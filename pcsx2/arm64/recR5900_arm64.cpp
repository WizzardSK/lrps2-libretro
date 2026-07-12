// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 EE (R5900) recompiler -- Phase C.3-2: native integer ALU translation.
//
// Builds on the C.3-1 code cache + dispatch. Each block now natively translates
// the leading run of simple integer ALU instructions to AArch64 via VIXL, then
// hands the rest of the basic block to the interpreter (eeRunBasicBlock_arm64),
// which keeps branch-delay-slot, memory, FPU/MMI/VU and exception semantics
// exact. The EE GPRs are 128-bit; integer ops touch only the low 64 bits
// (GPR.r[i].UD[0] at byte offset i*16). 32-bit MIPS ops produce a sign-extended
// 64-bit result; the D* forms are full 64-bit. Cycle accounting mirrors execI
// (cpuBlockCycles += opcode.cycles * (2 - CP0.Config[18])), summed per block via
// OpCycles() with the real per-class costs (Default 9, Branch 11, Mult 16, Div
// 112). Self-modifying code -> granular per-page (mirror-normalised) invalidation,
// as Cpu->Clear is called on EE writes.

#include "common/Pcsx2Types.h"
#include "R5900.h"
#include "R5900OpcodeTables.h"
#include "Memory.h"
#include "VU.h"
#include "common/Console.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <sys/syscall.h>
#include <unistd.h>
#include <sys/mman.h>

// #define EE_PC_SAMPLE 1 // TEMP: EE PC histogram diagnostic (disables block-linking when on)

#include "aarch64/macro-assembler-aarch64.h"

// C.30-2: native COP2 macro emission -- the vendored aVU_Macro entry points
// (aVU.cpp TU) + the AsmHelpers thread-locals its emitters write through, and
// the analysis-flag plumbing they gate on.
#include "arm64/AsmHelpers.h"
#include "arm64/aR5900Analysis.h"
#include "VUmicro.h" // _vu0FinishMicro

bool recVUMacroIsMode0(u32 op);   // aVU_Macro.inl (classify COP2 SPECIAL ALU)
bool recVUMacroEmitMode0(u32 op); // aVU_Macro.inl (emit via microVU0 single-op emitters)

using namespace vixl::aarch64;

extern "C" void eeRunBasicBlock_arm64(void); // Interpreter.cpp
extern u32 cpuBlockCycles;                    // Interpreter.cpp
// EE memory wrappers + misalign cancel (Interpreter.cpp), for native loads/stores.
extern "C" u32  eeRead8_arm64(u32),  eeRead16_arm64(u32), eeRead32_arm64(u32);
extern "C" u64  eeRead64_arm64(u32);
extern "C" void eeWrite8_arm64(u32, u32),  eeWrite16_arm64(u32, u32);
extern "C" void eeWrite32_arm64(u32, u32), eeWrite64_arm64(u32, u64);
extern "C" void eeRead128_arm64(u32, u128*), eeWrite128_arm64(u32, const u128*);
// Unaligned load/store family helpers (C.16): merge semantics, no misalign trap.
extern "C" void eeLWL_arm64(u32, u32), eeLWR_arm64(u32, u32);
extern "C" void eeLDL_arm64(u32, u32), eeLDR_arm64(u32, u32);
extern "C" void eeSWL_arm64(u32, u32), eeSWR_arm64(u32, u32);
extern "C" void eeSDL_arm64(u32, u32), eeSDR_arm64(u32, u32);
extern "C" void eeCancelInstruction_arm64(void);
extern "C" void eeEventTest_arm64(void);
extern "C" void eeUpdateCycles_arm64(void);
extern "C" void eeTraceHook(u32 pc); // Interpreter.cpp (TEMP diagnostic, LRPS2_TRACE)
extern "C" void eeLogExc(u32 pc);    // Interpreter.cpp (TEMP diagnostic, LRPS2_EXCLOG)

// Interpreter op implementations invoked directly from JIT blocks (C.18): ops
// with contained side effects (no control flow, no exception, no pc /
// cpuBlockCycles reads) execute via a plain call with cpuRegs.code staged,
// so the block continues natively instead of handing off its whole tail.
namespace R5900 { namespace Interpreter { namespace OpcodeImpl {
	void CACHE();
	void COP2(); // top-level COP2 dispatcher (Int_COP2PrintTable[rs])
	namespace COP0 { void MFC0(); void MTC0(); int CPCOND0(); }
	namespace MMI  { void QFSRV(); }
} } }

namespace
{
	typedef void (*BlockFn)(void);

	constexpr size_t kCodeCacheSize = 32 * 1024 * 1024;
	constexpr int    kMaxInsns      = 64;
	constexpr u32    kPageShift     = 12;

	u8*    s_code     = nullptr;
	size_t s_code_pos = 0;
	bool   s_ok       = false;

	// Block record with a snapshot of the NATIVE-range source words. The EE's
	// fault-based SMC protection must drop every block on a written page (after
	// the fault unprotects it, further writes to the page don't fault, so any
	// surviving block would go stale silently). That page-granular clear made
	// code/data-sharing pages recompile their blocks on every data write
	// (~8-12% of GT3 attract time inside vixl emission). Instead of erasing,
	// Clear marks blocks dead; the next dispatch revalidates the snapshot
	// against live memory and, when the code bytes are unchanged (the common
	// case -- the write hit data), reinstalls the existing native code and
	// re-protects the page. Bit-identical: a block is only revived if its
	// baked-in source is byte-identical.
	struct BlockRec
	{
		BlockFn fn;
		u32 end;  // Norm(one past the native-covered range); == Norm(pc) if none
		bool live;
		std::vector<u32> src; // words [Norm(pc), end) at compile time
	};
	std::unordered_map<u32, BlockRec>         s_blocks;
	std::unordered_map<u32, std::vector<u32>> s_page;

	// Direct-mapped block cache for the 32 MB EE RAM (the hot region): O(1) array
	// index instead of a hash lookup per block. ROM/scratchpad fall back to the
	// hash. (Foundation for cheaper block-ending; full block-linking is future.)
	constexpr u32 kRamBytes = 0x02000000;
	constexpr u32 kRamWords = kRamBytes >> 2;
	BlockFn*      s_lut      = nullptr;

	// C.40: chained blocks keep the 80-byte frame (and x19) live and jump
	// straight to the successor's body; only the return to the C++ dispatcher
	// pops the frame. The chain entry point skips the prologue, whose size is
	// therefore fixed: Sub + 5*Stp + exactly-4-instruction x19 materialization.
	constexpr u32 kChainEntryOffset = (1 + 5 + 4) * 4;

	inline u32  Norm(u32 a)  { return a & 0x1fffffff; }
	inline bool InRam(u32 np) { return np < kRamBytes; }
	inline void LutClearAll() { if (s_lut) madvise(s_lut, (size_t)kRamWords * sizeof(BlockFn), MADV_DONTNEED); }

	bool VixlEmitSelfTest()
	{
		MacroAssembler masm;
		masm.Add(x0, x0, 1);
		masm.Ret();
		masm.FinalizeCode();
		vixl::CodeBuffer* buf = masm.GetBuffer();
		buf->SetExecutable();
		auto fn = buf->GetStartAddress<int64_t (*)(int64_t)>();
		const int64_t r = fn(41);
		buf->SetWritable();
		return r == 42;
	}

	// Inline vtlb fast path (Phase C.12): with the guest address in w0 (upper
	// half of x0 zero, as left by 32-bit address arithmetic), emit the vmap
	// lookup and leave the HOST pointer in x10, branching to 'slow' when the
	// entry is a handler (hw register -> take the C wrapper call instead).
	// Mirrors vtlb_memRead/Write's direct case exactly: vmv = vmap[addr>>12];
	// handler iff (s64)(vmv.value + addr) < 0; host = vmv.value + addr.
	// x10/x11 are scratch; w0/x1/q0 (address/value) are preserved.
	inline bool InlineMemEnabled()
	{
		// LRPS2_WLOG needs every access to go through the wrappers (that's
		// where the watch hooks live); LRPS2_NO_INLINE_MEM forces it off for
		// A/B diagnosis.
		static int on = -1;
		if (on < 0) on = (getenv("LRPS2_WLOG") || getenv("LRPS2_NO_INLINE_MEM")) ? 0 : 1;
		return on != 0;
	}
	inline void EmitVmapLookup(MacroAssembler& m, Label* slow)
	{
		m.Mov(x10, reinterpret_cast<uint64_t>(&vtlb_private::vtlbdata.vmap));
		m.Ldr(x10, MemOperand(x10));                 // vmap array base (realloc-safe)
		m.Lsr(w11, w0, 12);
		m.Ldr(x10, MemOperand(x10, x11, LSL, 3));    // vmv.value
		m.Add(x10, x10, x0);                         // + guest addr
		m.Tbnz(x10, 63, slow);                       // sign bit = handler
	}

	// ---- C.27: block-local write-through GPR register cache ----
	// The low 64 bits of guest GPRs are mirrored in the callee-saved host regs
	// x21..x27 for the duration of a block: repeated reads become reg-reg Movs
	// instead of Ldrs. WRITE-THROUGH: every StoreGpr still writes cpuRegs.GPR
	// immediately, so memory stays authoritative at all times -- event tests,
	// misalign cancels (fastjmp restores callee-saved regs), interpreter
	// handoffs and the branch not-taken/taken fork need no flush logic at all.
	// The only hazard is a STALE CACHE after something writes GPR memory
	// without going through StoreGpr; those sites invalidate:
	//   MMI/PCPYH 128-bit results (rd), LQ (rt), the eeLWL/LWR/LDL/LDR helper
	//   calls (rt), and EmitInterpOpCall bodies (any reg -> invalidate all).
	// Helper C calls preserve x21..x27 (AAPCS64), so the cache survives them.
	// HI/LO live at byte offsets >= 512 and are never cached. Compilation is
	// EE-thread-only, so the compile-time state can be a plain static.
	constexpr int kCacheSlots = 7; // x21..x27
	inline Register CacheX(int slot) { return XRegister(21 + slot); }
	inline Register CacheW(int slot) { return WRegister(21 + slot); }
	struct RegCache
	{
		int8_t host_of[32];           // guest reg -> slot (-1 = uncached)
		int8_t guest_of[kCacheSlots]; // slot -> guest reg (-1 = free)
		bool   dirty[kCacheSlots];    // slot holds a value memory doesn't have yet
		u8 rr;                        // round-robin eviction cursor
		u8 pinned;                    // slots the CURRENT op holds Register handles to
		void Reset()
		{
			for (int i = 0; i < 32; i++) host_of[i] = -1;
			for (int i = 0; i < kCacheSlots; i++) { guest_of[i] = -1; dirty[i] = false; }
			rr = 0;
			pinned = 0;
		}
		void Invalidate(u32 g) // drop without write-back (memory was just overwritten)
		{
			if (g < 32 && host_of[g] >= 0)
			{
				const int s = host_of[g];
				guest_of[s] = -1; dirty[s] = false; host_of[g] = -1;
			}
		}
		// Claim a slot for guest reg g, write-back-evicting the round-robin
		// victim. Slots pinned by the current op (live Register handles from
		// SrcGpr/DstGpr) are skipped -- an op pins at most 3 of the 7 slots,
		// so the scan always terminates.
		int Alloc(MacroAssembler& m, const Register& gpr, u32 g)
		{
			int s;
			do
			{
				s = rr;
				rr = (u8)((rr + 1) % kCacheSlots);
			} while (pinned & (1u << s));
			if (guest_of[s] >= 0)
			{
				if (dirty[s]) m.Str(CacheX(s), MemOperand(gpr, (u32)guest_of[s] * 16));
				host_of[(int)guest_of[s]] = -1;
				dirty[s] = false;
			}
			guest_of[s] = (int8_t)g;
			host_of[g] = (int8_t)s;
			return s;
		}
		// Write one guest reg back to memory if dirty (mapping stays, now clean).
		void FlushReg(MacroAssembler& m, const Register& gpr, u32 g)
		{
			if (g >= 32 || host_of[g] < 0) return;
			const int s = host_of[g];
			if (!dirty[s]) return;
			m.Str(CacheX(s), MemOperand(gpr, g * 16));
			dirty[s] = false;
		}
		// Write all dirty regs back (mappings stay, now clean).
		void FlushDirty(MacroAssembler& m, const Register& gpr)
		{
			for (int s = 0; s < kCacheSlots; s++)
				if (guest_of[s] >= 0 && dirty[s])
				{
					m.Str(CacheX(s), MemOperand(gpr, (u32)guest_of[s] * 16));
					dirty[s] = false;
				}
		}
		// Emit write-backs WITHOUT touching compile-time state: for cold paths
		// (misalign -> eeCancelInstruction fastjmp) that leave the block while
		// the fall-through continuation keeps using the dirty state.
		void FlushDirtyColdPath(MacroAssembler& m, const Register& gpr)
		{
			for (int s = 0; s < kCacheSlots; s++)
				if (guest_of[s] >= 0 && dirty[s])
					m.Str(CacheX(s), MemOperand(gpr, (u32)guest_of[s] * 16));
		}
	};
	RegCache s_rc;
	inline bool RegCacheOn()
	{
		// LRPS2_TRACE_STEP's per-op hook hashes GPR MEMORY mid-block; with a
		// dirty write-back cache that state is deliberately stale, so the
		// trace tooling forces the cache off (block-entry tracing is fine --
		// every block exit flushes).
		static int on = -1;
		if (on < 0) on = (getenv("LRPS2_NO_EE_REGCACHE") || getenv("LRPS2_TRACE_STEP")) ? 0 : 1;
		return on != 0;
	}

	// EE GPRs are 128-bit; integer ops use the low 64 bits at byte offset idx*16.
	// A cached guest reg reads as a reg-reg Mov; a miss loads from memory and
	// fills a slot (the extra Mov is far cheaper than the next avoided load).
	inline void LoadGpr(MacroAssembler& m, const Register& xd, const Register& gpr, u32 idx)
	{
		if (idx == 0) { m.Mov(xd, 0); return; }
		if (!RegCacheOn()) { m.Ldr(xd, MemOperand(gpr, idx * 16)); return; }
		int s = s_rc.host_of[idx];
		if (s >= 0) { m.Mov(xd, CacheX(s)); return; }
		m.Ldr(xd, MemOperand(gpr, idx * 16));
		s = s_rc.Alloc(m, gpr, idx);
		m.Mov(CacheX(s), xd);
	}

	// Dirty write-back (C.27-2): the store lands only in the cache slot; memory
	// is written at the flush points (block exits, helper calls that observe
	// GPR memory, dirty-slot eviction, and the cold misalign-cancel paths).
	inline void StoreGpr(MacroAssembler& m, const Register& xs, const Register& gpr, u32 idx)
	{
		if (!RegCacheOn() || idx == 0)
		{
			m.Str(xs, MemOperand(gpr, idx * 16));
			return;
		}
		int s = s_rc.host_of[idx];
		if (s < 0) s = s_rc.Alloc(m, gpr, idx);
		m.Mov(CacheX(s), xs);
		s_rc.dirty[s] = true;
	}

	// ---- C.27-3: direct-to-cache emission API ----
	// Converted emitters get Register handles straight into the cache slots,
	// eliminating the load-into-x0 / store-from-x0 copy dance entirely:
	//   const Register a = SrcGpr(m, gpr, rs, x0);
	//   const Register d = DstGpr(m, gpr, rd, x0);
	//   m.Add(d.W(), a.W(), ...); m.Sxtw(d, d.W());
	//   FinishDst(m, gpr, rd, d);
	// With the cache ON: Src returns the (loaded) slot register or xzr for r0,
	// Dst returns the target slot marked dirty, FinishDst is a no-op; both pin
	// their slot so a later Alloc in the same op can't evict a live handle
	// (pins are cleared at every op boundary). With the cache OFF the same
	// call sequence degenerates to exactly the pre-C.27 code: Src loads into
	// the passed scratch, Dst returns the scratch, FinishDst stores it.
	// RULES for converted emitters: acquire all Srcs before the Dst; the
	// instruction that writes Dst must be the last reader of the Srcs (Dst
	// may alias a Src when the cache is off, or when rd==rs/rt).
	inline Register SrcGpr(MacroAssembler& m, const Register& gpr, u32 idx, const Register& scratch)
	{
		// r0 materializes as Mov scratch,0 -- NOT xzr: register 31 in the Rn
		// position of immediate/extended ADD/SUB/CMP encodes SP, so handing
		// out xzr would be a silent footgun for converted emitters.
		if (idx == 0) { m.Mov(scratch, 0); return scratch; }
		if (!RegCacheOn())
		{
			m.Ldr(scratch, MemOperand(gpr, idx * 16));
			return scratch;
		}
		int s = s_rc.host_of[idx];
		if (s < 0)
		{
			s = s_rc.Alloc(m, gpr, idx);
			m.Ldr(CacheX(s), MemOperand(gpr, idx * 16));
		}
		s_rc.pinned |= (u8)(1u << s);
		return CacheX(s);
	}
	// idx must be nonzero (callers guard r0 like they always have).
	inline Register DstGpr(MacroAssembler& m, const Register& gpr, u32 idx, const Register& scratch)
	{
		if (!RegCacheOn())
			return scratch;
		int s = s_rc.host_of[idx];
		if (s < 0) s = s_rc.Alloc(m, gpr, idx); // no load: fully overwritten
		s_rc.dirty[s] = true;
		s_rc.pinned |= (u8)(1u << s);
		return CacheX(s);
	}
	inline void FinishDst(MacroAssembler& m, const Register& gpr, u32 idx, const Register& d)
	{
		if (!RegCacheOn())
			m.Str(d, MemOperand(gpr, idx * 16));
	}

	// COP1 (FPU) register sub-ops that are bit-exact and safe to translate
	// natively: the pure bit moves (MFC1/MTC1/MOV.S/NEG.S/ABS.S) plus, as of
	// C.23/C.24, the S-format arithmetic (native AArch64 FP ops with the EE's
	// fpuDouble()-clamp / checkOverflow/checkUnderflow / checkDivideByZero
	// semantics -- see EmitFpuBinaryS/EmitDivS/EmitSqrtS/EmitRsqrtS/
	// EmitMaxMinS). CVT, C.cond, MADD/MSUB/*A_S, BC1 and CFC1/CTC1 stay with
	// the interpreter (a later increment).
	//   rs=0x00 MFC1, rs=0x04 MTC1,
	//   rs=0x10 + funct {0x00 ADD.S,0x01 SUB.S,0x02 MUL.S,0x03 DIV.S,0x04 SQRT.S,
	//                     0x05 ABS.S,0x06 MOV.S,0x07 NEG.S,0x16 RSQRT.S,
	//                     0x18 ADDA.S,0x19 SUBA.S,0x1a MULA.S,
	//                     0x1c MADD.S,0x1d MSUB.S,0x1e MADDA.S,0x1f MSUBA.S,
	//                     0x28 MAX.S,0x29 MIN.S}
	bool Cop1RegSupported(u32 insn)
	{
		const u32 rs = (insn >> 21) & 31;
		if (rs == 0x00 || rs == 0x04) return true;
		if (rs == 0x14) return (insn & 0x3f) == 0x20; // W format: CVT.S.W
		if (rs == 0x10)
		{
			const u32 funct = insn & 0x3f;
			return funct == 0x00 || funct == 0x01 || funct == 0x02 || funct == 0x03 ||
			       funct == 0x04 || funct == 0x05 || funct == 0x06 || funct == 0x07 ||
			       funct == 0x16 || (funct >= 0x18 && funct <= 0x1a) ||
			       (funct >= 0x1c && funct <= 0x1f) || funct == 0x24 ||
			       funct == 0x28 || funct == 0x29 ||
			       funct == 0x30 || funct == 0x32 || funct == 0x34 || funct == 0x36;
		}
		return false;
	}

	// Is this a Cop1RegSupported() instruction specifically one of the S-format
	// arithmetic/compare/convert ops (everything except the plain bit moves),
	// for the LRPS2_NO_EE_FPU_ARITH bisect toggle (finer-grained than the
	// blanket LRPS2_NO_EE_COP1, which also covers the plain bit-move ops).
	bool Cop1ArithSupported(u32 insn)
	{
		const u32 rs = (insn >> 21) & 31;
		if (rs == 0x14) return true; // CVT.S.W
		if (rs != 0x10) return false;
		const u32 funct = insn & 0x3f;
		return funct == 0x00 || funct == 0x01 || funct == 0x02 || funct == 0x03 ||
		       funct == 0x04 || funct == 0x16 || (funct >= 0x18 && funct <= 0x1a) ||
		       (funct >= 0x1c && funct <= 0x1f) || funct == 0x24 ||
		       funct == 0x28 || funct == 0x29 ||
		       funct == 0x30 || funct == 0x32 || funct == 0x34 || funct == 0x36;
	}

	// ---- C.23: native S-format ADD/SUB/MUL (bit-exact port of pcsx2/FPU.cpp) ----
	// The interpreter's ADD_S/SUB_S/MUL_S are, e.g.:
	//   _FdValf_ = fpuDouble(Fs) + fpuDouble(Ft);
	//   if (checkOverflow(_FdValUl_, O|SO)) return;
	//   checkUnderflow(_FdValUl_, U|SU);
	// where fpuDouble() clamps a raw single INPUT (denormal/zero -> signed zero;
	// inf/nan -> signed posFmax 0x7f7fffff; else passthrough) and the "+"/"-"/"*"
	// is a real IEEE single-precision op (fpuDouble returns a `float`, despite the
	// name). This is NOT the x86 recompiler's double-promotion "full" mode (see
	// armsx2-pcsx2's aR5900FPU.cpp emitToPS2FPUFullCore) -- that targets a
	// different ground truth (the x86 JIT) with extra magnitude-preserving range.
	// Our target is the plain interpreter, so the natural translation is: clamp
	// each operand's raw bits, do the arithmetic natively in AArch64 single
	// precision (Sn regs -- exactly matches host `float op float`, no guard-bit
	// masking trickery needed since we never promote to double), then clamp the
	// raw result bits the same way checkOverflow/checkUnderflow do. Verified safe
	// against host FPCR: EE blocks always run with FPCR left at its process
	// default (bitmask 0, no flush-to-zero) -- only VU1 briefly swaps FPCR
	// (recVU1_arm64.cpp, RAII-restored), so denormal results here are never
	// silently flushed by the host before we can inspect them.

	// Clamp a raw single bit pattern per fpuDouble(). wtmp is scratch.
	inline void EmitFpuClampBits(MacroAssembler& m, const Register& wd, const Register& wtmp)
	{
		Label notZero, done;
		m.And(wtmp, wd, 0x7f800000);      // exponent field
		m.Cbnz(wtmp, &notZero);
		m.And(wd, wd, 0x80000000);        // exp==0 (zero/denormal) -> signed zero
		m.B(&done);
		m.Bind(&notZero);
		m.Cmp(wtmp, 0x7f800000);
		m.B(&done, ne);                   // normal finite -> untouched
		m.And(wtmp, wd, 0x80000000);
		m.Orr(wd, wtmp, 0x7f7fffff);      // exp==0xff -> signed posFmax
		m.Bind(&done);
	}

	// checkOverflow(wd, O|SO) then, only if no overflow, checkUnderflow(wd, U|SU) --
	// exact mirror of pcsx2/FPU.cpp for the ADD_S/SUB_S/MUL_S family. wd holds the
	// raw result bits (updated in place with the clamped value); xfbase must hold
	// &fpuRegs. Scratch: w2, w3.
	inline void EmitFpuOverflowUnderflow(MacroAssembler& m, const Register& wd, const Register& xfbase)
	{
		Label overflow, notDenormal, done;

		m.And(w2, wd, 0x7fffffff);
		m.Cmp(w2, 0x7f800000);
		m.B(&overflow, eq);

		// no overflow: checkOverflow's else-branch clears FPUflagO only.
		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Bic(w3, w3, 0x8000);
		m.Str(w3, MemOperand(xfbase, 252));

		// checkUnderflow: exp==0 && mantissa!=0 (true denormal) -> flush + U|SU;
		// else clear U only (exact zero is NOT underflow).
		m.And(w2, wd, 0x7f800000);
		m.Cbnz(w2, &notDenormal);
		m.And(w2, wd, 0x7fffff);
		m.Cbz(w2, &notDenormal);
		m.And(wd, wd, 0x80000000);        // flush to signed zero
		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Orr(w3, w3, 0x4000);
		m.Orr(w3, w3, 0x8);
		m.Str(w3, MemOperand(xfbase, 252));
		m.B(&done);

		m.Bind(&notDenormal);
		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Bic(w3, w3, 0x4000);
		m.Str(w3, MemOperand(xfbase, 252));
		m.B(&done);

		m.Bind(&overflow);
		m.And(w3, wd, 0x80000000);
		m.Orr(wd, w3, 0x7f7fffff);        // signed posFmax
		m.Ldr(w2, MemOperand(xfbase, 252));
		m.Orr(w2, w2, 0x8000);
		m.Orr(w2, w2, 0x10);
		m.Str(w2, MemOperand(xfbase, 252));

		m.Bind(&done);
	}

	// The ACC accumulator lives right after fpr[32] (128 B) + fprc[32] (128 B).
	constexpr u32 kFpuAcc = 256;

	// op: 0 add, 1 sub, 2 mul (the low bits of both the ADD/SUB/MUL.S funct
	// 0x00/0x01/0x02 and the ADDA/SUBA/MULA.S funct 0x18/0x19/0x1a). x1 must
	// hold &fpuRegs (caller already loaded it -- see EmitCop1Reg). Result ->
	// fpuRegs byte offset dst_off (fd*4, or kFpuAcc for the A forms; the
	// interpreter's ADDA/SUBA/MULA are line-identical to ADD/SUB/MUL apart
	// from the destination, and none of them touch ACCflag -- that field is
	// x86-recompiler-internal).
	void EmitFpuBinaryS(MacroAssembler& m, const Register& xfbase, u32 op, u32 dst_off, u32 fs, u32 ft)
	{
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Ldr(w0, MemOperand(xfbase, ft * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s1, w0);
		switch (op)
		{
			case 0:  m.Fadd(s0, s0, s1); break;
			case 1:  m.Fsub(s0, s0, s1); break;
			default: m.Fmul(s0, s0, s1); break; // 2
		}
		m.Fmov(w0, s0);
		EmitFpuOverflowUnderflow(m, w0, xfbase);
		m.Str(w0, MemOperand(xfbase, dst_off));
	}

	// funct 0x1c MADD.S / 0x1d MSUB.S: temp = fpuDouble(Fs)*fpuDouble(Ft);
	// fd = fpuDouble(ACC) +/- fpuDouble(temp.UL) -- BOTH the intermediate
	// product and ACC are re-clamped through fpuDouble before the add/sub
	// (pcsx2/FPU.cpp MADD_S/MSUB_S). Then the usual O|SO / U|SU result check.
	// There is NO fused multiply-add on the EE -- the product is a rounded
	// single, so native Fmul + Fadd (not Fmadd) is the exact translation.
	void EmitFpuMaddS(MacroAssembler& m, const Register& xfbase, u32 fd, u32 fs, u32 ft, bool sub)
	{
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Ldr(w0, MemOperand(xfbase, ft * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s1, w0);
		m.Fmul(s0, s0, s1);
		m.Fmov(w0, s0);
		EmitFpuClampBits(m, w0, w4);   // fpuDouble(temp.UL)
		m.Fmov(s0, w0);
		m.Ldr(w0, MemOperand(xfbase, kFpuAcc));
		EmitFpuClampBits(m, w0, w4);   // fpuDouble(ACC.UL)
		m.Fmov(s1, w0);
		if (sub) m.Fsub(s0, s1, s0);   // fd = ACC - temp
		else     m.Fadd(s0, s1, s0);   // fd = ACC + temp
		m.Fmov(w0, s0);
		EmitFpuOverflowUnderflow(m, w0, xfbase);
		m.Str(w0, MemOperand(xfbase, fd * 4));
	}

	// funct 0x1e MADDA.S / 0x1f MSUBA.S: ACC.f +/-= fpuDouble(Fs)*fpuDouble(Ft).
	// UNLIKE MADD/MSUB, neither the product nor ACC goes through fpuDouble here
	// (the interpreter's `+=` reads the raw ACC float and adds the raw IEEE
	// product) -- a real asymmetry in the ground truth; replicate, don't
	// "harmonize". Then O|SO / U|SU on the accumulated result.
	void EmitFpuMaddaS(MacroAssembler& m, const Register& xfbase, u32 fs, u32 ft, bool sub)
	{
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Ldr(w0, MemOperand(xfbase, ft * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s1, w0);
		m.Fmul(s0, s0, s1);
		m.Ldr(w0, MemOperand(xfbase, kFpuAcc)); // raw ACC, no clamp
		m.Fmov(s1, w0);
		if (sub) m.Fsub(s0, s1, s0);
		else     m.Fadd(s0, s1, s0);
		m.Fmov(w0, s0);
		EmitFpuOverflowUnderflow(m, w0, xfbase);
		m.Str(w0, MemOperand(xfbase, kFpuAcc));
	}

	// checkOverflow(wd,0) + checkUnderflow(wd,0): DIV.S/SQRT.S/RSQRT.S pass
	// cFlagsToSet=0 to both checks (pcsx2/FPU.cpp), so the raw result bits get
	// value-clamped exactly like EmitFpuOverflowUnderflow but FCR31 is NEVER
	// touched (neither set nor cleared) -- a real ground-truth quirk, not an
	// oversight, so no fbase param here.
	inline void EmitFpuClampOutValueOnly(MacroAssembler& m, const Register& wd)
	{
		Label overflow, done;
		m.And(w2, wd, 0x7fffffff);
		m.Cmp(w2, 0x7f800000);
		m.B(&overflow, eq);
		m.And(w2, wd, 0x7f800000);
		m.Cbnz(w2, &done);
		m.And(w2, wd, 0x7fffff);
		m.Cbz(w2, &done);
		m.And(wd, wd, 0x80000000);
		m.B(&done);
		m.Bind(&overflow);
		m.And(w3, wd, 0x80000000);
		m.Orr(wd, w3, 0x7f7fffff);
		m.Bind(&done);
	}

	// funct 0x03 DIV.S: checkDivideByZero(fd, Ft, Fs, D|SD, I|SI) first -- if Ft's
	// raw exponent field is 0 (zero/denormal): set D|SD, or I|SI instead if Fs's
	// exponent field is ALSO 0 (0/0); fd = sign(Ft^Fs) | posFmax; EARLY RETURN, no
	// overflow/underflow check at all. Otherwise fd = fpuDouble(Fs)/fpuDouble(Ft),
	// then checkOverflow(0)/checkUnderflow(0) (value-clamp only, no flags).
	void EmitDivS(MacroAssembler& m, const Register& xfbase, u32 fd, u32 fs, u32 ft)
	{
		Label divByZero, done;

		m.Ldr(w5, MemOperand(xfbase, ft * 4)); // raw Ft: divisor sign + zero-check
		m.And(w2, w5, 0x7f800000);
		m.Cbz(w2, &divByZero);

		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Mov(w0, w5);
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s1, w0);
		m.Fdiv(s0, s0, s1);
		m.Fmov(w0, s0);
		EmitFpuClampOutValueOnly(m, w0);
		m.Str(w0, MemOperand(xfbase, fd * 4));
		m.B(&done);

		m.Bind(&divByZero);
		{
			Label zeroFs, flagsDone;
			m.Ldr(w6, MemOperand(xfbase, fs * 4)); // raw Fs: 0/0 check + result sign
			m.Ldr(w3, MemOperand(xfbase, 252));
			m.And(w2, w6, 0x7f800000);
			m.Cbz(w2, &zeroFs);
			m.Orr(w3, w3, 0x10000); // D
			m.Orr(w3, w3, 0x20);    // SD
			m.B(&flagsDone);
			m.Bind(&zeroFs);
			m.Orr(w3, w3, 0x20000); // I
			m.Orr(w3, w3, 0x40);    // SI
			m.Bind(&flagsDone);
			m.Str(w3, MemOperand(xfbase, 252));
			m.Eor(w0, w5, w6);
			m.And(w0, w0, 0x80000000);
			m.Orr(w0, w0, 0x7f7fffff);
			m.Str(w0, MemOperand(xfbase, fd * 4));
		}

		m.Bind(&done);
	}

	// funct 0x04 SQRT.S: single operand in the Ft field (not Fs -- matches the
	// x86 recompiler's XMMINFO_READT, a real EE quirk vs standard MIPS unary
	// COP1 encoding). Unconditionally clears I|D, then: Ft exponent-field==0 ->
	// fd = signed zero (Ft's sign, no extra flags); Ft negative -> sets I|SI,
	// fd = +sqrt(|fpuDouble(Ft)|) (positive magnitude only -- sqrt of a negative
	// does NOT reapply the sign); Ft positive -> fd = sqrt(fpuDouble(Ft)). No
	// overflow/underflow clamp at all in any path (ground truth has none).
	void EmitSqrtS(MacroAssembler& m, const Register& xfbase, u32 fd, u32 ft)
	{
		Label isZero, isNeg, done;

		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Bic(w3, w3, 0x30000); // clear I|D unconditionally
		m.Str(w3, MemOperand(xfbase, 252));

		m.Ldr(w0, MemOperand(xfbase, ft * 4));
		m.And(w2, w0, 0x7f800000);
		m.Cbz(w2, &isZero);
		m.Tst(w0, 0x80000000);
		m.B(&isNeg, ne);

		// positive, nonzero-exp
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Fsqrt(s0, s0);
		m.Fmov(w0, s0);
		m.Str(w0, MemOperand(xfbase, fd * 4));
		m.B(&done);

		m.Bind(&isNeg);
		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Orr(w3, w3, 0x20000); // I
		m.Orr(w3, w3, 0x40);    // SI
		m.Str(w3, MemOperand(xfbase, 252));
		EmitFpuClampBits(m, w0, w4);
		m.And(w0, w0, 0x7fffffff); // fabs
		m.Fmov(s0, w0);
		m.Fsqrt(s0, s0);
		m.Fmov(w0, s0);
		m.Str(w0, MemOperand(xfbase, fd * 4));
		m.B(&done);

		m.Bind(&isZero);
		m.And(w0, w0, 0x80000000);
		m.Str(w0, MemOperand(xfbase, fd * 4));

		m.Bind(&done);
	}

	// funct 0x16 RSQRT.S: fd = Fs / rsqrt-source(Ft). Unconditionally clears D|I,
	// then: Ft exponent-field==0 -> sets D|SD, fd = sign(Ft raw) | posFmax
	// (NOT xor'd with Fs, unlike DIV.S's inline zero-check -- a genuine
	// asymmetry in the ground truth), EARLY RETURN (no overflow/underflow).
	// Ft negative -> sets I|SI, divisor = fpuDouble(sqrt(|fpuDouble(Ft)|)) (the
	// sqrt RESULT is re-clamped through fpuDouble before dividing). Ft positive
	// -> divisor = sqrt(fpuDouble(Ft)) (sqrt result NOT re-clamped -- another
	// real asymmetry vs the negative path; replicate both exactly, do not
	// "simplify" to match). Both non-early-return paths fall through to
	// checkOverflow(0)/checkUnderflow(0) (value-clamp only, no flags).
	void EmitRsqrtS(MacroAssembler& m, const Register& xfbase, u32 fd, u32 fs, u32 ft)
	{
		Label isZero, isNeg, compute, done;

		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Bic(w3, w3, 0x30000); // clear D|I unconditionally
		m.Str(w3, MemOperand(xfbase, 252));

		m.Ldr(w5, MemOperand(xfbase, ft * 4)); // raw Ft
		m.And(w2, w5, 0x7f800000);
		m.Cbz(w2, &isZero);
		m.Tst(w5, 0x80000000);
		m.B(&isNeg, ne);

		// positive, nonzero-exp: s1 = sqrt(fpuDouble(Ft)), NOT re-clamped
		m.Mov(w0, w5);
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s1, w0);
		m.Fsqrt(s1, s1);
		m.B(&compute);

		m.Bind(&isNeg);
		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Orr(w3, w3, 0x20000); // I
		m.Orr(w3, w3, 0x40);    // SI
		m.Str(w3, MemOperand(xfbase, 252));
		m.Mov(w0, w5);
		EmitFpuClampBits(m, w0, w4);
		m.And(w0, w0, 0x7fffffff); // fabs
		m.Fmov(s1, w0);
		m.Fsqrt(s1, s1);
		m.Fmov(w0, s1);
		EmitFpuClampBits(m, w0, w4); // re-clamp temp.UL before dividing
		m.Fmov(s1, w0);
		m.B(&compute);

		m.Bind(&compute);
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Fdiv(s0, s0, s1);
		m.Fmov(w0, s0);
		EmitFpuClampOutValueOnly(m, w0);
		m.Str(w0, MemOperand(xfbase, fd * 4));
		m.B(&done);

		m.Bind(&isZero);
		m.Ldr(w3, MemOperand(xfbase, 252));
		m.Orr(w3, w3, 0x10000); // D
		m.Orr(w3, w3, 0x20);    // SD
		m.Str(w3, MemOperand(xfbase, 252));
		m.And(w0, w5, 0x80000000);
		m.Orr(w0, w0, 0x7f7fffff);
		m.Str(w0, MemOperand(xfbase, fd * 4));

		m.Bind(&done);
	}

	// funct 0x24 CVT.W.S: if the exponent field of the RAW single (no fpuDouble
	// clamp!) is <= 0x4E800000, fd = (s32)Fs.f (C cast = truncate toward zero,
	// exactly Fcvtzs); else out-of-range: positive -> 0x7fffffff, negative ->
	// 0x80000000. Denormals/zero have exp 0 -> in-range -> truncate to 0.
	void EmitCvtW(MacroAssembler& m, const Register& xfbase, u32 fd, u32 fs)
	{
		Label outOfRange, done;
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		m.And(w2, w0, 0x7f800000);
		m.Mov(w3, 0x4E800000);
		m.Cmp(w2, w3);
		m.B(&outOfRange, hi);
		m.Fmov(s0, w0);
		m.Fcvtzs(w2, s0);
		m.Str(w2, MemOperand(xfbase, fd * 4));
		m.B(&done);
		m.Bind(&outOfRange);
		m.Mov(w2, 0x7fffffff);
		m.Mov(w3, 0x80000000);
		m.Tst(w0, 0x80000000);
		m.Csel(w2, w3, w2, ne);
		m.Str(w2, MemOperand(xfbase, fd * 4));
		m.Bind(&done);
	}

	// rs=0x14 (W format) funct 0x20 CVT.S.W: fd = (float)Fs.SL -- plain signed
	// int32 -> single conversion (Scvtf, round-to-nearest = host default FPCR,
	// which EE blocks always run under). No flags, no clamps.
	void EmitCvtS(MacroAssembler& m, const Register& xfbase, u32 fd, u32 fs)
	{
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		m.Scvtf(s0, w0);
		m.Fmov(w0, s0);
		m.Str(w0, MemOperand(xfbase, fd * 4));
	}

	// funct 0x30 C.F / 0x32 C.EQ / 0x34 C.LT / 0x36 C.LE: compare
	// fpuDouble-clamped operands (comparePrecision is compiled out, so the
	// interpreter compares the exact clamped floats) and set/clear the FCR31
	// C flag (0x00800000). C.F unconditionally clears it. Post-clamp values
	// are never NaN, so the ordered integer-style conditions (eq/lt/le) after
	// Fcmp are exact.
	void EmitFpuCmpS(MacroAssembler& m, const Register& xfbase, u32 funct, u32 fs, u32 ft)
	{
		if (funct == 0x30) // C.F: clear C only
		{
			m.Ldr(w2, MemOperand(xfbase, 252));
			m.Bic(w2, w2, 0x00800000);
			m.Str(w2, MemOperand(xfbase, 252));
			return;
		}
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s0, w0);
		m.Ldr(w0, MemOperand(xfbase, ft * 4));
		EmitFpuClampBits(m, w0, w4);
		m.Fmov(s1, w0);
		m.Fcmp(s0, s1);
		m.Cset(w0, funct == 0x32 ? eq : (funct == 0x34 ? lt : le));
		m.Ldr(w2, MemOperand(xfbase, 252));
		m.Bic(w2, w2, 0x00800000);
		m.Orr(w2, w2, Operand(w0, LSL, 23));
		m.Str(w2, MemOperand(xfbase, 252));
	}

	// funct 0x28 MAX.S / 0x29 MIN.S: pure signed-int32 comparison on the raw
	// bits (pcsx2/FPU.cpp fp_max/fp_min) -- NOT a float compare: no fpuDouble
	// clamp, no overflow/underflow. Two negative operands invert the natural
	// signed-int order vs float magnitude, hence the bothNeg swap. Clears O|U
	// (same bits ABS_S/NEG_S clear). NOTE: xfbase is x1, so w1 must NOT be
	// used as a value register here -- the C.24 crash was exactly that
	// (`Ldr(w1, [x1, ...])` zeroed the base, sending the result store to
	// ~NULL); values live in w0/w2..w5 instead.
	void EmitMaxMinS(MacroAssembler& m, const Register& xfbase, u32 fd, u32 fs, u32 ft, bool isMax)
	{
		m.Ldr(w0, MemOperand(xfbase, fs * 4));
		m.Ldr(w2, MemOperand(xfbase, ft * 4));
		m.Cmp(w0, w2);           // signed s32 compare
		m.Csel(w3, w0, w2, gt);  // w3 = max(fs,ft)
		m.Csel(w4, w0, w2, lt);  // w4 = min(fs,ft)
		m.And(w5, w0, w2);
		m.Tst(w5, 0x80000000);   // both negative?
		if (isMax) m.Csel(w0, w4, w3, ne); // bothNeg -> min, else -> max
		else       m.Csel(w0, w3, w4, ne); // bothNeg -> max, else -> min
		m.Str(w0, MemOperand(xfbase, fd * 4));
		m.Ldr(w2, MemOperand(xfbase, 252));
		m.Bic(w2, w2, 0xC000); // clear O|U
		m.Str(w2, MemOperand(xfbase, 252));
	}

	// fpuRegisters layout: FPRreg fpr[32] (4 bytes each) at offset 0; u32 fprc[32]
	// at offset 128 -> FCR31 (the status/cause reg) at offset 128 + 31*4 = 252.
	void EmitCop1Reg(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		const u32 rs    = (insn >> 21) & 31;
		const u32 rt    = (insn >> 16) & 31;   // GPR for MFC1/MTC1
		const u32 fs    = (insn >> 11) & 31;
		const u32 fd    = (insn >>  6) & 31;
		const u32 funct = insn & 0x3f;
		const uint64_t fbase = reinterpret_cast<uint64_t>(&fpuRegs);

		if (rs == 0x00) // MFC1 rt, fs : GPR[rt] = sign_extend_32_to_64(fpr[fs].UL)
		{
			if (rt) { m.Mov(x1, fbase); m.Ldr(w0, MemOperand(x1, fs * 4)); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rt); }
			return;
		}
		if (rs == 0x04) // MTC1 rt, fs : fpr[fs].UL = GPR[rt].UL[0]
		{
			m.Mov(x1, fbase); LoadGpr(m, x0, gpr, rt); m.Str(w0, MemOperand(x1, fs * 4));
			return;
		}
		if (rs == 0x14) // W format: CVT.S.W only
		{
			m.Mov(x1, fbase);
			EmitCvtS(m, x1, fd, fs);
			return;
		}
		// rs == 0x10 single-precision ops
		const u32 ft = (insn >> 16) & 31;
		m.Mov(x1, fbase);
		if (funct == 0x00 || funct == 0x01 || funct == 0x02) // ADD.S/SUB.S/MUL.S
		{
			EmitFpuBinaryS(m, x1, funct, fd * 4, fs, ft);
			return;
		}
		if (funct == 0x18 || funct == 0x19 || funct == 0x1a) // ADDA.S/SUBA.S/MULA.S
		{
			EmitFpuBinaryS(m, x1, funct & 3, kFpuAcc, fs, ft);
			return;
		}
		if (funct == 0x03) { EmitDivS(m, x1, fd, fs, ft); return; }   // DIV.S
		if (funct == 0x04) { EmitSqrtS(m, x1, fd, ft); return; }     // SQRT.S (source is Ft, not Fs)
		if (funct == 0x16) { EmitRsqrtS(m, x1, fd, fs, ft); return; } // RSQRT.S
		if (funct == 0x1c) { EmitFpuMaddS(m, x1, fd, fs, ft, false); return; } // MADD.S
		if (funct == 0x1d) { EmitFpuMaddS(m, x1, fd, fs, ft, true); return; }  // MSUB.S
		if (funct == 0x1e) { EmitFpuMaddaS(m, x1, fs, ft, false); return; }    // MADDA.S
		if (funct == 0x1f) { EmitFpuMaddaS(m, x1, fs, ft, true); return; }     // MSUBA.S
		if (funct == 0x24) { EmitCvtW(m, x1, fd, fs); return; }               // CVT.W.S
		if (funct == 0x28) { EmitMaxMinS(m, x1, fd, fs, ft, true); return; }  // MAX.S
		if (funct == 0x29) { EmitMaxMinS(m, x1, fd, fs, ft, false); return; } // MIN.S
		if (funct == 0x30 || funct == 0x32 || funct == 0x34 || funct == 0x36) // C.F/C.EQ/C.LT/C.LE
		{
			EmitFpuCmpS(m, x1, funct, fs, ft);
			return;
		}
		m.Ldr(w0, MemOperand(x1, fs * 4));
		if (funct == 0x06) { m.Str(w0, MemOperand(x1, fd * 4)); return; } // MOV.S
		if (funct == 0x05) m.And(w0, w0, 0x7fffffff);                     // ABS.S
		else               m.Eor(w0, w0, 0x80000000);                    // NEG.S
		m.Str(w0, MemOperand(x1, fd * 4));
		// clear the O|U cause flags in FCR31 (ABS.S/NEG.S do clearFPUFlags(O|U))
		m.Ldr(w0, MemOperand(x1, 252));
		m.Bic(w0, w0, 0xC000);
		m.Str(w0, MemOperand(x1, 252));
	}

	// ---- MMI (opcode 0x1C): 128-bit parallel integer ops -> ARM NEON ----
	// The EE GPRs are 128 bits; MMI does packed byte/half/word arithmetic over the
	// whole register, which maps 1:1 onto AArch64 NEON Q-registers. We translate
	// only the bit-exact, side-effect-free ops (parallel add/sub incl. saturating,
	// logical, signed compares, signed max/min, the EXTL/EXTU interleaves, the
	// immediate shifts, the PPAC* even-element packs and PCPYLD/PCPYUD). PMFHL/
	// PMTHL, the HI/LO multiply-divide ops, variable shifts, PMADD, the remaining
	// shuffles (PINTH/PEXEH/PREVH/PCPYH...), etc. stay with the interpreter.
	enum { M_ADD, M_SUB, M_SQADD, M_UQADD, M_SQSUB, M_UQSUB, M_AND, M_ORR, M_EOR,
	       M_NOR, M_CMGT, M_CMEQ, M_SMAX, M_SMIN, M_ZIP1, M_ZIP2,
	       M_SLLI, M_SRLI, M_SRAI, // immediate shifts (fmt 1=.8H sa&0xf, 2=.4S sa)
	       M_PACK,                 // PPACB/H/W: even elements, rt -> low, rs -> high
	       M_CPYLD, M_CPYUD };     // PCPYLD/PCPYUD (64-bit lane zips)

	// Decode an MMI instruction to (action, lane fmt: 0=16B,1=8H,2=4S). Returns the
	// action or -1 if not natively supported. funct selects the sub-class
	// (MMI0=0x08, MMI1=0x28, MMI2=0x09, MMI3=0x29); (insn>>6)&0x1f the op.
	int MmiAction(u32 insn, int* fmt)
	{
		const u32 funct = insn & 0x3f;
		const u32 sub   = (insn >> 6) & 0x1f;
		switch (funct)
		{
			// top-level MMI functs: immediate shifts (sub = shift amount)
			case 0x34: *fmt = 1; return M_SLLI; // PSLLH
			case 0x36: *fmt = 1; return M_SRLI; // PSRLH
			case 0x37: *fmt = 1; return M_SRAI; // PSRAH
			case 0x3c: *fmt = 2; return M_SLLI; // PSLLW
			case 0x3e: *fmt = 2; return M_SRLI; // PSRLW
			case 0x3f: *fmt = 2; return M_SRAI; // PSRAW
			case 0x08: // MMI0
				switch (sub)
				{
					case 0x00: *fmt = 2; return M_ADD;   // PADDW
					case 0x01: *fmt = 2; return M_SUB;   // PSUBW
					case 0x02: *fmt = 2; return M_CMGT;  // PCGTW
					case 0x03: *fmt = 2; return M_SMAX;  // PMAXW
					case 0x04: *fmt = 1; return M_ADD;   // PADDH
					case 0x05: *fmt = 1; return M_SUB;   // PSUBH
					case 0x06: *fmt = 1; return M_CMGT;  // PCGTH
					case 0x07: *fmt = 1; return M_SMAX;  // PMAXH
					case 0x08: *fmt = 0; return M_ADD;   // PADDB
					case 0x09: *fmt = 0; return M_SUB;   // PSUBB
					case 0x0a: *fmt = 0; return M_CMGT;  // PCGTB
					case 0x10: *fmt = 2; return M_SQADD; // PADDSW
					case 0x11: *fmt = 2; return M_SQSUB; // PSUBSW
					case 0x12: *fmt = 2; return M_ZIP1;  // PEXTLW
					case 0x13: *fmt = 2; return M_PACK;  // PPACW
					case 0x14: *fmt = 1; return M_SQADD; // PADDSH
					case 0x15: *fmt = 1; return M_SQSUB; // PSUBSH
					case 0x16: *fmt = 1; return M_ZIP1;  // PEXTLH
					case 0x17: *fmt = 1; return M_PACK;  // PPACH
					case 0x18: *fmt = 0; return M_SQADD; // PADDSB
					case 0x19: *fmt = 0; return M_SQSUB; // PSUBSB
					case 0x1a: *fmt = 0; return M_ZIP1;  // PEXTLB
					case 0x1b: *fmt = 0; return M_PACK;  // PPACB
				}
				return -1;
			case 0x28: // MMI1
				switch (sub)
				{
					case 0x02: *fmt = 2; return M_CMEQ;  // PCEQW
					case 0x03: *fmt = 2; return M_SMIN;  // PMINW
					case 0x06: *fmt = 1; return M_CMEQ;  // PCEQH
					case 0x07: *fmt = 1; return M_SMIN;  // PMINH
					case 0x0a: *fmt = 0; return M_CMEQ;  // PCEQB
					case 0x10: *fmt = 2; return M_UQADD; // PADDUW
					case 0x11: *fmt = 2; return M_UQSUB; // PSUBUW
					case 0x12: *fmt = 2; return M_ZIP2;  // PEXTUW
					case 0x14: *fmt = 1; return M_UQADD; // PADDUH
					case 0x15: *fmt = 1; return M_UQSUB; // PSUBUH
					case 0x16: *fmt = 1; return M_ZIP2;  // PEXTUH
					case 0x18: *fmt = 0; return M_UQADD; // PADDUB
					case 0x19: *fmt = 0; return M_UQSUB; // PSUBUB
					case 0x1a: *fmt = 0; return M_ZIP2;  // PEXTUB
				}
				return -1;
			case 0x09: // MMI2
				if (sub == 0x0e) { *fmt = 3; return M_CPYLD; } // PCPYLD
				if (sub == 0x12) { *fmt = 0; return M_AND; }   // PAND
				if (sub == 0x13) { *fmt = 0; return M_EOR; }   // PXOR
				return -1;
			case 0x29: // MMI3
				if (sub == 0x0e) { *fmt = 3; return M_CPYUD; } // PCPYUD
				if (sub == 0x12) { *fmt = 0; return M_ORR; }   // POR
				if (sub == 0x13) { *fmt = 0; return M_NOR; }   // PNOR
				return -1;
		}
		return -1;
	}

	bool MmiSupported(u32 insn) { int f; return MmiAction(insn, &f) >= 0; }

	// Load a full 128-bit GPR into a NEON Q-register (r0 reads as all-zero).
	void LoadQ(MacroAssembler& m, const VRegister& q, const Register& gpr, u32 idx)
	{
		if (idx == 0) m.Movi(q.V2D(), (uint64_t)0);
		else          m.Ldr(q, MemOperand(gpr, idx * 16));
	}

	void EmitMmi(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		int fmt;
		const int act = MmiAction(insn, &fmt);
		const u32 rd = (insn >> 11) & 31, rs = (insn >> 21) & 31, rt = (insn >> 16) & 31;
		if (!rd) return; // result discarded (r0 stays zero)

		s_rc.FlushReg(m, gpr, rs); // 128-bit sources are read from memory
		s_rc.FlushReg(m, gpr, rt);
		LoadQ(m, q1, gpr, rs); // rs -> v1
		LoadQ(m, q2, gpr, rt); // rt -> v2
		VRegister d, n, o;     // n = rs(v1), o = rt(v2)
		switch (fmt)
		{
			case 0:  d = v0.V16B(); n = v1.V16B(); o = v2.V16B(); break;
			case 1:  d = v0.V8H();  n = v1.V8H();  o = v2.V8H();  break;
			case 2:  d = v0.V4S();  n = v1.V4S();  o = v2.V4S();  break;
			default: d = v0.V2D();  n = v1.V2D();  o = v2.V2D();  break;
		}
		// Immediate shift amount: PS*H uses sa & 0xf, PS*W the full 5-bit sa.
		const int shamt = (int)((insn >> 6) & (fmt == 1 ? 0xf : 0x1f));
		switch (act)
		{
			case M_ADD:   m.Add(d, n, o);   break;
			case M_SUB:   m.Sub(d, n, o);   break;
			case M_SQADD: m.Sqadd(d, n, o); break;
			case M_UQADD: m.Uqadd(d, n, o); break;
			case M_SQSUB: m.Sqsub(d, n, o); break;
			case M_UQSUB: m.Uqsub(d, n, o); break;
			case M_AND:   m.And(d, n, o);   break;
			case M_ORR:   m.Orr(d, n, o);   break;
			case M_EOR:   m.Eor(d, n, o);   break;
			case M_NOR:   m.Orr(d, n, o); m.Mvn(d, d); break;
			case M_CMGT:  m.Cmgt(d, n, o);  break;
			case M_CMEQ:  m.Cmeq(d, n, o);  break;
			case M_SMAX:  m.Smax(d, n, o);  break;
			case M_SMIN:  m.Smin(d, n, o);  break;
			case M_ZIP1:  m.Zip1(d, o, n);  break; // PEXTL: interleave rt then rs
			case M_ZIP2:  m.Zip2(d, o, n);  break; // PEXTU
			// NEON USHR/SSHR immediates encode 1..lanebits only; shift 0 is a copy.
			case M_SLLI:  m.Shl(d, o, shamt); break;                                      // PSLLH/PSLLW
			case M_SRLI:  if (shamt) m.Ushr(d, o, shamt); else m.Mov(v0.V16B(), v2.V16B()); break; // PSRLH/PSRLW
			case M_SRAI:  if (shamt) m.Sshr(d, o, shamt); else m.Mov(v0.V16B(), v2.V16B()); break; // PSRAH/PSRAW
			case M_PACK:  m.Uzp1(d, o, n);  break; // PPAC*: rt evens -> low, rs evens -> high
			case M_CPYLD: m.Zip1(d, o, n);  break; // PCPYLD: rd = {rt.UD[0], rs.UD[0]}
			case M_CPYUD: m.Zip2(d, n, o);  break; // PCPYUD: rd = {rs.UD[1], rt.UD[1]}
		}
		m.Str(q0, MemOperand(gpr, rd * 16));
		s_rc.Invalidate(rd); // 128-bit write bypasses StoreGpr
	}

	// ---- Integer mult/div + HI/LO moves (opcode 0x00) ----
	// All bit-exact, side-effect-free integer ops on the low 32 bits, with the
	// 64-bit HI/LO results stored sign-extended, matching the interpreter exactly
	// (R5900OpcodeImpl.cpp MULT/MULTU/DIV/DIVU). HI/LO live right after the 32
	// 128-bit GPRs: HI at byte offset 32*16, LO at 32*16+16 (low 64 bits used).
	constexpr u32 kHI = 32 * 16;
	constexpr u32 kLO = kHI + 16;

	// Load the low 32 bits of a GPR into a w-register (r0 reads as zero).
	inline void LoadW(MacroAssembler& m, const Register& wd, const Register& gpr, u32 idx)
	{
		if (idx == 0) { m.Mov(wd, 0); return; }
		if (RegCacheOn())
		{
			// A cached 64-bit value's W view is exactly the guest low 32 bits;
			// on a miss just load (no fill -- we only have half the value).
			const int s = s_rc.host_of[idx];
			if (s >= 0) { m.Mov(wd, CacheW(s)); return; }
		}
		m.Ldr(wd, MemOperand(gpr, idx * 16));
	}

	// off = 0 for the SPECIAL (pipeline 0) forms, 8 for the MMI pipeline-1
	// forms (MULT1/DIV1/MFHI1/...): HI1/LO1 are the upper 64 bits of HI/LO,
	// and the funct encodings line up exactly between the two op spaces.
	void EmitMulDiv(MacroAssembler& m, const Register& gpr, u32 funct, u32 rs, u32 rt, u32 rd, u32 off = 0)
	{
		const u32 kHIo = kHI + off, kLOo = kLO + off;
		switch (funct)
		{
			case 0x10: if (rd) { m.Ldr(x0, MemOperand(gpr, kHIo)); StoreGpr(m, x0, gpr, rd); } return; // MFHI
			case 0x12: if (rd) { m.Ldr(x0, MemOperand(gpr, kLOo)); StoreGpr(m, x0, gpr, rd); } return; // MFLO
			case 0x11: LoadGpr(m, x0, gpr, rs); m.Str(x0, MemOperand(gpr, kHIo)); return;              // MTHI
			case 0x13: LoadGpr(m, x0, gpr, rs); m.Str(x0, MemOperand(gpr, kLOo)); return;              // MTLO

			case 0x18: // MULT (signed)
			case 0x19: // MULTU (unsigned)
			{
				LoadW(m, w0, gpr, rs); LoadW(m, w1, gpr, rt);
				if (funct == 0x18) m.Smull(x0, w0, w1); else m.Umull(x0, w0, w1);
				m.Sxtw(x2, w0);                // LO = (s32)(res & 0xffffffff)
				m.Lsr(x3, x0, 32); m.Sxtw(x3, w3); // HI = (s32)(res >> 32)
				m.Str(x2, MemOperand(gpr, kLOo));
				m.Str(x3, MemOperand(gpr, kHIo));
				if (rd) StoreGpr(m, x2, gpr, rd); // rd = LO.UD[0]
				return;
			}
			case 0x1a: // DIV (signed)
			{
				LoadW(m, w0, gpr, rs); LoadW(m, w1, gpr, rt);
				Label zero, normal, store;
				m.Cbz(w1, &zero);
				m.Mov(w5, 0x80000000); m.Cmp(w0, w5); m.B(&normal, ne); // overflow: rs==INT_MIN
				m.Cmn(w1, 1); m.B(&normal, ne);                         // && rt==-1
				m.Mov(x2, (uint64_t)(int64_t)(s32)0x80000000); m.Mov(x3, 0); m.B(&store);
				m.Bind(&normal);
				m.Sdiv(w2, w0, w1); m.Msub(w4, w2, w1, w0);             // q, rem=rs-q*rt
				m.Sxtw(x2, w2); m.Sxtw(x3, w4); m.B(&store);
				m.Bind(&zero);                                          // div by 0
				m.Mov(w6, 1); m.Mov(w7, -1); m.Cmp(w0, 0); m.Csel(w2, w6, w7, lt); // LO=(rs<0)?1:-1
				m.Sxtw(x2, w2); m.Sxtw(x3, w0);                        // HI=sext(rs)
				m.Bind(&store);
				m.Str(x2, MemOperand(gpr, kLOo)); m.Str(x3, MemOperand(gpr, kHIo));
				return;
			}
			case 0x1b: // DIVU (unsigned)
			{
				LoadW(m, w0, gpr, rs); LoadW(m, w1, gpr, rt);
				Label zero, store;
				m.Cbz(w1, &zero);
				m.Udiv(w2, w0, w1); m.Msub(w4, w2, w1, w0);
				m.Sxtw(x2, w2); m.Sxtw(x3, w4); m.B(&store);
				m.Bind(&zero);
				m.Mov(x2, (uint64_t)-1); m.Sxtw(x3, w0);              // LO=-1, HI=sext(rs)
				m.Bind(&store);
				m.Str(x2, MemOperand(gpr, kLOo)); m.Str(x3, MemOperand(gpr, kHIo));
				return;
			}
		}
	}

	// MADD/MADDU (+ pipeline-1 MADD1/MADDU1, off=8): multiply-accumulate into
	// the 64-bit {HI.low32, LO.low32} accumulator, split back sign-extended.
	// Mirrors MMI.cpp MADD: temp = (u64)LO.UL[0] | ((u64)HI.UL[0]<<32) + rs*rt;
	// LO.SD = sext(temp[31:0]); HI.SD = sext(temp[63:32]); rd = LO.SD[0].
	void EmitMadd(MacroAssembler& m, const Register& gpr, bool is_unsigned, u32 rs, u32 rt, u32 rd, u32 off = 0)
	{
		const u32 kHIo = kHI + off, kLOo = kLO + off;
		m.Ldr(w0, MemOperand(gpr, kLOo));            // acc[31:0]  = LO.UL[0] (zero-ext)
		m.Ldr(w1, MemOperand(gpr, kHIo));            // acc[63:32] = HI.UL[0]
		m.Orr(x0, x0, Operand(x1, LSL, 32));         // acc
		LoadW(m, w2, gpr, rs); LoadW(m, w3, gpr, rt);
		if (is_unsigned) m.Umull(x2, w2, w3); else m.Smull(x2, w2, w3);
		m.Add(x0, x0, x2);                           // temp = acc + rs*rt
		m.Sxtw(x1, w0);                              // LO = sext(temp[31:0])
		m.Lsr(x2, x0, 32); m.Sxtw(x2, w2);           // HI = sext(temp[63:32])
		m.Str(x1, MemOperand(gpr, kLOo));
		m.Str(x2, MemOperand(gpr, kHIo));
		if (rd) StoreGpr(m, x1, gpr, rd);            // rd = LO.SD[0]
	}

	// Per-opcode cycle cost, mirroring R5900OpcodeTables.cpp exactly (Cycles::*):
	// Default=9, Branch=11, CopDefault=7, Mult=2*8=16, Div=14*8=112, Load=14,
	// Store=14, MMI_Default=14. Used to accumulate cpuBlockCycles accurately
	// (cpuBlockCycles += cost * (2 - CP0.Config[18]), summed over the block's ops).
	// NOTE: loads/stores/COP1-moves/MMI must NOT fall through to Default(9) --
	// undercounting their cost makes the JIT's cpuRegs.cycle lag the interpreter's,
	// so scheduled vsync/timer/DMA interrupts fire at different instruction
	// boundaries. That timing drift is what made MMX7 stall after the logo (the EE
	// idle loop 0x00081fc0 spins waiting for an interrupt that the JIT delivered
	// late relative to a NO_EE_MEM reference).
	int OpCycles(u32 insn)
	{
		const u32 op = insn >> 26, funct = insn & 0x3f;
		switch (op)
		{
			case 0x00: // SPECIAL
				if (funct == 0x18 || funct == 0x19) return 16;  // MULT/MULTU
				if (funct == 0x1a || funct == 0x1b) return 112; // DIV/DIVU
				return 9;                                       // ALU/shift/JR/JALR/MFHI...
			case 0x01:                                          // REGIMM
				// MTSAB/MTSAH are Default; the rest (BLTZ/BGEZ/B*ZAL/likely) Branch.
				if (((insn >> 16) & 31) == 0x18 || ((insn >> 16) & 31) == 0x19) return 9;
				return 11;                                      // Branch
			case 0x04: case 0x05: case 0x06: case 0x07:         // BEQ/BNE/BLEZ/BGTZ
			case 0x14: case 0x15: case 0x16: case 0x17:         // BEQL/BNEL/BLEZL/BGTZL
				return 11;                                      // Branch
			case 0x10:                                          // COP0
				if (((insn >> 21) & 31) == 0x08) return 11;     // BC0x (Branch)
				return 7;                                       // MFC0/MTC0 (CopDefault)
			case 0x11:                                          // COP1
				// Per R5900OpcodeTables.cpp: BC1F/T(L) = Branch(11); MUL/MULA/
				// MADD/MSUB/MADDA/MSUBA.S = FPU_Mult(4*8=32), DIV.S/SQRT.S =
				// 6*8=48, RSQRT.S = 8*8=64; everything else we translate (MFC1/
				// MTC1/MOV/NEG/ABS/ADD/SUB/ADDA/SUBA/MAX/MIN/CVT/C.cond) is
				// CopDefault(7).
				if (((insn >> 21) & 31) == 0x08) return 11;     // BC1x (Branch)
				if (((insn >> 21) & 31) == 0x10)
				{
					switch (insn & 0x3f)
					{
						case 0x02: case 0x1a:            // MUL.S/MULA.S
						case 0x1c: case 0x1d:            // MADD.S/MSUB.S
						case 0x1e: case 0x1f: return 32; // MADDA.S/MSUBA.S
						case 0x03: case 0x04: return 48; // DIV.S/SQRT.S
						case 0x16: return 64;            // RSQRT.S
					}
				}
				return 7;                                       // CopDefault
			case 0x1c:                                          // MMI
				if (funct == 0x18 || funct == 0x19) return 16;  // MULT1/MULTU1 (Mult)
				if (funct == 0x1a || funct == 0x1b) return 112; // DIV1/DIVU1 (Div)
				if (funct >= 0x10 && funct <= 0x13) return 9;   // MFHI1/MTHI1/MFLO1/MTLO1 (Default)
				if (funct == 0x00 || funct == 0x01 || funct == 0x20 || funct == 0x21) return 16; // MADD/MADDU(1) (Mult)
				return 14;                                      // MMI_Default
			case 0x1a: case 0x1b:                               // LDL/LDR
			case 0x1e:                                          // LQ
			case 0x20: case 0x21: case 0x22: case 0x23:         // LB/LH/LWL/LW
			case 0x24: case 0x25: case 0x26: case 0x27:         // LBU/LHU/LWR/LWU
			case 0x31:                                          // LWC1
			case 0x36:                                          // LQC2
			case 0x37:                                          // LD
				return 14;                                      // Load
			case 0x1f:                                          // SQ
			case 0x28: case 0x29: case 0x2a: case 0x2b:         // SB/SH/SWL/SW
			case 0x2c: case 0x2d: case 0x2e:                    // SDL/SDR/SWR
			case 0x39:                                          // SWC1
			case 0x3e:                                          // SQC2
			case 0x3f:                                          // SD
				return 14;                                      // Store
			default:
				return 9;                                       // Default (addi/ori/lui/J/JAL/daddi...)
		}
	}

	// Translate one simple integer ALU op (32- and 64-bit forms). Returns false,
	// emitting nothing, for control flow / memory / FPU / MMI / anything else.
	// TEMP diagnostic: env-gated toggles to force categories of EE ops back to
	// the interpreter, to bisect which native EE JIT feature breaks MMX7's
	// post-logo progression. Read once. Mirrored in IsTranslatable so the block
	// builder and delay-slot handling stay consistent with EmitSimple.
	struct EeDiagFlags { int no_mmi, no_muldiv, no_cop1, no_mem, no_load, no_store, no_branch, no_ld64, no_interpcall, no_fpu_arith; };
	const EeDiagFlags& eeDiag()
	{
		static EeDiagFlags f = {
			getenv("LRPS2_NO_EE_MMI")    ? 1 : 0,
			getenv("LRPS2_NO_EE_MULDIV") ? 1 : 0,
			getenv("LRPS2_NO_EE_COP1")   ? 1 : 0,
			getenv("LRPS2_NO_EE_MEM")    ? 1 : 0,
			(getenv("LRPS2_NO_EE_MEM") || getenv("LRPS2_NO_EE_LOAD"))  ? 1 : 0,
			(getenv("LRPS2_NO_EE_MEM") || getenv("LRPS2_NO_EE_STORE")) ? 1 : 0,
			getenv("LRPS2_NO_EE_BRANCH") ? 1 : 0,
			getenv("LRPS2_NO_EE_LD64")   ? 1 : 0,
			getenv("LRPS2_NO_EE_INTERPCALL") ? 1 : 0, // C.18 inline interp-op calls
			getenv("LRPS2_NO_EE_FPU_ARITH")  ? 1 : 0, // C.23 native ADD.S/SUB.S/MUL.S
		};
		return f;
	}

	// C.18: execute an interpreter op implementation from inside the block.
	// Only for ops with contained side effects: no control flow, no exception,
	// and no reads of cpuRegs.pc or cpuBlockCycles (MFC0's Count uses
	// cpuRegs.cycle, which advances at branch boundaries identically in both
	// execution modes, so mid-block reads match the interpreter handoff).
	void EmitInterpOpCall(MacroAssembler& m, const Register& gpr, u32 insn, void (*fn)())
	{
		// The interp op body reads guest GPRs from memory (MTC0 rt, QFSRV
		// rs/rt) -- write the dirty cache back first...
		s_rc.FlushDirty(m, gpr);
		m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.code));
		m.Mov(w0, insn);
		m.Str(w0, MemOperand(x10));
		m.Mov(x16, reinterpret_cast<uint64_t>(fn));
		m.Blr(x16);
		// ...and it may also write any guest GPR directly (MFC0 rt, QFSRV rd):
		// drop the whole cache afterwards.
		s_rc.Reset();
	}

	// C.30-2: native COP2 macro emission. The vendored aVU_Macro emitters gate
	// flag maintenance on g_pCurInstInfo's M1 analysis bits; we run no analysis
	// pass, so a static "everything live" record keeps every gate conservative
	// (always denormalize/update/normalize the flags -- correct, just not
	// minimal). The emitters write through the AsmHelpers thread-locals, so we
	// hand them OUR in-flight assembler for the duration of the op: armAsmPtr
	// must be the block buffer's start so armGetCurrentCodePointer() (used for
	// ADRP/call displacement math) computes true host addresses.
	EEINST s_cop2AllLive; // .info set on first use
	// TEMP bisect (C.30-2 black-texture hunt): LRPS2_COP2M_SKIP="a-b,c,d-e"
	// (hex funct ranges) forces the listed COP2 SPECIAL functs back to the
	// interp call while the rest stay native.
	bool Cop2MacroSkipped(u32 insn)
	{
		const u32 funct = insn & 0x3f;
		static int parsed = -1;
		static u64 mask = 0;  // bit per funct 0..0x3f
		static u64 smask[2] = {0, 0}; // bits per SPECIAL2 sub 0..0x43 ("sXX" entries)
		if (parsed < 0)
		{
			parsed = 0;
			// Also read the mask from a file so GUI-launched sessions (no env)
			// can be re-tested by editing the file + restarting the frontend.
			static char filebuf[256];
			const char* e = getenv("LRPS2_COP2M_SKIP");
			if (!e)
			{
				if (FILE* f = fopen("/home/user/lrps2_cop2skip.txt", "r"))
				{
					if (fgets(filebuf, sizeof(filebuf), f))
						e = filebuf;
					fclose(f);
				}
			}
			if (e)
			{
				char buf[256]; strncpy(buf, e, 255); buf[255] = 0;
				for (char* tok = strtok(buf, ","); tok; tok = strtok(nullptr, ","))
				{
					const bool sub = (tok[0] == 's');
					if (sub) tok++;
					unsigned lo, hi;
					if (sscanf(tok, "%x-%x", &lo, &hi) == 2) {}
					else if (sscanf(tok, "%x", &lo) == 1) hi = lo;
					else continue;
					// "sLO-sHI" tolerated too: strip a stray 's' on the hi bound
					if (sub)
						for (unsigned v = lo; v <= hi && v < 128; v++) smask[v >> 6] |= 1ull << (v & 63);
					else
						for (unsigned f = lo; f <= hi && f < 64; f++) mask |= 1ull << f;
				}
			}
		}
		if ((mask >> funct) & 1)
			return true;
		if (funct >= 0x3c) // SPECIAL2: match the sub index too
		{
			const u32 sub = (insn & 3) | ((insn >> 4) & 0x7c);
			if (sub < 128 && ((smask[sub >> 6] >> (sub & 63)) & 1))
				return true;
		}
		return false;
	}

	bool EmitCop2Macro(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		if (!recVUMacroIsMode0(insn))
			return false; // CALLMS/CALLMSR/unknown -> C.29-1 interp call
		if (Cop2MacroSkipped(insn))
			return false; // TEMP bisect exclusion -> interp call
		// VCLIP stays on the interpreter call PERMANENTLY (C.30-4 root cause,
		// GT3 black road): the interpreter's macro path keeps the live clip
		// shift register in VU->clipflag and commits it to VI[REG_CLIP_FLAG]
		// through the FMAC pipeline -- i.e. VI lags the live value by one op.
		// GT3 reads the flag via CFC2 tens of thousands of times per race and
		// depends on that delayed view; the native emitter's immediate
		// read-shift-write on VI hands CFC2 a value shifted 6 bits too far
		// and the road-chunk culling goes wrong. A native version would have
		// to replicate the delayed commit (VI = old clipflag; clipflag = new)
		// -- not worth it for an op this rare next to the ALU stream.
		if ((insn & 0x3f) >= 0x3c && ((insn & 3) | ((insn >> 4) & 0x7c)) == 0x1f)
			return false;

		// The macro emitters clobber gprF0 (w23 -- one of OUR cache slots) and
		// run their own VI-GPR allocator (macro mode excludes x19-x26; x27 is
		// untracked but our cache is dead after this anyway): retire the guest
		// GPR cache exactly like an interp-call would.
		s_rc.FlushDirty(m, gpr);
		s_rc.Reset();

		// Conservative VU0 sync (their aR5900 does this analysis-driven): a
		// macro op reads/writes VU0 state, so any in-flight VU0 microprogram
		// must finish first. The interpreter ops self-sync the same way.
		{
			Label vu0_idle;
			m.Mov(x10, reinterpret_cast<uint64_t>(&vuRegs[0].VI[REG_VPU_STAT].UL));
			m.Ldr(w0, MemOperand(x10));
			m.Tbz(w0, 0, &vu0_idle); // VPU_STAT bit0 = VU0 running
			m.Mov(x16, reinterpret_cast<uint64_t>(&_vu0FinishMicro));
			m.Blr(x16);
			m.Bind(&vu0_idle);
		}

		s_cop2AllLive.info = EEINST_COP2_DENORMALIZE_STATUS_FLAG | EEINST_COP2_NORMALIZE_STATUS_FLAG |
		                     EEINST_COP2_STATUS_FLAG | EEINST_COP2_MAC_FLAG | EEINST_COP2_CLIP_FLAG |
		                     EEINST_COP2_SYNC_VU0 | EEINST_COP2_FINISH_VU0 | EEINST_COP2_FLUSH_VU0_REGISTERS;
		EEINST* saved_inst = g_pCurInstInfo;
		g_pCurInstInfo = &s_cop2AllLive;
		cpuRegs.code = insn; // emit-time input: the emitters' _Fs_/_Ft_/... macros read it

		MacroAssembler* saved_asm = armAsm;
		u8* saved_ptr = armAsmPtr;
		size_t saved_cap = armAsmCapacity;
		armAsm = &m;
		armAsmPtr = m.GetBuffer()->GetStartAddress<u8*>();
		armAsmCapacity = m.GetBuffer()->GetCapacity();

		const bool ok = recVUMacroEmitMode0(insn);

		// TEMP diagnostic (LRPS2_JIT_STATS): prove the native macro path compiles.
		{
			static const bool stats = getenv("LRPS2_JIT_STATS") != nullptr;
			if (stats && ok)
			{
				static u64 n = 0;
				n++;
				if (n <= 5 || (n & 0xff) == 0)
					fprintf(stderr, "[jit] COP2 macro op #%llu native (insn=%08x funct=%02x)\n",
						(unsigned long long)n, insn, insn & 0x3f);
			}
		}

		armAsm = saved_asm;
		armAsmPtr = saved_ptr;
		armAsmCapacity = saved_cap;
		g_pCurInstInfo = saved_inst;
		return ok;
	}

	// QFSRV is MMI1 (funct 0x28) sub 0x1b -- NOT MMI3 (0x29), whose sub 0x1b is
	// PCPYH. The first C.18 cut called QFSRV() on PCPYH instructions (the
	// GT3-attract breaker at funct=29/sub=1b) and broke MMX7's post-logo
	// progression -- same shifted-table bug class as the C.10 DADDU miscompile.
	inline bool IsQfsrv(u32 insn) { return (insn & 0x3f) == 0x28 && ((insn >> 6) & 0x1f) == 0x1b; }
	inline bool IsPcpyh(u32 insn) { return (insn & 0x3f) == 0x29 && ((insn >> 6) & 0x1f) == 0x1b; }

	// TEMP bisect toggles for the C.18 inline-call op groups (finer than
	// LRPS2_NO_EE_INTERPCALL, which disables all of them).
	struct IcFlags { int no_cop0, no_cache, no_qfsrv, no_cop2, no_cop2macro; };
	const IcFlags& icDiag()
	{
		static IcFlags f = {
			getenv("LRPS2_NO_EE_IC_COP0")  ? 1 : 0,
			getenv("LRPS2_NO_EE_IC_CACHE") ? 1 : 0,
			getenv("LRPS2_NO_EE_IC_QFSRV") ? 1 : 0,
			getenv("LRPS2_NO_EE_IC_COP2")  ? 1 : 0,
			getenv("LRPS2_NO_EE_COP2MACRO") ? 1 : 0,
		};
		return f;
	}

	bool EmitSimple(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		s_rc.pinned = 0; // new op: previous op's Register handles are dead
		const u32 op = insn >> 26;
		const u32 rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, rd = (insn >> 11) & 31, sa = (insn >> 6) & 31;
		const u32 funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);
		const uint64_t simm64 = (uint64_t)(int64_t)simm;
		const u32 zimm = insn & 0xffff;

		// TEMP diagnostic toggles (see eeDiag): force a category of EE ops back
		// to the interpreter to bisect which native EE JIT feature breaks MMX7.
		const int no_mmi = eeDiag().no_mmi, no_muldiv = eeDiag().no_muldiv;
		const int no_cop1 = eeDiag().no_cop1;
		const int no_fpu_arith = eeDiag().no_fpu_arith;
		const int no_load = eeDiag().no_load, no_store = eeDiag().no_store;
		const int no_ld64 = eeDiag().no_ld64;

		switch (op)
		{
			case 0x00:
				switch (funct)
				{
					// 32-bit shifts -> sign-extend to 64 (direct-to-cache, C.27-3)
					case 0x00: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Lsl(d.W(), a.W(), sa); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SLL
					case 0x02: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Lsr(d.W(), a.W(), sa); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SRL
					case 0x03: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Asr(d.W(), a.W(), sa); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SRA
					case 0x04: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), b = SrcGpr(m, gpr, rs, x1), d = DstGpr(m, gpr, rd, x0); m.Lsl(d.W(), a.W(), b.W()); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SLLV
					case 0x06: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), b = SrcGpr(m, gpr, rs, x1), d = DstGpr(m, gpr, rd, x0); m.Lsr(d.W(), a.W(), b.W()); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SRLV
					case 0x07: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), b = SrcGpr(m, gpr, rs, x1), d = DstGpr(m, gpr, rd, x0); m.Asr(d.W(), a.W(), b.W()); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SRAV
					// 32-bit add/sub -> sign-extend to 64
					case 0x21: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Add(d.W(), a.W(), b.W()); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // ADDU
					case 0x23: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Sub(d.W(), a.W(), b.W()); m.Sxtw(d, d.W()); FinishDst(m, gpr, rd, d); } return true; // SUBU
					// 64-bit logic / set / add / sub
					case 0x24: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.And(d, a, b); FinishDst(m, gpr, rd, d); } return true; // AND
					case 0x25: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Orr(d, a, b); FinishDst(m, gpr, rd, d); } return true; // OR
					case 0x26: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Eor(d, a, b); FinishDst(m, gpr, rd, d); } return true; // XOR
					case 0x27: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Orr(d, a, b); m.Mvn(d, d); FinishDst(m, gpr, rd, d); } return true; // NOR
					case 0x2a: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Cmp(a, b); m.Cset(d, lt); FinishDst(m, gpr, rd, d); } return true; // SLT
					case 0x2b: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Cmp(a, b); m.Cset(d, lo); FinishDst(m, gpr, rd, d); } return true; // SLTU
					// funct map: 0x2c=DADD 0x2d=DADDU 0x2e=DSUB 0x2f=DSUBU (the
					// interpreter's DADD/DSUB don't trap on overflow, so both
					// pairs emit the same Add/Sub). 0x2d used to emit Sub --
					// every real DADDU rd,rs,rt computed rs-rt; it survived boot
					// because `move rd,rs` == `daddu rd,rs,$zero` where rs-0 is
					// accidentally correct (MMX7 post-logo stall, C.9).
					case 0x2c:
					case 0x2d: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Add(d, a, b); FinishDst(m, gpr, rd, d); } return true; // DADD/DADDU
					case 0x2e:
					case 0x2f: if (rd) { const Register a = SrcGpr(m, gpr, rs, x0), b = SrcGpr(m, gpr, rt, x1), d = DstGpr(m, gpr, rd, x0); m.Sub(d, a, b); FinishDst(m, gpr, rd, d); } return true; // DSUB/DSUBU
					// 64-bit shifts (variable)
					case 0x14: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), b = SrcGpr(m, gpr, rs, x1), d = DstGpr(m, gpr, rd, x0); m.Lsl(d, a, b); FinishDst(m, gpr, rd, d); } return true; // DSLLV
					case 0x16: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), b = SrcGpr(m, gpr, rs, x1), d = DstGpr(m, gpr, rd, x0); m.Lsr(d, a, b); FinishDst(m, gpr, rd, d); } return true; // DSRLV
					case 0x17: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), b = SrcGpr(m, gpr, rs, x1), d = DstGpr(m, gpr, rd, x0); m.Asr(d, a, b); FinishDst(m, gpr, rd, d); } return true; // DSRAV
					// 64-bit shifts (immediate)
					case 0x38: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Lsl(d, a, sa);      FinishDst(m, gpr, rd, d); } return true; // DSLL
					case 0x3a: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Lsr(d, a, sa);      FinishDst(m, gpr, rd, d); } return true; // DSRL
					case 0x3b: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Asr(d, a, sa);      FinishDst(m, gpr, rd, d); } return true; // DSRA
					case 0x3c: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Lsl(d, a, sa + 32); FinishDst(m, gpr, rd, d); } return true; // DSLL32
					case 0x3e: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Lsr(d, a, sa + 32); FinishDst(m, gpr, rd, d); } return true; // DSRL32
					case 0x3f: if (rd) { const Register a = SrcGpr(m, gpr, rt, x0), d = DstGpr(m, gpr, rd, x0); m.Asr(d, a, sa + 32); FinishDst(m, gpr, rd, d); } return true; // DSRA32
					// HI/LO moves + integer mult/div (bit-exact, sign-extended results)
					case 0x10: case 0x11: case 0x12: case 0x13: // MFHI/MTHI/MFLO/MTLO
					case 0x18: case 0x19: case 0x1a: case 0x1b: // MULT/MULTU/DIV/DIVU
						if (no_muldiv) return false;
						EmitMulDiv(m, gpr, funct, rs, rt, rd); return true;
					// SYNC is architecturally a barrier; the interpreter's SYNC()
					// body is empty, so it translates to pure cycle bookkeeping.
					case 0x0f: return true;
					// MFSA/MTSA: the shift-amount register (also written by
					// MTSAB/MTSAH, read by QFSRV). MFSA zero-extends sa into
					// rd (interp: GPR[rd].UD[0] = (u64)cpuRegs.sa).
					case 0x28:
						if (rd)
						{
							const Register d = DstGpr(m, gpr, rd, x0);
							m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.sa));
							m.Ldr(d.W(), MemOperand(x10)); // 32-bit load zero-extends
							FinishDst(m, gpr, rd, d);
						}
						return true;
					case 0x29:
						LoadGpr(m, x0, gpr, rs);
						m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.sa));
						m.Str(w0, MemOperand(x10));
						return true;
					// MOVZ/MOVN: rd = rs when rt ==/!= 0 (full 64-bit compare),
					// rd unchanged otherwise.
					case 0x0a:
					case 0x0b:
						if (rd)
						{
							const Register a = SrcGpr(m, gpr, rs, x0);
							const Register b = SrcGpr(m, gpr, rt, x1);
							const Register dold = SrcGpr(m, gpr, rd, x2); // OLD rd value
							const Register d = DstGpr(m, gpr, rd, x2);    // same slot / same scratch
							m.Cmp(b, 0);
							m.Csel(d, a, dold, funct == 0x0a ? eq : ne);
							FinishDst(m, gpr, rd, d);
						}
						return true;
					default: return false;
				}
			// ADDI/DADDI (0x08/0x18): the R5900's overflow trap is dropped, exactly
			// like the upstream x86 recompiler (recADDIU just calls recADDI, which
			// never checks overflow -- no game relies on the exception). So they are
			// emitted identically to ADDIU/DADDIU. C.42.
			case 0x08: // ADDI
			case 0x09: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(w1, (u32)simm); m.Add(d.W(), a.W(), w1); m.Sxtw(d, d.W()); FinishDst(m, gpr, rt, d); } return true; // ADDIU
			case 0x18: // DADDI
			case 0x19: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(x1, simm64); m.Add(d, a, x1); FinishDst(m, gpr, rt, d); } return true; // DADDIU
			case 0x0a: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(x1, simm64); m.Cmp(a, x1); m.Cset(d, lt); FinishDst(m, gpr, rt, d); } return true; // SLTI
			case 0x0b: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(x1, simm64); m.Cmp(a, x1); m.Cset(d, lo); FinishDst(m, gpr, rt, d); } return true; // SLTIU
			case 0x0c: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(x1, (uint64_t)zimm); m.And(d, a, x1); FinishDst(m, gpr, rt, d); } return true; // ANDI
			case 0x0d: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(x1, (uint64_t)zimm); m.Orr(d, a, x1); FinishDst(m, gpr, rt, d); } return true; // ORI
			case 0x0e: if (rt) { const Register a = SrcGpr(m, gpr, rs, x0), d = DstGpr(m, gpr, rt, x0); m.Mov(x1, (uint64_t)zimm); m.Eor(d, a, x1); FinishDst(m, gpr, rt, d); } return true; // XORI
			case 0x0f: if (rt) { const Register d = DstGpr(m, gpr, rt, x0); m.Mov(d.W(), zimm << 16); m.Sxtw(d, d.W()); FinishDst(m, gpr, rt, d); } return true; // LUI

			// REGIMM (0x01): only MTSAB/MTSAH here -- the branches (BLTZ/BGEZ/
			// B*ZAL/likely) are handled by EmitBranch. MTSAB/MTSAH write the
			// shift-amount register cpuRegs.sa (read by QFSRV). rt selects the op.
			case 0x01:
			{
				const u32 rtf = (insn >> 16) & 31;
				if (rtf == 0x18) // MTSAB: sa = (rs.UL[0] & 0xF) ^ (imm & 0xF)
				{
					LoadGpr(m, x0, gpr, rs); m.And(w0, w0, 0xF);
					m.Eor(w0, w0, (u32)(zimm & 0xF));
					m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.sa)); m.Str(w0, MemOperand(x10));
					return true;
				}
				if (rtf == 0x19) // MTSAH: sa = ((rs.UL[0] & 0x7) ^ (imm & 0x7)) << 1
				{
					LoadGpr(m, x0, gpr, rs); m.And(w0, w0, 0x7);
					m.Eor(w0, w0, (u32)(zimm & 0x7));
					m.Lsl(w0, w0, 1);
					m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.sa)); m.Str(w0, MemOperand(x10));
					return true;
				}
				return false; // branches -> EmitBranch
			}

			// Loads (LB/LBU/LH/LHU/LW/LWU/LD). Address = rs.UL[0] + simm (32-bit).
			// The interpreter cancels (fastjmp) on a misaligned LH/LW/LD, so do the
			// same; reads run even when rt==0 (side effects). Memory goes through
			// the EE vtlb wrappers. gpr (x19) is callee-saved across the calls.
			case 0x20: case 0x24: case 0x21: case 0x25: case 0x23: case 0x27: case 0x37:
			{
				if (no_load || (no_ld64 && op == 0x37)) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				const u32 amask = (op == 0x37) ? 7 : (op == 0x23 || op == 0x27) ? 3 : (op == 0x21 || op == 0x25) ? 1 : 0;
				if (amask)
			{
				// Cold path: the cancel fastjmp leaves the block and the
				// interpreter resumes from GPR MEMORY -- write the dirty cache
				// back on this (never-taken-in-practice) path only, leaving
				// the fall-through compile-time state untouched.
				Label ok; m.Tst(w0, amask); m.B(&ok, eq);
				s_rc.FlushDirtyColdPath(m, gpr);
				m.Mov(x16, reinterpret_cast<uint64_t>(&eeCancelInstruction_arm64)); m.Blr(x16);
				m.Bind(&ok);
			}
				uint64_t fn = (op == 0x37) ? reinterpret_cast<uint64_t>(&eeRead64_arm64)
				            : (op == 0x23 || op == 0x27) ? reinterpret_cast<uint64_t>(&eeRead32_arm64)
				            : (op == 0x21 || op == 0x25) ? reinterpret_cast<uint64_t>(&eeRead16_arm64)
				            : reinterpret_cast<uint64_t>(&eeRead8_arm64);
				if (InlineMemEnabled())
				{
					// Inline vtlb direct case; handlers (hw regs) fall back to
					// the wrapper call. Same value semantics: Ldrb/Ldrh/Ldr w0
					// zero-extend into x0 exactly like the u32-returning
					// wrappers.
					Label slow, done;
					EmitVmapLookup(m, &slow);
					switch (op)
					{
						case 0x20: case 0x24: m.Ldrb(w0, MemOperand(x10)); break; // LB/LBU
						case 0x21: case 0x25: m.Ldrh(w0, MemOperand(x10)); break; // LH/LHU
						case 0x23: case 0x27: m.Ldr (w0, MemOperand(x10)); break; // LW/LWU
						default:              m.Ldr (x0, MemOperand(x10)); break; // LD
					}
					m.B(&done);
					m.Bind(&slow);
					m.Mov(x16, fn); m.Blr(x16);
					m.Bind(&done);
				}
				else { m.Mov(x16, fn); m.Blr(x16); } // result in x0 (u32 returns are zero-extended in x0)
				if (rt)
				{
					switch (op)
					{
						case 0x20: m.Sxtb(x0, w0); break; // LB
						case 0x21: m.Sxth(x0, w0); break; // LH
						case 0x23: m.Sxtw(x0, w0); break; // LW
						default: break;                   // LBU/LHU/LWU (zero-extended), LD (full 64)
					}
					StoreGpr(m, x0, gpr, rt);
				}
				return true;
			}
			// Unaligned load family (LWL/LWR/LDL/LDR): merge with the existing rt
			// contents at the aligned address, no misalign exception -- a plain
			// helper call that mirrors the interpreter op exactly (C.16). These
			// were the top handoff cause in GT3's attract (LDL+LDR alone ~865M
			// interpreted ops / 10500 frames).
			case 0x22: case 0x26: case 0x1a: case 0x1b:
			{
				if (no_load) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				m.Mov(w1, rt);
				s_rc.FlushReg(m, gpr, rt); // the helper READS GPR[rt] for the merge
				const uint64_t fn = (op == 0x22) ? reinterpret_cast<uint64_t>(&eeLWL_arm64)
				                  : (op == 0x26) ? reinterpret_cast<uint64_t>(&eeLWR_arm64)
				                  : (op == 0x1a) ? reinterpret_cast<uint64_t>(&eeLDL_arm64)
				                  :                reinterpret_cast<uint64_t>(&eeLDR_arm64);
				m.Mov(x16, fn); m.Blr(x16);
				s_rc.Invalidate(rt); // ...and then writes it directly
				return true;
			}
			// Unaligned store family (SWL/SWR/SDL/SDR): read-modify-write of the
			// aligned word/dword, same helper-call pattern.
			case 0x2a: case 0x2e: case 0x2c: case 0x2d:
			{
				if (no_store) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				m.Mov(w1, rt);
				s_rc.FlushReg(m, gpr, rt); // the helper reads GPR[rt] from memory
				const uint64_t fn = (op == 0x2a) ? reinterpret_cast<uint64_t>(&eeSWL_arm64)
				                  : (op == 0x2e) ? reinterpret_cast<uint64_t>(&eeSWR_arm64)
				                  : (op == 0x2c) ? reinterpret_cast<uint64_t>(&eeSDL_arm64)
				                  :                reinterpret_cast<uint64_t>(&eeSDR_arm64);
				m.Mov(x16, fn); m.Blr(x16);
				return true;
			}
			// Stores (SB/SH/SW/SD).
			case 0x28: case 0x29: case 0x2b: case 0x3f:
			{
				if (no_store || (no_ld64 && op == 0x3f)) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				const u32 amask = (op == 0x3f) ? 7 : (op == 0x2b) ? 3 : (op == 0x29) ? 1 : 0;
				if (amask)
			{
				// Cold path: the cancel fastjmp leaves the block and the
				// interpreter resumes from GPR MEMORY -- write the dirty cache
				// back on this (never-taken-in-practice) path only, leaving
				// the fall-through compile-time state untouched.
				Label ok; m.Tst(w0, amask); m.B(&ok, eq);
				s_rc.FlushDirtyColdPath(m, gpr);
				m.Mov(x16, reinterpret_cast<uint64_t>(&eeCancelInstruction_arm64)); m.Blr(x16);
				m.Bind(&ok);
			}
				LoadGpr(m, x1, gpr, rt); // value (low bits used by the wrapper)
				uint64_t fn = (op == 0x3f) ? reinterpret_cast<uint64_t>(&eeWrite64_arm64)
				            : (op == 0x2b) ? reinterpret_cast<uint64_t>(&eeWrite32_arm64)
				            : (op == 0x29) ? reinterpret_cast<uint64_t>(&eeWrite16_arm64)
				            : reinterpret_cast<uint64_t>(&eeWrite8_arm64);
				if (InlineMemEnabled())
				{
					// Inline direct store. A store into a write-protected RAM
					// page (holding compiled blocks) SIGSEGVs -> vtlb
					// PageFaultHandler -> mmap_ClearCpuBlock unprotects and
					// drops the stale blocks -> the kernel retries this Str;
					// identical flow to interpreter stores.
					Label slow, done;
					EmitVmapLookup(m, &slow);
					switch (op)
					{
						case 0x28: m.Strb(w1, MemOperand(x10)); break; // SB
						case 0x29: m.Strh(w1, MemOperand(x10)); break; // SH
						case 0x2b: m.Str (w1, MemOperand(x10)); break; // SW
						default:   m.Str (x1, MemOperand(x10)); break; // SD
					}
					m.B(&done);
					m.Bind(&slow);
					m.Mov(x16, fn); m.Blr(x16);
					m.Bind(&done);
				}
				else { m.Mov(x16, fn); m.Blr(x16); }
				return true;
			}
			// PREF (0x33): the interpreter's PREF() body is empty (like SYNC),
			// so it translates to pure cycle bookkeeping (Default=9).
			case 0x33: return true;

			// LQ/SQ (128-bit; the hardware silently aligns -- addr & ~0xf, no
			// misalign exception). Previously untranslated: every LQ/SQ ended
			// the block with an interpreter handoff, and they are everywhere in
			// compiler-generated 128-bit copies.
			case 0x1e: // LQ
			{
				if (no_load || !rt) return false; // rt==0: rare, leave to interp
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				m.And(w0, w0, 0xfffffff0);
				Label slow, done;
				if (InlineMemEnabled())
				{
					EmitVmapLookup(m, &slow);
					m.Ldr(q0, MemOperand(x10));
					m.Str(q0, MemOperand(gpr, rt * 16));
					m.B(&done);
					m.Bind(&slow);
				}
				m.Add(x1, gpr, rt * 16);
				m.Mov(x16, reinterpret_cast<uint64_t>(&eeRead128_arm64)); m.Blr(x16);
				m.Bind(&done);
				s_rc.Invalidate(rt); // 128-bit write bypasses StoreGpr (both paths)
				return true;
			}
			case 0x1f: // SQ
			{
				if (no_store) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				m.And(w0, w0, 0xfffffff0);
				s_rc.FlushReg(m, gpr, rt); // the 128-bit source is read from memory
				Label slow, done;
				if (InlineMemEnabled())
				{
					EmitVmapLookup(m, &slow);
					m.Ldr(q0, MemOperand(gpr, rt * 16)); // GPR.r[0] is kept zero
					m.Str(q0, MemOperand(x10));
					m.B(&done);
					m.Bind(&slow);
				}
				m.Add(x1, gpr, rt * 16);
				m.Mov(x16, reinterpret_cast<uint64_t>(&eeWrite128_arm64)); m.Blr(x16);
				m.Bind(&done);
				return true;
			}

			// COP1 register ops (MFC1/MTC1/MOV.S/NEG.S/ABS.S only -- arithmetic and
			// BC1/CFC1/CTC1 hand off to the interpreter).
			case 0x11:
				if (no_cop1 || !Cop1RegSupported(insn)) return false;
				if (no_fpu_arith && Cop1ArithSupported(insn)) return false;
				EmitCop1Reg(m, gpr, insn);
				return true;

			// MMI (128-bit parallel integer) -> NEON, for the bit-exact subset.
			case 0x1c:
				if (no_mmi) return false;
				// Pipeline-1 mult/div + HI1/LO1 moves: the funct encodings match
				// the SPECIAL forms exactly, only HI/LO shift up by 8 bytes.
				// MADD/MADDU (funct 0x00/0x01) are pipeline 0; MADD1/MADDU1
				// (0x20/0x21) pipeline 1.
				switch (funct)
				{
					case 0x10: case 0x11: case 0x12: case 0x13: // MFHI1/MTHI1/MFLO1/MTLO1
					case 0x18: case 0x19: case 0x1a: case 0x1b: // MULT1/MULTU1/DIV1/DIVU1
						if (no_muldiv) return false;
						EmitMulDiv(m, gpr, funct, rs, rt, rd, 8);
						return true;
					case 0x00: case 0x01: // MADD/MADDU
						if (no_muldiv) return false;
						EmitMadd(m, gpr, funct == 0x01, rs, rt, rd, 0);
						return true;
					case 0x20: case 0x21: // MADD1/MADDU1
						if (no_muldiv) return false;
						EmitMadd(m, gpr, funct == 0x21, rs, rt, rd, 8);
						return true;
				}
				if (MmiSupported(insn)) { EmitMmi(m, gpr, insn); return true; }
				// PCPYH: broadcast halfword 0 of each 64-bit half of rt across
				// that half. Scalar: (u16)half * 0x0001000100010001.
				if (IsPcpyh(insn))
				{
					if (rd)
					{
						s_rc.FlushReg(m, gpr, rt); // low half is read from memory
						m.Ldrh(w0, MemOperand(gpr, rt * 16));     // rt.US[0] (r0 memory kept zero)
						m.Ldrh(w1, MemOperand(gpr, rt * 16 + 8)); // rt.US[4]
						m.Mov(x2, 0x0001000100010001ull);
						m.Mul(x0, x0, x2);
						m.Mul(x1, x1, x2);
						m.Str(x0, MemOperand(gpr, rd * 16));
						m.Str(x1, MemOperand(gpr, rd * 16 + 8));
						s_rc.Invalidate(rd); // direct write bypasses StoreGpr
					}
					return true;
				}
				// QFSRV (quadword funnel shift by the SA register): interpreter
				// implementation called inline (C.18).
				if (IsQfsrv(insn) && !eeDiag().no_interpcall && !icDiag().no_qfsrv)
				{
					EmitInterpOpCall(m, gpr, insn, &R5900::Interpreter::OpcodeImpl::MMI::QFSRV);
					return true;
				}
				return false;

			// CACHE (0x2f): emulated D$ maintenance, contained side effects ->
			// COP2 / VU0 macro mode (0x12), C.29-1: everything except the BC2
			// branches (rs=0x08, control flow -> EmitBranch) executes via the
			// interpreter's top-level COP2() dispatcher called inline, so VU0
			// macro ALU ops / VCALLMS(R) / Q(M)FC2/CTC2/CFC2 no longer BREAK
			// the block into an interpreter tail. All contained: no EE pc use,
			// no exceptions; VU0 timing reads cpuRegs.cycle which only
			// advances at branch boundaries -- identical in both execution
			// modes (same argument as MFC0 Count, C.18). EmitInterpOpCall's
			// FlushDirty-before covers QMTC2/CTC2 reading GPR memory and its
			// cache Reset-after covers QMFC2/CFC2 writing it. The whole COP2
			// opcode space is charged Default(9) by R5900OpcodeTables (the
			// class table is disabled upstream), which OpCycles already does.
			case 0x12:
				if (rs == 0x08) return false; // BC2F/T(L) -> EmitBranch
				// C.30-2: SPECIAL1/2 ALU ops emit natively via the microVU0
				// single-op emitters; LRPS2_NO_EE_COP2MACRO=1 forces them back
				// to the C.29-1 interp call. Everything else (transfers,
				// CALLMS/CALLMSR) stays on the interp call.
				if (rs >= 0x10 && !icDiag().no_cop2macro && EmitCop2Macro(m, gpr, insn))
					return true;
				if (eeDiag().no_interpcall || icDiag().no_cop2) return false;
				EmitInterpOpCall(m, gpr, insn, &R5900::Interpreter::OpcodeImpl::COP2);
				return true;

			// interpreter implementation called inline (C.18). Top GT3 attract
			// block-breaker (19.4M handoffs / 10500 frames).
			case 0x2f:
				if (eeDiag().no_interpcall || icDiag().no_cache) return false;
				EmitInterpOpCall(m, gpr, insn, &R5900::Interpreter::OpcodeImpl::CACHE);
				return true;

			// COP0 (0x10): MFC0/MTC0 via inline interpreter call (C.18) -- MFC0
			// Count reads cpuRegs.cycle which only advances at branch boundaries,
			// identical in both execution modes; MTC0 Status/Config side effects
			// live inside the op. BC0x (branches) and CO/TLB/ERET stay handoffs.
			case 0x10:
				if (eeDiag().no_interpcall || icDiag().no_cop0) return false;
				if (rs == 0x00) { EmitInterpOpCall(m, gpr, insn, &R5900::Interpreter::OpcodeImpl::COP0::MFC0); return true; }
				if (rs == 0x04) { EmitInterpOpCall(m, gpr, insn, &R5900::Interpreter::OpcodeImpl::COP0::MTC0); return true; }
				return false;

			// LWC1 ft, off(base): fpr[ft].UL = read32(addr). Unlike integer LW, a
			// misaligned address is *silently skipped* (no exception/cancel) -- match
			// the interpreter's `if (addr & 3) return;`.
			case 0x31:
			{
				if (no_cop1) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				Label skip;
				m.Tst(w0, 3); m.B(&skip, ne);
				m.Mov(x16, reinterpret_cast<uint64_t>(&eeRead32_arm64)); m.Blr(x16);
				m.Mov(x1, reinterpret_cast<uint64_t>(&fpuRegs)); m.Str(w0, MemOperand(x1, rt * 4));
				m.Bind(&skip);
				return true;
			}
			// SWC1 ft, off(base): write32(addr, fpr[ft].UL); misalign silently skipped.
			case 0x39:
			{
				if (no_cop1) return false;
				{ const Register a = SrcGpr(m, gpr, rs, x0); m.Mov(w2, (u32)simm); m.Add(w0, a.W(), w2); }
				Label skip;
				m.Tst(w0, 3); m.B(&skip, ne);
				m.Mov(x1, reinterpret_cast<uint64_t>(&fpuRegs)); m.Ldr(w1, MemOperand(x1, rt * 4));
				m.Mov(x16, reinterpret_cast<uint64_t>(&eeWrite32_arm64)); m.Blr(x16);
				m.Bind(&skip);
				return true;
			}
			default: return false;
		}
	}

	// Block epilogue with tail-chaining: restore the frame, then dispatch directly
	// to the next block (cpuRegs.pc) via the RAM LUT -- staying in JIT code instead
	// of returning to the C++ GAME_RUNNING loop. On a LUT miss (uncompiled block or
	// non-RAM pc) it returns to the dispatcher, which compiles it and re-enters.
	// x30 (the original return address into eeJitRunBlock) propagates through the
	// chain, so the final `ret` returns to the C++ caller.
	void EmitChainEpilogue(MacroAssembler& m)
	{
#ifdef EE_PC_SAMPLE
		m.Ldp(x19, x20, MemOperand(sp, 0));
		m.Ldp(x21, x30, MemOperand(sp, 16));
		m.Ldp(x22, x23, MemOperand(sp, 32));
		m.Ldp(x24, x25, MemOperand(sp, 48));
		m.Ldp(x26, x27, MemOperand(sp, 64));
		m.Add(sp, sp, 80);
		m.Ret(); // TEMP: disable block-linking so every block returns to the dispatcher (PC sampling)
		return;
#endif
		// C.40: try to chain FIRST, with the frame (and x19) still live --
		// the successor's chain entry skips its prologue, so back-to-back
		// blocks pay no frame pop/push or x19 rematerialization.
		m.Mov(x0, reinterpret_cast<uint64_t>(&cpuRegs.pc));
		m.Ldr(w1, MemOperand(x0));          // pc
		m.And(w2, w1, 0x1fffffff);          // np = pc & mirror mask
		m.Mov(w3, kRamBytes);
		Label ret_path;
		m.Cmp(w2, w3);
		m.B(&ret_path, hs);                 // not RAM -> return to dispatcher
		m.Mov(x0, reinterpret_cast<uint64_t>(&s_lut));
		m.Ldr(x0, MemOperand(x0));          // s_lut base
		m.Lsr(w2, w2, 2);                   // idx = np >> 2
		m.Ldr(x3, MemOperand(x0, w2, UXTW, 3)); // fn = s_lut[idx]
		m.Cbz(x3, &ret_path);               // uncompiled -> return
		m.Add(x3, x3, kChainEntryOffset);   // skip the successor's prologue
		m.Br(x3);                           // tail-jump to next block's body
		m.Bind(&ret_path);
		m.Ldp(x19, x20, MemOperand(sp, 0));
		m.Ldp(x21, x30, MemOperand(sp, 16));
		m.Ldp(x22, x23, MemOperand(sp, 32));
		m.Ldp(x24, x25, MemOperand(sp, 48));
		m.Ldp(x26, x27, MemOperand(sp, 64));
		m.Add(sp, sp, 80);
		m.Ret();
	}

	bool IsTranslatable(u32 insn)
	{
		const u32 op = insn >> 26, funct = insn & 0x3f;
		if (op == 0x00)
		{
			switch (funct)
			{
				case 0x00: case 0x02: case 0x03: case 0x04: case 0x06: case 0x07:
				case 0x21: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
				case 0x2a: case 0x2b: case 0x2c: case 0x2d: case 0x2e: case 0x2f:
				case 0x14: case 0x16: case 0x17: case 0x38: case 0x3a: case 0x3b:
				case 0x3c: case 0x3e: case 0x3f:
					return true;
				case 0x10: case 0x11: case 0x12: case 0x13: // MFHI/MTHI/MFLO/MTLO
				case 0x18: case 0x19: case 0x1a: case 0x1b: // MULT/MULTU/DIV/DIVU
					return !eeDiag().no_muldiv;
				case 0x0f: return true;                     // SYNC (empty body)
				case 0x0a: case 0x0b: return true;          // MOVZ/MOVN
				case 0x28: case 0x29: return true;          // MFSA/MTSA
				default: return false;
			}
		}
		if (op == 0x11)
			return !eeDiag().no_cop1 && Cop1RegSupported(insn) &&
			       !(eeDiag().no_fpu_arith && Cop1ArithSupported(insn));
		if (op == 0x1c)
		{
			const u32 f = insn & 0x3f;
			if ((f >= 0x10 && f <= 0x13) || (f >= 0x18 && f <= 0x1b) // pipeline-1 muldiv
				|| f == 0x00 || f == 0x01 || f == 0x20 || f == 0x21) // MADD/MADDU(1)
				return !eeDiag().no_mmi && !eeDiag().no_muldiv;
			return !eeDiag().no_mmi &&
				(MmiSupported(insn) || IsPcpyh(insn) ||
				 (IsQfsrv(insn) && !eeDiag().no_interpcall && !icDiag().no_qfsrv));
		}
		if (op == 0x01) // REGIMM: MTSAB/MTSAH only (branches handled elsewhere)
		{
			const u32 rtf = (insn >> 16) & 31;
			return rtf == 0x18 || rtf == 0x19;
		}
		if (op == 0x12) // COP2: all but BC2 (branches -> EmitBranch)
		return ((insn >> 21) & 31) != 0x08 && !eeDiag().no_interpcall && !icDiag().no_cop2;
	if (op == 0x2f) return !eeDiag().no_interpcall && !icDiag().no_cache; // CACHE
		if (op == 0x10) // COP0: MFC0/MTC0 only
		{
			const u32 crs = (insn >> 21) & 31;
			return !eeDiag().no_interpcall && !icDiag().no_cop0 && (crs == 0x00 || crs == 0x04);
		}
		switch (op)
		{
			case 0x08: case 0x18: // ADDI/DADDI (overflow trap dropped, like the x86 rec)
			case 0x09: case 0x19: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
			case 0x33: // PREF (empty body)
				return true;
			case 0x20: case 0x21: case 0x23: case 0x24: case 0x25: case 0x27:
			case 0x22: case 0x26: case 0x1a: case 0x1b: // LWL/LWR/LDL/LDR
				return !eeDiag().no_load;
			case 0x2a: case 0x2e: case 0x2c: case 0x2d: // SWL/SWR/SDL/SDR
				return !eeDiag().no_store;
			case 0x1e: return !eeDiag().no_load && ((insn >> 16) & 31) != 0; // LQ (rt==0 -> interp)
			case 0x37: return !eeDiag().no_load && !eeDiag().no_ld64; // LD
			case 0x28: case 0x29: case 0x2b:
				return !eeDiag().no_store;
			case 0x1f: return !eeDiag().no_store; // SQ
			case 0x3f: return !eeDiag().no_store && !eeDiag().no_ld64; // SD
			case 0x31: case 0x39: return !eeDiag().no_cop1; // LWC1/SWC1 are COP1 moves
			default: return false;
		}
	}

	// cyc = raw sum of the block's per-op cycle costs (see OpCycles); the
	// (2 - CP0.Config[18]) factor is applied here, matching execI.
	void EmitCycleBookkeeping(MacroAssembler& m, int cyc)
	{
		m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.CP0.n.Config));
		m.Ldr(w1, MemOperand(x10));
		m.Mov(w2, cyc); m.Mov(w3, cyc * 2);
		m.Tst(w1, 1u << 18);
		m.Csel(w2, w2, w3, ne);
		m.Mov(x10, reinterpret_cast<uint64_t>(&cpuBlockCycles));
		m.Ldr(w0, MemOperand(x10)); m.Add(w0, w0, w2); m.Str(w0, MemOperand(x10));
	}

	inline void StorePC(MacroAssembler& m, const Register& wsrc)
	{
		m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.pc));
		m.Str(wsrc, MemOperand(x10));
	}
	inline void StorePCImm(MacroAssembler& m, u32 v) { m.Mov(w0, v); StorePC(m, w0); }


	// Translate a branch/jump (ends the block via the chaining epilogue). EE: the
	// delay slot runs only when taken; intEventTest() runs on both paths; not-taken
	// continues at bpc+4. Likely branches / Goemon-HLE JAL / untranslatable delay
	// slot -> false (interpreter finishes the basic block).
	bool EmitBranch(MacroAssembler& m, const Register& gpr, u32 bpc, u32 insn, int cyc_leading)
	{
		if (eeDiag().no_branch) return false; // TEMP diagnostic: force branches to interpreter
		s_rc.pinned = 0; // new op: previous op's Register handles are dead
		const u32 op = insn >> 26, rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);

		bool uncond = false, is_jr = false, two = false, one = false, likely = false;
		bool bc0 = false; // condition = CPCOND0() (DMAC all-clear test), via helper call
		bool bc1 = false; // condition = FCR31 C flag (0x00800000), tested inline
		bool bc2 = false; // condition = CP2COND (VPU_STAT VU0-busy bit 0x100), tested inline
		int  link = -1;
		Condition cond = al;
		u32  tconst = 0;
		const u32 jtarget = (((bpc + 4) & 0xf0000000) | ((insn & 0x3ffffff) << 2));

		if      (op == 0x00 && funct == 0x08) { uncond = true; is_jr = true; }
		else if (op == 0x00 && funct == 0x09) { uncond = true; is_jr = true; link = (int)((insn >> 11) & 31); }
		else if (op == 0x02) { uncond = true; tconst = jtarget; }
		else if (op == 0x03) { uncond = true; link = 31; tconst = jtarget;
			if (jtarget == 0x3563b8 || jtarget == 0x35d628 || jtarget == 0x35c118) return false; }
		else if (op == 0x04) { two = true; cond = eq; tconst = bpc + 4 + simm * 4; }
		else if (op == 0x05) { two = true; cond = ne; tconst = bpc + 4 + simm * 4; }
		else if (op == 0x06) { one = true; cond = le; tconst = bpc + 4 + simm * 4; }
		else if (op == 0x07) { one = true; cond = gt; tconst = bpc + 4 + simm * 4; }
		// Likely branches: taken == normal (delay slot inline); not-taken SKIPS
		// the delay slot (continue at bpc+8) and, like the interpreter's
		// BEQL/BNEL/BLEZL/BGTZL/BLTZL/BGEZL, calls intEventTest there.
		else if (op == 0x14) { two = true; cond = eq; tconst = bpc + 4 + simm * 4; likely = true; }
		else if (op == 0x15) { two = true; cond = ne; tconst = bpc + 4 + simm * 4; likely = true; }
		else if (op == 0x16) { one = true; cond = le; tconst = bpc + 4 + simm * 4; likely = true; }
		else if (op == 0x17) { one = true; cond = gt; tconst = bpc + 4 + simm * 4; likely = true; }
		else if (op == 0x01)
		{
			const u32 t = bpc + 4 + simm * 4;
			if      (rt == 0x00) { one = true; cond = lt; tconst = t; }
			else if (rt == 0x01) { one = true; cond = ge; tconst = t; }
			else if (rt == 0x02) { one = true; cond = lt; tconst = t; likely = true; } // BLTZL
			else if (rt == 0x03) { one = true; cond = ge; tconst = t; likely = true; } // BGEZL
			else if (rt == 0x10) { one = true; cond = lt; tconst = t; link = 31; }
			else if (rt == 0x11) { one = true; cond = ge; tconst = t; link = 31; }
			else return false;
		}
		// BC0F/BC0T(+L): branch on CPCOND0 == 0/1 (COP0.cpp: DMAC CIS|~CPC
		// all-clear). Taken/not-taken semantics match the BLEZ class (interp
		// BC0F does nothing on not-taken; the likely forms skip the delay
		// slot + event test like the other likely branches).
		else if (op == 0x10 && rs == 0x08)
		{
			const u32 t = bpc + 4 + simm * 4;
			if      (rt == 0x00) { bc0 = true; cond = eq; tconst = t; }                // BC0F
			else if (rt == 0x01) { bc0 = true; cond = ne; tconst = t; }                // BC0T
			else if (rt == 0x02) { bc0 = true; cond = eq; tconst = t; likely = true; } // BC0FL
			else if (rt == 0x03) { bc0 = true; cond = ne; tconst = t; likely = true; } // BC0TL
			else return false;
		}
		// BC1F/BC1T(+L): branch on the FCR31 C flag (0x00800000) clear/set.
		// Same not-taken semantics as BC0x: nothing on the non-likely forms,
		// and the likely forms skip the delay slot WITHOUT an event test
		// (FPU.cpp's BC1L macro only does `cpuRegs.pc += 4`).
		else if (op == 0x11 && rs == 0x08)
		{
			const u32 t = bpc + 4 + simm * 4;
			if      (rt == 0x00) { bc1 = true; cond = eq; tconst = t; }                // BC1F
			else if (rt == 0x01) { bc1 = true; cond = ne; tconst = t; }                // BC1T
			else if (rt == 0x02) { bc1 = true; cond = eq; tconst = t; likely = true; } // BC1FL
			else if (rt == 0x03) { bc1 = true; cond = ne; tconst = t; likely = true; } // BC1TL
			else return false;
		}
		// BC2F/BC2T(+L): branch on CP2COND = VPU_STAT's VU0-busy bit (0x100).
		// Same not-taken semantics as BC0x/BC1x (COP2.cpp: BC2xL else pc+=4,
		// no event test). NOTE the whole COP2 space is charged Default(9),
		// not Branch(11) -- the upstream opcode table has no COP2 subclass.
		else if (op == 0x12 && rs == 0x08)
		{
			const u32 t = bpc + 4 + simm * 4;
			if      (rt == 0x00) { bc2 = true; cond = eq; tconst = t; }                // BC2F
			else if (rt == 0x01) { bc2 = true; cond = ne; tconst = t; }                // BC2T
			else if (rt == 0x02) { bc2 = true; cond = eq; tconst = t; likely = true; } // BC2FL
			else if (rt == 0x03) { bc2 = true; cond = ne; tconst = t; likely = true; } // BC2TL
			else return false;
		}
		else return false;

		const u32 ds = memRead32(bpc + 4);
		if (!IsTranslatable(ds))
			return false;

		// TEMP diagnostic (LRPS2_JIT_STATS): confirm the likely-branch path is
		// actually exercised (compiled), not just harmless.
		static const bool jit_stats = getenv("LRPS2_JIT_STATS") != nullptr;
		if (likely && jit_stats)
		{
			static u64 n = 0;
			n++;
			if (n <= 5 || (n & 0x3ff) == 0)
				fprintf(stderr, "[jit] likely branch #%llu compiled at %08x (op=%02x)\n",
					(unsigned long long)n, bpc, op);
		}

		// Taken/uncond runs the branch + its delay slot inline; not-taken runs only
		// the branch (the delay slot at bpc+4 is executed by the continuation).
		const int cyc_taken = cyc_leading + OpCycles(insn) + OpCycles(ds);
		const int cyc_nt    = cyc_leading + OpCycles(insn);

		const uint64_t evt = reinterpret_cast<uint64_t>(&eeEventTest_arm64);
		const uint64_t upd = reinterpret_cast<uint64_t>(&eeUpdateCycles_arm64);

		// C.35: gate the event test on cycle >= nextEventCycle, the upstream
		// x86 recompiler's dispatcher rule. The interpreter tests on EVERY
		// taken branch, and mirroring that costs ~28% of CPU time in
		// _cpuEventTest_Shared/iopEventTest bookkeeping (GT3 perf profile).
		// This intentionally trades interpreter bit-identity (events fire at
		// the next due branch, not the current one) for upstream-rec timing;
		// LRPS2_NO_EVTGATE=1 restores the every-branch behaviour.
		static const bool evt_gate = getenv("LRPS2_NO_EVTGATE") == nullptr;
		const auto EmitEventTest = [&m, evt]()
		{
			if (evt_gate)
			{
				Label skip;
				m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.cycle));
				m.Ldr(w0, MemOperand(x10));
				m.Mov(x11, reinterpret_cast<uint64_t>(&cpuRegs.nextEventCycle));
				m.Ldr(w1, MemOperand(x11));
				m.Subs(w0, w0, w1); // (s32)(cycle - nextEventCycle)
				m.B(&skip, mi);     // not due yet
				m.Mov(x16, evt);
				m.Blr(x16);
				m.Bind(&skip);
			}
			else
			{
				m.Mov(x16, evt);
				m.Blr(x16);
			}
		};

		// C.39: inline the default-cycle-rate body of intUpdateCPUCycles
		// (cycle += max(1, cpuBlockCycles >> 3); cpuBlockCycles &= 7) into the
		// block tail — it runs on every block exit and the call was 4.3% of
		// CPU time. Any non-zero EECycleRate falls back to the helper, checked
		// at runtime so settings changes need no block flush.
		// LRPS2_NO_INLINE_UPD=1 restores the call.
		static const bool upd_inline = getenv("LRPS2_NO_INLINE_UPD") == nullptr;
		const auto EmitUpdateCycles = [&m, upd]()
		{
			if (upd_inline)
			{
				Label slow, done;
				m.Mov(x10, reinterpret_cast<uint64_t>(&EmuConfig.Speedhacks.EECycleRate));
				m.Ldrsb(w1, MemOperand(x10));
				m.Cbnz(w1, &slow);
				m.Mov(x10, reinterpret_cast<uint64_t>(&cpuBlockCycles));
				m.Ldr(w0, MemOperand(x10));
				m.Lsr(w2, w0, 3);
				m.Cmp(w2, 0);
				m.Csinc(w2, w2, wzr, ne); // max(1, cpuBlockCycles >> 3)
				m.Mov(x11, reinterpret_cast<uint64_t>(&cpuRegs.cycle));
				m.Ldr(w3, MemOperand(x11));
				m.Add(w3, w3, w2);
				m.Str(w3, MemOperand(x11));
				m.And(w0, w0, 7);
				m.Str(w0, MemOperand(x10));
				m.B(&done);
				m.Bind(&slow);
				m.Mov(x16, upd);
				m.Blr(x16);
				m.Bind(&done);
			}
			else
			{
				m.Mov(x16, upd);
				m.Blr(x16);
			}
		};

		// WaitLoop speedhack for the EE kernel idle loop at 0x81fc0 (6 nops +
		// `beq zero,zero,-6`): the interpreter fast-forwards cpuRegs.cycle to
		// nextEventCycle when spinning there (_doBranch_shared, gated on
		// Cpu==&intCpu so the JIT never benefited). Mirror it on the taken
		// path: after the cycle flush (upd), before the event test, do
		// `if (nextEventCycle != cycle) cycle = nextEventCycle` -- the next
		// event then fires immediately instead of burning host time emulating
		// millions of idle iterations. Same effect as the interpreter's
		// `(s64)(u32(nev-cyc)) > 0` condition (true iff the u32s differ).
		const bool idle_skip = EmuConfig.Speedhacks.WaitLoop && !is_jr
			&& ((tconst & 0x1fffffff) == 0x00081fc0);
		const auto EmitIdleSkip = [&m]()
		{
			Label skip;
			m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.cycle));
			m.Mov(x11, reinterpret_cast<uint64_t>(&cpuRegs.nextEventCycle));
			m.Ldr(w0, MemOperand(x10));
			m.Ldr(w1, MemOperand(x11));
			m.Cmp(w1, w0);
			m.B(&skip, eq);
			m.Str(w1, MemOperand(x10)); // cycle = nextEventCycle
			m.Bind(&skip);
		};

		// Link value is ZERO-extended to match the interpreter's _SetLink
		// (R5900.h: UD[0] = u32 pc+4) -- NOT sign-extended as real MIPS64 would.
		// Kernel return addresses (0x8xxxxxxx) otherwise get 0xffffffff upper
		// halves that the interpreter path never produces.
		if (link > 0) { m.Mov(w0, bpc + 8); StoreGpr(m, x0, gpr, (u32)link); }
		if (is_jr) LoadGpr(m, x20, gpr, rs);

		if (uncond)
		{
			EmitSimple(m, gpr, ds);
			EmitCycleBookkeeping(m, cyc_taken);
			if (is_jr) StorePC(m, w20); else StorePCImm(m, tconst);
			s_rc.FlushDirty(m, gpr); // block exit: the next block reads GPR memory
			EmitUpdateCycles();
			if (idle_skip) EmitIdleSkip();
			EmitEventTest();
			EmitChainEpilogue(m);
			return true;
		}

		if (bc0)
		{
			// CPCOND0 is a plain int() reading dmacRegs; call it directly.
			m.Mov(x16, reinterpret_cast<uint64_t>(&R5900::Interpreter::OpcodeImpl::COP0::CPCOND0));
			m.Blr(x16);
			m.Cmp(w0, 0); // eq -> CPCOND0()==0 (BC0F taken), ne -> ==1 (BC0T taken)
		}
		else if (bc1)
		{
			// FCR31 lives at fpuRegs.fprc[31] (byte offset 252).
			m.Mov(x10, reinterpret_cast<uint64_t>(&fpuRegs));
			m.Ldr(w0, MemOperand(x10, 252));
			m.Tst(w0, 0x00800000); // eq -> C clear (BC1F taken), ne -> C set (BC1T taken)
		}
		else if (bc2)
		{
			m.Mov(x10, reinterpret_cast<uint64_t>(&vuRegs[0].VI[REG_VPU_STAT].UL));
			m.Ldr(w0, MemOperand(x10));
			m.Tst(w0, 0x100); // eq -> VU0 idle (BC2F taken), ne -> busy (BC2T taken)
		}
		else
		{
			const Register a = SrcGpr(m, gpr, rs, x0);
			if (two) { const Register b = SrcGpr(m, gpr, rt, x1); m.Cmp(a, b); } else m.Cmp(a, 0);
		}
		// The taken/not-taken paths fork the compile-time cache state: the
		// delay slot is only EXECUTED on the taken path, so its cache fills,
		// stores and evictions must not leak into the not-taken emission.
		// Snapshot here, emit the taken path (mutating), then restore for the
		// not-taken path. Both paths flush before their chain epilogue.
		const RegCache fork_state = s_rc;
		Label not_taken;
		m.B(&not_taken, InvertCondition(cond));
		EmitSimple(m, gpr, ds);
		EmitCycleBookkeeping(m, cyc_taken);
		StorePCImm(m, tconst);
		s_rc.FlushDirty(m, gpr); // block exit (taken)
		EmitUpdateCycles();
		if (idle_skip) EmitIdleSkip();
		EmitEventTest();
		EmitChainEpilogue(m);
		s_rc = fork_state;
		m.Bind(&not_taken);
		EmitCycleBookkeeping(m, cyc_nt);
		// Likely branches skip their delay slot when not taken.
		StorePCImm(m, bpc + (likely ? 8 : 4));
		s_rc.FlushDirty(m, gpr); // block exit (not taken)
		// Mirror the interpreter EXACTLY: of the non-likely conditionals, only
		// BEQ/BNE call intEventTest() on the not-taken path (BGEZ/BGTZ/BLEZ/BLTZ
		// and the AL forms do nothing there -- see Interpreter.cpp); the likely
		// branches BEQL/BNEL/BLEZL/BGTZL/BLTZL/BGEZL do. EXCEPTION: BC0FL/BC0TL
		// and BC1FL/BC1TL are likely (skip the delay slot) but do NOT call
		// intEventTest on not-taken -- COP0.cpp's BC0xL and FPU.cpp's BC1L
		// macro only do `cpuRegs.pc += 4`. Always
		// WITHOUT intUpdateCPUCycles (no cpuBlockCycles flush). Deviating in
		// either direction (extra/missing test points, or flushing first) shifts
		// interrupt delivery (EPC) inside wait loops relative to the interpreter,
		// which cascades into different kernel-scheduler decisions (MMX7 FMV
		// loader wedge).
		if (op == 0x04 || op == 0x05 || (likely && !bc0 && !bc1 && !bc2)) EmitEventTest();
		EmitChainEpilogue(m);
		return true;
	}

	// Register a block on its source pages (dedup: revive cycles re-push after
	// eeJitClear erased only the WRITTEN page's list -- a spanning block's entry
	// on the other page survives) and write-protect them so ANY host write
	// (interpreter store, JIT wrapper store, SIF/IPU DMA memcpy, overlay load)
	// faults -> vtlb PageFaultHandler -> mmap_ClearCpuBlock -> Cpu->Clear ->
	// eeJitClear_arm64 drops the page's blocks. Without this nothing ever
	// invalidates EE blocks on self-modifying code / game overlay loads
	// (MMX7 loads an overlay ~frame 111 and later runs STALE translated
	// code at 0x105138 -> its FMV loader wedges -> post-logo black stall).
	void RegisterPages(u32 pc, u32 ns, u32 ne)
	{
		if (ne <= ns)
			return; // no native-covered bytes -> content-independent block
		for (u32 pg = ns >> kPageShift; pg <= (ne - 1) >> kPageShift; pg++)
		{
			auto& vec = s_page[pg];
			if (std::find(vec.begin(), vec.end(), pc) == vec.end())
				vec.push_back(pc);
			if (InRam(pg << kPageShift))
				mmap_MarkCountedRamPage(pg << kPageShift);
		}
	}

	int s_cache_wraps = 0; // TEMP DEBUG (C.24 crash hunt)

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 8192 > kCodeCacheSize)
		{
			s_cache_wraps++;
			if (getenv("LRPS2_FAULT_LOG"))
				fprintf(stderr, "[ee-cache-WRAP #%d]\n", s_cache_wraps);
			s_blocks.clear();
			s_page.clear();
			LutClearAll();
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);
		// The aVU macro emitters (C.30-2) hold RSCRATCHADDR (x17) and q31 across
		// vixl macro expansions -- keep the assembler from synthesizing into them
		// (mirrors armStartBlock in AsmHelpers.cpp).
		masm.GetScratchRegisterList()->Remove(17);
		masm.GetScratchVRegisterList()->Remove(31);

		const Register gpr = x19;
		s_rc.Reset(); // fresh per-block register-cache state (C.27)
		// Uniform 80-byte frame across ALL blocks (the chain epilogue of any
		// block must match any successor's prologue): x19/x20 (gpr base, jr
		// target), x21..x27 (GPR register cache), x30 (chain return).
		masm.Sub(sp, sp, 80);
		masm.Stp(x19, x20, MemOperand(sp, 0));
		masm.Stp(x21, x30, MemOperand(sp, 16));
		masm.Stp(x22, x23, MemOperand(sp, 32));
		masm.Stp(x24, x25, MemOperand(sp, 48));
		masm.Stp(x26, x27, MemOperand(sp, 64));
		{
			// Exactly 4 instructions so kChainEntryOffset stays constant
			// (VIXL's Mov would shrink the sequence for compressible values).
			const uint64_t gv = reinterpret_cast<uint64_t>(&cpuRegs.GPR.r[0]);
			vixl::ExactAssemblyScope scope(&masm, 4 * kInstructionSize);
			masm.movz(gpr, gv & 0xffff);
			masm.movk(gpr, (gv >> 16) & 0xffff, 16);
			masm.movk(gpr, (gv >> 32) & 0xffff, 32);
			masm.movk(gpr, (gv >> 48) & 0xffff, 48);
		}

		// TEMP diagnostic (LRPS2_EXCLOG): only the EE exception vector blocks
		// (pc 0x8000_0180/0200) call eeLogExc, dumping COP0 Cause/EPC + cycle.
		if ((pc & 0x1fffffff) == 0x200 || (pc & 0x1fffffff) == 0x180)
		{
			masm.Mov(w0, pc);
			masm.Mov(x16, reinterpret_cast<uint64_t>(&eeLogExc));
			masm.Blr(x16);
		}

		// TEMP diagnostic (LRPS2_TRACE): call eeTraceHook(pc) at block entry so a
		// full-JIT run can be state-diffed against a pure-interpreter run.
		{
			static int trc = -1; static u32 tlo = 0, thi = 0;
			if (trc < 0)
			{
				const char* l = getenv("LRPS2_TRACE_LO"); const char* h = getenv("LRPS2_TRACE_HI");
				trc = (getenv("LRPS2_TRACE") && !getenv("LRPS2_TRACE_STEP") && l && h) ? 1 : 0;
				if (trc) { tlo = (u32)strtoul(l, 0, 16); thi = (u32)strtoul(h, 0, 16); }
			}
			if (trc && (pc & 0x1fffffff) >= tlo && (pc & 0x1fffffff) < thi)
			{
				masm.Mov(w0, pc);
				masm.Mov(x16, reinterpret_cast<uint64_t>(&eeTraceHook));
				masm.Blr(x16);
			}
		}

		// TEMP: one-shot disasm dump of the MMX7 stall loop block range.
		if (getenv("LRPS2_DUMP_STALL") && (pc & 0x1fffffff) >= 0x0010b7a8 && (pc & 0x1fffffff) <= 0x0010bd74)
		{
			static u32 dumped = 0;
			if (!(dumped & (1u << ((pc >> 4) & 31))))
			{
				dumped |= (1u << ((pc >> 4) & 31));
				for (u32 q = pc, k = 0; k < 24; q += 4, k++)
				{
					u32 i = memRead32(q);
					fprintf(stderr, "[stall] %08x: %08x op=%02x funct=%02x rs=%u rt=%u rd=%u imm=%04x xlat=%d\n",
						q, i, i >> 26, i & 0x3f, (i >> 21) & 31, (i >> 16) & 31, (i >> 11) & 31, i & 0xffff, IsTranslatable(i));
					if ((i >> 26) == 0x02 || (i >> 26) == 0x03 || ((i >> 26) == 0 && ((i & 0x3f) == 8 || (i & 0x3f) == 9))
						|| ((i >> 26) >= 0x04 && (i >> 26) <= 0x07) || (i >> 26) == 0x01) { fprintf(stderr, "[stall]   ^branch, +delayslot\n"); break; }
				}
			}
		}

		u32 p = pc;
		int n = 0;
		int cyc = 0; // raw sum of the leading translated ops' cycle costs
		bool done = false;
		bool branch_end = false; // ended via EmitBranch: branch + delay slot baked
		// TEMP diagnostic (LRPS2_TRACE_STEP): per-instruction trace-hook calls so
		// the JIT pc stream is 1:1 comparable with a pure-interpreter run.
		static int trcstep = -1; static u32 slo = 0, shi = 0;
		if (trcstep < 0)
		{
			const char* l = getenv("LRPS2_TRACE_LO"); const char* h = getenv("LRPS2_TRACE_HI");
			trcstep = (getenv("LRPS2_TRACE_STEP") && l && h) ? 1 : 0;
			if (trcstep) { slo = (u32)strtoul(l, 0, 16); shi = (u32)strtoul(h, 0, 16); }
		}
		while (n < kMaxInsns)
		{
			const u32 insn = memRead32(p);
			if (trcstep && (p & 0x1fffffff) >= slo && (p & 0x1fffffff) < shi && IsTranslatable(insn))
			{
				masm.Mov(w0, p);
				masm.Mov(x16, reinterpret_cast<uint64_t>(&eeTraceHook));
				masm.Blr(x16);
			}
			if (EmitSimple(masm, gpr, insn))
			{
				cyc += OpCycles(insn); p += 4; n++;
				// TEMP diagnostic (LRPS2_EE_SPLIT_MEM): execute mem ops natively but
				// end the block right after each one (shortest-possible native blocks,
				// like NO_EE_MEM's block shape but keeping native mem execution) -- to
				// bisect "native mem op is wrong" vs "long native block interaction".
				static int split_mem = -1;
				if (split_mem < 0) split_mem = getenv("LRPS2_EE_SPLIT_MEM") ? 1 : 0;
				const u32 mop = insn >> 26;
				const bool is_mem = (mop >= 0x20 && mop <= 0x2b) || mop == 0x37 || mop == 0x3f || mop == 0x31 || mop == 0x39;
				if (split_mem && is_mem)
				{
					StorePCImm(masm, p);
					EmitCycleBookkeeping(masm, cyc);
					s_rc.FlushDirty(masm, gpr); // block exit: memory must be current
					EmitChainEpilogue(masm);
					done = true; break;
				}
				continue;
			}
			if (EmitBranch(masm, gpr, p, insn, cyc)) { done = true; branch_end = true; break; } // emits bookkeeping + pc + eventtest + chain epilogue
			break; // unsupported -> interpreter finishes the basic block
		}

		if (!done)
		{
			if (n > 0)
			{
				masm.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.pc));
				masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, 4 * n); masm.Str(w0, MemOperand(x10));
				EmitCycleBookkeeping(masm, cyc);
			}
			// The interpreter tail reads AND writes guest GPR memory: flush the
			// dirty cache before the call, and treat everything as unknown after
			// (the block ends right away, but keep the invariant explicit).
			s_rc.FlushDirty(masm, gpr);
			masm.Mov(x16, reinterpret_cast<uint64_t>(&eeRunBasicBlock_arm64));
			masm.Blr(x16);
			s_rc.Reset();
			EmitChainEpilogue(masm);
		}

		masm.FinalizeCode();

		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15);

		BlockFn fn = reinterpret_cast<BlockFn>(start);
		// Native code covers the leading translated run; a translated branch also
		// bakes the branch + delay slot (p still points at the branch there).
		const u32 ns = Norm(pc), ne = Norm(branch_end ? p + 8 : p);
		BlockRec rec{fn, ne, true, {}};
		if (ne > ns)
		{
			rec.src.resize((ne - ns) >> 2);
			for (u32 q = 0; q < (u32)rec.src.size(); q++)
				rec.src[q] = memRead32(pc + q * 4);
		}
		s_blocks[pc] = std::move(rec); // overwrites a dead record after SMC
		if (InRam(ns)) s_lut[ns >> 2] = fn;
		RegisterPages(pc, ns, ne);
		return fn;
	}

	// Revive a dead block if its baked-in source words are unchanged in live
	// memory (the page-clearing write hit data, not this code). Reinstalls the
	// existing native code and re-protects the page; returns nullptr when the
	// source really changed (caller erases + recompiles).
	BlockFn Revive(u32 pc, BlockRec& r)
	{
		for (u32 q = 0; q < (u32)r.src.size(); q++)
			if (memRead32(pc + q * 4) != r.src[q])
				return nullptr;
		r.live = true;
		const u32 ns = Norm(pc);
		if (InRam(ns)) s_lut[ns >> 2] = r.fn;
		RegisterPages(pc, ns, r.end);
		// TEMP diagnostic (LRPS2_JIT_STATS): prove revives happen (vs recompiles).
		static const bool stats = getenv("LRPS2_JIT_STATS") != nullptr;
		if (stats)
		{
			static u64 nrev = 0;
			if ((++nrev & 0xfff) == 0)
				fprintf(stderr, "[eerec] %llu block revives\n", (unsigned long long)nrev);
		}
		return r.fn;
	}

	inline BlockFn BlockForPC(u32 pc)
	{
		const u32 np = Norm(pc);
		if (InRam(np))
		{
			BlockFn f = s_lut[np >> 2];
			if (f)
				return f;
			auto it = s_blocks.find(pc);
			if (it != s_blocks.end())
			{
				if (BlockFn g = Revive(pc, it->second))
					return g;
				s_blocks.erase(it); // source changed -> compile fresh
			}
			return CompileBlock(pc);
		}
		auto it = s_blocks.find(pc);
		return (it != s_blocks.end() && it->second.live) ? it->second.fn : CompileBlock(pc);
	}
}

void eeJitReserve_arm64(void)
{
	s_ok = VixlEmitSelfTest();
	if (!s_code)
	{
		s_code = (u8*)mmap(nullptr, kCodeCacheSize, PROT_READ | PROT_WRITE | PROT_EXEC,
		                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (s_code == MAP_FAILED) s_code = nullptr;
	}
	if (!s_lut)
	{
		s_lut = (BlockFn*)mmap(nullptr, (size_t)kRamWords * sizeof(BlockFn), PROT_READ | PROT_WRITE,
		                       MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (s_lut == MAP_FAILED) s_lut = nullptr;
	}
	s_ok = s_ok && s_code && s_lut;
	Console.WriteLn("arm64 EE rec (C.7): %s.", s_ok ? "native ALU+mem+branch+FPU-mov+MMI+muldiv JIT (block-linking)" : "FAILED");
}

void eeJitReset_arm64(void)
{
	s_blocks.clear();
	s_page.clear();
	LutClearAll();
	s_code_pos = 0;
}

void eeJitClear_arm64(u32 addr, u32 size)
{
	// The clear MUST cover every block on the touched pages: the fault handler
	// unprotects the whole page, so later writes to it don't fault and any
	// surviving block would go stale silently. Blocks are marked dead (records
	// and native code kept); the next dispatch revalidates the source snapshot
	// and revives the block without recompiling when the code bytes survived.
	if (s_blocks.empty())
		return;
	const u32 a0 = Norm(addr);
	const u32 a1 = Norm(addr + (size ? size : 1) * 4 - 1);
	for (u32 pg = a0 >> kPageShift; pg <= a1 >> kPageShift; pg++)
	{
		auto it = s_page.find(pg);
		if (it == s_page.end())
			continue;
		for (u32 spc : it->second)
		{
			auto bit = s_blocks.find(spc);
			if (bit != s_blocks.end())
				bit->second.live = false;
			const u32 np = Norm(spc);
			if (InRam(np)) s_lut[np >> 2] = nullptr;
		}
		s_page.erase(it);
	}
}

void eeJitShutdown_arm64(void)
{
	s_blocks.clear();
	s_page.clear();
	if (s_code) { munmap(s_code, kCodeCacheSize); s_code = nullptr; }
	if (s_lut) { munmap(s_lut, (size_t)kRamWords * sizeof(BlockFn)); s_lut = nullptr; }
	s_code_pos = 0;
}

// TEMP DEBUG (C.24 crash hunt, LRPS2_FAULT_LOG): given a host fault pc, say
// whether it lies in the EE JIT cache, which block contains it, and hexdump
// the emitted code around it (offline-disassemblable). Not signal-safe --
// debug only.
extern "C" void eeJitDebugLocate_arm64(uintptr_t pc)
{
	if (!s_code || pc < (uintptr_t)s_code || pc >= (uintptr_t)s_code + kCodeCacheSize)
	{
		fprintf(stderr, "[locate] pc=%p NOT in EE cache (%p..%p)\n",
			(void*)pc, (void*)s_code, (void*)(s_code + kCodeCacheSize));
		return;
	}
	u32 best_pc = 0; uintptr_t best_fn = 0; bool best_live = false;
	for (const auto& kv : s_blocks)
	{
		const uintptr_t fn = (uintptr_t)kv.second.fn;
		if (fn <= pc && fn > best_fn) { best_fn = fn; best_pc = kv.first; best_live = kv.second.live; }
	}
	fprintf(stderr, "[locate] pc=%p in EE cache; block guest=%08x host=%p off=+%#lx live=%d\n",
		(void*)pc, best_pc, (void*)best_fn, (unsigned long)(pc - best_fn), (int)best_live);
	fprintf(stderr, "[locate] cache=%p pos=%#zx pc-off=%#lx wraps=%d tid=%ld\n",
		(void*)s_code, s_code_pos, (unsigned long)(pc - (uintptr_t)s_code), s_cache_wraps,
		(long)syscall(SYS_gettid));
	// Dump the whole emitted block from its start through a bit past the fault.
	{
		const u32* w = (const u32*)best_fn;
		const int nwords = (int)((pc - best_fn) / 4) + 24;
		for (int i = 0; i < nwords; i++)
			fprintf(stderr, "[code+%04x] %08x%s\n", i * 4, w[i],
				((uintptr_t)(w + i) == pc) ? "  <-- FAULT" : "");
	}
	// And the block's guest source snapshot (MIPS words) for cross-reference.
	auto it = s_blocks.find(best_pc);
	if (it != s_blocks.end())
	{
		const u32 ns = Norm(best_pc);
		fprintf(stderr, "[guest] block %08x native-end %08x src words %zu\n",
			best_pc, it->second.end, it->second.src.size());
		for (size_t i = 0; i < it->second.src.size(); i++)
			fprintf(stderr, "[mips %08x] %08x\n", ns + (u32)i * 4, it->second.src[i]);
	}
}

extern "C" void eeJitRunBlock_arm64(void)
{
	// TEMP DEBUG (C.24 crash hunt): identify the EE-dispatch thread once.
	if (getenv("LRPS2_FAULT_LOG"))
	{
		static bool once = false;
		if (!once) { once = true; fprintf(stderr, "[ee-thread] tid=%ld\n", (long)syscall(SYS_gettid)); }
	}
#ifdef EE_PC_SAMPLE
	// TEMP diagnostic: sample EE PC 1/256 dispatches, dump the hottest 14 PCs
	// every ~3M samples to find the loop the game spins on while stalled.
	{
		static std::unordered_map<u32, u64> hist;
		static u64 samples = 0;
		if (samples == 0) Console.WriteLn("[ee-sample] dispatch hook live");
		{
			hist[cpuRegs.pc & 0x1fffffff]++;
			if (++samples % 20000 == 0)
			{
				std::vector<std::pair<u32, u64>> v(hist.begin(), hist.end());
				std::partial_sort(v.begin(), v.begin() + (v.size() < 14 ? v.size() : 14), v.end(),
					[](auto& a, auto& b) { return a.second > b.second; });
				Console.WriteLn("=== EE PC histogram (uniq=%zu) ===", v.size());
				for (size_t i = 0; i < v.size() && i < 14; i++)
					Console.WriteLn("  pc=0x%08x  %llu", v[i].first, (unsigned long long)v[i].second);
				hist.clear();
			}
		}
	}
#endif
	BlockForPC(cpuRegs.pc)();
}

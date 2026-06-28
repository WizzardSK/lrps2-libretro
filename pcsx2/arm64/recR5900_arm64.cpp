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
#include "common/Console.h"

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <sys/mman.h>

// #define EE_PC_SAMPLE 1 // TEMP: EE PC histogram diagnostic (disables block-linking when on)

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

extern "C" void eeRunBasicBlock_arm64(void); // Interpreter.cpp
extern u32 cpuBlockCycles;                    // Interpreter.cpp
// EE memory wrappers + misalign cancel (Interpreter.cpp), for native loads/stores.
extern "C" u32  eeRead8_arm64(u32),  eeRead16_arm64(u32), eeRead32_arm64(u32);
extern "C" u64  eeRead64_arm64(u32);
extern "C" void eeWrite8_arm64(u32, u32),  eeWrite16_arm64(u32, u32);
extern "C" void eeWrite32_arm64(u32, u32), eeWrite64_arm64(u32, u64);
extern "C" void eeCancelInstruction_arm64(void);
extern "C" void eeEventTest_arm64(void);
extern "C" void eeUpdateCycles_arm64(void);

namespace
{
	typedef void (*BlockFn)(void);

	constexpr size_t kCodeCacheSize = 32 * 1024 * 1024;
	constexpr int    kMaxInsns      = 64;
	constexpr u32    kPageShift     = 12;

	u8*    s_code     = nullptr;
	size_t s_code_pos = 0;
	bool   s_ok       = false;

	std::unordered_map<u32, BlockFn>          s_blocks;
	std::unordered_map<u32, std::vector<u32>> s_page;

	// Direct-mapped block cache for the 32 MB EE RAM (the hot region): O(1) array
	// index instead of a hash lookup per block. ROM/scratchpad fall back to the
	// hash. (Foundation for cheaper block-ending; full block-linking is future.)
	constexpr u32 kRamBytes = 0x02000000;
	constexpr u32 kRamWords = kRamBytes >> 2;
	BlockFn*      s_lut      = nullptr;

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

	// EE GPRs are 128-bit; integer ops use the low 64 bits at byte offset idx*16.
	inline void LoadGpr(MacroAssembler& m, const Register& xd, const Register& gpr, u32 idx)
	{
		if (idx == 0) m.Mov(xd, 0);
		else          m.Ldr(xd, MemOperand(gpr, idx * 16));
	}
	inline void StoreGpr(MacroAssembler& m, const Register& xs, const Register& gpr, u32 idx)
	{
		m.Str(xs, MemOperand(gpr, idx * 16));
	}

	// COP1 (FPU) register sub-ops that are *pure bit moves* -- no PS2 FPU
	// arithmetic, so they are bit-exact and safe to translate natively. The
	// quirky non-IEEE arithmetic (ADD.S/MUL.S/DIV.S, CVT, C.cond, MADD, ...),
	// BC1 and CFC1/CTC1 stay with the interpreter.
	//   rs=0x00 MFC1, rs=0x04 MTC1, rs=0x10 + funct {0x05 ABS.S,0x06 MOV.S,0x07 NEG.S}
	bool Cop1RegSupported(u32 insn)
	{
		const u32 rs = (insn >> 21) & 31;
		if (rs == 0x00 || rs == 0x04) return true;
		if (rs == 0x10)
		{
			const u32 funct = insn & 0x3f;
			return funct == 0x05 || funct == 0x06 || funct == 0x07;
		}
		return false;
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
		// rs == 0x10 single-precision bit ops
		m.Mov(x1, fbase);
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
	// logical, signed compares, signed max/min, and the EXTL/EXTU interleaves).
	// PMFHL/PMTHL, the HI/LO multiply-divide ops, pack, variable shifts, PMADD,
	// PCPY*, etc. stay with the interpreter.
	enum { M_ADD, M_SUB, M_SQADD, M_UQADD, M_SQSUB, M_UQSUB, M_AND, M_ORR, M_EOR,
	       M_NOR, M_CMGT, M_CMEQ, M_SMAX, M_SMIN, M_ZIP1, M_ZIP2 };

	// Decode an MMI instruction to (action, lane fmt: 0=16B,1=8H,2=4S). Returns the
	// action or -1 if not natively supported. funct selects the sub-class
	// (MMI0=0x08, MMI1=0x28, MMI2=0x09, MMI3=0x29); (insn>>6)&0x1f the op.
	int MmiAction(u32 insn, int* fmt)
	{
		const u32 funct = insn & 0x3f;
		const u32 sub   = (insn >> 6) & 0x1f;
		switch (funct)
		{
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
					case 0x14: *fmt = 1; return M_SQADD; // PADDSH
					case 0x15: *fmt = 1; return M_SQSUB; // PSUBSH
					case 0x16: *fmt = 1; return M_ZIP1;  // PEXTLH
					case 0x18: *fmt = 0; return M_SQADD; // PADDSB
					case 0x19: *fmt = 0; return M_SQSUB; // PSUBSB
					case 0x1a: *fmt = 0; return M_ZIP1;  // PEXTLB
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
				if (sub == 0x12) { *fmt = 0; return M_AND; } // PAND
				if (sub == 0x13) { *fmt = 0; return M_EOR; } // PXOR
				return -1;
			case 0x29: // MMI3
				if (sub == 0x12) { *fmt = 0; return M_ORR; } // POR
				if (sub == 0x13) { *fmt = 0; return M_NOR; } // PNOR
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

		LoadQ(m, q1, gpr, rs); // rs -> v1
		LoadQ(m, q2, gpr, rt); // rt -> v2
		VRegister d, n, o;     // n = rs(v1), o = rt(v2)
		switch (fmt)
		{
			case 0:  d = v0.V16B(); n = v1.V16B(); o = v2.V16B(); break;
			case 1:  d = v0.V8H();  n = v1.V8H();  o = v2.V8H();  break;
			default: d = v0.V4S();  n = v1.V4S();  o = v2.V4S();  break;
		}
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
		}
		m.Str(q0, MemOperand(gpr, rd * 16));
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
		if (idx == 0) m.Mov(wd, 0);
		else          m.Ldr(wd, MemOperand(gpr, idx * 16));
	}

	void EmitMulDiv(MacroAssembler& m, const Register& gpr, u32 funct, u32 rs, u32 rt, u32 rd)
	{
		switch (funct)
		{
			case 0x10: if (rd) { m.Ldr(x0, MemOperand(gpr, kHI)); StoreGpr(m, x0, gpr, rd); } return; // MFHI
			case 0x12: if (rd) { m.Ldr(x0, MemOperand(gpr, kLO)); StoreGpr(m, x0, gpr, rd); } return; // MFLO
			case 0x11: LoadGpr(m, x0, gpr, rs); m.Str(x0, MemOperand(gpr, kHI)); return;              // MTHI
			case 0x13: LoadGpr(m, x0, gpr, rs); m.Str(x0, MemOperand(gpr, kLO)); return;              // MTLO

			case 0x18: // MULT (signed)
			case 0x19: // MULTU (unsigned)
			{
				LoadW(m, w0, gpr, rs); LoadW(m, w1, gpr, rt);
				if (funct == 0x18) m.Smull(x0, w0, w1); else m.Umull(x0, w0, w1);
				m.Sxtw(x2, w0);                // LO = (s32)(res & 0xffffffff)
				m.Lsr(x3, x0, 32); m.Sxtw(x3, w3); // HI = (s32)(res >> 32)
				m.Str(x2, MemOperand(gpr, kLO));
				m.Str(x3, MemOperand(gpr, kHI));
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
				m.Str(x2, MemOperand(gpr, kLO)); m.Str(x3, MemOperand(gpr, kHI));
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
				m.Str(x2, MemOperand(gpr, kLO)); m.Str(x3, MemOperand(gpr, kHI));
				return;
			}
		}
	}

	// Per-opcode cycle cost, mirroring R5900OpcodeTables.cpp: Default=9, Branch=11,
	// Mult=2*8=16, Div=14*8=112. Used to accumulate cpuBlockCycles accurately
	// (cpuBlockCycles += cost * (2 - CP0.Config[18]), summed over the block's ops).
	int OpCycles(u32 insn)
	{
		const u32 op = insn >> 26, funct = insn & 0x3f;
		if (op == 0x00)
		{
			if (funct == 0x18 || funct == 0x19) return 16;  // MULT/MULTU
			if (funct == 0x1a || funct == 0x1b) return 112; // DIV/DIVU
		}
		if (op == 0x01 || op == 0x04 || op == 0x05 || op == 0x06 || op == 0x07) return 11; // Branch
		return 9; // Default (incl. JR/JALR/J/JAL)
	}

	// Translate one simple integer ALU op (32- and 64-bit forms). Returns false,
	// emitting nothing, for control flow / memory / FPU / MMI / anything else.
	// TEMP diagnostic: env-gated toggles to force categories of EE ops back to
	// the interpreter, to bisect which native EE JIT feature breaks MMX7's
	// post-logo progression. Read once. Mirrored in IsTranslatable so the block
	// builder and delay-slot handling stay consistent with EmitSimple.
	struct EeDiagFlags { int no_mmi, no_muldiv, no_cop1, no_mem, no_load, no_store, no_branch, no_ld64; };
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
		};
		return f;
	}

	bool EmitSimple(MacroAssembler& m, const Register& gpr, u32 insn)
	{
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
		const int no_load = eeDiag().no_load, no_store = eeDiag().no_store;
		const int no_ld64 = eeDiag().no_ld64;

		switch (op)
		{
			case 0x00:
				switch (funct)
				{
					// 32-bit shifts -> sign-extend to 64
					case 0x00: if (rd) { LoadGpr(m, x0, gpr, rt); m.Lsl(w0, w0, sa); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SLL
					case 0x02: if (rd) { LoadGpr(m, x0, gpr, rt); m.Lsr(w0, w0, sa); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SRL
					case 0x03: if (rd) { LoadGpr(m, x0, gpr, rt); m.Asr(w0, w0, sa); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SRA
					case 0x04: if (rd) { LoadGpr(m, x0, gpr, rt); LoadGpr(m, x1, gpr, rs); m.Lsl(w0, w0, w1); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SLLV
					case 0x06: if (rd) { LoadGpr(m, x0, gpr, rt); LoadGpr(m, x1, gpr, rs); m.Lsr(w0, w0, w1); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SRLV
					case 0x07: if (rd) { LoadGpr(m, x0, gpr, rt); LoadGpr(m, x1, gpr, rs); m.Asr(w0, w0, w1); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SRAV
					// 32-bit add/sub -> sign-extend to 64
					case 0x21: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Add(w0, w0, w1); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // ADDU
					case 0x23: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Sub(w0, w0, w1); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rd); } return true; // SUBU
					// 64-bit logic / set / add / sub
					case 0x24: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.And(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // AND
					case 0x25: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Orr(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // OR
					case 0x26: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Eor(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // XOR
					case 0x27: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Orr(x0, x0, x1); m.Mvn(x0, x0); StoreGpr(m, x0, gpr, rd); } return true; // NOR
					case 0x2a: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Cmp(x0, x1); m.Cset(x0, lt); StoreGpr(m, x0, gpr, rd); } return true; // SLT
					case 0x2b: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Cmp(x0, x1); m.Cset(x0, lo); StoreGpr(m, x0, gpr, rd); } return true; // SLTU
					case 0x2c: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Add(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // DADDU
					case 0x2d: if (rd) { LoadGpr(m, x0, gpr, rs); LoadGpr(m, x1, gpr, rt); m.Sub(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // DSUBU
					// 64-bit shifts (variable)
					case 0x14: if (rd) { LoadGpr(m, x0, gpr, rt); LoadGpr(m, x1, gpr, rs); m.Lsl(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // DSLLV
					case 0x16: if (rd) { LoadGpr(m, x0, gpr, rt); LoadGpr(m, x1, gpr, rs); m.Lsr(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // DSRLV
					case 0x17: if (rd) { LoadGpr(m, x0, gpr, rt); LoadGpr(m, x1, gpr, rs); m.Asr(x0, x0, x1); StoreGpr(m, x0, gpr, rd); } return true; // DSRAV
					// 64-bit shifts (immediate)
					case 0x38: if (rd) { LoadGpr(m, x0, gpr, rt); m.Lsl(x0, x0, sa);      StoreGpr(m, x0, gpr, rd); } return true; // DSLL
					case 0x3a: if (rd) { LoadGpr(m, x0, gpr, rt); m.Lsr(x0, x0, sa);      StoreGpr(m, x0, gpr, rd); } return true; // DSRL
					case 0x3b: if (rd) { LoadGpr(m, x0, gpr, rt); m.Asr(x0, x0, sa);      StoreGpr(m, x0, gpr, rd); } return true; // DSRA
					case 0x3c: if (rd) { LoadGpr(m, x0, gpr, rt); m.Lsl(x0, x0, sa + 32); StoreGpr(m, x0, gpr, rd); } return true; // DSLL32
					case 0x3e: if (rd) { LoadGpr(m, x0, gpr, rt); m.Lsr(x0, x0, sa + 32); StoreGpr(m, x0, gpr, rd); } return true; // DSRL32
					case 0x3f: if (rd) { LoadGpr(m, x0, gpr, rt); m.Asr(x0, x0, sa + 32); StoreGpr(m, x0, gpr, rd); } return true; // DSRA32
					// HI/LO moves + integer mult/div (bit-exact, sign-extended results)
					case 0x10: case 0x11: case 0x12: case 0x13: // MFHI/MTHI/MFLO/MTLO
					case 0x18: case 0x19: case 0x1a: case 0x1b: // MULT/MULTU/DIV/DIVU
						if (no_muldiv) return false;
						EmitMulDiv(m, gpr, funct, rs, rt, rd); return true;
					default: return false;
				}
			case 0x09: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(w1, (u32)simm); m.Add(w0, w0, w1); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rt); } return true; // ADDIU
			case 0x19: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(x1, simm64); m.Add(x0, x0, x1); StoreGpr(m, x0, gpr, rt); } return true; // DADDIU
			case 0x0a: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(x1, simm64); m.Cmp(x0, x1); m.Cset(x0, lt); StoreGpr(m, x0, gpr, rt); } return true; // SLTI
			case 0x0b: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(x1, simm64); m.Cmp(x0, x1); m.Cset(x0, lo); StoreGpr(m, x0, gpr, rt); } return true; // SLTIU
			case 0x0c: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(x1, (uint64_t)zimm); m.And(x0, x0, x1); StoreGpr(m, x0, gpr, rt); } return true; // ANDI
			case 0x0d: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(x1, (uint64_t)zimm); m.Orr(x0, x0, x1); StoreGpr(m, x0, gpr, rt); } return true; // ORI
			case 0x0e: if (rt) { LoadGpr(m, x0, gpr, rs); m.Mov(x1, (uint64_t)zimm); m.Eor(x0, x0, x1); StoreGpr(m, x0, gpr, rt); } return true; // XORI
			case 0x0f: if (rt) { m.Mov(w0, zimm << 16); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, rt); } return true; // LUI

			// Loads (LB/LBU/LH/LHU/LW/LWU/LD). Address = rs.UL[0] + simm (32-bit).
			// The interpreter cancels (fastjmp) on a misaligned LH/LW/LD, so do the
			// same; reads run even when rt==0 (side effects). Memory goes through
			// the EE vtlb wrappers. gpr (x19) is callee-saved across the calls.
			case 0x20: case 0x24: case 0x21: case 0x25: case 0x23: case 0x27: case 0x37:
			{
				if (no_load || (no_ld64 && op == 0x37)) return false;
				LoadGpr(m, x0, gpr, rs); m.Mov(w2, (u32)simm); m.Add(w0, w0, w2);
				const u32 amask = (op == 0x37) ? 7 : (op == 0x23 || op == 0x27) ? 3 : (op == 0x21 || op == 0x25) ? 1 : 0;
				if (amask) { Label ok; m.Tst(w0, amask); m.B(&ok, eq); m.Mov(x16, reinterpret_cast<uint64_t>(&eeCancelInstruction_arm64)); m.Blr(x16); m.Bind(&ok); }
				uint64_t fn = (op == 0x37) ? reinterpret_cast<uint64_t>(&eeRead64_arm64)
				            : (op == 0x23 || op == 0x27) ? reinterpret_cast<uint64_t>(&eeRead32_arm64)
				            : (op == 0x21 || op == 0x25) ? reinterpret_cast<uint64_t>(&eeRead16_arm64)
				            : reinterpret_cast<uint64_t>(&eeRead8_arm64);
				m.Mov(x16, fn); m.Blr(x16); // result in x0 (u32 returns are zero-extended in x0)
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
			// Stores (SB/SH/SW/SD).
			case 0x28: case 0x29: case 0x2b: case 0x3f:
			{
				if (no_store || (no_ld64 && op == 0x3f)) return false;
				LoadGpr(m, x0, gpr, rs); m.Mov(w2, (u32)simm); m.Add(w0, w0, w2);
				const u32 amask = (op == 0x3f) ? 7 : (op == 0x2b) ? 3 : (op == 0x29) ? 1 : 0;
				if (amask) { Label ok; m.Tst(w0, amask); m.B(&ok, eq); m.Mov(x16, reinterpret_cast<uint64_t>(&eeCancelInstruction_arm64)); m.Blr(x16); m.Bind(&ok); }
				LoadGpr(m, x1, gpr, rt); // value (low bits used by the wrapper)
				uint64_t fn = (op == 0x3f) ? reinterpret_cast<uint64_t>(&eeWrite64_arm64)
				            : (op == 0x2b) ? reinterpret_cast<uint64_t>(&eeWrite32_arm64)
				            : (op == 0x29) ? reinterpret_cast<uint64_t>(&eeWrite16_arm64)
				            : reinterpret_cast<uint64_t>(&eeWrite8_arm64);
				m.Mov(x16, fn); m.Blr(x16);
				return true;
			}

			// COP1 register ops (MFC1/MTC1/MOV.S/NEG.S/ABS.S only -- arithmetic and
			// BC1/CFC1/CTC1 hand off to the interpreter).
			case 0x11:
				if (no_cop1 || !Cop1RegSupported(insn)) return false;
				EmitCop1Reg(m, gpr, insn);
				return true;

			// MMI (128-bit parallel integer) -> NEON, for the bit-exact subset.
			case 0x1c:
				if (no_mmi || !MmiSupported(insn)) return false;
				EmitMmi(m, gpr, insn);
				return true;

			// LWC1 ft, off(base): fpr[ft].UL = read32(addr). Unlike integer LW, a
			// misaligned address is *silently skipped* (no exception/cancel) -- match
			// the interpreter's `if (addr & 3) return;`.
			case 0x31:
			{
				if (no_cop1) return false;
				LoadGpr(m, x0, gpr, rs); m.Mov(w2, (u32)simm); m.Add(w0, w0, w2);
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
				LoadGpr(m, x0, gpr, rs); m.Mov(w2, (u32)simm); m.Add(w0, w0, w2);
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
		m.Ldp(x19, x20, MemOperand(sp, 0));
		m.Ldp(x21, x30, MemOperand(sp, 16));
		m.Add(sp, sp, 32);

#ifdef EE_PC_SAMPLE
		m.Ret(); // TEMP: disable block-linking so every block returns to the dispatcher (PC sampling)
		return;
#endif
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
		m.Br(x3);                           // tail-jump to next block
		m.Bind(&ret_path);
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
				case 0x2a: case 0x2b: case 0x2c: case 0x2d:
				case 0x14: case 0x16: case 0x17: case 0x38: case 0x3a: case 0x3b:
				case 0x3c: case 0x3e: case 0x3f:
					return true;
				case 0x10: case 0x11: case 0x12: case 0x13: // MFHI/MTHI/MFLO/MTLO
				case 0x18: case 0x19: case 0x1a: case 0x1b: // MULT/MULTU/DIV/DIVU
					return !eeDiag().no_muldiv;
				default: return false;
			}
		}
		if (op == 0x11) return !eeDiag().no_cop1 && Cop1RegSupported(insn);
		if (op == 0x1c) return !eeDiag().no_mmi && MmiSupported(insn);
		switch (op)
		{
			case 0x09: case 0x19: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
				return true;
			case 0x20: case 0x21: case 0x23: case 0x24: case 0x25: case 0x27:
				return !eeDiag().no_load;
			case 0x37: return !eeDiag().no_load && !eeDiag().no_ld64; // LD
			case 0x28: case 0x29: case 0x2b:
				return !eeDiag().no_store;
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
		const u32 op = insn >> 26, rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);

		bool uncond = false, is_jr = false, two = false, one = false;
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
		else if (op == 0x01)
		{
			const u32 t = bpc + 4 + simm * 4;
			if      (rt == 0x00) { one = true; cond = lt; tconst = t; }
			else if (rt == 0x01) { one = true; cond = ge; tconst = t; }
			else if (rt == 0x10) { one = true; cond = lt; tconst = t; link = 31; }
			else if (rt == 0x11) { one = true; cond = ge; tconst = t; link = 31; }
			else return false;
		}
		else return false;

		const u32 ds = memRead32(bpc + 4);
		if (!IsTranslatable(ds))
			return false;

		// Taken/uncond runs the branch + its delay slot inline; not-taken runs only
		// the branch (the delay slot at bpc+4 is executed by the continuation).
		const int cyc_taken = cyc_leading + OpCycles(insn) + OpCycles(ds);
		const int cyc_nt    = cyc_leading + OpCycles(insn);

		const uint64_t evt = reinterpret_cast<uint64_t>(&eeEventTest_arm64);
		const uint64_t upd = reinterpret_cast<uint64_t>(&eeUpdateCycles_arm64);
		const bool nt_evt = (op == 0x04 || op == 0x05); // only BEQ/BNE event-test on not-taken

		if (link > 0) { m.Mov(w0, bpc + 8); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, (u32)link); }
		if (is_jr) LoadGpr(m, x20, gpr, rs);

		if (uncond)
		{
			EmitSimple(m, gpr, ds);
			EmitCycleBookkeeping(m, cyc_taken);
			if (is_jr) StorePC(m, w20); else StorePCImm(m, tconst);
			m.Mov(x16, upd); m.Blr(x16);
			m.Mov(x16, evt); m.Blr(x16);
			EmitChainEpilogue(m);
			return true;
		}

		LoadGpr(m, x0, gpr, rs);
		if (two) { LoadGpr(m, x1, gpr, rt); m.Cmp(x0, x1); } else m.Cmp(x0, 0);
		Label not_taken;
		m.B(&not_taken, InvertCondition(cond));
		EmitSimple(m, gpr, ds);
		EmitCycleBookkeeping(m, cyc_taken);
		StorePCImm(m, tconst);
		m.Mov(x16, upd); m.Blr(x16);
		m.Mov(x16, evt); m.Blr(x16);
		EmitChainEpilogue(m);
		m.Bind(&not_taken);
		EmitCycleBookkeeping(m, cyc_nt);
		StorePCImm(m, bpc + 4);
		// Must flush cpuBlockCycles into cpuRegs.cycle (eeUpdateCycles) BEFORE the
		// event test, exactly like the taken path -- otherwise a spin/poll loop that
		// keeps falling through this branch event-tests with a stale cpuRegs.cycle,
		// so scheduled IPU/DMA/counter events never reach their trigger and the EE
		// livelocks (MMX7 stuck polling IPU after the logo).
		if (nt_evt) { m.Mov(x16, upd); m.Blr(x16); m.Mov(x16, evt); m.Blr(x16); }
		EmitChainEpilogue(m);
		return true;
	}

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 8192 > kCodeCacheSize)
		{
			s_blocks.clear();
			s_page.clear();
			LutClearAll();
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		const Register gpr = x19;
		masm.Sub(sp, sp, 32);
		masm.Stp(x19, x20, MemOperand(sp, 0));
		masm.Stp(x21, x30, MemOperand(sp, 16));
		masm.Mov(gpr, reinterpret_cast<uint64_t>(&cpuRegs.GPR.r[0]));

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
		while (n < kMaxInsns)
		{
			const u32 insn = memRead32(p);
			if (EmitSimple(masm, gpr, insn)) { cyc += OpCycles(insn); p += 4; n++; continue; }
			if (EmitBranch(masm, gpr, p, insn, cyc)) { done = true; break; } // emits bookkeeping + pc + eventtest + chain epilogue
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
			masm.Mov(x16, reinterpret_cast<uint64_t>(&eeRunBasicBlock_arm64));
			masm.Blr(x16);
			EmitChainEpilogue(masm);
		}

		masm.FinalizeCode();

		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15);

		BlockFn fn = reinterpret_cast<BlockFn>(start);
		s_blocks.emplace(pc, fn);
		const u32 ns = Norm(pc), ne = Norm(p > pc ? p : pc + 4);
		if (InRam(ns)) s_lut[ns >> 2] = fn;
		for (u32 pg = ns >> kPageShift; pg <= (ne - 1) >> kPageShift; pg++)
			s_page[pg].push_back(pc);
		return fn;
	}

	inline BlockFn BlockForPC(u32 pc)
	{
		const u32 np = Norm(pc);
		if (InRam(np))
		{
			BlockFn f = s_lut[np >> 2];
			return f ? f : CompileBlock(pc);
		}
		auto it = s_blocks.find(pc);
		return it != s_blocks.end() ? it->second : CompileBlock(pc);
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
			s_blocks.erase(spc);
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

extern "C" void eeJitRunBlock_arm64(void)
{
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

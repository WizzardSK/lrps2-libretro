// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 VU1 recompiler -- Phase C.14-1: skeleton (code cache + per-PC block
// dispatch + interpreter-executed blocks).
//
// Mirrors the proven C.2b-1 (IOP) / C.3-1 (EE) bring-up pattern: the provider
// (CpuRecVU1_arm64) replaces InterpVU1 as CpuVU1, its Execute() keeps the
// interpreter's outer loop semantics EXACTLY (same cycle-budget expression,
// same VPU_STAT busy check, same branch fixup on stop), but dispatches through
// a direct-mapped LUT of compiled blocks over the 16KB VU1 micro memory. Each
// skeleton block is native AArch64 code that just calls vu1RunBlock_arm64(pc),
// which executes interpreter instruction pairs (vu1Exec) until the block ends:
// a taken branch/jump (TPC leaves sequential flow), the E-bit clearing busy,
// or the caller's cycle budget expiring -- checked per instruction, exactly
// like InterpVU1::Execute, so behaviour is byte-identical while the JIT
// plumbing (cache, dispatch, invalidation, MTVU-worker execution) runs for
// real. Later phases translate the VU upper/lower pairs natively.
//
// SMC note: VU1 microprograms are re-uploaded constantly (VIF MPG / MTVU
// micro writes). The skeleton is immune by construction -- blocks read the
// live micro memory through the interpreter -- but Clear() already drops the
// LUT so the invalidation contract is exercised before native translation
// needs it. Clear is cheap: programs are tiny and recompile in microseconds.

#include "common/Pcsx2Types.h"
#include "R5900.h"
#include "VUmicro.h"
#include "VUops.h"
#include "Vif.h"
#include "Hw.h"
#include "Dmac.h"
#include "Config.h"
#include "common/Console.h"
#include "common/FPControl.h"

#include <algorithm>
#include <cstring>
#include <sys/mman.h>

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

extern void _vuFlushAll(VURegs* VU); // VUops.cpp (declared locally by the interpreter too)

namespace
{
	constexpr u32    kProgSize      = 0x4000; // VU1 micro memory (16KB)
	constexpr u32    kProgMask      = kProgSize - 1;
	constexpr u32    kLutEntries    = kProgSize >> 3; // one per upper/lower pair
	constexpr size_t kCodeCacheSize = 2 * 1024 * 1024;

	using BlockFn = void (*)();

	u8*     s_code = nullptr;
	size_t  s_code_pos = 0;
	BlockFn s_lut[kLutEntries];
	bool    s_ok = false;

	// Cycle budget of the current Execute() call, visible to the block runner
	// so it can stop exactly where the interpreter's outer loop would have.
	// Single-writer: only one thread executes VU1 at a time (the EE thread, or
	// the MTVU worker when vuThread is on).
	u64 s_run_start = 0;
	u32 s_run_budget = 0;
}

// C.14-2: pre-decoded instruction pair. Everything that depends only on the
// instruction words is resolved at compile time: the two opcode dispatch
// pointers, both _VURegsNum decode snapshots (built via the VU1regs_* tables),
// and the E/D/T/I bit tests. The execution helper below replays _vu1Exec
// (VU1microInterp.cpp) verbatim against these cached values, so behaviour --
// including the full FMAC/FDIV/EFU/IALU stall+flag pipeline model and the
// XGKICK drip -- stays bit-identical to the interpreter. Records live in a
// fixed per-slot array; Clear() drops the LUT slot, and recompilation refills
// the record, which is what makes the cached decode SMC-safe.
struct VU1DecodedPair
{
	u32 ucode, lcode;
	FnPtr_VuVoid ufn, lfn;
	_VURegsNum uregs, lregs; // lregs zeroed for I-bit pairs (VUPIPE_NONE)
	bool ebit, dbit, tbit, ibit;
};

namespace
{
	VU1DecodedPair s_pairs[kLutEntries];
}

// Runs interpreter instruction pairs from the current TPC until the basic
// block ends. Kept extern "C" so compiled blocks can call it directly.
extern "C" void vu1RunBlock_arm64(void)
{
	VURegs& r = vuRegs[1];
	for (;;)
	{
		if (!((vuRegs[1].cycle - s_run_start) < s_run_budget))
			return; // budget expired (same expression as InterpVU1::Execute)
		if (!(vuRegs[0].VI[REG_VPU_STAT].UL & 0x100))
			return; // program ended (E-bit) -- outer loop does the branch fixup
		r.VI[REG_TPC].UL &= kProgMask;
		const u32 expect = (r.VI[REG_TPC].UL + 8) & kProgMask;
		vu1Exec(&r);
		if (r.VI[REG_TPC].UL != expect)
			return; // taken branch/jump: block boundary, re-dispatch
	}
}

// Executes one pre-decoded upper/lower pair. Structured as a line-for-line
// mirror of vu1Exec + _vu1Exec (VU1microInterp.cpp) with the decode replaced
// by the cached record -- any change there must be replicated here.
extern "C" void vu1ExecPair_arm64(const VU1DecodedPair* d)
{
	VURegs* const VU = &vuRegs[1];
	VU->cycle++;

	_VURegsNum uregs = d->uregs; // locals: the stall helpers take mutable ptrs
	_VURegsNum lregs = d->lregs;

	VU->VI[REG_TPC].UL += 8;

	if (d->ebit)
		VU->ebit = 2;
	if (d->dbit && (vuRegs[0].VI[REG_FBRST].UL & 0x400))
	{
		vuRegs[0].VI[REG_VPU_STAT].UL |= 0x200;
		hwIntcIrq(INTC_VU1);
		VU->ebit = 1;
	}
	if (d->tbit && (vuRegs[0].VI[REG_FBRST].UL & 0x800))
	{
		vuRegs[0].VI[REG_VPU_STAT].UL |= 0x400;
		hwIntcIrq(INTC_VU1);
		VU->ebit = 1;
	}

	VU->code = d->ucode;
	const u64 cyclesBeforeOp = vuRegs[1].cycle - 1;

	_vuTestUpperStalls(VU, &uregs);

	if (d->ibit)
	{
		_vuTestPipes(VU);
		if (VU->VIBackupCycles > 0)
			VU->VIBackupCycles -= std::min((u8)(vuRegs[1].cycle - cyclesBeforeOp), VU->VIBackupCycles);
		VU->code = d->ucode;
		d->ufn();
		VU->VI[REG_I].UL = d->lcode;
		// lregs stays zeroed (VUPIPE_NONE), matching the interpreter's memset.
	}
	else
	{
		VECTOR _VF;
		VECTOR _VFc;
		REG_VI _VI;
		REG_VI _VIc;
		int vfreg = 0;
		int vireg = 0;
		int discard = 0;

		VU->code = d->lcode;
		_vuTestLowerStalls(VU, &lregs);
		_vuTestPipes(VU);

		if (VU->VIBackupCycles > 0)
			VU->VIBackupCycles -= std::min((u8)(vuRegs[1].cycle - cyclesBeforeOp), VU->VIBackupCycles);

		if (uregs.VFwrite)
		{
			if (lregs.VFwrite == uregs.VFwrite)
				discard = 1;
			if (lregs.VFread0 == uregs.VFwrite ||
				lregs.VFread1 == uregs.VFwrite)
			{
				_VF = VU->VF[uregs.VFwrite];
				vfreg = uregs.VFwrite;
			}
		}
		if (uregs.VIwrite & (1 << REG_CLIP_FLAG))
		{
			if (lregs.VIwrite & (1 << REG_CLIP_FLAG))
				discard = 1;
			if (lregs.VIread & (1 << REG_CLIP_FLAG))
			{
				_VI = VU->VI[REG_CLIP_FLAG];
				vireg = REG_CLIP_FLAG;
			}
		}

		VU->code = d->ucode;
		d->ufn();

		if (discard == 0)
		{
			if (vfreg)
			{
				_VFc = VU->VF[vfreg];
				VU->VF[vfreg] = _VF;
			}
			if (vireg)
			{
				_VIc = VU->VI[vireg];
				VU->VI[vireg] = _VI;
			}

			VU->code = d->lcode;
			d->lfn();

			if (vfreg)
				VU->VF[vfreg] = _VFc;
			if (vireg)
				VU->VI[vireg] = _VIc;
		}
	}

	if (uregs.pipe == VUPIPE_FMAC || lregs.pipe == VUPIPE_FMAC)
		_vuClearFMAC(VU);

	_vuAddUpperStalls(VU, &uregs);
	_vuAddLowerStalls(VU, &lregs);

	if (VU->branch > 0)
	{
		if (VU->branch-- == 1)
		{
			VU->VI[REG_TPC].UL = VU->branchpc;

			if (VU->takedelaybranch)
			{
				VU->branch = 1;
				VU->branchpc = VU->delaybranchpc;
				VU->takedelaybranch = false;
			}
		}
	}

	if (VU->ebit > 0)
	{
		if (VU->ebit-- == 1)
		{
			VU->VIBackupCycles = 0;
			_vuFlushAll(VU);
			vuRegs[0].VI[REG_VPU_STAT].UL &= ~0x100;
			vif1Regs.stat.VEW = false;

			if (vuRegs[1].xgkickenable)
				_vuXGKICKTransfer(0, true);
			if (INSTANT_VU1)
				vuRegs[1].xgkicklastcycle = cpuRegs.cycle;
		}
	}

	if (uregs.pipe == VUPIPE_FMAC || lregs.pipe == VUPIPE_FMAC)
		VU->fmacwritepos = (VU->fmacwritepos + 1) & 3;

	// vu1Exec tail: XGKICK drip once per executed instruction.
	if (vuRegs[1].xgkickenable)
		_vuXGKICKTransfer((vuRegs[1].cycle - vuRegs[1].xgkicklastcycle) - 1, false);
}

namespace
{
	void DecodePair(u32 pc)
	{
		VU1DecodedPair& d = s_pairs[(pc & kProgMask) >> 3];
		const u32* ptr = (u32*)&vuRegs[1].Micro[pc & kProgMask];
		d.lcode = ptr[0];
		d.ucode = ptr[1];
		d.ebit  = (ptr[1] & 0x40000000) != 0;
		d.dbit  = (ptr[1] & 0x10000000) != 0;
		d.tbit  = (ptr[1] & 0x08000000) != 0;
		d.ibit  = (ptr[1] & 0x80000000) != 0;
		d.ufn   = VU1_UPPER_OPCODE[d.ucode & 0x3f];
		d.lfn   = VU1_LOWER_OPCODE[d.lcode >> 25];

		// Regs-decode via the same tables the interpreter uses. They read
		// VU->code, so stage it; compile time only, VU->code is scratch.
		vuRegs[1].code = d.ucode;
		VU1regs_UPPER_OPCODE[d.ucode & 0x3f](&d.uregs);
		memset(&d.lregs, 0, sizeof(d.lregs)); // I-bit form (VUPIPE_NONE)
		if (!d.ibit)
		{
			vuRegs[1].code = d.lcode;
			d.lregs.cycles = 0;
			VU1regs_LOWER_OPCODE[d.lcode >> 25](&d.lregs);
		}
	}

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 256 > kCodeCacheSize)
		{
			// Cache full: drop everything (VU programs are tiny; this is rare).
			memset(s_lut, 0, sizeof(s_lut));
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		// C.14-2 block: pre-decode the pair, emit a call to the specialized
		// pair executor with the record pointer. Later phases replace this
		// with inline native translations for supported pairs.
		DecodePair(pc);
		masm.Stp(x29, x30, MemOperand(sp, -16, PreIndex));
		masm.Mov(x0, reinterpret_cast<uint64_t>(&s_pairs[(pc & kProgMask) >> 3]));
		masm.Mov(x16, reinterpret_cast<uint64_t>(&vu1ExecPair_arm64));
		masm.Blr(x16);
		masm.Ldp(x29, x30, MemOperand(sp, 16, PostIndex));
		masm.Ret();
		masm.FinalizeCode();

		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15);

		BlockFn fn = reinterpret_cast<BlockFn>(start);
		s_lut[(pc & kProgMask) >> 3] = fn;
		// TEMP diagnostic (LRPS2_JIT_STATS): prove blocks are compiled+dispatched.
		{
			static int log_on = -1; static u64 n = 0;
			if (log_on < 0) log_on = getenv("LRPS2_JIT_STATS") ? 1 : 0;
			if (log_on)
			{
				n++;
				if (n <= 4 || (n & 0xff) == 0)
					fprintf(stderr, "[vu1rec] block #%llu compiled at VU1 pc=%04x\n",
						(unsigned long long)n, pc & kProgMask);
			}
		}
		return fn;
	}
}

recVU1_arm64 CpuRecVU1_arm64;

recVU1_arm64::recVU1_arm64()
{
	m_Idx = 1;
	IsInterpreter = false;
}

void recVU1_arm64::Reserve()
{
	if (!s_code)
	{
		s_code = (u8*)mmap(nullptr, kCodeCacheSize, PROT_READ | PROT_WRITE | PROT_EXEC,
		                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (s_code == MAP_FAILED)
			s_code = nullptr;
	}
	s_ok = s_code != nullptr;
	Console.WriteLn("arm64 VU1 rec (C.14-1): %s.",
		s_ok ? "block dispatch skeleton active (interpreter-executed blocks)" : "FAILED (interpreter fallback)");
}

void recVU1_arm64::Shutdown()
{
	if (s_code)
	{
		munmap(s_code, kCodeCacheSize);
		s_code = nullptr;
	}
	s_code_pos = 0;
	memset(s_lut, 0, sizeof(s_lut));
}

void recVU1_arm64::Reset()
{
	// Same VU-side state reset as the interpreter provider...
	CpuIntVU1.Reset();
	// ...plus a fresh block cache.
	memset(s_lut, 0, sizeof(s_lut));
	s_code_pos = 0;
}

void recVU1_arm64::SetStartPC(u32 startPC)
{
	vuRegs[1].start_pc = startPC;
}

void recVU1_arm64::Clear(u32 addr, u32 size)
{
	// Micro memory changed (VIF MPG upload / MTVU micro write). Skeleton
	// blocks read live micro memory so this is not yet load-bearing, but the
	// invalidation contract has to be exercised before native translation
	// depends on it.
	const u32 first = (addr & kProgMask) >> 3;
	const u32 last  = ((addr + (size ? size : 1) - 1) & kProgMask) >> 3;
	if (first <= last)
		memset(&s_lut[first], 0, (last - first + 1) * sizeof(BlockFn));
	else
	{
		memset(&s_lut[first], 0, (kLutEntries - first) * sizeof(BlockFn));
		memset(&s_lut[0], 0, (last + 1) * sizeof(BlockFn));
	}
}

void recVU1_arm64::Execute(u32 cycles)
{
	if (!s_ok)
	{
		CpuIntVU1.Execute(cycles);
		return;
	}

	// Outer loop mirrors InterpVU1::Execute exactly (VU1microInterp.cpp);
	// only the per-instruction vu1Exec is replaced by block dispatch.
	const FPControlRegisterBackup fpcr_backup(EmuConfig.Cpu.VU1FPCR);

	vuRegs[1].VI[REG_TPC].UL <<= 3;
	u64 startcycles = vuRegs[1].cycle;
	s_run_start = startcycles;
	s_run_budget = cycles;

	while ((vuRegs[1].cycle - startcycles) < cycles)
	{
		if (!(vuRegs[0].VI[REG_VPU_STAT].UL & 0x100))
		{
			if (vuRegs[1].branch == 1)
			{
				vuRegs[1].VI[REG_TPC].UL = vuRegs[1].branchpc;
				vuRegs[1].branch = 0;
			}
			break;
		}
		vuRegs[1].VI[REG_TPC].UL &= kProgMask;

		BlockFn fn = s_lut[vuRegs[1].VI[REG_TPC].UL >> 3];
		if (!fn)
			fn = CompileBlock(vuRegs[1].VI[REG_TPC].UL);
		fn();
	}
	vuRegs[1].VI[REG_TPC].UL >>= 3;
	vuRegs[1].nextBlockCycles = (vuRegs[1].cycle - cpuRegs.cycle) + 1;
}

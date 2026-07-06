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
#include "Config.h"
#include "common/Console.h"
#include "common/FPControl.h"

#include <cstring>
#include <sys/mman.h>

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

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

namespace
{
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

		// Skeleton block: frame + call the interpreter runner. Later phases
		// emit native translations of the upper/lower pairs here instead.
		masm.Stp(x29, x30, MemOperand(sp, -16, PreIndex));
		masm.Mov(x16, reinterpret_cast<uint64_t>(&vu1RunBlock_arm64));
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

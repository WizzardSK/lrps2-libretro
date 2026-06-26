// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 EE (R5900) recompiler -- Phase C.3-1: code cache + dispatch (opt-in).
//
// The JIT execution infrastructure for the EE, mirroring the IOP recompiler:
// an mmap'd RWX code cache, a per-PC block cache with lazy compilation, granular
// per-page (mirror-normalised) invalidation, and a per-block dispatch entry
// (eeJitRunBlock_arm64) that the interpreter's GAME_RUNNING loop calls when the
// EE recompiler provider (recCpu) is selected. Each compiled block currently
// calls eeRunBasicBlock_arm64 (the interpreter's per-basic-block loop), so
// behaviour is identical while the JIT plumbing -- emit, cache, execute,
// invalidate -- runs natively. Phase C.3-2 translates EE opcodes natively
// (64-bit GPR, then FPU/MMI/VU). The provider stays opt-in (VMManager keeps the
// interpreter as the default EE on arm64) until the JIT is proven.

#include "common/Pcsx2Types.h"
#include "R5900.h"
#include "common/Console.h"

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <sys/mman.h>

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

extern "C" void eeRunBasicBlock_arm64(void); // Interpreter.cpp

namespace
{
	typedef void (*BlockFn)(void);

	constexpr size_t kCodeCacheSize = 32 * 1024 * 1024;
	constexpr u32    kPageShift     = 12;

	u8*    s_code     = nullptr;
	size_t s_code_pos = 0;
	bool   s_ok       = false;

	std::unordered_map<u32, BlockFn>          s_blocks;
	std::unordered_map<u32, std::vector<u32>> s_page;

	inline u32 Norm(u32 a) { return a & 0x1fffffff; }

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

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 256 > kCodeCacheSize)
		{
			s_blocks.clear();
			s_page.clear();
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		masm.Sub(sp, sp, 16);
		masm.Str(x30, MemOperand(sp, 0));
		masm.Mov(x16, reinterpret_cast<uint64_t>(&eeRunBasicBlock_arm64));
		masm.Blr(x16);
		masm.Ldr(x30, MemOperand(sp, 0));
		masm.Add(sp, sp, 16);
		masm.Ret();
		masm.FinalizeCode();

		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15);

		BlockFn fn = reinterpret_cast<BlockFn>(start);
		s_blocks.emplace(pc, fn);
		s_page[Norm(pc) >> kPageShift].push_back(pc);
		return fn;
	}

	inline BlockFn BlockForPC(u32 pc)
	{
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
	s_ok = s_ok && (s_code != nullptr);
	Console.WriteLn("arm64 EE rec (C.3-1): %s.", s_ok ? "code cache + dispatch ready" : "FAILED");
}

void eeJitReset_arm64(void)
{
	s_blocks.clear();
	s_page.clear();
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
			s_blocks.erase(spc);
		s_page.erase(it);
	}
}

void eeJitShutdown_arm64(void)
{
	s_blocks.clear();
	s_page.clear();
	if (s_code) { munmap(s_code, kCodeCacheSize); s_code = nullptr; }
	s_code_pos = 0;
}

// Called by the interpreter's GAME_RUNNING loop (one EE basic block per call).
extern "C" void eeJitRunBlock_arm64(void)
{
	BlockForPC(cpuRegs.pc)();
}

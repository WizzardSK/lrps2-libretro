// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 IOP (R3000A) recompiler -- Phase C.2b-1: code cache + dispatch.
//
// Builds the real JIT execution infrastructure on VIXL (the AArch64 emitter
// vendored in 3rdparty/vixl): an executable code cache, a per-PC block cache,
// lazy compilation, and a dispatch loop that mirrors the interpreter's
// intExecuteBlock. Each compiled block currently just calls back into the
// interpreter for its basic block (iopRunBasicBlock_arm64), so behaviour is
// identical to the interpreter while the JIT plumbing -- emit, cache, execute,
// invalidate -- runs natively and is proven end-to-end.
//
// Phase C.2b-2 replaces the per-block interpreter call with natively translated
// IOP instructions (interpreter fallback for unsupported opcodes); the cache,
// dispatch and invalidation built here stay.

#include "R3000A.h"
#include "common/Console.h"

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <sys/mman.h>

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

// Interpreter entry that runs one IOP basic block (R3000AInterpreter.cpp).
extern "C" void iopRunBasicBlock_arm64(void);

namespace
{
	typedef void (*BlockFn)(void);

	constexpr size_t kCodeCacheSize = 16 * 1024 * 1024;
	u8*    s_code      = nullptr;   // RWX code cache
	size_t s_code_pos  = 0;
	std::unordered_map<u32, BlockFn> s_blocks;

	bool s_self_test_ok = false;

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

	// Emit a block at the current cache cursor. The block sets up a frame, calls
	// the interpreter for one IOP basic block, then returns. Per-PC so Phase
	// C.2b-2 can specialise the body to the actual instructions at `pc`.
	BlockFn CompileBlock(u32 /*pc*/)
	{
		if (s_code_pos + 256 > kCodeCacheSize)
		{
			// Cache full: flush everything and start over (blocks are pure, the
			// interpreter holds all real state, so this is always safe).
			s_blocks.clear();
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		masm.Sub(sp, sp, 16);
		masm.Str(x30, MemOperand(sp, 0));
		masm.Mov(x16, reinterpret_cast<uint64_t>(&iopRunBasicBlock_arm64));
		masm.Blr(x16);
		masm.Ldr(x30, MemOperand(sp, 0));
		masm.Add(sp, sp, 16);
		masm.Ret();
		masm.FinalizeCode();

		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15); // keep 16-byte alignment

		return reinterpret_cast<BlockFn>(start);
	}

	inline BlockFn BlockForPC(u32 pc)
	{
		auto it = s_blocks.find(pc);
		if (it != s_blocks.end())
			return it->second;
		BlockFn fn = CompileBlock(pc);
		s_blocks.emplace(pc, fn);
		return fn;
	}
}

static void recReserve(void)
{
	s_self_test_ok = VixlEmitSelfTest();
	if (!s_self_test_ok)
		Console.Error("arm64 IOP rec: VIXL emit self-test FAILED; recompiler disabled.");

	if (!s_code)
	{
		s_code = (u8*)mmap(nullptr, kCodeCacheSize, PROT_READ | PROT_WRITE | PROT_EXEC,
		                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (s_code == MAP_FAILED)
		{
			s_code = nullptr;
			Console.Error("arm64 IOP rec: failed to map code cache.");
		}
	}
	Console.WriteLn("arm64 IOP rec (C.2b-1): code cache %s, self-test %s.",
		s_code ? "mapped" : "FAILED", s_self_test_ok ? "OK" : "FAILED");
}

static void recResetIOP(void)
{
	s_blocks.clear();
	s_code_pos = 0;
}

static void recClearIOP(u32 /*Addr*/, u32 /*Size*/)
{
	// Conservative invalidation: drop the whole block cache. Blocks hold no
	// state of their own (the interpreter does), so this is always safe.
	s_blocks.clear();
	s_code_pos = 0;
}

// Dispatch loop -- mirrors intExecuteBlock, but runs each IOP basic block via a
// cached VIXL-emitted block instead of the interpreter's inner execI loop.
static s32 recExecuteBlock(s32 eeCycles)
{
	if (!s_code || !s_self_test_ok)
		return psxInt.ExecuteBlock(eeCycles); // safety fallback

	psxRegs.iopBreak    = 0;
	psxRegs.iopCycleEE  = eeCycles;

	while (psxRegs.iopCycleEE > 0)
	{
		BlockFn fn = BlockForPC(psxRegs.pc);
		fn();
	}

	return psxRegs.iopBreak + psxRegs.iopCycleEE;
}

static void recShutdown(void)
{
	s_blocks.clear();
	if (s_code)
	{
		munmap(s_code, kCodeCacheSize);
		s_code = nullptr;
	}
	s_code_pos = 0;
}

R3000Acpu psxRec = {
	recReserve,
	recResetIOP,
	recExecuteBlock,
	recClearIOP,
	recShutdown,
};

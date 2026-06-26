// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 IOP (R3000A) recompiler -- Phase C.2a: skeleton + emit pipeline.
//
// This is the seed of the AArch64 IOP recompiler. The x86 recompiler
// (x86/iR3000A.cpp) is excluded on aarch64 and cannot be retargeted (it emits
// directly through the x86 emitter), so the arm64 JIT is built fresh on top of
// VIXL (3rdparty/vixl), the AArch64 assembler upstream PCSX2 also uses.
//
// What C.2a establishes:
//   * the recompiler module compiles and links into the core,
//   * VIXL is actually exercised end-to-end inside the real build -- recReserve
//     emits a tiny function with VIXL, runs it, and asserts the result, proving
//     the emit -> finalize -> cache-flush -> execute pipeline works on the host,
//   * the R3000Acpu provider (psxRec) exists and is selectable.
//
// What C.2a deliberately does NOT do yet: translate IOP opcodes or run a
// VM-integrated dispatcher. Until real block recompilation lands (Phase C.2b),
// ExecuteBlock delegates to the interpreter so that, if psxRec is ever selected,
// behaviour stays correct. The interpreter remains the default provider
// (VMManager forces it on aarch64), so this module changes nothing at runtime
// until it is explicitly opted into and validated against a BIOS.

#include "R3000A.h"
#include "common/Console.h"

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

namespace
{
	// Proof-of-pipeline: emit `int64_t f(int64_t x) { return x + 1; }` with VIXL,
	// make it executable, run it and check the result. This validates the whole
	// emit/finalize/icache-flush/execute chain through the in-tree VIXL build the
	// real recompiler (Phase C.2b) will rely on.
	bool VixlEmitSelfTest()
	{
		MacroAssembler masm;
		masm.Add(x0, x0, 1); // x0 = arg0 + 1
		masm.Ret();
		masm.FinalizeCode();

		vixl::CodeBuffer* buf = masm.GetBuffer();
		buf->SetExecutable();
		auto fn = buf->GetStartAddress<int64_t (*)(int64_t)>();
		const int64_t result = fn(41);
		buf->SetWritable(); // hand the page back before it is freed
		return result == 42;
	}

	bool s_vixl_ok = false;
}

static void recReserve(void)
{
	// Bring up / validate the VIXL emit pipeline. A failure here means the JIT
	// backend is unusable on this host; we keep running (the interpreter is the
	// real execution path for now) but flag it loudly.
	s_vixl_ok = VixlEmitSelfTest();
	if (s_vixl_ok)
		Console.WriteLn("arm64 IOP rec (C.2a): VIXL emit pipeline OK.");
	else
		Console.Error("arm64 IOP rec (C.2a): VIXL emit self-test FAILED.");
}

static void recResetIOP(void)
{
	// No recompiled-block cache to flush yet (Phase C.2b). Nothing to do.
}

static void recClearIOP(u32 Addr, u32 Size)
{
	// No recompiled blocks to invalidate yet (Phase C.2b).
}

static s32 recExecuteBlock(s32 eeCycles)
{
	// Phase C.2a: real arm64 block recompilation is not implemented yet, so run
	// the interpreter. This keeps psxRec behaviourally correct if selected; the
	// VIXL dispatcher + opcode translation replace this in Phase C.2b.
	return psxInt.ExecuteBlock(eeCycles);
}

static void recShutdown(void)
{
}

R3000Acpu psxRec = {
	recReserve,
	recResetIOP,
	recExecuteBlock,
	recClearIOP,
	recShutdown,
};

// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 IOP (R3000A) recompiler -- Phase C.2b-2: native opcode translation.
//
// Builds on the C.2b-1 code cache + dispatch. Each block now natively
// translates the leading run of simple, side-effect-free ALU instructions
// (shifts, ADDU/SUBU/logic/SLT, and the reg-immediate forms) straight to
// AArch64 via VIXL, operating directly on psxRegs.GPR. At the first
// branch/jump/load/store/coprocessor/unsupported instruction it hands the rest
// of the basic block to the interpreter (iopRunBasicBlock_arm64), which keeps
// MIPS branch-delay-slot and memory/exception semantics exactly correct.
//
// Self-modifying code: psxCpu->Clear() is called on every IOP write, so blocks
// whose baked instruction range is written are invalidated (granular, per 4 KiB
// page, address-mirror-normalised).

#include "R3000A.h"
#include "common/Console.h"
#include "IopMem.h"
#include "IopHw.h"

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>
#include <sys/mman.h>

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

extern "C" void iopRunBasicBlock_arm64(void);

namespace
{
	typedef void (*BlockFn)(void);

	constexpr size_t kCodeCacheSize = 16 * 1024 * 1024;
	constexpr int    kMaxInsns      = 64;
	constexpr u32    kPageShift     = 12;

	u8*    s_code     = nullptr;
	size_t s_code_pos = 0;
	bool   s_ok       = false;

	std::unordered_map<u32, BlockFn>            s_blocks; // startpc -> fn
	std::unordered_map<u32, std::vector<u32>>   s_page;   // norm page -> startpcs touching it

	inline u32 Norm(u32 a) { return a & 0x1fffffff; } // collapse KSEG0/1 mirrors

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

	// Load IOP GPR[idx] into wd (r0 reads as 0). gpr = base of psxRegs.GPR.r.
	inline void LoadGpr(MacroAssembler& m, const Register& wd, const Register& gpr, u32 idx)
	{
		if (idx == 0)
			m.Mov(wd, 0);
		else
			m.Ldr(wd, MemOperand(gpr, idx * 4));
	}
	inline void StoreGpr(MacroAssembler& m, const Register& ws, const Register& gpr, u32 idx)
	{
		m.Str(ws, MemOperand(gpr, idx * 4)); // caller guarantees idx != 0
	}

	// Emit native code for one instruction if it is a simple ALU op. Returns
	// false (emitting nothing) for anything that must go through the interpreter
	// (branches, jumps, loads, stores, mult/div, coprocessor, traps, ...).
	bool EmitSimple(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		const u32 op = insn >> 26;
		const u32 rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, rd = (insn >> 11) & 31, sa = (insn >> 6) & 31;
		const u32 funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);
		const u32 zimm = insn & 0xffff;

		switch (op)
		{
			case 0x00: // SPECIAL
				switch (funct)
				{
					case 0x00: if (rd) { LoadGpr(m, w0, gpr, rt); m.Lsl(w0, w0, sa); StoreGpr(m, w0, gpr, rd); } return true; // SLL / NOP
					case 0x02: if (rd) { LoadGpr(m, w0, gpr, rt); m.Lsr(w0, w0, sa); StoreGpr(m, w0, gpr, rd); } return true; // SRL
					case 0x03: if (rd) { LoadGpr(m, w0, gpr, rt); m.Asr(w0, w0, sa); StoreGpr(m, w0, gpr, rd); } return true; // SRA
					case 0x04: if (rd) { LoadGpr(m, w0, gpr, rt); LoadGpr(m, w1, gpr, rs); m.Lsl(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // SLLV
					case 0x06: if (rd) { LoadGpr(m, w0, gpr, rt); LoadGpr(m, w1, gpr, rs); m.Lsr(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // SRLV
					case 0x07: if (rd) { LoadGpr(m, w0, gpr, rt); LoadGpr(m, w1, gpr, rs); m.Asr(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // SRAV
					case 0x21: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Add(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // ADDU
					case 0x23: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Sub(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // SUBU
					case 0x24: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.And(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // AND
					case 0x25: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Orr(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // OR
					case 0x26: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Eor(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true; // XOR
					case 0x27: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Orr(w0, w0, w1); m.Mvn(w0, w0); StoreGpr(m, w0, gpr, rd); } return true; // NOR
					case 0x2a: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Cmp(w0, w1); m.Cset(w0, lt); StoreGpr(m, w0, gpr, rd); } return true; // SLT
					case 0x2b: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Cmp(w0, w1); m.Cset(w0, lo); StoreGpr(m, w0, gpr, rd); } return true; // SLTU
					default: return false;
				}
			case 0x09: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, (u32)simm); m.Add(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true; // ADDIU
			case 0x0a: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, (u32)simm); m.Cmp(w0, w1); m.Cset(w0, lt); StoreGpr(m, w0, gpr, rt); } return true; // SLTI
			case 0x0b: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, (u32)simm); m.Cmp(w0, w1); m.Cset(w0, lo); StoreGpr(m, w0, gpr, rt); } return true; // SLTIU
			case 0x0c: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, zimm); m.And(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true; // ANDI
			case 0x0d: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, zimm); m.Orr(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true; // ORI
			case 0x0e: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, zimm); m.Eor(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true; // XORI
			case 0x0f: if (rt) { m.Mov(w0, zimm << 16); StoreGpr(m, w0, gpr, rt); } return true; // LUI
			default: return false;
		}
	}

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 4096 > kCodeCacheSize)
		{
			s_blocks.clear();
			s_page.clear();
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		const Register gpr = x9;
		masm.Sub(sp, sp, 16);
		masm.Str(x30, MemOperand(sp, 0));
		masm.Mov(gpr, reinterpret_cast<uint64_t>(&psxRegs.GPR.r[0]));

		u32 p = pc;
		int n = 0;
		while (n < kMaxInsns)
		{
			const u32 insn = iopMemRead32(p);
			if (!EmitSimple(masm, gpr, insn))
				break;
			p += 4;
			n++;
		}

		if (n > 0)
		{
			// pc += 4n ; cycle += n ; iopCycleEE -= (ICFG&8 ? 9 : 8) * n
			masm.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.pc));
			masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, 4 * n); masm.Str(w0, MemOperand(x10));
			masm.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.cycle));
			masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, n); masm.Str(w0, MemOperand(x10));
			masm.Mov(x10, reinterpret_cast<uint64_t>(&psxHu32(HW_ICFG)));
			masm.Ldr(w1, MemOperand(x10));
			masm.Mov(w2, 9 * n); masm.Mov(w3, 8 * n);
			masm.Tst(w1, 8);
			masm.Csel(w2, w2, w3, ne);
			masm.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.iopCycleEE));
			masm.Ldr(w0, MemOperand(x10)); masm.Sub(w0, w0, w2); masm.Str(w0, MemOperand(x10));
		}

		// finish the basic block (branch + delay slot, or any non-simple op) via the interpreter
		masm.Mov(x16, reinterpret_cast<uint64_t>(&iopRunBasicBlock_arm64));
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
		// index by every page the baked range [pc, p) touches (mirror-normalised)
		const u32 ns = Norm(pc), ne = Norm(p > pc ? p : pc + 4);
		for (u32 pg = ns >> kPageShift; pg <= (ne - 1) >> kPageShift; pg++)
			s_page[pg].push_back(pc);
		return fn;
	}

	inline BlockFn BlockForPC(u32 pc)
	{
		auto it = s_blocks.find(pc);
		return it != s_blocks.end() ? it->second : CompileBlock(pc);
	}
}

static void recReserve(void)
{
	s_ok = VixlEmitSelfTest();
	if (!s_code)
	{
		s_code = (u8*)mmap(nullptr, kCodeCacheSize, PROT_READ | PROT_WRITE | PROT_EXEC,
		                   MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
		if (s_code == MAP_FAILED) { s_code = nullptr; }
	}
	s_ok = s_ok && (s_code != nullptr);
	Console.WriteLn("arm64 IOP rec (C.2b-2): %s.", s_ok ? "native ALU JIT active" : "FAILED -> interpreter fallback");
}

static void recResetIOP(void)
{
	s_blocks.clear();
	s_page.clear();
	s_code_pos = 0;
}

static void recClearIOP(u32 addr, u32 size)
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

static s32 recExecuteBlock(s32 eeCycles)
{
	if (!s_ok)
		return psxInt.ExecuteBlock(eeCycles);

	psxRegs.iopBreak   = 0;
	psxRegs.iopCycleEE = eeCycles;
	while (psxRegs.iopCycleEE > 0)
		BlockForPC(psxRegs.pc)();
	return psxRegs.iopBreak + psxRegs.iopCycleEE;
}

static void recShutdown(void)
{
	s_blocks.clear();
	s_page.clear();
	if (s_code) { munmap(s_code, kCodeCacheSize); s_code = nullptr; }
	s_code_pos = 0;
}

R3000Acpu psxRec = {
	recReserve,
	recResetIOP,
	recExecuteBlock,
	recClearIOP,
	recShutdown,
};

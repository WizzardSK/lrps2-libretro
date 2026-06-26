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
// (cpuBlockCycles += opcode.cycles * (2 - CP0.Config[18]); all translated ALU
// ops use the Default 9-cycle cost). Self-modifying code -> granular per-page
// (mirror-normalised) invalidation, as Cpu->Clear is called on EE writes.

#include "common/Pcsx2Types.h"
#include "R5900.h"
#include "R5900OpcodeTables.h"
#include "Memory.h"
#include "common/Console.h"

#include <cstdint>
#include <unordered_map>
#include <vector>
#include <sys/mman.h>

#include "aarch64/macro-assembler-aarch64.h"

using namespace vixl::aarch64;

extern "C" void eeRunBasicBlock_arm64(void); // Interpreter.cpp
extern u32 cpuBlockCycles;                    // Interpreter.cpp

namespace
{
	typedef void (*BlockFn)(void);

	constexpr size_t kCodeCacheSize = 32 * 1024 * 1024;
	constexpr int    kMaxInsns      = 64;
	constexpr u32    kPageShift     = 12;
	constexpr int    kEECycles      = 9; // Default cost; every ALU op we translate uses it

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

	// Translate one simple integer ALU op (32- and 64-bit forms). Returns false,
	// emitting nothing, for control flow / memory / FPU / MMI / anything else.
	bool EmitSimple(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		const u32 op = insn >> 26;
		const u32 rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, rd = (insn >> 11) & 31, sa = (insn >> 6) & 31;
		const u32 funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);
		const uint64_t simm64 = (uint64_t)(int64_t)simm;
		const u32 zimm = insn & 0xffff;

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
			default: return false;
		}
	}

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 8192 > kCodeCacheSize)
		{
			s_blocks.clear();
			s_page.clear();
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		const Register gpr = x19;
		masm.Sub(sp, sp, 16);
		masm.Stp(x19, x30, MemOperand(sp, 0));
		masm.Mov(gpr, reinterpret_cast<uint64_t>(&cpuRegs.GPR.r[0]));

		u32 p = pc;
		int n = 0;
		while (n < kMaxInsns)
		{
			const u32 insn = memRead32(p);
			if (!EmitSimple(masm, gpr, insn))
				break;
			p += 4;
			n++;
		}

		if (n > 0)
		{
			// cpuRegs.pc += 4n
			masm.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.pc));
			masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, 4 * n); masm.Str(w0, MemOperand(x10));
			// cpuBlockCycles += 9n * (2 - CP0.Config[18])  ->  9n if bit set else 18n
			masm.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.CP0.n.Config));
			masm.Ldr(w1, MemOperand(x10));
			masm.Mov(w2, kEECycles * n); masm.Mov(w3, kEECycles * n * 2);
			masm.Tst(w1, 1u << 18);
			masm.Csel(w2, w2, w3, ne);
			masm.Mov(x10, reinterpret_cast<uint64_t>(&cpuBlockCycles));
			masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, w2); masm.Str(w0, MemOperand(x10));
		}

		// finish the basic block (branch + everything not translated) via interpreter
		masm.Mov(x16, reinterpret_cast<uint64_t>(&eeRunBasicBlock_arm64));
		masm.Blr(x16);
		masm.Ldp(x19, x30, MemOperand(sp, 0));
		masm.Add(sp, sp, 16);
		masm.Ret();
		masm.FinalizeCode();

		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15);

		BlockFn fn = reinterpret_cast<BlockFn>(start);
		s_blocks.emplace(pc, fn);
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
	Console.WriteLn("arm64 EE rec (C.3-2): %s.", s_ok ? "native integer-ALU JIT active" : "FAILED");
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

extern "C" void eeJitRunBlock_arm64(void)
{
	BlockForPC(cpuRegs.pc)();
}

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
	constexpr int    kEECycles      = 9; // Default cost; every ALU op we translate uses it

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

			// Loads (LB/LBU/LH/LHU/LW/LWU/LD). Address = rs.UL[0] + simm (32-bit).
			// The interpreter cancels (fastjmp) on a misaligned LH/LW/LD, so do the
			// same; reads run even when rt==0 (side effects). Memory goes through
			// the EE vtlb wrappers. gpr (x19) is callee-saved across the calls.
			case 0x20: case 0x24: case 0x21: case 0x25: case 0x23: case 0x27: case 0x37:
			{
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
				case 0x3c: case 0x3e: case 0x3f: return true;
				default: return false;
			}
		}
		switch (op)
		{
			case 0x09: case 0x19: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
			case 0x20: case 0x21: case 0x23: case 0x24: case 0x25: case 0x27: case 0x37:
			case 0x28: case 0x29: case 0x2b: case 0x3f: return true;
			default: return false;
		}
	}

	void EmitCycleBookkeeping(MacroAssembler& m, int count)
	{
		m.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.CP0.n.Config));
		m.Ldr(w1, MemOperand(x10));
		m.Mov(w2, kEECycles * count); m.Mov(w3, kEECycles * count * 2);
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
	bool EmitBranch(MacroAssembler& m, const Register& gpr, u32 bpc, u32 insn, int n_leading)
	{
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

		const uint64_t evt = reinterpret_cast<uint64_t>(&eeEventTest_arm64);
		const uint64_t upd = reinterpret_cast<uint64_t>(&eeUpdateCycles_arm64);
		const bool nt_evt = (op == 0x04 || op == 0x05); // only BEQ/BNE event-test on not-taken

		if (link > 0) { m.Mov(w0, bpc + 8); m.Sxtw(x0, w0); StoreGpr(m, x0, gpr, (u32)link); }
		if (is_jr) LoadGpr(m, x20, gpr, rs);

		if (uncond)
		{
			EmitSimple(m, gpr, ds);
			EmitCycleBookkeeping(m, n_leading + 2);
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
		EmitCycleBookkeeping(m, n_leading + 2);
		StorePCImm(m, tconst);
		m.Mov(x16, upd); m.Blr(x16);
		m.Mov(x16, evt); m.Blr(x16);
		EmitChainEpilogue(m);
		m.Bind(&not_taken);
		EmitCycleBookkeeping(m, n_leading + 1);
		StorePCImm(m, bpc + 4);
		if (nt_evt) { m.Mov(x16, evt); m.Blr(x16); }
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

		u32 p = pc;
		int n = 0;
		bool done = false;
		while (n < kMaxInsns)
		{
			const u32 insn = memRead32(p);
			if (EmitSimple(masm, gpr, insn)) { p += 4; n++; continue; }
			if (EmitBranch(masm, gpr, p, insn, n)) { done = true; break; } // emits bookkeeping + pc + eventtest + chain epilogue
			break; // unsupported -> interpreter finishes the basic block
		}

		if (!done)
		{
			if (n > 0)
			{
				masm.Mov(x10, reinterpret_cast<uint64_t>(&cpuRegs.pc));
				masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, 4 * n); masm.Str(w0, MemOperand(x10));
				EmitCycleBookkeeping(masm, n);
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
	Console.WriteLn("arm64 EE rec (C.3-7): %s.", s_ok ? "native ALU+mem+branch JIT active (block-linking)" : "FAILED");
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
	BlockForPC(cpuRegs.pc)();
}

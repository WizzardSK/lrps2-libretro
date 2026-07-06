// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// arm64 IOP (R3000A) recompiler -- Phase C.2b-4: branches + delay slots.
//
// Builds on the C.2b-1..3 code cache, dispatch, ALU and load/store translation.
// Blocks now also translate MIPS branches/jumps natively, each ending the block:
// the (always-executed) delay slot is emitted inline, the condition is evaluated
// from the pre-delay-slot register values, and psxRegs.pc is set to the taken
// target or the fall-through (bpc+8). iopEventTest() is called only on the taken
// path, matching the interpreter's doBranch(). Link (JAL/JALR/B*AL) writes pc+8
// before the delay slot. J (0x02) is left to the interpreter because it carries
// IOP module-import HLE; unaligned LWL/LWR/SWL/SWR and any opcode not covered by
// EmitSimple/EmitBranch also hand the rest of the basic block to the interpreter.
//
// Self-modifying code is handled by granular per-4KiB-page, mirror-normalised
// invalidation in recClearIOP (psxCpu->Clear is called on every IOP write).

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

	// end = Norm(one past the last byte covered by NATIVE code). The interpreter
	// tail of a partial block reads live memory, so only [Norm(pc), end) needs
	// SMC invalidation; end == Norm(pc) for blocks with no native lead.
	struct BlockRec { BlockFn fn; u32 end; };
	std::unordered_map<u32, BlockRec>         s_blocks;
	std::unordered_map<u32, std::vector<u32>> s_page;

	inline u32 Norm(u32 a) { return a & 0x1fffffff; }

	// Direct-mapped block cache for the 2 MB IOP RAM (hot region) + asm block-linking
	// (block-to-block tail-chaining), mirroring the EE recompiler. ROM/scratchpad
	// fall back to the hash.
	constexpr u32 kRamBytes = 0x00200000;
	constexpr u32 kRamWords = kRamBytes >> 2;
	BlockFn*      s_lut      = nullptr;
	inline bool InRam(u32 np) { return np < kRamBytes; }
	inline void LutClearAll() { if (s_lut) madvise(s_lut, (size_t)kRamWords * sizeof(BlockFn), MADV_DONTNEED); }

	// Word-granular "native code covers this RAM word" bitmap (64KB). Every IOP
	// RAM store lands in recClearIOP, so the common case (word with no compiled
	// code on it) must be a single bit test, not a hash lookup. Bits can go
	// stale after a block is erased; the slow path self-cleans them.
	u8 s_covered[kRamWords >> 3];
	inline void CoverRange(u32 ns, u32 ne)
	{
		for (u32 w = ns >> 2; w < ((ne + 3) >> 2); w++)
			s_covered[w >> 3] |= 1u << (w & 7);
	}

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

	inline void LoadGpr(MacroAssembler& m, const Register& wd, const Register& gpr, u32 idx)
	{
		if (idx == 0) m.Mov(wd, 0);
		else          m.Ldr(wd, MemOperand(gpr, idx * 4));
	}
	inline void StoreGpr(MacroAssembler& m, const Register& ws, const Register& gpr, u32 idx)
	{
		m.Str(ws, MemOperand(gpr, idx * 4));
	}

	// Translate one side-effect-light instruction (ALU + aligned load/store) to
	// native AArch64. Returns false (emitting nothing) for control flow and
	// anything not covered. gpr (x19) is callee-saved so it survives mem helpers.
	bool EmitSimple(MacroAssembler& m, const Register& gpr, u32 insn)
	{
		const u32 op = insn >> 26;
		const u32 rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, rd = (insn >> 11) & 31, sa = (insn >> 6) & 31;
		const u32 funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);
		const u32 zimm = insn & 0xffff;

		switch (op)
		{
			case 0x00:
				switch (funct)
				{
					case 0x00: if (rd) { LoadGpr(m, w0, gpr, rt); m.Lsl(w0, w0, sa); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x02: if (rd) { LoadGpr(m, w0, gpr, rt); m.Lsr(w0, w0, sa); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x03: if (rd) { LoadGpr(m, w0, gpr, rt); m.Asr(w0, w0, sa); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x04: if (rd) { LoadGpr(m, w0, gpr, rt); LoadGpr(m, w1, gpr, rs); m.Lsl(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x06: if (rd) { LoadGpr(m, w0, gpr, rt); LoadGpr(m, w1, gpr, rs); m.Lsr(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x07: if (rd) { LoadGpr(m, w0, gpr, rt); LoadGpr(m, w1, gpr, rs); m.Asr(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x21: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Add(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x23: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Sub(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x24: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.And(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x25: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Orr(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x26: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Eor(w0, w0, w1); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x27: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Orr(w0, w0, w1); m.Mvn(w0, w0); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x2a: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Cmp(w0, w1); m.Cset(w0, lt); StoreGpr(m, w0, gpr, rd); } return true;
					case 0x2b: if (rd) { LoadGpr(m, w0, gpr, rs); LoadGpr(m, w1, gpr, rt); m.Cmp(w0, w1); m.Cset(w0, lo); StoreGpr(m, w0, gpr, rd); } return true;
					default: return false;
				}
			case 0x09: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, (u32)simm); m.Add(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true;
			case 0x0a: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, (u32)simm); m.Cmp(w0, w1); m.Cset(w0, lt); StoreGpr(m, w0, gpr, rt); } return true;
			case 0x0b: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, (u32)simm); m.Cmp(w0, w1); m.Cset(w0, lo); StoreGpr(m, w0, gpr, rt); } return true;
			case 0x0c: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, zimm); m.And(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true;
			case 0x0d: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, zimm); m.Orr(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true;
			case 0x0e: if (rt) { LoadGpr(m, w0, gpr, rs); m.Mov(w1, zimm); m.Eor(w0, w0, w1); StoreGpr(m, w0, gpr, rt); } return true;
			case 0x0f: if (rt) { m.Mov(w0, zimm << 16); StoreGpr(m, w0, gpr, rt); } return true;

			case 0x20: case 0x24: case 0x21: case 0x25: case 0x23:
			{
				LoadGpr(m, w0, gpr, rs); m.Mov(w2, (u32)simm); m.Add(w0, w0, w2);
				uint64_t fn = (op == 0x23) ? reinterpret_cast<uint64_t>(&iopMemRead32)
				            : (op == 0x21 || op == 0x25) ? reinterpret_cast<uint64_t>(&iopMemRead16)
				            : reinterpret_cast<uint64_t>(&iopMemRead8);
				m.Mov(x16, fn); m.Blr(x16);
				if (rt)
				{
					switch (op)
					{
						case 0x20: m.Sxtb(w0, w0); break;
						case 0x24: m.Uxtb(w0, w0); break;
						case 0x21: m.Sxth(w0, w0); break;
						case 0x25: m.Uxth(w0, w0); break;
						default: break;
					}
					StoreGpr(m, w0, gpr, rt);
				}
				return true;
			}
			case 0x28: case 0x29: case 0x2b:
			{
				LoadGpr(m, w0, gpr, rs); m.Mov(w2, (u32)simm); m.Add(w0, w0, w2);
				LoadGpr(m, w1, gpr, rt);
				uint64_t fn = (op == 0x2b) ? reinterpret_cast<uint64_t>(&iopMemWrite32)
				            : (op == 0x29) ? reinterpret_cast<uint64_t>(&iopMemWrite16)
				            : reinterpret_cast<uint64_t>(&iopMemWrite8);
				m.Mov(x16, fn); m.Blr(x16);
				return true;
			}
			default: return false;
		}
	}

	// True iff EmitSimple would translate this instruction (no emission). Used to
	// decide whether a branch's delay slot can be inlined.
	bool IsTranslatable(u32 insn)
	{
		const u32 op = insn >> 26, funct = insn & 0x3f;
		if (op == 0x00)
		{
			switch (funct)
			{
				case 0x00: case 0x02: case 0x03: case 0x04: case 0x06: case 0x07:
				case 0x21: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
				case 0x2a: case 0x2b: return true;
				default: return false;
			}
		}
		switch (op)
		{
			case 0x09: case 0x0a: case 0x0b: case 0x0c: case 0x0d: case 0x0e: case 0x0f:
			case 0x20: case 0x21: case 0x23: case 0x24: case 0x25:
			case 0x28: case 0x29: case 0x2b: return true;
			default: return false;
		}
	}

	// cycle += count ; iopCycleEE -= (ICFG&8 ? 9 : 8) * count
	void EmitCycleBookkeeping(MacroAssembler& m, int count)
	{
		m.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.cycle));
		m.Ldr(w0, MemOperand(x10)); m.Add(w0, w0, count); m.Str(w0, MemOperand(x10));
		m.Mov(x10, reinterpret_cast<uint64_t>(&psxHu32(HW_ICFG)));
		m.Ldr(w1, MemOperand(x10));
		m.Mov(w2, 9 * count); m.Mov(w3, 8 * count);
		m.Tst(w1, 8);
		m.Csel(w2, w2, w3, ne);
		m.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.iopCycleEE));
		m.Ldr(w0, MemOperand(x10)); m.Sub(w0, w0, w2); m.Str(w0, MemOperand(x10));
	}

	// Restore the frame, then tail-chain straight to the next block via the RAM LUT
	// (staying in JIT code). IOP has a cycle budget, so only chain while
	// iopCycleEE > 0; otherwise (budget spent / LUT miss / non-RAM pc) return to
	// recExecuteBlock. x30 propagates so the final ret unwinds to the dispatcher.
	void EmitEpilogue(MacroAssembler& m)
	{
		m.Ldp(x19, x20, MemOperand(sp, 0));
		m.Ldp(x21, x30, MemOperand(sp, 16));
		m.Add(sp, sp, 32);
		Label ret_path;
		m.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.iopCycleEE));
		m.Ldr(w0, MemOperand(x10));
		m.Cmp(w0, 0);
		m.B(&ret_path, le);                  // budget exhausted -> return to dispatcher
		m.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.pc));
		m.Ldr(w1, MemOperand(x10));
		m.And(w2, w1, 0x1fffffff);
		m.Mov(w3, kRamBytes);
		m.Cmp(w2, w3);
		m.B(&ret_path, hs);                  // not RAM -> return
		m.Mov(x0, reinterpret_cast<uint64_t>(&s_lut));
		m.Ldr(x0, MemOperand(x0));
		m.Lsr(w2, w2, 2);
		m.Ldr(x3, MemOperand(x0, w2, UXTW, 3));
		m.Cbz(x3, &ret_path);
		m.Br(x3);
		m.Bind(&ret_path);
		m.Ret();
	}

	inline void StorePC(MacroAssembler& m, const Register& wsrc)
	{
		m.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.pc));
		m.Str(wsrc, MemOperand(x10));
	}

	// Translate a branch/jump (ending the block). Returns false, emitting nothing,
	// if it can't be handled (then the caller hands the rest to the interpreter).
	bool EmitBranch(MacroAssembler& m, const Register& gpr, u32 bpc, u32 insn, int n_leading)
	{
		const u32 op = insn >> 26, rs = (insn >> 21) & 31, rt = (insn >> 16) & 31, funct = insn & 0x3f;
		const int32_t simm = (int16_t)(insn & 0xffff);

		bool uncond = false, is_jr = false, two = false, one = false;
		int  link = -1;
		Condition cond = al;
		u32  tconst = 0;

		if      (op == 0x00 && funct == 0x08) { uncond = true; is_jr = true; }
		else if (op == 0x00 && funct == 0x09) { uncond = true; is_jr = true; link = (int)((insn >> 11) & 31); }
		else if (op == 0x03) { uncond = true; link = 31; tconst = (((bpc + 4) & 0xf0000000) | ((insn & 0x3ffffff) << 2)); } // JAL
		else if (op == 0x02) return false; // J: IOP module-import HLE -> interpreter
		else if (op == 0x04) { two = true; cond = eq; tconst = bpc + 4 + simm * 4; } // BEQ
		else if (op == 0x05) { two = true; cond = ne; tconst = bpc + 4 + simm * 4; } // BNE
		else if (op == 0x06) { one = true; cond = le; tconst = bpc + 4 + simm * 4; } // BLEZ
		else if (op == 0x07) { one = true; cond = gt; tconst = bpc + 4 + simm * 4; } // BGTZ
		else if (op == 0x01)
		{
			const u32 t = bpc + 4 + simm * 4;
			if      (rt == 0x00) { one = true; cond = lt; tconst = t; }            // BLTZ
			else if (rt == 0x01) { one = true; cond = ge; tconst = t; }            // BGEZ
			else if (rt == 0x10) { one = true; cond = lt; tconst = t; link = 31; } // BLTZAL
			else if (rt == 0x11) { one = true; cond = ge; tconst = t; link = 31; } // BGEZAL
			else return false;
		}
		else return false;

		const u32 ds = iopMemRead32(bpc + 4);
		if (!IsTranslatable(ds))
			return false;

		// link (before the delay slot, unconditionally)
		if (link > 0) { m.Mov(w0, bpc + 8); StoreGpr(m, w0, gpr, (u32)link); }
		// snapshot condition operands / jr target before the delay slot can clobber them
		if (two)   { LoadGpr(m, w20, gpr, rs); LoadGpr(m, w21, gpr, rt); }
		else if (one) { LoadGpr(m, w20, gpr, rs); }
		if (is_jr) { LoadGpr(m, w21, gpr, rs); }
		// the always-executed delay slot
		EmitSimple(m, gpr, ds);
		// time: n_leading native ops + the branch + the delay slot
		EmitCycleBookkeeping(m, n_leading + 2);

		if (uncond)
		{
			if (is_jr) StorePC(m, w21);
			else { m.Mov(w0, tconst); StorePC(m, w0); }
			m.Mov(x16, reinterpret_cast<uint64_t>(&iopEventTest)); m.Blr(x16);
		}
		else
		{
			if (two) m.Cmp(w20, w21);
			else     m.Cmp(w20, 0);
			m.Mov(w0, tconst); m.Mov(w1, bpc + 8);
			m.Csel(w0, w0, w1, cond);
			StorePC(m, w0);
			Label not_taken;
			m.B(&not_taken, InvertCondition(cond));
			m.Mov(x16, reinterpret_cast<uint64_t>(&iopEventTest)); m.Blr(x16);
			m.Bind(&not_taken);
		}
		EmitEpilogue(m);
		return true;
	}

	BlockFn CompileBlock(u32 pc)
	{
		if (s_code_pos + 8192 > kCodeCacheSize)
		{
			s_blocks.clear();
			s_page.clear();
			LutClearAll();
			memset(s_covered, 0, sizeof(s_covered));
			s_code_pos = 0;
		}

		u8* start = s_code + s_code_pos;
		MacroAssembler masm(start, kCodeCacheSize - s_code_pos, PositionDependentCode);

		const Register gpr = x19;
		masm.Sub(sp, sp, 32);
		masm.Stp(x19, x20, MemOperand(sp, 0));
		masm.Stp(x21, x30, MemOperand(sp, 16));
		masm.Mov(gpr, reinterpret_cast<uint64_t>(&psxRegs.GPR.r[0]));

		u32 p = pc;
		int n = 0;
		bool done = false;
		while (n < kMaxInsns)
		{
			const u32 insn = iopMemRead32(p);
			if (EmitSimple(masm, gpr, insn)) { p += 4; n++; continue; }
			if (EmitBranch(masm, gpr, p, insn, n)) { done = true; break; } // emits bookkeeping + pc + epilogue
			break; // unsupported -> interpreter handles the rest of the block
		}

		if (!done)
		{
			if (n > 0)
			{
				masm.Mov(x10, reinterpret_cast<uint64_t>(&psxRegs.pc));
				masm.Ldr(w0, MemOperand(x10)); masm.Add(w0, w0, 4 * n); masm.Str(w0, MemOperand(x10));
				EmitCycleBookkeeping(masm, n);
			}
			masm.Mov(x16, reinterpret_cast<uint64_t>(&iopRunBasicBlock_arm64));
			masm.Blr(x16);
			EmitEpilogue(masm);
		}

		masm.FinalizeCode();
		const size_t sz = masm.GetSizeOfCodeGenerated();
		__builtin___clear_cache(reinterpret_cast<char*>(start), reinterpret_cast<char*>(start + sz));
		s_code_pos += (sz + 15) & ~size_t(15);

		BlockFn fn = reinterpret_cast<BlockFn>(start);
		// Native code covers the leading run; a translated branch also bakes the
		// branch + delay slot (p still points at the branch instruction there).
		const u32 ns = Norm(pc), ne = Norm(done ? p + 8 : p);
		s_blocks.emplace(pc, BlockRec{fn, ne});
		if (InRam(ns)) s_lut[ns >> 2] = fn;
		if (ne > ns)
		{
			for (u32 pg = ns >> kPageShift; pg <= (ne - 1) >> kPageShift; pg++)
				s_page[pg].push_back(pc);
			if (InRam(ne - 1))
				CoverRange(ns, ne);
		}
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
		return it != s_blocks.end() ? it->second.fn : CompileBlock(pc);
	}
}

static void recReserve(void)
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
	Console.WriteLn("arm64 IOP rec (C.2b-4): %s.", s_ok ? "native ALU+mem+branch JIT active" : "FAILED -> interpreter fallback");
}

static void recResetIOP(void)
{
	s_blocks.clear();
	s_page.clear();
	LutClearAll();
	memset(s_covered, 0, sizeof(s_covered));
	s_code_pos = 0;
}

// Erase only the blocks whose NATIVE range [Norm(pc), end) intersects the
// (inclusive) byte range [a0, a1]. Page lists keep the survivors; entries for
// blocks already erased via a neighbouring page are dropped lazily.
static void EraseBlocksInRange(u32 a0, u32 a1)
{
	for (u32 pg = a0 >> kPageShift; pg <= a1 >> kPageShift; pg++)
	{
		auto it = s_page.find(pg);
		if (it == s_page.end())
			continue;
		auto& vec = it->second;
		size_t out = 0;
		for (size_t i = 0; i < vec.size(); i++)
		{
			const u32 spc = vec[i];
			auto bit = s_blocks.find(spc);
			if (bit == s_blocks.end())
				continue; // stale (erased through another page it spanned)
			const u32 ns = Norm(spc), ne = bit->second.end;
			if (ns <= a1 && ne > a0)
			{
				s_blocks.erase(bit);
				if (InRam(ns)) s_lut[ns >> 2] = nullptr;
				continue;
			}
			vec[out++] = spc;
		}
		if (out == 0)
			s_page.erase(it);
		else
			vec.resize(out);
	}
}

static void recClearIOP(u32 addr, u32 size)
{
	const u32 a0 = Norm(addr);
	const u32 a1 = Norm(addr + (size ? size : 1) * 4 - 1);
	if (a0 <= a1 && a1 < kRamBytes)
	{
		// RAM fast path (every IOP store lands here, typically 1 word with no
		// code on it). Ranges are whole words: callers pass addr&~3 + word
		// counts, and block bounds are 4-aligned, so word-level tests match
		// the byte-level intersection exactly.
		bool covered = false;
		for (u32 w = a0 >> 2; w <= (a1 >> 2) && !covered; w++)
			covered = (s_covered[w >> 3] >> (w & 7)) & 1;
		if (!covered)
			return;
		EraseBlocksInRange(a0, a1);
		// Nothing overlaps [a0, a1] anymore -- clear the bits so the next
		// store to these words takes the fast path again (self-cleans bits
		// left stale by blocks erased through other words).
		for (u32 w = a0 >> 2; w <= (a1 >> 2); w++)
			s_covered[w >> 3] &= ~(1u << (w & 7));
		return;
	}
	if (s_blocks.empty())
		return;
	EraseBlocksInRange(a0, a1);
}

static s32 recExecuteBlock(s32 eeCycles)
{
	// TEMP diagnostic toggle: LRPS2_NO_IOPREC=1 forces the IOP interpreter.
	static int no_ioprec = -1;
	if (no_ioprec < 0) no_ioprec = getenv("LRPS2_NO_IOPREC") ? 1 : 0;
	if (!s_ok || no_ioprec)
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
	memset(s_covered, 0, sizeof(s_covered));
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

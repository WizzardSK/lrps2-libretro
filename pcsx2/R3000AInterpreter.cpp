/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2021  PCSX2 Dev Team
 *
 *  PCSX2 is free software: you can redistribute it and/or modify it under the terms
 *  of the GNU Lesser General Public License as published by the Free Software Found-
 *  ation, either version 3 of the License, or (at your option) any later version.
 *
 *  PCSX2 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 *  without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 *  PURPOSE.  See the GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with PCSX2.
 *  If not, see <http://www.gnu.org/licenses/>.
 */


#include "R3000A.h"
#include "Common.h"
#include "Config.h"

#include "R5900OpcodeTables.h"
#include "IopBios.h"
#include "IopHw.h"


static bool branch2 = 0;
static u32 branchPC;

static __fi void execI(void)
{
	// Inject IRX hack
	if (psxRegs.pc == 0x1630 && EmuConfig.CurrentIRX.length() > 3)
	{
		// FIXME do I need to increase the module count (0x1F -> 0x20)
		if (iopMemRead32(0x20018) == 0x1F)
			iopMemWrite32(0x20094, 0xbffc0000);
	}

	psxRegs.code = iopMemRead32(psxRegs.pc);

	psxRegs.pc+= 4;
	psxRegs.cycle++;

	//One of the Iop to EE delta clocks to be set in PS1 mode.
	if ((psxHu32(HW_ICFG) & (1 << 3)))
		psxRegs.iopCycleEE -= 9;
	else //default ps2 mode value
		psxRegs.iopCycleEE -= 8;
	psxBSC[psxRegs.code >> 26]();
}

static void doBranch(s32 tar)
{
	branch2 = iopIsDelaySlot = true;
	branchPC = tar;
	execI();
	iopIsDelaySlot = false;
	psxRegs.pc = branchPC;

	iopEventTest();
}


/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, offset                                 *
*********************************************************/

void psxBGEZ()         // Branch if Rs >= 0
{
	if (_i32(_rRs_) >= 0) doBranch(_BranchTarget_);
}

void psxBGEZAL()   // Branch if Rs >= 0 and link
{
	_SetLink(31);
	if (_i32(_rRs_) >= 0)
	{
		doBranch(_BranchTarget_);
	}
}

void psxBGTZ()          // Branch if Rs >  0
{
	if (_i32(_rRs_) > 0) doBranch(_BranchTarget_);
}

void psxBLEZ()         // Branch if Rs <= 0
{
	if (_i32(_rRs_) <= 0) doBranch(_BranchTarget_);
}
void psxBLTZ()          // Branch if Rs <  0
{
	if (_i32(_rRs_) < 0) doBranch(_BranchTarget_);
}

void psxBLTZAL()    // Branch if Rs <  0 and link
{
	_SetLink(31);
	if (_i32(_rRs_) < 0)
		{
			doBranch(_BranchTarget_);
		}
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, rt, offset                             *
*********************************************************/

void psxBEQ()   // Branch if Rs == Rt
{
	if (_i32(_rRs_) == _i32(_rRt_)) doBranch(_BranchTarget_);
}

void psxBNE()   // Branch if Rs != Rt
{
	if (_i32(_rRs_) != _i32(_rRt_)) doBranch(_BranchTarget_);
}

/*********************************************************
* Jump to target                                         *
* Format:  OP target                                     *
*********************************************************/
void psxJ(void)
{
	// check for iop module import table magic
	u32 delayslot = iopMemRead32(psxRegs.pc);
	if (delayslot >> 16 == 0x2400 && R3000A::irxImportExec(R3000A::irxImportTableAddr(psxRegs.pc), delayslot & 0xffff))
		return;

	doBranch(_JumpTarget_);
}

void psxJAL(void)
{
	_SetLink(31);
	doBranch(_JumpTarget_);
}

/*********************************************************
* Register jump                                          *
* Format:  OP rs, rd                                     *
*********************************************************/
void psxJR(void)
{
	doBranch(_u32(_rRs_));
}

void psxJALR(void)
{
	if (_Rd_)
	{
		_SetLink(_Rd_);
	}
	doBranch(_u32(_rRs_));
}

static void intReserve(void) { }
static void intAlloc(void) { }
static void intReset(void) { intAlloc(); }
static void intClear(u32 Addr, u32 Size) { }
static void intShutdown(void) { }

static s32 intExecuteBlock( s32 eeCycles )
{
	psxRegs.iopBreak = 0;
	psxRegs.iopCycleEE = eeCycles;

	while (psxRegs.iopCycleEE > 0)
	{
		if ((psxHu32(HW_ICFG) & 8) && ((psxRegs.pc & 0x1fffffffU) == 0xa0 || (psxRegs.pc & 0x1fffffffU) == 0xb0 || (psxRegs.pc & 0x1fffffffU) == 0xc0))
			psxBiosCall();

		branch2 = 0;
		while (!branch2)
			execI();
	}

	return psxRegs.iopBreak + psxRegs.iopCycleEE;
}

#ifdef ARCH_ARM64
// TEMP diagnostic (LRPS2_IOP_HANDOFF_STATS=1): dynamic histogram of the opcodes
// the IOP JIT hands off to the interpreter, keyed (op, funct/rs/rt sub-field).
// Mirrors the EE's LRPS2_HANDOFF_STATS (Interpreter.cpp): h = every interpreted
// op, hf = only the FIRST op of each handoff (the actual block breaker). Dumps
// at exit. Delay-slot ops run inside doBranch and are not counted (same caveat
// as the EE tool).
#include <unordered_map>
#include <vector>
#include <algorithm>
static const bool s_iopHandoffStats = getenv("LRPS2_IOP_HANDOFF_STATS") != nullptr;
namespace
{
	static void iopHandoffDump(const char* tag, std::unordered_map<u32, u64>& m, u64 total)
	{
		std::vector<std::pair<u32, u64>> v(m.begin(), m.end());
		const size_t k = v.size() < 24 ? v.size() : 24;
		std::partial_sort(v.begin(), v.begin() + k, v.end(), [](auto& x, auto& y) { return x.second > y.second; });
		fprintf(stderr, "=== IOP handoff %s (%llu ops, uniq=%zu) ===\n", tag,
			(unsigned long long)total, v.size());
		for (size_t i = 0; i < k; i++)
			fprintf(stderr, "  op=%02x sub=%02x  %llu\n",
				v[i].first >> 8, v[i].first & 0xff, (unsigned long long)v[i].second);
	}

	struct IopHandoffTables
	{
		std::unordered_map<u32, u64> h, hf;
		u64 n = 0, nf = 0;
		~IopHandoffTables()
		{
			if (!s_iopHandoffStats)
				return;
			iopHandoffDump("histogram", h, n);
			iopHandoffDump("BREAKERS (first op)", hf, nf);
		}
	};
	IopHandoffTables s_iopHandoff;

	static void iopHandoffCount(u32 insn, bool first)
	{
		const u32 op = insn >> 26;
		u32 key = op << 8;
		if      (op == 0x00) key |= insn & 0x3f;         // SPECIAL: funct
		else if (op == 0x01) key |= (insn >> 16) & 0x1f; // REGIMM: rt
		else if (op == 0x10 || op == 0x12) key |= (insn >> 21) & 0x1f; // COP0/COP2: rs
		s_iopHandoff.h[key]++; s_iopHandoff.n++;
		if (first) { s_iopHandoff.hf[key]++; s_iopHandoff.nf++;
			// TEMP: pinpoint unexplained breakers (LRPS2_IOP_HANDOFF_PC=<key>)
			static const char* want = getenv("LRPS2_IOP_HANDOFF_PC");
			static u32 wkey = want ? (u32)strtoul(want, nullptr, 0) : 0xffffffff;
			static int budget = 20;
			if (key == wkey && budget > 0) { budget--; fprintf(stderr, "[iop-hf] pc=%08x insn=%08x\n", psxRegs.pc, insn); }
		}
	}
}

// arm64 recompiler (Phase C.2b) helper: run a single IOP basic block through the
// interpreter -- instructions until the next taken branch (and its delay slot,
// which execI()/doBranch() handle). The arm64 JIT currently emits one per-PC
// block per basic block, each of which calls this; Phase C.2b-2 replaces the
// body with natively translated instructions. Mirrors intExecuteBlock's inner
// loop (incl. the BIOS HLE entry hook) so behaviour is identical.
extern "C" void iopRunBasicBlock_arm64(void)
{
	if ((psxHu32(HW_ICFG) & 8) && ((psxRegs.pc & 0x1fffffffU) == 0xa0 || (psxRegs.pc & 0x1fffffffU) == 0xb0 || (psxRegs.pc & 0x1fffffffU) == 0xc0))
		psxBiosCall();

	branch2 = 0;
	if (s_iopHandoffStats)
	{
		bool first = true;
		while (!branch2)
		{
			iopHandoffCount(iopMemRead32(psxRegs.pc), first);
			first = false;
			execI();
		}
		return;
	}
	while (!branch2)
		execI();
}
#endif

R3000Acpu psxInt = {
	intReserve,
	intReset,
	intExecuteBlock,
	intClear,
	intShutdown
};

/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2010  PCSX2 Dev Team
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

#include "Common.h"
#include "R5900OpcodeTables.h"
#include "Elfheader.h"

#include "../common/FastJmp.h"

#include <float.h>
#include <unordered_map>
#include <vector>
#include <algorithm>

static int branch2 = 0;
#ifdef ARCH_ARM64
u32 cpuBlockCycles = 0;			// 3 bit fixed point version of cycle count (shared with the arm64 EE JIT)
#else
static u32 cpuBlockCycles = 0;		// 3 bit fixed point version of cycle count
#endif
static bool intExitExecution = false;
static fastjmp_buf intJmpBuf;
static u32 intLastBranchTo;

#ifdef ARCH_ARM64
// arm64 EE recompiler (Phase C.3) hook: when set, the GAME_RUNNING loop runs
// JIT-dispatched blocks instead of one interpreter instruction per iteration.
// nullptr => plain interpreter. The JIT cache machinery lives in
// arm64/recR5900_arm64.cpp; recCpu is assembled at the bottom of this file so
// it can reuse the static execute/exit/cancel helpers and intJmpBuf.
void (*g_ee_block_runner)(void) = nullptr;
extern "C" void eeJitRunBlock_arm64(void);
extern void eeJitReserve_arm64(void);
extern void eeJitShutdown_arm64(void);
extern void eeJitReset_arm64(void);
extern void eeJitClear_arm64(u32 addr, u32 size);
#endif

static void intEventTest(void);

void intUpdateCPUCycles()
{
	const bool lowcycles = (cpuBlockCycles <= 40);
	const s8 cyclerate = EmuConfig.Speedhacks.EECycleRate;
	u32 scale_cycles = 0;

	if (cyclerate == 0 || lowcycles || cyclerate < -99 || cyclerate > 3)
		scale_cycles = cpuBlockCycles >> 3;

	else if (cyclerate > 1)
		scale_cycles = cpuBlockCycles >> (2 + cyclerate);

	else if (cyclerate == 1)
		scale_cycles = (cpuBlockCycles >> 3) / 1.3f; // Adds a mild 30% increase in clockspeed for value 1.

	else if (cyclerate == -1) // the mildest value.
		// These values were manually tuned to yield mild speedup with high compatibility
		scale_cycles = (cpuBlockCycles <= 80 || cpuBlockCycles > 168 ? 5 : 7) * cpuBlockCycles / 32;

	else
		scale_cycles = ((5 + (-2 * (cyclerate + 1))) * cpuBlockCycles) >> 5;

	// Ensure block cycle count is never less than 1.
	cpuRegs.cycle += (scale_cycles < 1) ? 1 : scale_cycles;

	if (cyclerate > 1)
		cpuBlockCycles &= (0x1 << (cyclerate + 2)) - 1;
	else
		cpuBlockCycles &= 0x7;
}

// These macros are used to assemble the repassembler functions

#ifdef ARCH_ARM64
// TEMP diagnostic (LRPS2_TRACE=<path>, LRPS2_TRACE_LO/HI=<hex phys pc>): binary
// (pc, GPR-state-hash) trace for A/B-diffing JIT vs interpreter execution of a
// pc range. Called per-instruction from execI and per-block-entry from the
// arm64 EE JIT (recR5900_arm64.cpp CompileBlock).
volatile u64 g_diag_frame = 0; // set per-frame from retro_run (TEMP diagnostic)
u32 GetEECycle(void) { return cpuRegs.cycle; } // TEMP diagnostic (LRPS2_RAMCRC)

// Diagnostic hooks cost two calls per interpreted op even when disabled (they
// showed up in GT3 attract profiles); execI gates them on this one bool,
// resolved once at load (plain load, no magic-static guard in the hot path).
static const bool s_eeDiagHooks = getenv("LRPS2_TRACE") || getenv("LRPS2_EXCLOG");

// TEMP diagnostic (LRPS2_EXCLOG=<path>): log COP0 Cause/EPC/Status/Count +
// cpuRegs.cycle + frame at every entry to the EE interrupt vector 0x80000200,
// so a full-JIT run and a NO_EE_MEM run can be diffed to see whether interrupts
// are taken at different cycles / with different pending-cause bits (the suspected
// sub-frame interrupt-timing residual behind the MMX7 post-logo stall). Called at
// block entry from the arm64 EE JIT and per-instruction from execI.
extern "C" void eeLogExc(u32 pc)
{
	static FILE* f = nullptr; static int on = -1;
	static u32 last_pc = 0; static u32 last_epc = 0; static u64 last_n = 0;
	if (on < 0) { const char* p = getenv("LRPS2_EXCLOG"); f = p ? fopen(p, "w") : nullptr; on = f ? 1 : 0; }
	if (!on) return;
	const u32 pp = pc & 0x1fffffff;
	if (pp != 0x200 && pp != 0x180) return;
	// Dedup: with NO_EE_MEM the vector block may be entered via the JIT block
	// hook AND execI at the same pc/EPC -- log each vector entry once.
	static u64 n = 0; n++;
	if (pp == last_pc && cpuRegs.CP0.n.EPC == last_epc && n == last_n + 1) { last_n = n; return; }
	last_pc = pp; last_epc = cpuRegs.CP0.n.EPC; last_n = n;
	// Cause IP field (bits 10-15) = pending HW interrupt lines (IP2/bit10 is the
	// INTC/DMAC aggregate on the EE); EXCCODE = Cause bits 2-6.
	const u32 cause = cpuRegs.CP0.n.Cause;
	fprintf(f, "frame=%llu vec=%03x cyc=%u Cause=%08x(exc=%u,IP=%02x) EPC=%08x",
		(unsigned long long)g_diag_frame, pp, cpuRegs.cycle, cause,
		(cause >> 2) & 0x1f, (cause >> 10) & 0x3f, cpuRegs.CP0.n.EPC);
	if (pp == 0x180) // syscall/common vector: log the call# ($v1) and args
		fprintf(f, " v1=%02llx a=%08llx,%08llx,%08llx,%08llx",
			(unsigned long long)cpuRegs.GPR.n.v1.UL[0], (unsigned long long)cpuRegs.GPR.n.a0.UL[0],
			(unsigned long long)cpuRegs.GPR.n.a1.UL[0], (unsigned long long)cpuRegs.GPR.n.a2.UL[0],
			(unsigned long long)cpuRegs.GPR.n.a3.UL[0]);
	fprintf(f, "\n");
}
extern "C" void eeTraceHook(u32 pc)
{
	static FILE* f = nullptr; static int on = -1; static u32 lo = 0, hi = 0; static u64 n = 0;
	static u64 tframe = ~0ull;
	if (on < 0)
	{
		const char* p = getenv("LRPS2_TRACE");
		const char* l = getenv("LRPS2_TRACE_LO");
		const char* h = getenv("LRPS2_TRACE_HI");
		const char* fr = getenv("LRPS2_TRACE_FRAME");
		if (p && l && h) { f = fopen(p, "wb"); lo = (u32)strtoul(l, 0, 16); hi = (u32)strtoul(h, 0, 16); }
		if (fr) tframe = (u64)strtoull(fr, 0, 10);
		on = f ? 1 : 0;
	}
	if (!on || n >= 200000000ull) return;
	if (tframe != ~0ull && g_diag_frame != tframe) return;
	const u32 pp = pc & 0x1fffffff;
	if (pp < lo || pp >= hi) return;
	u64 hsh = 0;
	for (int i = 1; i < 32; i++) { hsh ^= cpuRegs.GPR.r[i].UD[0] + 0x9e3779b97f4a7c15ull * (u64)i; hsh = (hsh << 7) | (hsh >> 57); }
	hsh ^= cpuRegs.HI.UD[0]; hsh = (hsh << 7) | (hsh >> 57);
	hsh ^= cpuRegs.LO.UD[0];
	u64 rec[2] = { pc, hsh };
	fwrite(rec, 16, 1, f);
	// Targeted register dump around the frame-217 divergence at 0x001051d0
	// (`daddu v1,v1,a1`): dump v0,v1,a0,a1,a2 before each op of the byte-
	// assembly tail of 0x105138.
	if (pp >= 0x001051b0 && pp <= 0x001051e0)
	{
		static u64 dn = 0;
		if (dn < 60) { dn++; fprintf(stderr, "[div] pc=%08x v0=%016llx v1=%016llx a0=%016llx a1=%016llx a2=%016llx\n",
			pp, (unsigned long long)cpuRegs.GPR.r[2].UD[0], (unsigned long long)cpuRegs.GPR.r[3].UD[0],
			(unsigned long long)cpuRegs.GPR.r[4].UD[0], (unsigned long long)cpuRegs.GPR.r[5].UD[0],
			(unsigned long long)cpuRegs.GPR.r[6].UD[0]); }
	}
	// Compact per-op register line (LRPS2_TRACE_SHOW): dumps a few regs for the
	// first ~400 in-range ops so a JIT vs interp diff pinpoints the first op that
	// produces a divergent (sign-extended) value.
	if (getenv("LRPS2_TRACE_SHOW"))
	{
		static u64 sn = 0;
		if (sn < 400) { sn++; fprintf(stderr, "[op] pc=%08x v0=%016llx v1=%016llx a1=%016llx a2=%016llx t2=%016llx\n",
			pp, (unsigned long long)cpuRegs.GPR.r[2].UD[0], (unsigned long long)cpuRegs.GPR.r[3].UD[0],
			(unsigned long long)cpuRegs.GPR.r[5].UD[0], (unsigned long long)cpuRegs.GPR.r[6].UD[0],
			(unsigned long long)cpuRegs.GPR.r[10].UD[0]); }
	}
	if ((++n & 0xffff) == 0) fflush(f);
}
#endif

static void execI(void)
{
	// execI is called for every instruction so it must remains as light as possible.
	// If you enable the next define, Interpreter will be much slower (around
	// ~4fps on 3.9GHz Haswell vs ~8fps (even 10fps on dev build))
	// Extra note: due to some cycle count issue PCSX2's internal debugger is
	// not yet usable with the interpreter
	u32 pc = cpuRegs.pc;
#ifdef ARCH_ARM64
	if (s_eeDiagHooks)
	{
		eeTraceHook(pc);
		eeLogExc(pc);
	}
#endif
	// We need to increase the pc before executing the memRead32. An exception could appears
	// and it expects the PC counter to be pre-incremented
	cpuRegs.pc += 4;

	// interprete instruction
	cpuRegs.code = memRead32( pc );

	const R5900::OPCODE& opcode = R5900::GetCurrentInstruction();
	cpuBlockCycles += opcode.cycles * (2 - ((cpuRegs.CP0.n.Config >> 18) & 0x1));

	opcode.interpret();
}

static __fi void _doBranch_shared(u32 tar)
{
	branch2 = cpuRegs.branch = 1;
	execI();

	// branch being 0 means an exception was thrown, since only the exception
	// handler should ever clear it.

	if( cpuRegs.branch != 0 )
	{
		if (Cpu == &intCpu)
		{
			if (intLastBranchTo == tar && EmuConfig.Speedhacks.WaitLoop)
			{
				intUpdateCPUCycles();
				bool can_skip = true;
				if (tar != 0x81fc0)
				{
					if ((cpuRegs.pc - tar) < (4 * 10))
					{
						for (u32 i = tar; i < cpuRegs.pc; i += 4)
						{
							if (PSM(i) != 0)
							{
								can_skip = false;
								break;
							}
						}
					}
					else
						can_skip = false;
				}

				if (can_skip)
				{
					if (static_cast<s64>(cpuRegs.nextEventCycle - cpuRegs.cycle) > 0)
						cpuRegs.cycle = cpuRegs.nextEventCycle;
					else
						cpuRegs.nextEventCycle = cpuRegs.cycle;
				}
			}
		}
		intLastBranchTo = tar;
		cpuRegs.pc = tar;
		cpuRegs.branch = 0;
	}
}

static void doBranch( u32 target )
{
	_doBranch_shared( target );
	intUpdateCPUCycles();
	intEventTest();
}

void intDoBranch(u32 target)
{
	_doBranch_shared( target );

	if( Cpu == &intCpu )
	{
		intUpdateCPUCycles();
		intEventTest();
	}
}

void intSetBranch(void)
{
	branch2 = 1;
}

////////////////////////////////////////////////////////////////////
// R5900 Branching Instructions!
// These are the interpreter versions of the branch instructions.  Unlike other
// types of interpreter instructions which can be called safely from the recompilers,
// these instructions are not "recSafe" because they may not invoke the
// necessary branch test logic that the recs need to maintain sync with the
// cpuRegs.pc and delaySlot instruction and such.

namespace R5900 {
namespace Interpreter {
namespace OpcodeImpl {

/*********************************************************
* Jump to target                                         *
* Format:  OP target                                     *
*********************************************************/
// fixme: looking at the other branching code, shouldn't those _SetLinks in BGEZAL and such only be set
// if the condition is true? --arcum42

void J()
{
	doBranch(_JumpTarget_);
}

void JAL()
{
	// 0x3563b8 is the start address of the function that invalidate entry in TLB cache
	// 0x35c118 is the address in the June 22 prototype
	// 0x35d628 is the address in the August 26 prototype
	if (EmuConfig.Gamefixes.GoemonTlbHack) {
		if (_JumpTarget_ == 0x3563b8 || _JumpTarget_ == 0x35d628 || _JumpTarget_ == 0x35c118)
			GoemonUnloadTlb(cpuRegs.GPR.n.a0.UL[0]);
	}
	_SetLink(31);
	doBranch(_JumpTarget_);
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, rt, offset                             *
*********************************************************/

void BEQ()  // Branch if Rs == Rt
{
	if (cpuRegs.GPR.r[_Rs_].SD[0] == cpuRegs.GPR.r[_Rt_].SD[0])
		doBranch(_BranchTarget_);
	else
		intEventTest();
}

void BNE()  // Branch if Rs != Rt
{
	if (cpuRegs.GPR.r[_Rs_].SD[0] != cpuRegs.GPR.r[_Rt_].SD[0])
		doBranch(_BranchTarget_);
	else
		intEventTest();
}

/*********************************************************
* Register branch logic                                  *
* Format:  OP rs, offset                                 *
*********************************************************/

void BGEZ()    // Branch if Rs >= 0
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] >= 0)
	{
		doBranch(_BranchTarget_);
	}
}

void BGEZAL() // Branch if Rs >= 0 and link
{
	_SetLink(31);
	if (cpuRegs.GPR.r[_Rs_].SD[0] >= 0)
	{
		doBranch(_BranchTarget_);
	}
}

void BGTZ()    // Branch if Rs >  0
{
	if (cpuRegs.GPR.r[_Rs_].SD[0] > 0)
	{
		doBranch(_BranchTarget_);
	}
}

void BLEZ()   // Branch if Rs <= 0
{
	if (cpuRegs.GPR.r[_Rs_].SD[0] <= 0)
	{
		doBranch(_BranchTarget_);
	}
}

void BLTZ()    // Branch if Rs <  0
{
	if (cpuRegs.GPR.r[_Rs_].SD[0] < 0)
	{
		doBranch(_BranchTarget_);
	}
}

void BLTZAL()  // Branch if Rs <  0 and link
{
	_SetLink(31);
	if (cpuRegs.GPR.r[_Rs_].SD[0] < 0)
	{
		doBranch(_BranchTarget_);
	}
}

/*********************************************************
* Register branch logic  Likely                          *
* Format:  OP rs, offset                                 *
*********************************************************/


void BEQL()    // Branch if Rs == Rt
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] == cpuRegs.GPR.r[_Rt_].SD[0])
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BNEL()     // Branch if Rs != Rt
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] != cpuRegs.GPR.r[_Rt_].SD[0])
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BLEZL()    // Branch if Rs <= 0
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] <= 0)
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BGTZL()     // Branch if Rs >  0
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] > 0)
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BLTZL()     // Branch if Rs <  0
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] < 0)
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BGEZL()     // Branch if Rs >= 0
{
	if(cpuRegs.GPR.r[_Rs_].SD[0] >= 0)
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BLTZALL()   // Branch if Rs <  0 and link
{
	_SetLink(31);
	if(cpuRegs.GPR.r[_Rs_].SD[0] < 0)
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

void BGEZALL()   // Branch if Rs >= 0 and link
{
	_SetLink(31);
	if(cpuRegs.GPR.r[_Rs_].SD[0] >= 0)
	{
		doBranch(_BranchTarget_);
	}
	else
	{
		cpuRegs.pc +=4;
		intEventTest();
	}
}

/*********************************************************
* Register jump                                          *
* Format:  OP rs, rd                                     *
*********************************************************/
void JR()
{
	// 0x33ad48 and 0x35060c are the return address of the function (0x356250) that populate the TLB cache
	// 0x340600 and 0x356334 are the addresses in the June 22 prototype
	// 0x341ad0 and 0x357844 are the addresses in the August 26 prototype
	if (EmuConfig.Gamefixes.GoemonTlbHack) {
		u32 add = cpuRegs.GPR.r[_Rs_].UL[0];
		if (add == 0x33ad48 || add == 0x35060c || add == 0x341ad0 || add == 0x357844 || add == 0x357844 || add == 0x356334)
			GoemonPreloadTlb();
	}
	doBranch(cpuRegs.GPR.r[_Rs_].UL[0]);
}

void JALR()
{
	u32 temp = cpuRegs.GPR.r[_Rs_].UL[0];

	if (_Rd_)  _SetLink(_Rd_);

	doBranch(temp);
}

} } }		// end namespace R5900::Interpreter::OpcodeImpl


// --------------------------------------------------------------------------------------
//  R5900cpu/intCpu interface (implementations)
// --------------------------------------------------------------------------------------

static void intReserve()
{
	// fixme : detect cpu for use the optimize asm code
}

static void intReset()
{
	cpuRegs.branch = 0;
	branch2 = 0;
}

static void intEventTest()
{
	// TEMP diagnostic (LRPS2_EVTLOG=<path>, LRPS2_EVT_LO/HI=<frame range>): log
	// every EE event test (cycle vs nextEventCycle, pc, pending INTC/DMAC bits)
	// so a full-JIT and a NO_EE_MEM run can be diffed to find the first test
	// where interrupt recognition diverges.
	{
		static FILE* f = nullptr; static int on = -1; static u64 flo = 0, fhi = 0;
		if (on < 0)
		{
			const char* p = getenv("LRPS2_EVTLOG");
			f = p ? fopen(p, "w") : nullptr; on = f ? 1 : 0;
			const char* l = getenv("LRPS2_EVT_LO"); const char* h = getenv("LRPS2_EVT_HI");
			flo = l ? strtoull(l, 0, 10) : 0; fhi = h ? strtoull(h, 0, 10) : ~0ull;
		}
		if (on && g_diag_frame >= flo && g_diag_frame <= fhi)
			fprintf(f, "f=%llu cyc=%u nev=%d pc=%08x int=%08x is=%04x im=%04x\n",
				(unsigned long long)g_diag_frame, cpuRegs.cycle,
				(int)(s32)(cpuRegs.nextEventCycle - cpuRegs.cycle), cpuRegs.pc,
				cpuRegs.interrupt, psHu16(INTC_STAT), psHu16(INTC_MASK));
	}
	// Perform counters, ints, and IOP updates:
	_cpuEventTest_Shared();

	if (intExitExecution)
	{
		intExitExecution = false;
		fastjmp_jmp(&intJmpBuf, 1);
	}
}

static void intSafeExitExecution()
{
	// If we're currently processing events, we can't safely jump out of the interpreter here, because we'll
	// leave things in an inconsistent state. So instead, we flag it for exiting once cpuEventTest() returns.
	if (eeEventTestIsActive)
		intExitExecution = true;
	else
		fastjmp_jmp(&intJmpBuf, 1);
}

static void intCancelInstruction(void)
{
	// See execute function.
	fastjmp_jmp(&intJmpBuf, 0);
}

static void eeExecuteLoop(void)
{
	enum ExecuteState {
		RESET,
		GAME_LOADING,
		GAME_RUNNING
	};
	// The RESET stage interprets until the pc reaches the BIOS's EELOAD, which is
	// how a cold boot walks itself to the game's entry point. Re-entering the loop
	// with the game ALREADY running -- which is what loading a save state does,
	// since it exits execution and comes back with the VM mid-game -- can never
	// satisfy that condition: the pc is somewhere in the game, and it never
	// returns to EELOAD. The loop then interprets forever and the recompiler is
	// never reached (the JIT lives in the GAME_RUNNING stage), so a state load
	// silently dropped the EE onto the interpreter for the rest of the session.
	// g_GameStarted means exactly "we are past the game's entry point" and is part
	// of the save state, so it is the right thing to resume on.
	ExecuteState state = g_GameStarted ? GAME_RUNNING : RESET;

	// This will come back as zero the first time it runs, or on instruction cancel.
	// It will come back as nonzero when we exit execution.
	if (fastjmp_set(&intJmpBuf) != 0)
		return;

	// I hope this doesn't cause issues with the optimizer... infinite loop with a constant expression.
	for (;;)
	{
		// The execution was splited in three parts so it is easier to
		// resume it after a cancelled instruction.
		switch (state) {
		case RESET:
			{
				do
				{
					execI();
				} while (cpuRegs.pc != (g_eeloadMain ? g_eeloadMain : EELOAD_START));

				if (cpuRegs.pc == EELOAD_START)
				{
					// The EELOAD _start function is the same across all BIOS versions afaik
					u32 mainjump = memRead32(EELOAD_START + 0x9c);
					if (mainjump >> 26 == 3) // JAL
						g_eeloadMain = ((EELOAD_START + 0xa0) & 0xf0000000U) | (mainjump << 2 & 0x0fffffffU);
				}
				else if (cpuRegs.pc == g_eeloadMain)
				{
					eeloadHook();
					if (g_SkipBiosHack)
					{
						// See comments on this code in iR5900-32.cpp's recRecompile()
						u32 typeAexecjump = memRead32(EELOAD_START + 0x470);
						u32 typeBexecjump = memRead32(EELOAD_START + 0x5B0);
						u32 typeCexecjump = memRead32(EELOAD_START + 0x618);
						u32 typeDexecjump = memRead32(EELOAD_START + 0x600);
						if ((typeBexecjump >> 26 == 3) || (typeCexecjump >> 26 == 3) || (typeDexecjump >> 26 == 3)) // JAL to 0x822B8
							g_eeloadExec = EELOAD_START + 0x2B8;
						else if (typeAexecjump >> 26 == 3) // JAL to 0x82170
							g_eeloadExec = EELOAD_START + 0x170;
					}
				}
				else if (cpuRegs.pc == g_eeloadExec)
					eeloadHook2();

				if (!g_GameLoading)
					break;

				state = GAME_LOADING;
				// fallthrough
			}

		case GAME_LOADING:
			if (ElfEntry != 0xFFFFFFFF)
			{
				do
				{
					execI();
				} while (cpuRegs.pc != ElfEntry);
				eeGameStarting();
			}
			state = GAME_RUNNING;
			// fallthrough

		case GAME_RUNNING:
#ifdef ARCH_ARM64
			if (g_ee_block_runner)
				for (;;) g_ee_block_runner();
			else
#endif
				for (;;) execI();
			break;
		}
	}
}

static void intExecute(void)
{
#ifdef ARCH_ARM64
	g_ee_block_runner = nullptr; // interpreter: one instruction per iteration
#endif
	eeExecuteLoop();
}

#ifdef ARCH_ARM64
// arm64 EE recompiler entry points (provider recCpu, assembled below). They live
// here so they can use the static execute loop, branch2, execI and intJmpBuf.

// TEMP diagnostic (LRPS2_HANDOFF_STATS=1): dynamic histogram of the opcodes the
// JIT hands off to the interpreter, keyed by (op, funct, subop) so MMI/REGIMM/
// COPx sub-ops separate. Dumps the top of the table every 100M interpreted ops;
// this is what decides which ops are worth translating next.
static const bool s_handoffStats = getenv("LRPS2_HANDOFF_STATS") != nullptr;
static void eeHandoffCount(u32 insn, bool first)
{
	// Two tables: h = every interpreted op (shows total interp volume, mostly
	// already-translated ops stuck in handed-off block tails), hf = only the
	// FIRST op of each handoff = the untranslated op that actually broke the
	// block. hf is what decides which op to translate next.
	static std::unordered_map<u32, u64> h, hf; static u64 n = 0, nf = 0;
	const u32 op = insn >> 26;
	u32 key = op << 16;
	if      (op == 0x00) key |= (insn & 0x3f) << 8;                            // SPECIAL: funct
	else if (op == 0x01) key |= ((insn >> 16) & 0x1f) << 8;                    // REGIMM: rt
	else if (op == 0x1c) key |= ((insn & 0x3f) << 8) | ((insn >> 6) & 0x1f);   // MMI: funct+subop
	else if (op == 0x10 || op == 0x11 || op == 0x12)
		key |= ((insn >> 21) & 0x1f) << 8;                                     // COPx: rs field
	h[key]++;
	if (first) { hf[key]++; nf++; }
	if (++n % 100000000 != 0)
		return;
	const auto dump = [](const char* tag, std::unordered_map<u32, u64>& m, u64 total) {
		std::vector<std::pair<u32, u64>> v(m.begin(), m.end());
		const size_t k = v.size() < 24 ? v.size() : 24;
		std::partial_sort(v.begin(), v.begin() + k, v.end(), [](auto& x, auto& y) { return x.second > y.second; });
		fprintf(stderr, "=== EE handoff %s (%lluM ops, uniq=%zu) ===\n", tag,
			(unsigned long long)(total / 1000000), v.size());
		for (size_t i = 0; i < k; i++)
			fprintf(stderr, "  op=%02x funct=%02x sub=%02x  %llu\n",
				v[i].first >> 16, (v[i].first >> 8) & 0xff, v[i].first & 0xff,
				(unsigned long long)v[i].second);
	};
	dump("histogram", h, n);
	dump("BREAKERS (first op)", hf, nf);
}

// Run a single EE basic block through the interpreter (instructions until the
// next taken branch). The arm64 JIT's per-PC blocks call this for now (Phase
// C.3-1); later phases translate the instructions natively.
extern "C" void eeRunBasicBlock_arm64(void)
{
	branch2 = 0;
	if (s_handoffStats)
	{
		bool first = true;
		do { eeHandoffCount(memRead32(cpuRegs.pc), first); first = false; execI(); } while (!branch2);
		return;
	}
	do { execI(); } while (!branch2);
}

void eeRecExecute_arm64(void)
{
	// TEMP diagnostic toggle: LRPS2_NO_EEREC=1 runs the EE through the pure
	// interpreter even though the recompiler provider is selected, to bisect
	// whether a stall is a JIT correctness bug or shared C++ (IPU/DMA/timing).
	static int no_eerec = -1;
	if (no_eerec < 0) no_eerec = getenv("LRPS2_NO_EEREC") ? 1 : 0;
	g_ee_block_runner = no_eerec ? nullptr : &eeJitRunBlock_arm64;
	eeExecuteLoop();
}

// Non-template EE memory wrappers + misalign cancel, called from the arm64 EE
// JIT's natively-translated loads/stores (Phase C.3-3). Defined here so they see
// Memory.h / Cpu. Cpu->CancelInstruction() fastjmps to intJmpBuf, which is safe
// from JIT code (fastjmp restores sp and callee-saved regs).
// #define EE_RD_SAMPLE 1 // TEMP: EE read32 address histogram diagnostic (off)
extern "C" u32  eeRead8_arm64 (u32 a) { return memRead8(a); }
extern "C" u32  eeRead16_arm64(u32 a) { return memRead16(a); }
#ifdef EE_RD_SAMPLE
#include <unordered_map>
#include <vector>
#include <algorithm>
extern "C" u32  eeRead32_arm64(u32 a)
{
	// TEMP diagnostic: histogram read32 addresses; dump top 16 every 4M reads
	// (cleared each dump, so the final dump reflects the current/stall phase).
	static std::unordered_map<u32, u64> h; static u64 n = 0;
	h[a]++;
	if (++n % 4000000 == 0)
	{
		std::vector<std::pair<u32,u64>> v(h.begin(), h.end());
		size_t k = v.size() < 16 ? v.size() : 16;
		std::partial_sort(v.begin(), v.begin()+k, v.end(), [](auto&x,auto&y){return x.second>y.second;});
		fprintf(stderr, "=== EE read32 addr histogram (uniq=%zu) ===\n", v.size());
		for (size_t i=0;i<k;i++) fprintf(stderr, "  addr=0x%08x  %llu\n", v[i].first,(unsigned long long)v[i].second);
		h.clear();
	}
	return memRead32(a);
}
#else
extern "C" void eeDiagLogMem(int rd, u32 a, u64 v, int w); // fwd (TEMP diagnostic, defined below)
extern "C" u32  eeRead32_arm64(u32 a) { u32 v = memRead32(a); eeDiagLogMem(1, a, v, 4); return v; }
#endif
extern "C" u64  eeRead64_arm64(u32 a) { u64 v = memRead64(a); eeDiagLogMem(1, a, v, 8); return v; }
// TEMP diagnostic (LRPS2_WLOG): log EE loads/stores touching the MMX7
// IPU-handshake RAM regions and the IPU registers (0x10002000+) so a JIT run
// and a NO_MEM (interpreter) run can be diffed to find the first divergent
// value and the CMD-write/CTRL-poll/CMD-read ordering around it. Called from
// the JIT mem wrappers below and the interpreter LW/LD/SW/SD ops
// (R5900OpcodeImpl.cpp).
extern "C" void eeDiagLogMem(int rd, u32 a, u64 v, int w)
{
	static int on = -1; static u64 seq = 0; static u64 n = 0;
	static u32 wlo = 0, whi = 0; static u64 wframe = 0;
	if (on < 0)
	{
		on = getenv("LRPS2_WLOG") ? 1 : 0;
		// Watch window: LRPS2_WLO/LRPS2_WHI (hex phys addr, half-open range) and
		// LRPS2_WFRAME (log only from this frame on) -- env-tunable so retargeting
		// the watch needs no rebuild.
		const char* s;
		wlo = (s = getenv("LRPS2_WLO")) ? (u32)strtoul(s, 0, 16) : 0x00018c20;
		whi = (s = getenv("LRPS2_WHI")) ? (u32)strtoul(s, 0, 16) : 0x00018c30;
		wframe = (s = getenv("LRPS2_WFRAME")) ? (u64)strtoull(s, 0, 10) : 0;
	}
	if (!on) return;
	const u32 p = a & 0x1fffffff;
	if (p + (u32)w <= wlo || p >= whi) return;
	if (g_diag_frame < wframe) return;
	seq++;
	if (n < 20000) { n++; fprintf(stderr, "[%c] f=%llu seq=%llu a=%08x v=%016llx w=%d pc=%08x\n",
		rd ? 'r' : 'w', (unsigned long long)g_diag_frame, (unsigned long long)seq, p, (unsigned long long)v, w, cpuRegs.pc); }
	// Sync-point RAM dump (LRPS2_DUMP=<path>): on the first D4_MADR write of the
	// MMX7 stream buffer address, dump main RAM + scratchpad for A/B diffing.
	if (!rd && p == 0x1000b410 && (u32)v == 0x0162d210)
	{
		static int dumped = 0;
		const char* dp = getenv("LRPS2_DUMP");
		if (dp && !dumped)
		{
			dumped = 1;
			FILE* f = fopen(dp, "wb");
			if (f) { fwrite(eeMem->Main, 1, Ps2MemSize::MainRam, f);
			         fwrite(eeMem->Scratch, 1, Ps2MemSize::Scratch, f); fclose(f); }
			fprintf(stderr, "[dump] RAM+SPR written to %s at seq=%llu\n", dp, (unsigned long long)seq);
		}
	}
}
extern "C" void eeDiagLogWrite(u32 a, u64 v, int w) { eeDiagLogMem(0, a, v, w); }
// 128-bit LQ/SQ wrappers (Phase C.12). LQ/SQ silently align (addr & ~0xf on the
// caller side); dst/src is &cpuRegs.GPR.r[rt]. These are the slow path -- the
// JIT inlines the vtlb direct-pointer case and only calls here for handlers
// (or with LRPS2_WLOG, where inlining is disabled so the watch hooks fire).
extern "C" void eeRead128_arm64 (u32 a, u128* dst)       { memRead128(a, dst); eeDiagLogMem(1, a, dst->lo, 16); }
extern "C" void eeWrite128_arm64(u32 a, const u128* src) { eeDiagLogMem(0, a, src->lo, 16); memWrite128(a, src); }
extern "C" void eeWrite8_arm64 (u32 a, u32 v) { eeDiagLogWrite(a, v, 1); memWrite8(a, (u8)v); }
extern "C" void eeWrite16_arm64(u32 a, u32 v) { eeDiagLogWrite(a, v, 2); memWrite16(a, (u16)v); }
extern "C" void eeWrite32_arm64(u32 a, u32 v) { eeDiagLogWrite(a, v, 4); memWrite32(a, v); }
extern "C" void eeWrite64_arm64(u32 a, u64 v) { eeDiagLogWrite(a, v, 8); memWrite64(a, v); }
extern "C" void eeCancelInstruction_arm64(void) { Cpu->CancelInstruction(); }
// Unaligned load/store family (LWL/LWR/LDL/LDR, SWL/SWR/SDL/SDR), called from
// the arm64 EE JIT (C.16) with the decoded rt index. Line-for-line mirrors of
// the interpreter ops (R5900OpcodeImpl.cpp) -- they merge with the existing
// register/memory contents at the aligned address and never take a misalign
// exception, so translating them is a plain helper call.
extern "C" void eeLWL_arm64(u32 addr, u32 rt)
{
	static const u32 MASK[4]  = { 0xffffff, 0x0000ffff, 0x000000ff, 0x00000000 };
	static const u8  SHIFT[4] = { 24, 16, 8, 0 };
	const u32 shift = addr & 3;
	const u32 mem   = memRead32(addr & ~3);
	if (!rt) return;
	cpuRegs.GPR.r[rt].SD[0] = (s32)((cpuRegs.GPR.r[rt].UL[0] & MASK[shift]) | (mem << SHIFT[shift]));
}
extern "C" void eeLWR_arm64(u32 addr, u32 rt)
{
	static const u32 MASK[4]  = { 0x000000, 0xff000000, 0xffff0000, 0xffffff00 };
	static const u8  SHIFT[4] = { 0, 8, 16, 24 };
	const u32 shift = addr & 3;
	u32 mem         = memRead32(addr & ~3);
	if (!rt) return;
	mem = (cpuRegs.GPR.r[rt].UL[0] & MASK[shift]) | (mem >> SHIFT[shift]);
	if (shift == 0) cpuRegs.GPR.r[rt].SD[0] = (s32)mem; // sign-extends into 64
	else            cpuRegs.GPR.r[rt].UL[0] = mem;      // upper 32 preserved
}
static const u64 s_LDL_MASK[8] =
{	0x00ffffffffffffffULL, 0x0000ffffffffffffULL, 0x000000ffffffffffULL, 0x00000000ffffffffULL,
	0x0000000000ffffffULL, 0x000000000000ffffULL, 0x00000000000000ffULL, 0x0000000000000000ULL
};
static const u64 s_LDR_MASK[8] =
{	0x0000000000000000ULL, 0xff00000000000000ULL, 0xffff000000000000ULL, 0xffffff0000000000ULL,
	0xffffffff00000000ULL, 0xffffffffff000000ULL, 0xffffffffffff0000ULL, 0xffffffffffffff00ULL
};
extern "C" void eeLDL_arm64(u32 addr, u32 rt)
{
	static const u8 SHIFT[8] = { 56, 48, 40, 32, 24, 16, 8, 0 };
	const u32 shift = addr & 7;
	const u64 mem   = memRead64(addr & ~7);
	if (!rt) return;
	cpuRegs.GPR.r[rt].UD[0] = (cpuRegs.GPR.r[rt].UD[0] & s_LDL_MASK[shift]) | (mem << SHIFT[shift]);
}
extern "C" void eeLDR_arm64(u32 addr, u32 rt)
{
	static const u8 SHIFT[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	const u32 shift = addr & 7;
	const u64 mem   = memRead64(addr & ~7);
	if (!rt) return;
	cpuRegs.GPR.r[rt].UD[0] = (cpuRegs.GPR.r[rt].UD[0] & s_LDR_MASK[shift]) | (mem >> SHIFT[shift]);
}
extern "C" void eeSWL_arm64(u32 addr, u32 rt)
{
	static const u32 MASK[4]  = { 0xffffff00, 0xffff0000, 0xff000000, 0x00000000 };
	static const u8  SHIFT[4] = { 24, 16, 8, 0 };
	const u32 shift = addr & 3;
	const u32 mem   = memRead32(addr & ~3);
	memWrite32(addr & ~3, (cpuRegs.GPR.r[rt].UL[0] >> SHIFT[shift]) | (mem & MASK[shift]));
}
extern "C" void eeSWR_arm64(u32 addr, u32 rt)
{
	static const u32 MASK[4]  = { 0x00000000, 0x000000ff, 0x0000ffff, 0x00ffffff };
	static const u8  SHIFT[4] = { 0, 8, 16, 24 };
	const u32 shift = addr & 3;
	const u32 mem   = memRead32(addr & ~3);
	memWrite32(addr & ~3, (cpuRegs.GPR.r[rt].UL[0] << SHIFT[shift]) | (mem & MASK[shift]));
}
extern "C" void eeSDL_arm64(u32 addr, u32 rt)
{
	static const u64 MASK[8] =
	{	0xffffffffffffff00ULL, 0xffffffffffff0000ULL, 0xffffffffff000000ULL, 0xffffffff00000000ULL,
		0xffffff0000000000ULL, 0xffff000000000000ULL, 0xff00000000000000ULL, 0x0000000000000000ULL
	};
	static const u8 SHIFT[8] = { 56, 48, 40, 32, 24, 16, 8, 0 };
	const u32 shift = addr & 7;
	const u64 mem   = memRead64(addr & ~7);
	memWrite64(addr & ~7, (cpuRegs.GPR.r[rt].UD[0] >> SHIFT[shift]) | (mem & MASK[shift]));
}
extern "C" void eeSDR_arm64(u32 addr, u32 rt)
{
	static const u64 MASK[8] =
	{	0x0000000000000000ULL, 0x00000000000000ffULL, 0x000000000000ffffULL, 0x0000000000ffffffULL,
		0x00000000ffffffffULL, 0x000000ffffffffffULL, 0x0000ffffffffffffULL, 0x00ffffffffffffffULL
	};
	static const u8 SHIFT[8] = { 0, 8, 16, 24, 32, 40, 48, 56 };
	const u32 shift = addr & 7;
	const u64 mem   = memRead64(addr & ~7);
	memWrite64(addr & ~7, (cpuRegs.GPR.r[rt].UD[0] << SHIFT[shift]) | (mem & MASK[shift]));
}
// EE event test, called by the JIT after BEQ/BNE not-taken and (with the cycle
// update) after every taken/unconditional branch -- matching doBranch() and the
// interpreter branch handlers. intUpdateCPUCycles() flushes cpuBlockCycles into
// cpuRegs.cycle so events actually fire (without it the JIT livelocks).
extern "C" void eeEventTest_arm64(void)   { intEventTest(); }
extern "C" void eeUpdateCycles_arm64(void){ intUpdateCPUCycles(); }
#endif

static void intClear(u32 Addr, u32 Size) { }
static void intShutdown(void) { }

R5900cpu intCpu =
{
	intReserve,
	intShutdown,

	intReset,
	intExecute,

	intSafeExitExecution,
	intCancelInstruction,

	intClear
};

#ifdef ARCH_ARM64
// arm64 EE recompiler provider (Phase C.3). Execution loop/exit/cancel are shared
// with the interpreter (same intJmpBuf); only Reserve/Reset/Shutdown/Clear and the
// per-PC block dispatch (eeRecExecute_arm64 -> eeJitRunBlock_arm64) are the JIT's.
R5900cpu recCpu =
{
	eeJitReserve_arm64,
	eeJitShutdown_arm64,

	eeJitReset_arm64,
	eeRecExecute_arm64,

	intSafeExitExecution,
	intCancelInstruction,

	eeJitClear_arm64
};
#endif

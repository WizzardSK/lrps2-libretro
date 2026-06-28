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

static void execI(void)
{
	// execI is called for every instruction so it must remains as light as possible.
	// If you enable the next define, Interpreter will be much slower (around
	// ~4fps on 3.9GHz Haswell vs ~8fps (even 10fps on dev build))
	// Extra note: due to some cycle count issue PCSX2's internal debugger is
	// not yet usable with the interpreter
	u32 pc = cpuRegs.pc;
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
	ExecuteState state = RESET;

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

// Run a single EE basic block through the interpreter (instructions until the
// next taken branch). The arm64 JIT's per-PC blocks call this for now (Phase
// C.3-1); later phases translate the instructions natively.
extern "C" void eeRunBasicBlock_arm64(void)
{
	branch2 = 0;
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
extern "C" u32  eeRead32_arm64(u32 a) { return memRead32(a); }
#endif
extern "C" u64  eeRead64_arm64(u32 a) { return memRead64(a); }
// TEMP diagnostic (LRPS2_WLOG): log EE stores to the MMX7 IPU-handshake RAM
// regions so a JIT run and a NO_MEM (interpreter) run can be diffed to find the
// first divergent written value. Called from both the JIT store wrappers below
// and the interpreter SW/SD ops (R5900OpcodeImpl.cpp).
extern "C" void eeDiagLogWrite(u32 a, u64 v, int w)
{
	static int on = -1; static u64 seq = 0; static u64 n = 0;
	if (on < 0) on = getenv("LRPS2_WLOG") ? 1 : 0;
	if (!on) return;
	const u32 p = a & 0x1fffffff;
	const bool hot = (p >= 0x01400800 && p <= 0x0140087f) || (p >= 0x004009c0 && p <= 0x004009ff);
	if (!hot) return;
	seq++;
	if (n < 6000) { n++; fprintf(stderr, "[w] seq=%llu a=%08x v=%016llx w=%d\n",
		(unsigned long long)seq, p, (unsigned long long)v, w); }
}
extern "C" void eeWrite8_arm64 (u32 a, u32 v) { eeDiagLogWrite(a, v, 1); memWrite8(a, (u8)v); }
extern "C" void eeWrite16_arm64(u32 a, u32 v) { eeDiagLogWrite(a, v, 2); memWrite16(a, (u16)v); }
extern "C" void eeWrite32_arm64(u32 a, u32 v) { eeDiagLogWrite(a, v, 4); memWrite32(a, v); }
extern "C" void eeWrite64_arm64(u32 a, u64 v) { eeDiagLogWrite(a, v, 8); memWrite64(a, v); }
extern "C" void eeCancelInstruction_arm64(void) { Cpu->CancelInstruction(); }
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

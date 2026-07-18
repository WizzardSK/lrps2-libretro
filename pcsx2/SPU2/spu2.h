/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2023  PCSX2 Dev Team
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

#pragma once

#include "../SaveState.h"
#include "../IopCounters.h"

struct Pcsx2Config;

namespace SPU2
{
	/// Initialization/cleanup, call at process startup/shutdown.
	void Initialize(void);
	void Shutdown(void);

	/// Open/close, call at VM startup/shutdown.
	void Open(void);
	void Close(void);

	/// Reset, rebooting VM or going into PSX mode.
	void Reset(bool psxmode);

	/// Returns true if we're currently running in PSX mode.
	bool IsRunningPSXMode(void);
}

void SPU2write(u32 mem, u16 value);
u16 SPU2read(u32 mem);

s32 SPU2freeze(FreezeAction mode, freezeData* data);

extern u32 lClocks;
typedef void RegWriteHandler(u16 value);
extern RegWriteHandler* const tbl_reg_writes[0x401];

extern void TimeUpdate(u32 cClocks);

// TEMP diagnostic (LRPS2_SPU_SYNC_STATS=1): counts the operations that would be
// synchronization points for a prospective SPU worker thread. Reads force a
// drain (wait for the worker to catch up), writes/DMA can be queued, and an
// armed IRQ forces fully synchronous mixing (an IRQA crossing discovered late
// would deliver the IOP interrupt at the wrong emulated time -- the C.8 bug
// class). The verdict this collects: reads/sec (drain rate), the armed-tick
// fraction (how much of the run must stay synchronous), and which registers
// the game actually polls.
struct SpuSyncStats
{
	unsigned long long reads = 0, writes = 0, dmar = 0, dmaw = 0,
	                   dmar_words = 0, dmaw_words = 0, irqs = 0,
	                   timeupd = 0, ticks = 0, ticks_armed = 0,
	                   // IRQA-match sites (what raised has_to_call_irq*): mixer
	                   // voice reads, reverb work area, ADMA input reads, DMA
	                   // transfers, direct register-path TSA checks. The mixer/
	                   // reverb/input ones are the async-mixing killers; the DMA/
	                   // register ones happen on the EE thread synchronously
	                   // regardless of a worker.
	                   irq_mix = 0, irq_rvb = 0, irq_inp = 0, irq_dma = 0, irq_reg = 0;
	unsigned long long rd_reg[0x400] = {}; // (rmem & 0x7ff) >> 1
	void Dump(const char* tag) const;
};
extern SpuSyncStats g_spuSync;
extern const bool g_spuSyncOn;

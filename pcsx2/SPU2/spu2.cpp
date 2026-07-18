/*  PCSX2 - PS2 Emulator for PCs
 *  Copyright (C) 2002-2023 PCSX2 Dev Team
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

#include "Global.h"
#include "spu2.h"
#include "Dma.h"

#include "../R3000A.h"

static bool s_psxmode = false;

u32 lClocks = 0;

// TEMP diagnostic (LRPS2_SPU_SYNC_STATS=1): see spu2.h. Dumped at exit and
// every ~2M sample ticks (~43 s of emulated audio).
SpuSyncStats g_spuSync;
const bool g_spuSyncOn = getenv("LRPS2_SPU_SYNC_STATS") != nullptr;

void SpuSyncStats::Dump(const char* tag) const
{
	const double sec = ticks / 48000.0; // one stereo sample pair per tick
	if (!sec)
		return;
	fprintf(stderr, "[spu-sync %s] audio %.1fs | reads %llu (%.0f/s) writes %llu (%.0f/s) | "
		"dmaR %llu (%llu w) dmaW %llu (%llu w, %.0f/s) | irqs %llu | "
		"TimeUpdate %llu | armed ticks %llu/%llu (%.2f%%)\n",
		tag, sec, reads, reads / sec, writes, writes / sec,
		dmar, dmar_words, dmaw, dmaw_words, dmaw / sec, irqs,
		timeupd, ticks_armed, ticks, ticks ? 100.0 * ticks_armed / ticks : 0.0);
	fprintf(stderr, "[spu-sync %s] irq sites: mixer %llu reverb %llu input %llu dma %llu reg %llu\n",
		tag, irq_mix, irq_rvb, irq_inp, irq_dma, irq_reg);
	// Top polled registers (read side = the drain-forcing accesses).
	struct E { unsigned r; unsigned long long n; } top[12] = {};
	for (unsigned r = 0; r < 0x400; r++)
	{
		unsigned long long n = rd_reg[r];
		for (int i = 0; i < 12; i++)
			if (n > top[i].n) { for (int j = 11; j > i; j--) top[j] = top[j - 1]; top[i] = { r, n }; break; }
	}
	fprintf(stderr, "[spu-sync %s] top reads:", tag);
	for (int i = 0; i < 12 && top[i].n; i++)
		fprintf(stderr, " %03x:%llu", top[i].r << 1, top[i].n); // raw reg offset (core1 = +0x400)
	fprintf(stderr, "\n");
}

namespace { struct SpuSyncDumper { ~SpuSyncDumper() { if (g_spuSyncOn) g_spuSync.Dump("exit"); } } s_spuSyncDumper; }

// --------------------------------------------------------------------------------------
//  DMA 4/7 Callbacks from Core Emulator
// --------------------------------------------------------------------------------------

static void SPU2_InternalReset(bool psxmode)
{
	s_psxmode = psxmode;
	if (!s_psxmode)
	{
		memset(spu2regs, 0, 0x010000);
		memset(_spu2mem, 0, 0x200000);
		memset(_spu2mem + 0x2800, 7, 0x10); // from BIOS reversal. Locks the voices so they don't run free.
		memset(_spu2mem + 0xe870, 7, 0x10); // Loop which gets left over by the BIOS, Megaman X7 relies on it being there.

		Spdif.Info = 0; // Reset IRQ Status if it got set in a previously run game

		Cores[0].Init(0);
		Cores[1].Init(1);
	}
}

void SPU2::Reset(bool psxmode) { SPU2_InternalReset(psxmode); }
void SPU2::Initialize(void)    { }

void SPU2::Open()
{
	lClocks = psxRegs.cycle;

	SPU2_InternalReset(false);
}

void SPU2::Close() { }
void SPU2::Shutdown() { }
bool SPU2::IsRunningPSXMode() { return s_psxmode; }

u16 SPU2read(u32 rmem)
{
	u16 ret = 0xDEAD;
	u32 core = 0;
	const u32 mem = rmem & 0xFFFF;
	u32 omem = mem;

	if (g_spuSyncOn) { g_spuSync.reads++; g_spuSync.rd_reg[(rmem & 0x7ff) >> 1]++; }

	if (mem & 0x400)
	{
		omem ^= 0x400;
		core = 1;
	}

	if (omem == 0x1f9001AC)
	{
		Cores[core].ActiveTSA = Cores[core].TSA;
		for (int i = 0; i < 2; i++)
		{
			if (Cores[i].IRQEnable && (Cores[i].IRQA == Cores[core].ActiveTSA))
				{ if (g_spuSyncOn) g_spuSync.irq_reg++; has_to_call_irq[i] = true; }
		}
		ret = Cores[core].DmaRead();
	}
	else
	{
		TimeUpdate(psxRegs.cycle);

		if (rmem >> 16 == 0x1f80)
			ret = Cores[0].ReadRegPS1(rmem);
		else if (mem >= 0x800)
			ret = spu2Ru16(mem);
		else
			ret = *(regtable[(mem >> 1)]);
	}

	return ret;
}

void SPU2write(u32 rmem, u16 value)
{
	// Note: Reverb/Effects are very sensitive to having precise update timings.
	// If the SPU2 isn't in in sync with the IOP, samples can end up playing at rather
	// incorrect pitches and loop lengths.

	if (g_spuSyncOn) g_spuSync.writes++;

	TimeUpdate(psxRegs.cycle);

	if (rmem >> 16 == 0x1f80)
		Cores[0].WriteRegPS1(rmem, value);
	else
		tbl_reg_writes[(rmem & 0x7ff) / 2](value);
}

s32 SPU2freeze(FreezeAction mode, freezeData* data)
{
	if (!data)
		return -1;

	if (mode == FreezeAction::Size)
		data->size = SPU2Savestate::SizeIt();
	else
	{
		if (data->data == nullptr)
			return -1;

		auto& spud = (SPU2Savestate::DataBlock&)*(data->data);

		switch (mode)
		{
			case FreezeAction::Load:
				return SPU2Savestate::ThawIt(spud);
			case FreezeAction::Save:
				SPU2Savestate::FreezeIt(spud);
				break;
			default:
				break;
		}
	}

	return 0;
}

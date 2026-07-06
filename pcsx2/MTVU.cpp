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

#include <cstring> /* memset */
#include <thread>

#include "Common.h"
#include "Gif_Unit.h"
#include "MTVU.h"
#include "VMManager.h"
#include "Vif_Dynarec.h"

#include "../common/Threading.h"

VU_Thread vu1Thread;

// Rounds up a size in bytes for size in u32's
static __fi u32 SIZE_U32(u32 x) { return (x + 3) >> 2; }

enum MTVU_EVENT
{
	MTVU_VU_EXECUTE,     // Execute VU program
	MTVU_VU_WRITE_MICRO, // Write to VU micro-mem
	MTVU_VU_WRITE_DATA,  // Write to VU data-mem
	MTVU_VU_WRITE_VIREGS,// Write to VU registers
	MTVU_VU_WRITE_VFREGS,// Write to VU registers
	MTVU_VIF_WRITE_COL,  // Write to Vif col reg
	MTVU_VIF_WRITE_ROW,  // Write to Vif row reg
	MTVU_VIF_UNPACK,     // Execute Vif Unpack
	MTVU_NULL_PACKET,    // Go back to beginning of buffer
	MTVU_RESET
};

// Calls the vif unpack functions from the MTVU thread
static void MTVU_Unpack(void* data, VIFregisters& vifRegs)
{
	u16 wl = vifRegs.cycle.wl > 0 ? vifRegs.cycle.wl : 256;
	bool isFill = vifRegs.cycle.cl < wl;
	dVifUnpack<1>((u8*)data, isFill);
}

// Called on Saving/Loading states...
bool SaveStateBase::mtvuFreeze()
{
	if (!(FreezeTag("MTVU")))
		return false;

	if (!IsSaving())
	{
		vu1Thread.Reset();
		vu1Thread.WriteCol(vif1);
		vu1Thread.WriteRow(vif1);
		vu1Thread.WriteMicroMem(0, vuRegs[1].Micro, 0x4000);
		vu1Thread.WriteDataMem(0, vuRegs[1].Mem, 0x4000);
		vu1Thread.WriteVIRegs(&vuRegs[1].VI[0]);
		vu1Thread.WriteVFRegs(&vuRegs[1].VF[0]);
	}
	for (size_t i = 0; i < 4; ++i)
	{
		unsigned int v = vu1Thread.vuCycles[i].load();
		Freeze(v);
	}

	u32 gsInterrupts = vu1Thread.mtvuInterrupts.load();
	Freeze(gsInterrupts);
	vu1Thread.mtvuInterrupts.store(gsInterrupts);
	u64 gsSignal = vu1Thread.gsSignal.load();
	Freeze(gsSignal);
	vu1Thread.gsSignal.store(gsSignal);
	u64 gsLabel = vu1Thread.gsLabel.load();
	Freeze(gsLabel);
	vu1Thread.gsLabel.store(gsLabel);

	Freeze(vu1Thread.vuCycleIdx);

	return IsOkay();
}

VU_Thread::VU_Thread()
{
}

VU_Thread::~VU_Thread()
{
	Close();
}

void VU_Thread::Open()
{
	if (IsOpen())
		return;

	Reset();
	semaEvent.Reset();
	m_shutdown_flag.store(false, std::memory_order_release);
	m_thread.SetStackSize(VMManager::EMU_THREAD_STACK_SIZE);
	m_thread.Start([this]() { ExecuteRingBuffer(); });
}

void VU_Thread::Close()
{
	if (!IsOpen())
		return;

	m_shutdown_flag.store(true, std::memory_order_release);
	semaEvent.NotifyOfWorkIfRunning();
	m_thread.Join();
}

void VU_Thread::Reset()
{
	size_t i;

	vuCycleIdx = 0;
	m_ato_write_pos = 0;
	m_write_pos = 0;
	m_ato_read_pos = 0;
	m_read_pos = 0;
	memset(&vif, 0, sizeof(vif));
	memset(&vifRegs, 0, sizeof(vifRegs));
	for (i = 0; i < 4; ++i)
		vu1Thread.vuCycles[i] = 0;
	vu1Thread.mtvuInterrupts = 0;
}

void VU_Thread::ExecuteRingBuffer(void)
{
	for (;;)
	{
		semaEvent.WaitForWork();
		if (m_shutdown_flag.load(std::memory_order_acquire))
			break;

		while (m_ato_read_pos.load(std::memory_order_relaxed) != GetWritePos())
		{
			u32 tag = Read();
			switch (tag)
			{
				case MTVU_VU_EXECUTE:
				{
					vuRegs[1].cycle = 0;
					s32 addr = Read();
					vifRegs.top = Read();
					vifRegs.itop = Read();
					vuFBRST = Read();
					if (addr != -1)
						vuRegs[1].VI[REG_TPC].UL = addr & 0x7FF;
					CpuVU1->SetStartPC(vuRegs[1].VI[REG_TPC].UL << 3);
					// The EE-side vu1ExecMicro intentionally CLEARS VPU_STAT's VU1
					// busy bit in MTVU mode ("pretend the VU finished instantly").
					// The x86 microVU worker doesn't care, but the VU1 INTERPRETER's
					// Execute loop breaks immediately when the busy bit is clear --
					// the worker then executes ZERO instructions, produces no XGKICK
					// output, and every MTVU GS packet arrives empty (GT3's VU1-drawn
					// FMV went black). Set busy for the duration of the program like
					// the non-MTVU path does; the interpreter's E-bit end clears it.
					vuRegs[0].VI[REG_VPU_STAT].UL |= 0x0100;
					CpuVU1->Execute(vu1RunCycles);
					// TEMP diagnostic (LRPS2_MTVU_LOG): count worker-side VU1 programs
					// and the GS-packet bytes they produced.
					{
						static int log_on = -1; static u64 n = 0, bytes = 0;
						if (log_on < 0) log_on = getenv("LRPS2_MTVU_LOG") ? 1 : 0;
						if (log_on)
						{
							n++; bytes += gifUnit.gifPath[GIF_PATH_1].gsPack.size;
							if (n <= 4 || (n & 0xf) == 0)
								fprintf(stderr, "[mtvu-w] progs=%llu path1_bytes=%llu\n",
									(unsigned long long)n, (unsigned long long)bytes);
						}
					}
					gifUnit.gifPath[GIF_PATH_1].FinishGSPacketMTVU();
					semaXGkick.Post(); // Tell MTGS a path1 packet is complete
					vuCycles[vuCycleIdx].store(vuRegs[1].cycle, std::memory_order_release);
					vuCycleIdx = (vuCycleIdx + 1) & 3;
					break;
				}
				case MTVU_VU_WRITE_MICRO:
				{
					u32 vu_micro_addr = Read();
					u32 size = Read();
					CpuVU1->Clear(vu_micro_addr, size);
					Read(&vuRegs[1].Micro[vu_micro_addr], size);
					break;
				}
				case MTVU_VU_WRITE_DATA:
				{
					u32 vu_data_addr = Read();
					u32 size = Read();
					Read(&vuRegs[1].Mem[vu_data_addr], size);
					break;
				}
				case MTVU_VU_WRITE_VIREGS:
					Read(&vuRegs[1].VI, /*size_u32(32)*/8);
					break;
				case MTVU_VU_WRITE_VFREGS:
					Read(&vuRegs[1].VF, /*size_u32(4*32)*/32);
					break;
				case MTVU_VIF_WRITE_COL:
					Read(&vif.MaskCol, sizeof(vif.MaskCol));
					break;
				case MTVU_VIF_WRITE_ROW:
					Read(&vif.MaskRow, sizeof(vif.MaskRow));
					break;
				case MTVU_VIF_UNPACK:
				{
					// TEMP diagnostic (LRPS2_MTVU_LOG)
					{
						static int log_on = -1; static u64 n = 0;
						if (log_on < 0) log_on = getenv("LRPS2_MTVU_LOG") ? 1 : 0;
						if (log_on) { n++; if (n <= 4 || (n & 0x3ff) == 0) fprintf(stderr, "[mtvu-u] unpacks=%llu\n", (unsigned long long)n); }
					}
					u32 vif_copy_size = (uptr)&vif.StructEnd - (uptr)&vif.tag;
					Read(&vif.tag, vif_copy_size);
					ReadRegs(&vifRegs);
					u32 size = Read();
					MTVU_Unpack(&buffer[m_read_pos], vifRegs);
					m_read_pos += SIZE_U32(size);
					break;
				}
				case MTVU_NULL_PACKET:
					m_read_pos = 0;
					break;
				default:
					break;
			}

			m_ato_read_pos.store(m_read_pos, std::memory_order_release);
		}
	}

	semaEvent.Kill();
}


// Should only be called by ReserveSpace()
__ri void VU_Thread::WaitOnSize(s32 size)
{
	for (;;)
	{
		s32 readPos = GetReadPos();
		if (readPos <= m_write_pos)
			break; // MTVU is reading in back of write_pos
		// FIXME greg: there is a bug somewhere in the queue pointer
		// management. It creates a deadlock/corruption in SotC intro (before
		// the first menu). I added a 4KB safety net which seem to avoid to
		// trigger the bug.
		// Note: a wait lock instead of a yield also helps to avoid the bug.
		if (readPos > m_write_pos + size + _4kb)
			break; // Enough free front space
		{          // Let MTVU run to free up buffer space
			KickStart();
			// Locking might trigger a full flush of the ring buffer. Yield
			// will be more aggressive, and only flush the minimal size.
			// Performance will be smoother but it will consume extra CPU cycle
			// on the EE thread (not an issue on 4 cores).
			std::this_thread::yield();
		}
	}
}

// Makes sure theres enough room in the ring buffer
// to write a continuous 'size * sizeof(u32)' bytes
void VU_Thread::ReserveSpace(s32 size)
{
	if (m_write_pos + size > (buffer_size - 1))
	{
		WaitOnSize(1); // Size of MTVU_NULL_PACKET
		Write(MTVU_NULL_PACKET);
		// Reset local write pointer/position
		m_write_pos = 0;
		m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	}

	WaitOnSize(size);
}

// Use this when reading read_pos from ee thread
__fi s32 VU_Thread::GetReadPos()
{
	return m_ato_read_pos.load(std::memory_order_acquire);
}

// Use this when reading write_pos from vu thread
__fi s32 VU_Thread::GetWritePos()
{
	return m_ato_write_pos.load(std::memory_order_acquire);
}

// Gets the effective write pointer after
__fi u32* VU_Thread::GetWritePtr()
{
	return &buffer[m_write_pos];
}

__fi u32 VU_Thread::Read()
{
	u32 ret = buffer[m_read_pos];
	m_read_pos++;
	return ret;
}

__fi void VU_Thread::Read(void* dest, u32 size)
{
	memcpy(dest, &buffer[m_read_pos], size);
	m_read_pos += SIZE_U32(size);
}

__fi void VU_Thread::ReadRegs(VIFregisters* dest)
{
	VIFregistersMTVU* src = (VIFregistersMTVU*)&buffer[m_read_pos];
	dest->cycle = src->cycle;
	dest->mode = src->mode;
	dest->num = src->num;
	dest->mask = src->mask;
	dest->itop = src->itop;
	dest->top = src->top;
	m_read_pos += SIZE_U32(sizeof(VIFregistersMTVU));
}

__fi void VU_Thread::Write(u32 val)
{
	GetWritePtr()[0] = val;
	m_write_pos += 1;
}

__fi void VU_Thread::Write(const void* src, u32 size)
{
	memcpy(GetWritePtr(), src, size);
	m_write_pos += SIZE_U32(size);
}

__fi void VU_Thread::WriteRegs(VIFregisters* src)
{
	VIFregistersMTVU* dest = (VIFregistersMTVU*)GetWritePtr();
	dest->cycle = src->cycle;
	dest->mode = src->mode;
	dest->num = src->num;
	dest->mask = src->mask;
	dest->top = src->top;
	dest->itop = src->itop;
	m_write_pos += SIZE_U32(sizeof(VIFregistersMTVU));
}

// Returns Average number of vu Cycles from last 4 runs
// Used for vu cycle stealing hack
u32 VU_Thread::Get_vuCycles()
{
	return (vuCycles[0].load(std::memory_order_relaxed) +
			vuCycles[1].load(std::memory_order_relaxed) +
			vuCycles[2].load(std::memory_order_relaxed) +
			vuCycles[3].load(std::memory_order_relaxed)) >>
		   2;
}

void VU_Thread::Get_MTVUChanges()
{
	// Note: Atomic communication is with Gif_Unit.cpp Gif_HandlerAD_MTVU
	u32 interrupts = mtvuInterrupts.load(std::memory_order_relaxed);
	if (!interrupts)
		return;

	if (interrupts & InterruptFlagSignal)
	{
		/* Clear the flag with acquire FIRST (Label-style), then
		 * atomically take ownership of gsSignal by exchanging it
		 * to zero. acquire on the flag-clear synchronizes with the
		 * producer's release fetch_or, so the gsSignal value we
		 * read below is the one the producer wrote before setting
		 * the flag.
		 *
		 * Order matters: if we read gsSignal first and then cleared
		 * the flag (the previous design), the producer could
		 * overwrite gsSignal between our read and our clear, and
		 * we'd silently drop the second value. With clear-then-
		 * exchange the producer-side spin-wait (Gif_Unit.cpp) sees
		 * the cleared flag and is free to publish a new value; if
		 * it raced into the gap before our exchange, our exchange
		 * picks up the new value and the spurious flag re-set is
		 * handled by the signal==0 guard below on a subsequent
		 * Get_MTVUChanges call. */
		mtvuInterrupts.fetch_and(~InterruptFlagSignal, std::memory_order_acquire);
		const u64 signal = gsSignal.exchange(0, std::memory_order_relaxed);
		if (signal != 0)
		{
			const u32 signalMsk = (u32)(signal >> 32);
			const u32 signalData = (u32)signal;
			if (CSRreg.SIGNAL)
			{
				/* Queue signal */
				gifUnit.gsSIGNAL.queued = true;
				gifUnit.gsSIGNAL.data[0] = signalData;
				gifUnit.gsSIGNAL.data[1] = signalMsk;
			}
			else
			{
				CSRreg.SIGNAL    = true;
				GSSIGLBLID.SIGID = (GSSIGLBLID.SIGID & ~signalMsk) | (signalData & signalMsk);

				if (!GSIMR.SIGMSK)
					hwIntcIrq(INTC_GS);
			}
		}
		/* signal == 0: flag was re-set by the producer after we
		 * cleared it but before its gsSignal store was visible to
		 * us, OR a previous Get_MTVUChanges already drained the
		 * value. The producer will set the flag again once its
		 * store is visible; the next Get_MTVUChanges call will
		 * process it. Treating zero as a no-op also prevents a
		 * spurious IRQ with stale SIGID bits. */
	}
	if (interrupts & InterruptFlagFinish)
	{
		mtvuInterrupts.fetch_and(~InterruptFlagFinish, std::memory_order_relaxed);
		/* Finish firing */
		gifUnit.gsFINISH.gsFINISHFired = false;
		gifUnit.gsFINISH.gsFINISHPending = true;

		if (!gifUnit.checkPaths(false, true, true, true))
			Gif_FinishIRQ();
	}
	if (interrupts & InterruptFlagLabel)
	{
		mtvuInterrupts.fetch_and(~InterruptFlagLabel, std::memory_order_acquire);
		// If other thread updates gsLabel for a second interrupt, that's okay.  Worst case we think there's a label interrupt but gsLabel is 0
		// We do not want the exchange of gsLabel to move ahead of clearing the flag, or the other thread could add more work before we clear the flag, resulting in an update with the flag unset
		// acquire semantics should supply that guarantee
		/* LABEL firing */
		const u64 label = gsLabel.exchange(0, std::memory_order_relaxed);
		const u32 labelMsk = (u32)(label >> 32);
		const u32 labelData = (u32)label;
		GSSIGLBLID.LBLID = (GSSIGLBLID.LBLID & ~labelMsk) | (labelData & labelMsk);
	}
	if (interrupts & InterruptFlagVUEBit)
	{
		mtvuInterrupts.fetch_and(~InterruptFlagVUEBit, std::memory_order_relaxed);

		if(INSTANT_VU1)
			vuRegs[0].VI[REG_VPU_STAT].UL &= ~0xFF00;
	}
	if (interrupts & InterruptFlagVUTBit)
	{
		mtvuInterrupts.fetch_and(~InterruptFlagVUTBit, std::memory_order_relaxed);
		vuRegs[0].VI[REG_VPU_STAT].UL &= ~0xFF00;
		vuRegs[0].VI[REG_VPU_STAT].UL |= 0x0400;
		hwIntcIrq(7);
	}
}

void VU_Thread::KickStart()
{
	semaEvent.NotifyOfWork();
}

// Called from Gif_Path::CopyGSPacketData (MTVU thread) after handing MTGS a
// partial packet, so MTGS's semaXGkick wait wakes up and drains it.
void Gif_MTVU_KickSema()
{
	vu1Thread.semaXGkick.Post();
}

void VU_Thread::WaitVU()
{
	semaEvent.WaitForEmpty();
}

void VU_Thread::ExecuteVU(u32 vu_addr, u32 vif_top, u32 vif_itop, u32 fbrst)
{
	Get_MTVUChanges(); // Clear any pending interrupts
	ReserveSpace(5);
	Write(MTVU_VU_EXECUTE);
	Write(vu_addr);
	Write(vif_top);
	Write(vif_itop);
	Write(fbrst);
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	gifUnit.TransferGSPacketData(GIF_TRANS_MTVU, NULL, 0);
	KickStart();
	u32 cycles = std::max(Get_vuCycles(), 4u);
	u32 skip_cycles = std::min(cycles, 3000u);
	cpuRegs.cycle += skip_cycles * EmuConfig.Speedhacks.EECycleSkip;
	vuRegs[0].cycle += skip_cycles * EmuConfig.Speedhacks.EECycleSkip;
	Get_MTVUChanges();

	if (!INSTANT_VU1)
	{
		vuRegs[0].VI[REG_VPU_STAT].UL |= 0x100;
		CPU_INT(VU_MTVU_BUSY, cycles);
	}
}

void VU_Thread::VifUnpack(vifStruct& _vif, VIFregisters& _vifRegs, const u8* data, u32 size)
{
	u32 vif_copy_size = (uptr)&_vif.StructEnd - (uptr)&_vif.tag;
	ReserveSpace(1 + SIZE_U32(vif_copy_size) + SIZE_U32(sizeof(VIFregistersMTVU)) + 1 + SIZE_U32(size));
	Write(MTVU_VIF_UNPACK);
	Write(&_vif.tag, vif_copy_size);
	WriteRegs(&_vifRegs);
	Write(size);
	Write(data, size);
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

void VU_Thread::WriteMicroMem(u32 vu_micro_addr, const void* data, u32 size)
{
	ReserveSpace(3 + SIZE_U32(size));
	Write(MTVU_VU_WRITE_MICRO);
	Write(vu_micro_addr);
	Write(size);
	Write(data, size);
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

void VU_Thread::WriteDataMem(u32 vu_data_addr, const void* data, u32 size)
{
	ReserveSpace(3 + SIZE_U32(size));
	Write(MTVU_VU_WRITE_DATA);
	Write(vu_data_addr);
	Write(size);
	Write(data, size);
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

void VU_Thread::WriteVIRegs(REG_VI* viRegs)
{
	ReserveSpace(1 + /*size_u32(32)*/8);
	Write(MTVU_VU_WRITE_VIREGS);
	Write(viRegs, /*size_u32(32)*/8);
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

void VU_Thread::WriteVFRegs(VECTOR* vfRegs)
{
	ReserveSpace(1 + /*size_u32(32*4)*/32);
	Write(MTVU_VU_WRITE_VFREGS);
	Write(vfRegs, /*size_u32(32*4)*/32);
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

void VU_Thread::WriteCol(vifStruct& _vif)
{
	ReserveSpace(1 + SIZE_U32(sizeof(_vif.MaskCol)));
	Write(MTVU_VIF_WRITE_COL);
	Write(&_vif.MaskCol, sizeof(_vif.MaskCol));
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

void VU_Thread::WriteRow(vifStruct& _vif)
{
	ReserveSpace(1 + SIZE_U32(sizeof(_vif.MaskRow)));
	Write(MTVU_VIF_WRITE_ROW);
	Write(&_vif.MaskRow, sizeof(_vif.MaskRow));
	m_ato_write_pos.store(m_write_pos, std::memory_order_release);
	KickStart();
}

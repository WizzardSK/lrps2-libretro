// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-FileCopyrightText: 2026 isztld <https://isztld.com/>
// SPDX-License-Identifier: GPL-3.0+
//
// Minimal subset of the ARMSX2/upstream-PCSX2 arm64 AsmHelpers (isztldav/pcsx2
// @ c89cb8a), kept API-compatible so code transplanted from that tree (C.19:
// Vif_UnpackNEON; later candidates: aR5900FPU, aVU) compiles with few or no
// edits. Simplifications vs the original: no ArmConstantPool, no macOS W^X
// (our code regions are RWX mmaps), plain Mov instead of adrp+add address
// materialization.

#pragma once

#include "common/Pcsx2Defs.h" // u8/u32 + __fi
#include "aarch64/macro-assembler-aarch64.h"

#define RXARG1 vixl::aarch64::x0
#define RXARG2 vixl::aarch64::x1
#define RXARG3 vixl::aarch64::x2

// vixl reserves x16/x17 as macro-assembler scratch (ip0/ip1); armStartBlock
// removes x17 from the scratch list so helper code can use it explicitly.
#define RXVIXLSCRATCH vixl::aarch64::x16
#define RSCRATCHADDR vixl::aarch64::x17

const vixl::aarch64::VRegister& armQRegister(int n);

extern thread_local vixl::aarch64::MacroAssembler* armAsm;
extern thread_local u8* armAsmPtr;
extern thread_local size_t armAsmCapacity;

static __fi u8* armGetCurrentCodePointer()
{
	return armAsmPtr + armAsm->GetCursorOffset();
}

static __fi u8* armGetAsmPtr()
{
	return armAsmPtr;
}

// Points the (thread-local) emitter at a code region. Pass the remaining
// capacity of the region; blocks are then bracketed with armStartBlock() /
// armEndBlock(), which advance the pointer past the emitted code.
void armSetAsmPtr(void* ptr, size_t capacity);

u8* armStartBlock();
u8* armEndBlock();

void armMoveAddressToReg(const vixl::aarch64::Register& reg, const void* addr);
void armLoadPtr(const vixl::aarch64::CPURegister& reg, const void* addr);
void armStorePtr(const vixl::aarch64::CPURegister& reg, const void* addr);

vixl::aarch64::MemOperand armOffsetMemOperand(const vixl::aarch64::MemOperand& op, s64 offset);
void armGetMemOperandInRegister(const vixl::aarch64::Register& addr_reg,
	const vixl::aarch64::MemOperand& op, s64 extra_offset = 0);

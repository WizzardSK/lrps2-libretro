// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-FileCopyrightText: 2026 isztld <https://isztld.com/>
// SPDX-License-Identifier: GPL-3.0+
//
// See AsmHelpers.h -- minimal ARMSX2-compatible emitter plumbing.

#include "AsmHelpers.h"

namespace a64 = vixl::aarch64;

const a64::VRegister& armQRegister(int n)
{
	using namespace vixl::aarch64;
	static constexpr const VRegister* regs[32] = {&q0, &q1, &q2, &q3, &q4, &q5, &q6, &q7, &q8, &q9, &q10,
		&q11, &q12, &q13, &q14, &q15, &q16, &q17, &q18, &q19, &q20, &q21, &q22, &q23, &q24, &q25, &q26, &q27, &q28,
		&q29, &q30, &q31};
	return *regs[n];
}

// thread_local: VIF0 unpacks compile on the EE thread while VIF1's can compile
// on the MTVU worker; each thread gets its own in-flight assembler. The per-idx
// write pointers (nVif[idx].recWritePtr) keep the code regions disjoint.
thread_local a64::MacroAssembler* armAsm;
thread_local u8* armAsmPtr;
thread_local size_t armAsmCapacity;

// Placement storage instead of heap new/delete per compiled block (pattern
// from ARMSX2; also one fewer alloc/free pair per block).
alignas(a64::MacroAssembler) static thread_local u8 s_armAsmStorage[sizeof(a64::MacroAssembler)];

void armSetAsmPtr(void* ptr, size_t capacity)
{
	armAsmPtr = static_cast<u8*>(ptr);
	armAsmCapacity = capacity;
}

static void armAlignAsmPtr()
{
	static constexpr uintptr_t ALIGNMENT = 16;
	u8* new_ptr = reinterpret_cast<u8*>((reinterpret_cast<uintptr_t>(armAsmPtr) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1));
	armAsmCapacity -= (new_ptr - armAsmPtr);
	armAsmPtr = new_ptr;
}

u8* armStartBlock()
{
	armAlignAsmPtr();
	armAsm = new (s_armAsmStorage) a64::MacroAssembler(
		static_cast<vixl::byte*>(armAsmPtr), armAsmCapacity, a64::PositionDependentCode);
	// Helper code uses x17 (RSCRATCHADDR) and q29-q31 explicitly; keep the
	// macro-assembler from clobbering them during macro expansion.
	armAsm->GetScratchRegisterList()->Remove(RSCRATCHADDR.GetCode());
	armAsm->GetScratchVRegisterList()->Remove(31);
	return armAsmPtr;
}

u8* armEndBlock()
{
	armAsm->FinalizeCode();
	const u32 size = static_cast<u32>(armAsm->GetSizeOfCodeGenerated());
	armAsm->~MacroAssembler();
	armAsm = nullptr;
	__builtin___clear_cache(reinterpret_cast<char*>(armAsmPtr), reinterpret_cast<char*>(armAsmPtr + size));
	armAsmPtr = armAsmPtr + size;
	armAsmCapacity -= size;
	return armAsmPtr;
}

void armMoveAddressToReg(const a64::Register& reg, const void* addr)
{
	armAsm->Mov(reg, reinterpret_cast<uintptr_t>(addr));
}

void armLoadPtr(const a64::CPURegister& reg, const void* addr)
{
	armMoveAddressToReg(RSCRATCHADDR, addr);
	armAsm->Ldr(reg, a64::MemOperand(RSCRATCHADDR));
}

void armStorePtr(const a64::CPURegister& reg, const void* addr)
{
	armMoveAddressToReg(RSCRATCHADDR, addr);
	armAsm->Str(reg, a64::MemOperand(RSCRATCHADDR));
}

a64::MemOperand armOffsetMemOperand(const a64::MemOperand& op, s64 offset)
{
	return a64::MemOperand(op.GetBaseRegister(), op.GetOffset() + offset, op.GetAddrMode());
}

void armGetMemOperandInRegister(const a64::Register& addr_reg, const a64::MemOperand& op, s64 extra_offset)
{
	armAsm->Add(addr_reg, op.GetBaseRegister(), op.GetOffset() + extra_offset);
}

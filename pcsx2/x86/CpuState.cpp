#include "common/Pcsx2Types.h"
#include "R5900.h"
#include "R3000A.h"

alignas(16) cpuRegisters cpuRegs_;
alignas(16) fpuRegisters fpuRegs_;
alignas(16) psxRegisters psxRegs_;

cpuRegisters& cpuRegs = cpuRegs_;
fpuRegisters& fpuRegs = fpuRegs_;
psxRegisters& psxRegs = psxRegs_;
bool iopIsDelaySlot_;
bool& iopIsDelaySlot = iopIsDelaySlot_;

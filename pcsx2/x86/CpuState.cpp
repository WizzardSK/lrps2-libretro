#include "common/Pcsx2Types.h"
#include "R5900.h"

alignas(16) cpuRegisters cpuRegs_;
alignas(16) fpuRegisters fpuRegs_;

cpuRegisters& cpuRegs = cpuRegs_;
fpuRegisters& fpuRegs = fpuRegs_;

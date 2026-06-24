// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// Interpreter-only arm64 build: the x86 VIF dynarec (newVif_Dynarec.cpp) is not
// compiled, but the VIF unpack hot path (Vif_Unpack.cpp / MTVU.cpp) still calls
// dVifReset/dVifUnpack. Provide no-op stubs so the core links and boots; a real
// arm64 VIF unpack is future work, so VIF-dynarec paths are degraded for now.

#include "common/Pcsx2Types.h"

void dVifReset(int idx) {}

template <int idx>
void dVifUnpack(const u8* data, bool isFill) {}

template void dVifUnpack<0>(const u8* data, bool isFill);
template void dVifUnpack<1>(const u8* data, bool isFill);

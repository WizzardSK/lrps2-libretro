// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
//
// Interpreter-only arm64 build: the x86 VIF dynarec (newVif_Dynarec.cpp) is not
// compiled. Rather than no-op'ing the VIF unpack hot path (which silently
// dropped all VU data), route dVifUnpack to _nVifUnpack -- the portable,
// scalar reference unpack that the SSE path itself falls back to when VU memory
// wraps (see newVif_Dynarec.cpp). This unpacks VIF data correctly on arm64,
// just without the recompiled fast path. A real arm64 VIF recompiler is future
// work.

#include "../Common.h"
#include "../Vif.h"
#include "../Vif_Dma.h"
#include "../Vif_Dynarec.h"
#include "../MTVU.h"

// No recompiler block cache / write pointer on arm64 (dVifUnpack goes straight
// to the reference path), so the per-idx reset has nothing to do here.
void dVifReset(int idx) {}

template <int idx>
void dVifUnpack(const u8* data, bool isFill)
{
	VIFregisters& vifRegs = MTVU_VifXRegs;
	_nVifUnpack(idx, data, vifRegs.mode, isFill);
}

template void dVifUnpack<0>(const u8* data, bool isFill);
template void dVifUnpack<1>(const u8* data, bool isFill);

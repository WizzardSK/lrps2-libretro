// SPDX-FileCopyrightText: lrps2 arm64 port
// SPDX-License-Identifier: LGPL-3.0+
// Stub of upstream common/Perf.h (perf-map JIT symbol registration) for code
// transplanted from armsx2; the libretro fork has no perf integration.
#pragma once
#include "Pcsx2Defs.h"
namespace Perf
{
	struct Group
	{
		void Register(const void*, size_t, const char*, ...) {}
		void RegisterPC(const void*, size_t, u32) {}
		void RegisterKey(const void*, size_t, const char*, u64) {}
	};
	extern Group any, ee, iop, vu, vu0, vu1, vif;
}

// SPDX-License-Identifier: LGPL-3.0+
//
// Sampling profiler for the arm64 JIT bring-up (LRPS2_PROF=1).
//
// perf(1) is not available on the target, and the emulator's hot code lives in
// three anonymous mmap'd code caches (EE, IOP, microVU), so a normal profiler
// would attribute all of it to one nameless address range anyway. This samples
// the PC of the threads we care about with a per-thread CPU-time timer and
// buckets each sample by the code cache it landed in; everything else is
// resolved to a symbol with dladdr() when the run ends.
//
// Off unless LRPS2_PROF is set: no timer is armed and no samples are taken.
// LRPS2_PROF_HZ (default 1000) sets the per-thread sampling rate.

#pragma once

#include <cstddef>

namespace ArmProf
{
	bool Enabled();

	// Name a code cache so samples inside it are attributed to it by name.
	void RegisterRegion(const char* name, const void* base, size_t size);

	// Arm the sampling timer for the calling thread. Idempotent per thread.
	void AttachThread(const char* name);

	// Tell the profiler which guest block a freshly compiled chunk of a code
	// cache belongs to, so samples inside it can be reported per guest PC.
	// Cheap no-op unless profiling is on. Later notes win over earlier ones,
	// which is what we want when a block is recompiled into reused cache space.
	void NoteBlock(const char* region, const void* host_start, size_t host_size, unsigned guest_pc);

	// Print the breakdown. Idempotent; also runs from atexit().
	void Report();
} // namespace ArmProf

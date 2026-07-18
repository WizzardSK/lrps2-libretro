// TEMP diagnostic (LRPS2_SYNC_STATS=1): where does the EE thread's WALL time go?
//
// The sampling profiler measures per-thread CPU time, so it is blind to blocking:
// it happily reports EE 39 % / GS 37 % / MTVU 24 % while the process as a whole
// only gets ~1.2 cores out of three busy threads. That gap is the interesting
// part -- the three threads barely overlap -- and finding it needs wall time at
// the points where one thread waits for another.
#pragma once

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <ctime>

struct SyncCounter
{
	const char*        name;
	std::atomic<double> seconds{0.0};
	std::atomic<long>   count{0};
};

extern SyncCounter g_sync_waitgs;   // EE waits for the GS thread to drain the frame
extern SyncCounter g_sync_waitvu;   // EE waits for the MTVU thread to go idle
extern SyncCounter g_sync_xgkick;   // GS waits for MTVU to hand over an XGKICK packet

bool SyncStatsEnabled();

struct SyncStat
{
	SyncCounter& c;
	double       t0;

	explicit SyncStat(SyncCounter& counter) : c(counter), t0(0.0)
	{
		if (!SyncStatsEnabled())
			return;
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		t0 = ts.tv_sec + ts.tv_nsec * 1e-9;
	}

	~SyncStat()
	{
		if (!SyncStatsEnabled())
			return;
		struct timespec ts;
		clock_gettime(CLOCK_MONOTONIC, &ts);
		const double dt = (ts.tv_sec + ts.tv_nsec * 1e-9) - t0;
		double cur = c.seconds.load(std::memory_order_relaxed);
		while (!c.seconds.compare_exchange_weak(cur, cur + dt, std::memory_order_relaxed))
			;
		c.count.fetch_add(1, std::memory_order_relaxed);
	}
};

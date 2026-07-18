#include "SyncStats_arm64.h"

SyncCounter g_sync_waitgs{"EE waits for GS (vsync drain)"};
SyncCounter g_sync_waitvu{"EE waits for MTVU (WaitVU)"};
SyncCounter g_sync_xgkick{"GS waits for MTVU (XGKICK)"};

bool SyncStatsEnabled()
{
	static const bool on = getenv("LRPS2_SYNC_STATS") != nullptr;
	return on;
}

namespace
{
	struct SyncReport
	{
		~SyncReport()
		{
			if (!SyncStatsEnabled())
				return;
			const SyncCounter* all[] = {&g_sync_waitgs, &g_sync_waitvu, &g_sync_xgkick};
			fprintf(stderr, "[sync] blocking (wall seconds):\n");
			for (const SyncCounter* c : all)
				fprintf(stderr, "[sync]   %-32s %7.2f s over %ld waits\n",
					c->name, c->seconds.load(), c->count.load());
		}
	};
	SyncReport s_report;
}

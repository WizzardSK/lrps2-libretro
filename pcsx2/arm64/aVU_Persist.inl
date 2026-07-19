// SPDX-FileCopyrightText: 2002-2026 PCSX2 Dev Team
// SPDX-License-Identifier: GPL-3.0+

// Persisted-JIT VU program cache — emit-time recorder (phase 1: capture only).
//
// While recording is enabled (LRPS2_VU_PROGCACHE=1, default off), every aVU
// emission episode (the region emitted between the armSetAsmPtr/armStartBlock
// open in mVUblockFetch's cold path and the matching armEndBlock) is captured
// as a "chunk" on the owning microProgram's persist log: the raw code bytes
// plus the blocks whose entry points live inside it.
//
// Phase 1 records chunks and blocks but performs no address classification
// yet, so every log is marked non-persistable; the value is the plumbing and
// the stats (chunk/block counts per program observable on real games via
// LRPS2_VU_PROGCACHE_STATS=1). The fixup recorder, serializer and hydration
// land on top of this. Reference design: yaps2 microVU_Persist-arm64.
//
// Fail-safe invariant carried over from the reference: anything the recorder
// cannot account for marks the episode non-persistable and drops it; dropped
// state can only ever cause a fallback to a normal recompile, never wrong
// code.

#include <cstdlib>
#include <unordered_map>
#include <vector>

namespace aVUPersist
{
	struct PersistChunk
	{
		std::vector<u8> code;
		// Fixups land here in phase 2.
	};

	struct PersistBlockRec
	{
		u32 chunkIndex;
		u32 entryOffset; // within chunk
		u32 startPC;     // microMem byte offset
		microRegInfo pState;
		microRegInfo pStateEnd;
		const microBlock* live; // manager's copy; not serialized
	};

} // namespace aVUPersist

// Global-scope definition (fwd-declared in aVU.h so microProgram can hold a
// pointer without pulling the implementation types).
struct MvuPersistLog
{
	std::vector<aVUPersist::PersistChunk> chunks;
	std::vector<aVUPersist::PersistBlockRec> blocks;
	// host entry -> blocks[] index, for phase-2 cross-chunk branch resolution.
	std::unordered_map<const void*, u32> blockByEntry;
};

namespace aVUPersist
{
	struct Stats
	{
		u64 episodes = 0;
		u64 chunksRecorded = 0;
		u64 chunksDropped = 0;
		u64 blocksRecorded = 0;
		u64 orphanBlocks = 0; // block registered with no episode open
	};

	static bool s_enabled = false;
	static bool s_statsDump = false;
	static bool s_initDone = false;
	static Stats s_stats[2];

	// Per-VU in-flight episode state. Episodes never nest: the cold path in
	// mVUblockFetch opens exactly one MacroAssembler session and the
	// recursive mVUcompile calls append into it.
	struct Episode
	{
		bool open = false;
		bool dropped = false;
		u8* base = nullptr;
		microProgram* prog = nullptr;
	};
	static Episode s_episode[2];

	static void Init()
	{
		if (s_initDone)
			return;
		s_initDone = true;
		const char* env = getenv("LRPS2_VU_PROGCACHE");
		s_enabled = (env && env[0] == '1');
		const char* stats = getenv("LRPS2_VU_PROGCACHE_STATS");
		s_statsDump = (stats && stats[0] == '1');
	}

	__fi bool Enabled()
	{
		return s_enabled;
	}

	// mVUblockFetch cold path bound armAsm at `base`: begin a chunk.
	static void BeginEpisode(microVU& mVU, u8* base)
	{
		Init();
		if (!s_enabled)
			return;
		Episode& ep = s_episode[mVU.index];
		ep.open = true;
		ep.dropped = false;
		ep.base = base;
		ep.prog = nullptr;
		s_stats[mVU.index].episodes++;
	}

	// armEndBlock finalized at `end`: capture the chunk onto the owning
	// program's log, or account a drop. Empty episodes (search hit, nothing
	// emitted) are ignored.
	static void EndEpisode(microVU& mVU, u8* end)
	{
		if (!s_enabled)
			return;
		Episode& ep = s_episode[mVU.index];
		if (!ep.open)
			return;
		ep.open = false;
		if (end == ep.base)
			return; // nothing emitted
		if (ep.dropped || !ep.prog || !ep.prog->persist)
		{
			s_stats[mVU.index].chunksDropped++;
			return;
		}
		MvuPersistLog& log = *ep.prog->persist;
		// Blocks recorded during the episode carry entry offsets relative to
		// ep.base under chunkIndex == chunks.size(); commit the code bytes.
		PersistChunk& chunk = log.chunks.emplace_back();
		chunk.code.assign(ep.base, end);
		s_stats[mVU.index].chunksRecorded++;
	}

	// microBlockManager::add registered `block` (manager's stable copy) with
	// entry `entry`. Called for every block, recording or not, so the episode
	// can learn its owning program (mVU.prog.cur is not yet final there).
	static void OnBlockAdded(microVU& mVU, microProgram* prog, const microBlock* block, u8* entry, u32 startPC_bytes)
	{
		if (!s_enabled)
			return;
		Episode& ep = s_episode[mVU.index];
		if (!ep.open)
		{
			s_stats[mVU.index].orphanBlocks++;
			return;
		}
		if (!ep.prog)
		{
			ep.prog = prog;
			if (!prog->persist)
				prog->persist = new MvuPersistLog();
		}
		else if (ep.prog != prog)
		{
			// One episode spanning two programs would make chunk ownership
			// ambiguous; drop it (never observed, but fail safe).
			ep.dropped = true;
			return;
		}
		if (entry < ep.base)
		{
			// Entry before the episode base = a re-registered block whose code
			// lives in an earlier chunk; the manager dedup path handles it.
			return;
		}
		MvuPersistLog& log = *prog->persist;
		PersistBlockRec& rec = log.blocks.emplace_back();
		rec.chunkIndex = static_cast<u32>(log.chunks.size()); // current in-flight chunk
		rec.entryOffset = static_cast<u32>(entry - ep.base);
		rec.startPC = startPC_bytes;
		rec.pState = block->pState;
		rec.pStateEnd = block->pStateEnd;
		rec.live = block;
		log.blockByEntry[entry] = static_cast<u32>(log.blocks.size() - 1);
		s_stats[mVU.index].blocksRecorded++;
	}

	static void OnProgramDeleted(microProgram& prog)
	{
		delete prog.persist;
		prog.persist = nullptr;
	}

	static void DumpStats(u32 vuIndex)
	{
		if (!s_statsDump)
			return;
		const Stats& st = s_stats[vuIndex];
		Console.WriteLn("aVUPersist[VU%u]: episodes=%llu chunks=%llu dropped=%llu blocks=%llu orphans=%llu",
			vuIndex, (unsigned long long)st.episodes, (unsigned long long)st.chunksRecorded,
			(unsigned long long)st.chunksDropped, (unsigned long long)st.blocksRecorded,
			(unsigned long long)st.orphanBlocks);
	}
} // namespace aVUPersist

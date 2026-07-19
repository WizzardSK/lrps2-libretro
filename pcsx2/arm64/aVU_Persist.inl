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
#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <dlfcn.h>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aVUPersist
{
	// A baked address the persisted code cannot use verbatim in another run.
	// The class picks how hydration re-derives it: image/arena/code deltas are
	// applied as a base shift, block absolutes point into the freshly-allocated
	// block graph, ADRP pages are re-paged. `kFixUnclassifiable` never lands in
	// a serialized chunk — it marks the whole episode non-persistable.
	// Which memory region a baked address lands in — picks the rebase delta at
	// hydration (each region loads at its own ASLR base in a future process).
	enum TargetClass : u8
	{
		kClassImage = 0, // the core .so image incl. its bss statics (microVU/vuRegs)
		kClassArena,     // the VU code-cache arena (stubs + cross-chunk code)
		kClassBlock,     // a microBlock object of this program (heap)
		kClassVuMem,     // this VU's Mem/Micro buffer (separately mmap'd)
	};

	// How the address is encoded in the instruction stream — picks the patch.
	enum RelocForm : u8
	{
		kFormMovzMovk = 0, // canonical movz+movk*3 quartet at codeOffset
		kFormAdrp,         // ADRP (page) at codeOffset; low-12 offset is invariant
		kFormBranch,       // B/BL imm26 at codeOffset (code target, arena or image)
	};

	struct PersistFixup
	{
		u32 codeOffset;  // chunk-relative byte offset of the patched insn
		u8 form;         // RelocForm
		u8 targetClass;  // TargetClass
		u16 _pad;
		u64 target;      // resolved absolute (debug aid + hydration input)
	};

	struct PersistChunk
	{
		std::vector<u8> code;
		std::vector<PersistFixup> fixups;
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
	// Cleared the first time a chunk holds an unclassifiable baked address; a
	// non-persistable program is never serialized (its chunk indices stay valid
	// for the blocks that reference them, so we keep the chunks in place).
	bool persistable = true;
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
		// Fixup classification (phase 2 measurement).
		u64 fixImage = 0;
		u64 fixArena = 0;
		u64 fixBlock = 0;
		u64 fixAdrp = 0;
		u64 fixBranchArena = 0;
		u64 fixBranchImage = 0;
		u64 fixVuMem = 0;
		u64 fixUnclassifiable = 0; // mapped pointers we could not place -> drops
	};

	// Run-invariant-under-rebase ranges, resolved once per process.
	struct Ranges
	{
		uptr imgBegin = 0, imgEnd = 0; // the core .so's mapped image
		std::vector<std::pair<uptr, uptr>> mapped; // every mapping, sorted by begin
		bool valid = false;
	};
	static Ranges s_ranges;

	// True if `v` points into any current process mapping. A movz/movk that
	// materializes a value outside every mapping is a run-invariant constant
	// (mask, immediate, packed field), not an address — it needs no fixup and
	// must not drop the episode. A value inside a mapping is a real pointer.
	static bool IsMapped(uptr v)
	{
		const auto& m = s_ranges.mapped;
		// binary search: last range whose begin <= v
		size_t lo = 0, hi = m.size();
		while (lo < hi)
		{
			size_t mid = (lo + hi) / 2;
			if (m[mid].first <= v)
				lo = mid + 1;
			else
				hi = mid;
		}
		return lo > 0 && v < m[lo - 1].second;
	}

	// Resolve the core .so image span from /proc/self/maps using dladdr to find
	// any address inside it (a function of this TU). Best-effort: on failure the
	// scanner treats would-be image absolutes as unclassifiable, which only
	// costs cache coverage, never correctness.
	static void ResolveRanges()
	{
		if (s_ranges.valid)
			return;
		s_ranges.valid = true; // attempt once regardless of outcome
		Dl_info info;
		if (!dladdr(reinterpret_cast<const void*>(&ResolveRanges), &info) || !info.dli_fbase)
			return;
		const uptr anchor = reinterpret_cast<uptr>(info.dli_fbase);
		std::FILE* maps = std::fopen("/proc/self/maps", "r");
		if (!maps)
			return;
		// First pass: find the pathname of the mapping holding the anchor.
		char line[512];
		char anchorPath[256] = {};
		while (std::fgets(line, sizeof(line), maps))
		{
			unsigned long long b = 0, e = 0;
			int off = 0;
			if (std::sscanf(line, "%llx-%llx %*s %*s %*s %*s %n", &b, &e, &off) < 2)
				continue;
			if (anchor >= (uptr)b && anchor < (uptr)e)
			{
				const char* p = line + off;
				const char* nl = std::strchr(p, '\n');
				size_t len = nl ? (size_t)(nl - p) : std::strlen(p);
				if (len >= sizeof(anchorPath))
					len = sizeof(anchorPath) - 1;
				std::memcpy(anchorPath, p, len);
				anchorPath[len] = 0;
				break;
			}
		}
		// Second pass: union every mapping of that same file (text/rodata/data/bss)
		// for the image span, record every mapping for the pointer test, and find
		// the mapping that holds &microVU0 (a .so bss static) so the bss can be
		// folded into the image span below.
		const uptr bssAnchor = reinterpret_cast<uptr>(&microVU0);
		uptr bssBegin = 0, bssEnd = 0;
		std::rewind(maps);
		while (std::fgets(line, sizeof(line), maps))
		{
			unsigned long long b = 0, e = 0;
			int off = 0;
			if (std::sscanf(line, "%llx-%llx %*s %*s %*s %*s %n", &b, &e, &off) < 2)
				continue;
			s_ranges.mapped.emplace_back((uptr)b, (uptr)e);
			if (bssAnchor >= (uptr)b && bssAnchor < (uptr)e)
			{
				bssBegin = (uptr)b;
				bssEnd = (uptr)e;
			}
			const char* p = line + off;
			if (anchorPath[0] && (p[0] == '/' || p[0] == '[') && std::strncmp(p, anchorPath, std::strlen(anchorPath)) == 0)
			{
				if (s_ranges.imgBegin == 0 || (uptr)b < s_ranges.imgBegin)
					s_ranges.imgBegin = (uptr)b;
				if ((uptr)e > s_ranges.imgEnd)
					s_ranges.imgEnd = (uptr)e;
			}
		}
		std::fclose(maps);
		// The .so's .bss is the anonymous mapping directly after its last file-
		// backed segment; static globals (microVU0/microVU1, vuRegs, the VU
		// register files, emit-constant tables) live there and rebase with the
		// same load delta as the image. Fold it into the image span only when it
		// is contiguous with the image (so a stray heap/arena mapping cannot be
		// mistaken for it).
		if (bssBegin && bssBegin <= s_ranges.imgEnd && bssEnd > s_ranges.imgEnd)
			s_ranges.imgEnd = bssEnd;
		std::sort(s_ranges.mapped.begin(), s_ranges.mapped.end());
	}

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
		// Force canonical fixed-length address materializations so every baked
		// absolute occupies a re-encodable slot for hydration. Identical values,
		// so guest output is unchanged.
		g_armCanonicalAddrForms = true;
		s_stats[mVU.index].episodes++;
	}

	// True if `v` points inside one of this program's recorded microBlock
	// objects (its &pState = the object base, since pState is the first member).
	static bool IsBlockAbs(const MvuPersistLog& log, uptr v)
	{
		for (const PersistBlockRec& b : log.blocks)
		{
			const uptr base = reinterpret_cast<uptr>(b.live);
			if (v >= base && v < base + sizeof(microBlock))
				return true;
		}
		return false;
	}

	// Decode the chunk's ARM64 stream and classify every baked address into a
	// fixup. movz/movk immediate materializations, ADRP pages, and B/BL targets
	// are the only run-variant forms the aVU emitter produces; anything whose
	// resolved value falls outside the known image / arena / block ranges marks
	// the episode non-persistable (returns false). `chunkBase` is the address
	// the code was emitted at (needed to resolve PC-relative operands).
	static bool ScanChunkForFixups(microVU& mVU, const MvuPersistLog& log, PersistChunk& chunk, uptr chunkBase)
	{
		ResolveRanges();
		Stats& st = s_stats[mVU.index];
		const uptr arenaBegin = reinterpret_cast<uptr>(mVU.cache);
		const uptr arenaEnd = reinterpret_cast<uptr>(mVU.prog.codeReserveEnd);
		const uptr chunkEnd = chunkBase + chunk.code.size();
		// This VU's memory + microprogram buffers (VURegs::Mem/Micro are pointers
		// to separately-mmap'd regions the JIT bakes absolute addresses into).
		const uptr vuMemBegin = reinterpret_cast<uptr>(mVU.regs().Mem);
		const uptr vuMemEnd = vuMemBegin + mVU.vuMemSize;
		const uptr vuMicroBegin = reinterpret_cast<uptr>(mVU.regs().Micro);
		const uptr vuMicroEnd = vuMicroBegin + mVU.microMemSize;

		bool ok = true;
		// Place a resolved absolute into a target class; -1 = run-invariant
		// constant (no fixup), -2 = mapped but unplaceable (drop the episode).
		auto classify = [&](uptr v) -> int {
			if (v == 0)
				return -1;
			if (s_ranges.imgBegin && v >= s_ranges.imgBegin && v < s_ranges.imgEnd)
				return kClassImage;
			if (v >= arenaBegin && v < arenaEnd)
				return kClassArena;
			if (IsBlockAbs(log, v))
				return kClassBlock;
			if ((v >= vuMemBegin && v < vuMemEnd) || (v >= vuMicroBegin && v < vuMicroEnd))
				return kClassVuMem;
			if (!IsMapped(v))
				return -1; // constant (mask/immediate), not an address
			return -2;     // mapped but unplaced
		};
		auto bumpClass = [&](int cls) {
			switch (cls)
			{
				case kClassImage: st.fixImage++; break;
				case kClassArena: st.fixArena++; break;
				case kClassBlock: st.fixBlock++; break;
				case kClassVuMem: st.fixVuMem++; break;
			}
		};
		auto classifyAbs = [&](u32 off, uptr v) {
			const int cls = classify(v);
			if (cls == -1)
				return;
			if (cls == -2)
			{
				st.fixUnclassifiable++; ok = false;
				return;
			}
			chunk.fixups.push_back({off, (u8)kFormMovzMovk, (u8)cls, 0, v});
			bumpClass(cls);
		};

		const u8* code = chunk.code.data();
		const size_t n = chunk.code.size() / 4;
		int curReg = -1;
		uptr curVal = 0;
		u32 curOff = 0;
		for (size_t i = 0; i < n; i++)
		{
			u32 insn;
			std::memcpy(&insn, code + i * 4, 4);
			const u32 off = static_cast<u32>(i * 4);
			const int rd = insn & 0x1F;
			const bool isMovz = (insn & 0x7F800000u) == 0x52800000u; // 32/64-bit MOVZ
			const bool isMovk = (insn & 0x7F800000u) == 0x72800000u; // 32/64-bit MOVK
			const bool is64 = (insn >> 31) & 1;

			if (isMovz)
			{
				if (curReg >= 0)
					classifyAbs(curOff, curVal); // previous sequence ended
				const u32 hw = (insn >> 21) & 0x3;
				const u64 imm = (insn >> 5) & 0xFFFF;
				curReg = rd;
				curVal = static_cast<uptr>(imm << (hw * 16));
				curOff = off;
				(void)is64;
				continue;
			}
			if (isMovk && curReg == rd)
			{
				const u32 hw = (insn >> 21) & 0x3;
				const u64 imm = (insn >> 5) & 0xFFFF;
				curVal &= ~(static_cast<uptr>(0xFFFF) << (hw * 16));
				curVal |= static_cast<uptr>(imm << (hw * 16));
				continue;
			}
			// Any other instruction terminates a pending mov-immediate sequence.
			if (curReg >= 0)
			{
				classifyAbs(curOff, curVal);
				curReg = -1;
			}

			// ADRP: op | immlo(2) | 10000 | immhi(19) | Rd. Page target must land
			// in a rebasable region (the low-12 offset in the paired add/ldr is
			// invariant); classify it so hydration picks the right delta.
			if ((insn & 0x9F000000u) == 0x90000000u)
			{
				const s64 immlo = (insn >> 29) & 0x3;
				s64 immhi = (insn >> 5) & 0x7FFFF;
				if (immhi & 0x40000) immhi |= ~0x7FFFFll; // sign-extend 19 bits
				const s64 page = (immhi << 2) | immlo;
				const uptr pc = chunkBase + off;
				const uptr target = (pc & ~static_cast<uptr>(0xFFF)) + static_cast<uptr>(page << 12);
				const int cls = classify(target);
				if (cls == -1)
					continue; // page of a constant pool that moves with the chunk? leave it
				if (cls == -2)
				{ st.fixUnclassifiable++; ok = false; continue; }
				chunk.fixups.push_back({off, (u8)kFormAdrp, (u8)cls, 0, target});
				st.fixAdrp++;
				continue;
			}
			// B (0x14000000) / BL (0x94000000): imm26 << 2, PC-relative.
			if ((insn & 0x7C000000u) == 0x14000000u)
			{
				s64 imm26 = insn & 0x03FFFFFF;
				if (imm26 & 0x02000000) imm26 |= ~0x03FFFFFFll; // sign-extend
				const uptr pc = chunkBase + off;
				const uptr target = pc + static_cast<uptr>(imm26 << 2);
				if (target >= chunkBase && target < chunkEnd)
					continue; // intra-chunk: invariant when the chunk moves as a unit
				const int cls = classify(target);
				if (cls == kClassArena || cls == kClassImage)
				{ chunk.fixups.push_back({off, (u8)kFormBranch, (u8)cls, 0, target});
				  (cls == kClassArena ? st.fixBranchArena : st.fixBranchImage)++; continue; }
				st.fixUnclassifiable++; ok = false;
				continue;
			}
		}
		if (curReg >= 0)
			classifyAbs(curOff, curVal);
		return ok;
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
		g_armCanonicalAddrForms = false;
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
		if (!ScanChunkForFixups(mVU, log, chunk, reinterpret_cast<uptr>(ep.base)))
		{
			// Unclassifiable baked address: this chunk is not safely relocatable,
			// so the whole program becomes non-persistable. Keep the chunk in
			// place (its index is referenced by recorded blocks); serialization
			// (phase 3) refuses a non-persistable log.
			log.persistable = false;
			s_stats[mVU.index].chunksDropped++;
			return;
		}
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

	//------------------------------------------------------------------
	// Serialization (phase 3): a program's recorded block graph -> a flat,
	// content-addressed image. Layout bases (image / arena / per-block heap)
	// are stored so hydration can rebase the fixups by delta.
	//------------------------------------------------------------------

	static constexpr u32 kImageMagic = 0x32555641; // 'AVU2'
	static constexpr u32 kImageVersion = 1;

	struct ImageHeader
	{
		u32 magic;
		u32 version;
		u32 vuIndex;
		u32 startPC;      // microMem byte offset of the creating entry
		u64 contentHash;  // hash of the guest microprogram this was compiled from
		u64 imageBase;    // s_ranges.imgBegin at record time
		u64 arenaBase;    // mVU.cache at record time
		u32 blockCount;
		u32 chunkCount;
		u64 payloadHash;  // hash of every byte after this header (bit-rot guard)
	};
	static_assert(sizeof(ImageHeader) == 56);

	struct DiskBlock
	{
		u32 chunkIndex;
		u32 entryOffset;
		u32 startPC;
		u32 _pad;
		u64 liveBase; // record-time address of the microBlock (for kFixBlockAbs rebase)
		u8 pState[sizeof(microRegInfo)];
		u8 pStateEnd[sizeof(microRegInfo)];
	};

	static u64 HashBytes(const void* data, size_t len, u64 seed = 1469598103934665603ull)
	{
		const u8* p = static_cast<const u8*>(data);
		u64 h = seed;
		for (size_t i = 0; i < len; i++)
		{
			h ^= p[i];
			h *= 1099511628211ull;
		}
		return h;
	}

	template <typename T>
	static void AppendPod(std::vector<u8>& out, const T& v)
	{
		const u8* p = reinterpret_cast<const u8*>(&v);
		out.insert(out.end(), p, p + sizeof(T));
	}

	// Serialize `prog`'s recorded graph into `out`. Returns false when the
	// program has no persistable log or recorded nothing.
	static bool SerializeProgram(microVU& mVU, const microProgram& prog, std::vector<u8>& out)
	{
		const MvuPersistLog* log = prog.persist;
		if (!log || !log->persistable || log->blocks.empty() || log->chunks.empty())
			return false;

		out.clear();
		ImageHeader hdr = {};
		hdr.magic = kImageMagic;
		hdr.version = kImageVersion;
		hdr.vuIndex = mVU.index;
		hdr.startPC = static_cast<u32>(prog.startPC) * 8u;
		hdr.contentHash = HashBytes(prog.data, sizeof(prog.data));
		hdr.imageBase = s_ranges.imgBegin;
		hdr.arenaBase = reinterpret_cast<u64>(mVU.cache);
		hdr.blockCount = static_cast<u32>(log->blocks.size());
		hdr.chunkCount = static_cast<u32>(log->chunks.size());
		AppendPod(out, hdr); // payloadHash filled in after the body

		for (const PersistBlockRec& b : log->blocks)
		{
			DiskBlock db = {};
			db.chunkIndex = b.chunkIndex;
			db.entryOffset = b.entryOffset;
			db.startPC = b.startPC;
			db.liveBase = reinterpret_cast<u64>(b.live);
			std::memcpy(db.pState, &b.pState, sizeof(microRegInfo));
			std::memcpy(db.pStateEnd, &b.pStateEnd, sizeof(microRegInfo));
			AppendPod(out, db);
		}
		for (const PersistChunk& c : log->chunks)
		{
			const u32 codeSize = static_cast<u32>(c.code.size());
			const u32 fixupCount = static_cast<u32>(c.fixups.size());
			AppendPod(out, codeSize);
			AppendPod(out, fixupCount);
			out.insert(out.end(), c.code.begin(), c.code.end());
			for (const PersistFixup& f : c.fixups)
				AppendPod(out, f);
		}

		// Fill payloadHash over everything after the header.
		const u64 ph = HashBytes(out.data() + sizeof(ImageHeader), out.size() - sizeof(ImageHeader));
		std::memcpy(out.data() + offsetof(ImageHeader, payloadHash), &ph, sizeof(ph));
		return true;
	}

	// Parse a serialized image and verify it structurally round-trips against
	// the live `log` it was produced from: header sanity, payload hash, and
	// byte-exact chunk code + fixup records. Returns true on full agreement.
	// This proves the format is complete and lossless before any hydration
	// consumes it. Test-only (LRPS2_VU_PROGCACHE_SELFTEST).
	static bool VerifyRoundTrip(const std::vector<u8>& img, const MvuPersistLog& log)
	{
		if (img.size() < sizeof(ImageHeader))
			return false;
		ImageHeader hdr;
		std::memcpy(&hdr, img.data(), sizeof(hdr));
		if (hdr.magic != kImageMagic || hdr.version != kImageVersion)
			return false;
		if (hdr.blockCount != log.blocks.size() || hdr.chunkCount != log.chunks.size())
			return false;
		if (HashBytes(img.data() + sizeof(ImageHeader), img.size() - sizeof(ImageHeader)) != hdr.payloadHash)
			return false;

		size_t pos = sizeof(ImageHeader);
		for (u32 i = 0; i < hdr.blockCount; i++)
		{
			if (pos + sizeof(DiskBlock) > img.size())
				return false;
			DiskBlock db;
			std::memcpy(&db, img.data() + pos, sizeof(db));
			pos += sizeof(db);
			const PersistBlockRec& b = log.blocks[i];
			if (db.chunkIndex != b.chunkIndex || db.entryOffset != b.entryOffset || db.startPC != b.startPC)
				return false;
			if (std::memcmp(db.pState, &b.pState, sizeof(microRegInfo)) != 0)
				return false;
		}
		for (u32 i = 0; i < hdr.chunkCount; i++)
		{
			if (pos + 8 > img.size())
				return false;
			u32 codeSize, fixupCount;
			std::memcpy(&codeSize, img.data() + pos, 4);
			std::memcpy(&fixupCount, img.data() + pos + 4, 4);
			pos += 8;
			const PersistChunk& c = log.chunks[i];
			if (codeSize != c.code.size() || fixupCount != c.fixups.size())
				return false;
			if (pos + codeSize > img.size() || std::memcmp(img.data() + pos, c.code.data(), codeSize) != 0)
				return false;
			pos += codeSize;
			for (u32 j = 0; j < fixupCount; j++)
			{
				if (pos + sizeof(PersistFixup) > img.size())
					return false;
				PersistFixup f;
				std::memcpy(&f, img.data() + pos, sizeof(f));
				pos += sizeof(f);
				if (f.codeOffset != c.fixups[j].codeOffset || f.form != c.fixups[j].form || f.targetClass != c.fixups[j].targetClass || f.target != c.fixups[j].target)
					return false;
			}
		}
		return pos == img.size();
	}

	// Walk every recorded program and round-trip serialize/verify it. Called
	// from mVUclose before programs are freed. Gated by
	// LRPS2_VU_PROGCACHE_SELFTEST; counts pass/fail into the stats.
	static u64 s_selftestPass[2] = {};
	static u64 s_selftestFail[2] = {};
	static void RoundTripSelfTest(microVU& mVU)
	{
		if (!s_enabled)
			return;
		static const bool run = getenv("LRPS2_VU_PROGCACHE_SELFTEST") != nullptr;
		if (!run)
			return;
		for (u32 i = 0; i < (mVU.progSize / 2); i++)
		{
			if (!mVU.prog.prog[i])
				continue;
			for (microProgram* p : *mVU.prog.prog[i])
			{
				if (!p || !p->persist || !p->persist->persistable || p->persist->blocks.empty())
					continue;
				std::vector<u8> img;
				if (SerializeProgram(mVU, *p, img) && VerifyRoundTrip(img, *p->persist))
					s_selftestPass[mVU.index]++;
				else
					s_selftestFail[mVU.index]++;
			}
		}
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
		Console.WriteLn("aVUPersist[VU%u]: fixups img=%llu arena=%llu block=%llu vumem=%llu adrp=%llu bra=%llu bimg=%llu UNCLASS=%llu",
			vuIndex, (unsigned long long)st.fixImage, (unsigned long long)st.fixArena,
			(unsigned long long)st.fixBlock, (unsigned long long)st.fixVuMem, (unsigned long long)st.fixAdrp,
			(unsigned long long)st.fixBranchArena, (unsigned long long)st.fixBranchImage,
			(unsigned long long)st.fixUnclassifiable);
		if (s_selftestPass[vuIndex] || s_selftestFail[vuIndex])
			Console.WriteLn("aVUPersist[VU%u]: round-trip selftest pass=%llu fail=%llu",
				vuIndex, (unsigned long long)s_selftestPass[vuIndex],
				(unsigned long long)s_selftestFail[vuIndex]);
	}
} // namespace aVUPersist

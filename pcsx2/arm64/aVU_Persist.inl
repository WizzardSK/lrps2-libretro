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
#include <dirent.h>
#include <dlfcn.h>
#include <link.h>
#include <unistd.h>
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
		kFormCondBranch,   // B.cond / CBZ / CBNZ imm19 (bits 23:5), +/-1 MB
		kFormTestBranch,   // TBZ / TBNZ imm14 (bits 18:5), +/-32 KB
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
		uptr recordBase = 0; // arena address the chunk was emitted at (record run)
	};

	struct PersistBlockRec
	{
		u32 chunkIndex;
		u32 entryOffset; // within chunk
		u32 startPC;     // microMem byte offset
		microRegInfo pState;
		microRegInfo pStateEnd;
		const microBlock* live; // manager's copy; not serialized (jumpCache read from it at serialize)
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
	// Content key captured at compile time from the live full micro memory (the
	// program's own, at dispatch), so a lookup before recompile recomputes the
	// same value — the cached copy prog.data only holds the program's range.
	u64 contentHash = 0;
	bool hasHash = false;
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
		u64 fixCondBranch = 0; // cross-chunk B.cond/CBZ/CBNZ/TBZ/TBNZ
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
	static bool s_hydrateEnabled = false;
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
		const char* hyd = getenv("LRPS2_VU_PROGCACHE_HYDRATE");
		s_hydrateEnabled = (hyd && hyd[0] == '1');
	}

	__fi bool Enabled()
	{
		return s_enabled;
	}

	// Defined later; needed by the recorder to key programs at compile time.
	static u64 HashBytes(const void* data, size_t len, u64 seed);

	// Identity of the running core build, folded from the .so's GNU build-id
	// note. Baked image offsets are only meaningful inside the exact build that
	// recorded them (the hydration rebase shifts every image reference by one
	// delta, assuming an identical layout), so images written by any other
	// build must be rejected. 0 when the note is absent; such builds only
	// accept caches that were also written without one.
	struct BuildIdCtx
	{
		uptr base;
		u64 id;
	};

	static int BuildIdPhdrCb(struct dl_phdr_info* info, size_t, void* data)
	{
		BuildIdCtx* ctx = static_cast<BuildIdCtx*>(data);
		if (info->dlpi_addr != ctx->base)
			return 0;
		for (int i = 0; i < info->dlpi_phnum; i++)
		{
			const ElfW(Phdr)& ph = info->dlpi_phdr[i];
			if (ph.p_type != PT_NOTE)
				continue;
			const u8* p = reinterpret_cast<const u8*>(info->dlpi_addr + ph.p_vaddr);
			const u8* end = p + ph.p_memsz;
			while (p + sizeof(ElfW(Nhdr)) <= end)
			{
				const ElfW(Nhdr)* nh = reinterpret_cast<const ElfW(Nhdr)*>(p);
				const u8* name = p + sizeof(ElfW(Nhdr));
				const u8* desc = name + ((nh->n_namesz + 3) & ~3u);
				if (desc + nh->n_descsz > end)
					break;
				if (nh->n_type == NT_GNU_BUILD_ID && nh->n_namesz == 4 && !std::memcmp(name, "GNU", 4))
				{
					ctx->id = HashBytes(desc, nh->n_descsz, 1469598103934665603ull);
					return 1;
				}
				p = desc + ((nh->n_descsz + 3) & ~3u);
			}
		}
		return 1; // reached our module and found no note: stop, id stays 0
	}

	static u64 OwnBuildId()
	{
		static const u64 s_id = []() -> u64 {
			Dl_info info;
			if (!dladdr(reinterpret_cast<const void*>(&ResolveRanges), &info) || !info.dli_fbase)
				return 0;
			BuildIdCtx ctx{reinterpret_cast<uptr>(info.dli_fbase), 0};
			dl_iterate_phdr(&BuildIdPhdrCb, &ctx);
			return ctx.id;
		}();
		return s_id;
	}

	// Accessors for the live-hydration glue in aVU.cpp (mVUtryHydrate).
	static u64 s_hydrated[2] = {};
	__fi bool HydrateEnabled() { Init(); return s_hydrateEnabled; }
	__fi void ResolveRangesPublic() { ResolveRanges(); }
	__fi u64 CurrentImageBegin() { return s_ranges.imgBegin; }
	__fi void NoteHydrated(u32 vu) { s_hydrated[vu]++; }

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
			// Every remaining PC-relative form. Intra-chunk targets need no fixup
			// (the distance is invariant when the chunk moves as a unit), but a
			// target outside the chunk must be re-encoded at hydration, because
			// chunks recorded far apart are packed contiguously there.
			const uptr pc = chunkBase + off;
			uptr target = 0;
			u8 form = 0;
			if ((insn & 0x7C000000u) == 0x14000000u) // B / BL: imm26 << 2
			{
				s64 imm26 = insn & 0x03FFFFFF;
				if (imm26 & 0x02000000) imm26 |= ~0x03FFFFFFll; // sign-extend
				target = pc + static_cast<uptr>(imm26 << 2);
				form = kFormBranch;
			}
			// B.cond (0x54000000) / CBZ|CBNZ (0x34000000): imm19 at bits 23:5.
			// Both share the encoding slot, so one patch form covers them.
			else if ((insn & 0xFF000010u) == 0x54000000u || (insn & 0x7E000000u) == 0x34000000u)
			{
				s64 imm19 = (insn >> 5) & 0x7FFFF;
				if (imm19 & 0x40000) imm19 |= ~0x7FFFFll;
				target = pc + static_cast<uptr>(imm19 << 2);
				form = kFormCondBranch;
			}
			// TBZ / TBNZ (0x36000000): imm14 at bits 18:5.
			else if ((insn & 0x7E000000u) == 0x36000000u)
			{
				s64 imm14 = (insn >> 5) & 0x3FFF;
				if (imm14 & 0x2000) imm14 |= ~0x3FFFll;
				target = pc + static_cast<uptr>(imm14 << 2);
				form = kFormTestBranch;
			}
			// ADR (0x10000000) and LDR-literal (0x18000000) are PC-relative forms
			// the aVU emitter is not known to produce. There is no patch for them,
			// so an escaping one must not be silently persisted: drop the episode.
			else if ((insn & 0x9F000000u) == 0x10000000u || (insn & 0x3B000000u) == 0x18000000u)
			{
				s64 imm = (insn >> 5) & 0x7FFFF;
				if (imm & 0x40000) imm |= ~0x7FFFFll;
				const uptr t = ((insn & 0x9F000000u) == 0x10000000u)
					? pc + static_cast<uptr>((imm << 2) | ((insn >> 29) & 0x3))
					: pc + static_cast<uptr>(imm << 2);
				if (t < chunkBase || t >= chunkEnd)
				{ st.fixUnclassifiable++; ok = false; }
				continue;
			}
			else
				continue;

			if (target >= chunkBase && target < chunkEnd)
				continue; // intra-chunk: invariant when the chunk moves as a unit
			const int cls = classify(target);
			if (cls != kClassArena && cls != kClassImage)
			{ st.fixUnclassifiable++; ok = false; continue; }
			chunk.fixups.push_back({off, form, (u8)cls, 0, target});
			if (form == kFormBranch)
				(cls == kClassArena ? st.fixBranchArena : st.fixBranchImage)++;
			else
				st.fixCondBranch++;
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
		chunk.recordBase = reinterpret_cast<uptr>(ep.base);
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
			if (!prog->persist->hasHash)
			{
				// Key on the live full micro memory now (compile time = the
				// program's own micro memory) so a future lookup recomputes it.
				const u32 entryPC = static_cast<u32>(prog->startPC) * 8u;
				prog->persist->contentHash = HashBytes(mVU.regs().Micro, mVU.microMemSize,
					HashBytes(&entryPC, sizeof(entryPC), 1469598103934665603ull));
				prog->persist->hasHash = true;
			}
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
	static constexpr u32 kImageVersion = 6; // v6: cross-chunk conditional branches
	                                        // (B.cond/CBZ/TBZ) are recorded as fixups.
	                                        // v5 images have none, so their multi-chunk
	                                        // programs mis-execute once chunks are packed.
	                                        // v5: header carries the writer's buildId
	                                        // (image offsets don't survive a relink).
	                                        // v4: recorded code no longer routes far
	                                        // calls/jumps through per-process trampolines
	                                        // (see armEmitCall/armEmitJmp); v3 images bake
	                                        // trampoline slots that fault after hydration.

	struct ImageHeader
	{
		u32 magic;
		u32 version;
		u32 vuIndex;
		u32 startPC;      // microMem byte offset of the creating entry
		u64 contentHash;  // hash of the guest microprogram this was compiled from
		u64 codegenFingerprint; // hash of every runtime config flag the microVU codegen
		                  // reads while emitting (MTVU, VU clamp/sync/I-bit/XGKICK hacks,
		                  // EE cycle rate). A block baked under one fingerprint is not
		                  // executable under another, so hydration must reject a mismatch.
		u64 buildId;      // OwnBuildId() of the writing core (folded GNU build-id);
		                  // image offsets don't survive a relink, reject any other build
		u64 imageBase;    // s_ranges.imgBegin at record time
		u64 arenaBase;    // mVU.cache at record time
		u64 memBase;      // mVU.regs().Mem at record time
		u64 microBase;    // mVU.regs().Micro at record time
		u32 blockCount;
		u32 chunkCount;
		u64 payloadHash;  // hash of every byte after this header (bit-rot guard)
	};
	static_assert(sizeof(ImageHeader) == 88);

	struct DiskBlock
	{
		u32 chunkIndex;
		u32 entryOffset;
		u32 startPC;
		u32 flags; // bit0: block ends in JR/JALR (needs a jumpCache array on hydrate)
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

	// Every runtime config flag the microVU codegen branches on while emitting a
	// block. A .vuprog baked under one combination bakes a different VU1 interrupt
	// path (MTVU vs instant), different clamp/overflow code and different sync
	// cycles, so it cannot be executed under another combination — hydrating a
	// mismatched cache deadlocks the MTVU worker. Fold this into both the cache
	// header and the file name so mismatched flavours are rejected and coexist.
	static u64 CodegenFingerprint()
	{
		const auto& R = EmuConfig.Cpu.Recompiler;
		const auto& S = EmuConfig.Speedhacks;
		const auto& G = EmuConfig.Gamefixes;
		u8 f[20] = {};
		f[0]  = (u8)R.EnableVU1;       // } together = THREAD_VU1 (MTVU): flips the VU1
		f[1]  = (u8)S.vuThread;        // }   interrupt-signalling path in every block
		f[2]  = (u8)S.vu1Instant;      // INSTANT_VU1
		f[3]  = (u8)G.IbitHack;
		f[4]  = (u8)G.VUSyncHack;
		f[5]  = (u8)G.XgKickHack;
		f[6]  = (u8)G.VUOverflowHack;
		f[7]  = (u8)G.FullVU0SyncHack;
		f[8]  = (u8)R.vu0Overflow;
		f[9]  = (u8)R.vu1Overflow;
		f[10] = (u8)R.vu0ExtraOverflow;
		f[11] = (u8)R.vu1ExtraOverflow;
		f[12] = (u8)R.vu0SignOverflow;
		f[13] = (u8)R.vu1SignOverflow;
		f[14] = (u8)S.EECycleRate;
		f[15] = (u8)S.EECycleSkip;
		return HashBytes(f, sizeof(f));
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
		// The content key was captured at compile time from the live full micro
		// memory (mVUcacheProg only snapshots the program's range into prog.data,
		// so hashing prog.data here would not match a lookup over live memory).
		hdr.contentHash = log->contentHash;
		hdr.codegenFingerprint = CodegenFingerprint();
		hdr.buildId = OwnBuildId();
		hdr.imageBase = s_ranges.imgBegin;
		hdr.arenaBase = reinterpret_cast<u64>(mVU.cache);
		hdr.memBase = reinterpret_cast<u64>(mVU.regs().Mem);
		hdr.microBase = reinterpret_cast<u64>(mVU.regs().Micro);
		hdr.blockCount = static_cast<u32>(log->blocks.size());
		hdr.chunkCount = static_cast<u32>(log->chunks.size());
		AppendPod(out, hdr); // payloadHash filled in after the body

		for (const PersistBlockRec& b : log->blocks)
		{
			DiskBlock db = {};
			db.chunkIndex = b.chunkIndex;
			db.entryOffset = b.entryOffset;
			db.startPC = b.startPC;
			// jumpCache is allocated mid-compile (after OnBlockAdded ran), so read the
			// final state off the manager's live block at serialize time, not the
			// stale add-time snapshot.
			db.flags = (b.live && b.live->jumpCache) ? 1u : 0u;
			db.liveBase = reinterpret_cast<u64>(b.live);
			std::memcpy(db.pState, &b.pState, sizeof(microRegInfo));
			std::memcpy(db.pStateEnd, &b.pStateEnd, sizeof(microRegInfo));
			AppendPod(out, db);
		}
		for (const PersistChunk& c : log->chunks)
		{
			const u32 codeSize = static_cast<u32>(c.code.size());
			const u32 fixupCount = static_cast<u32>(c.fixups.size());
			const u64 recordBase = c.recordBase;
			AppendPod(out, codeSize);
			AppendPod(out, fixupCount);
			AppendPod(out, recordBase);
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
		if (hdr.magic != kImageMagic || hdr.version != kImageVersion || hdr.buildId != OwnBuildId())
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
			if (pos + 16 > img.size())
				return false;
			u32 codeSize, fixupCount;
			std::memcpy(&codeSize, img.data() + pos, 4);
			std::memcpy(&fixupCount, img.data() + pos + 4, 4);
			pos += 16; // codeSize + fixupCount + recordBase(u64)
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

	// Defined in the on-disk store section below.
	static const char* CacheDir();
	static void MakeCachePath(char* out, size_t cap, u32 vuIndex, u64 hash, u64 fingerprint);

	// A parsed serialized image, ready for hydration. Mirrors the on-disk layout
	// (header + blocks + chunks) in live vectors.
	struct ParsedImage
	{
		ImageHeader hdr = {};
		std::vector<DiskBlock> blocks;
		std::vector<PersistChunk> chunks; // code + fixups + recordBase per chunk
	};

	// Parse + integrity-check a serialized image into `out`. Returns false on any
	// malformed field or a payload-hash mismatch (caller falls back to recompile).
	static bool ParseImage(const std::vector<u8>& img, ParsedImage& out)
	{
		if (img.size() < sizeof(ImageHeader))
			return false;
		std::memcpy(&out.hdr, img.data(), sizeof(ImageHeader));
		if (out.hdr.magic != kImageMagic || out.hdr.version != kImageVersion || out.hdr.buildId != OwnBuildId())
			return false;
		if (HashBytes(img.data() + sizeof(ImageHeader), img.size() - sizeof(ImageHeader)) != out.hdr.payloadHash)
			return false;
		size_t pos = sizeof(ImageHeader);
		out.blocks.resize(out.hdr.blockCount);
		for (u32 i = 0; i < out.hdr.blockCount; i++)
		{
			if (pos + sizeof(DiskBlock) > img.size())
				return false;
			std::memcpy(&out.blocks[i], img.data() + pos, sizeof(DiskBlock));
			pos += sizeof(DiskBlock);
		}
		out.chunks.resize(out.hdr.chunkCount);
		for (u32 i = 0; i < out.hdr.chunkCount; i++)
		{
			if (pos + 16 > img.size())
				return false;
			u32 codeSize, fixupCount;
			u64 recordBase;
			std::memcpy(&codeSize, img.data() + pos, 4);
			std::memcpy(&fixupCount, img.data() + pos + 4, 4);
			std::memcpy(&recordBase, img.data() + pos + 8, 8);
			pos += 16;
			PersistChunk& c = out.chunks[i];
			c.recordBase = recordBase;
			if (pos + codeSize > img.size())
				return false;
			c.code.assign(img.data() + pos, img.data() + pos + codeSize);
			pos += codeSize;
			c.fixups.resize(fixupCount);
			for (u32 j = 0; j < fixupCount; j++)
			{
				if (pos + sizeof(PersistFixup) > img.size())
					return false;
				std::memcpy(&c.fixups[j], img.data() + pos, sizeof(PersistFixup));
				pos += sizeof(PersistFixup);
			}
		}
		return pos == img.size();
	}

	// Compute the content-address key for the program that would be compiled at
	// `startPC_bytes` from the live guest micro memory — the same key
	// SerializeProgram writes into the header.
	static u64 LookupKey(microVU& mVU, u32 startPC_bytes)
	{
		return HashBytes(mVU.regs().Micro, mVU.microMemSize,
			HashBytes(&startPC_bytes, sizeof(startPC_bytes), 1469598103934665603ull));
	}

	// Read the .vuprog for (this VU, startPC) if one exists and parses cleanly,
	// and its content key matches the live micro memory. Returns true + fills
	// `out`. No-op (false) unless hydration is enabled with a cache dir.
	static bool TryReadProgram(microVU& mVU, u32 startPC_bytes, ParsedImage& out)
	{
		if (!s_hydrateEnabled || !CacheDir())
			return false;
		const u64 key = LookupKey(mVU, startPC_bytes);
		const u64 fpr = CodegenFingerprint();
		char path[512];
		MakeCachePath(path, sizeof(path), mVU.index, key, fpr);
		static const bool dbg = getenv("LRPS2_VU_PROGCACHE_DUMP") != nullptr;
		std::FILE* fp = std::fopen(path, "rb");
		if (!fp)
		{ if (dbg) Console.WriteLn("TryReadProgram: no file %s", path); return false; }
		std::fseek(fp, 0, SEEK_END);
		long sz = std::ftell(fp);
		std::fseek(fp, 0, SEEK_SET);
		std::vector<u8> img(sz > 0 ? sz : 0);
		const bool okr = sz > 0 && std::fread(img.data(), 1, sz, fp) == (size_t)sz;
		std::fclose(fp);
		if (!okr || !ParseImage(img, out))
		{ if (dbg) Console.WriteLn("TryReadProgram: parse fail %s", path); return false; }
		// Guard against a hash collision: the recorded key must match and the
		// entry PC must be the one we are looking up.
		const bool m = out.hdr.contentHash == key && out.hdr.startPC == startPC_bytes
			&& out.hdr.vuIndex == (u32)mVU.index && out.hdr.codegenFingerprint == fpr;
		if (dbg && !m) Console.WriteLn("TryReadProgram: mismatch hdrHash=%llx key=%llx hdrPC=%x wantPC=%x hdrFp=%llx wantFp=%llx",
			(unsigned long long)out.hdr.contentHash, (unsigned long long)key, out.hdr.startPC, startPC_bytes,
			(unsigned long long)out.hdr.codegenFingerprint, (unsigned long long)fpr);
		return m;
	}

	//------------------------------------------------------------------
	// On-disk store (phase 3c): persist serialized programs to a per-VU cache
	// directory (LRPS2_VU_PROGCACHE_DIR) so a future process can hydrate them.
	// Content-addressed by the guest-microprogram hash. This step writes and
	// verifies (parse + payload-hash) across processes; live hydration wiring is
	// the next step.
	//------------------------------------------------------------------

	static u64 s_diskSaved[2] = {};
	static u64 s_diskLoaded[2] = {};
	static u64 s_diskRejected[2] = {};

	static const char* CacheDir()
	{
		static const char* dir = getenv("LRPS2_VU_PROGCACHE_DIR");
		return dir;
	}

	static void MakeCachePath(char* out, size_t cap, u32 vuIndex, u64 hash, u64 fingerprint)
	{
		// fingerprint segregates codegen flavours (e.g. MTVU on vs off) into distinct
		// files so a config toggle between launches does not overwrite the other's cache.
		std::snprintf(out, cap, "%s/vu%u_%016llx_%016llx.vuprog", CacheDir(), vuIndex,
			(unsigned long long)hash, (unsigned long long)fingerprint);
	}

	// Serialize `prog` and write it to the cache dir (tmp + rename for atomicity).
	static void SaveProgramToDisk(microVU& mVU, const microProgram& prog)
	{
		if (!CacheDir())
			return;
		std::vector<u8> img;
		if (!SerializeProgram(mVU, prog, img))
			return;
		u64 hash, fpr;
		std::memcpy(&hash, img.data() + offsetof(ImageHeader, contentHash), sizeof(hash));
		std::memcpy(&fpr, img.data() + offsetof(ImageHeader, codegenFingerprint), sizeof(fpr));
		char path[512], tmp[540];
		MakeCachePath(path, sizeof(path), mVU.index, hash, fpr);
		std::snprintf(tmp, sizeof(tmp), "%s.tmp%d", path, (int)getpid());
		std::FILE* fp = std::fopen(tmp, "wb");
		if (!fp)
			return;
		const bool okw = std::fwrite(img.data(), 1, img.size(), fp) == img.size();
		std::fclose(fp);
		if (okw && std::rename(tmp, path) == 0)
			s_diskSaved[mVU.index]++;
		else
			std::remove(tmp);
	}

	static void SaveAllToDisk(microVU& mVU)
	{
		if (!s_enabled || !CacheDir())
			return;
		for (u32 i = 0; i < (mVU.progSize / 2); i++)
		{
			if (!mVU.prog.prog[i])
				continue;
			for (microProgram* p : *mVU.prog.prog[i])
				if (p && p->persist && p->persist->persistable && !p->persist->blocks.empty())
					SaveProgramToDisk(mVU, *p);
		}
	}

	// Read one .vuprog and check it is a well-formed, uncorrupted image (magic,
	// version, payload hash). Cross-process proof that the store survives a
	// restart. Returns true if the image is loadable.
	static bool LoadAndVerifyImage(const char* path)
	{
		std::FILE* fp = std::fopen(path, "rb");
		if (!fp)
			return false;
		std::fseek(fp, 0, SEEK_END);
		long sz = std::ftell(fp);
		std::fseek(fp, 0, SEEK_SET);
		if (sz < (long)sizeof(ImageHeader))
		{ std::fclose(fp); return false; }
		std::vector<u8> img(sz);
		const bool okr = std::fread(img.data(), 1, sz, fp) == (size_t)sz;
		std::fclose(fp);
		if (!okr)
			return false;
		ImageHeader hdr;
		std::memcpy(&hdr, img.data(), sizeof(hdr));
		if (hdr.magic != kImageMagic || hdr.version != kImageVersion || hdr.buildId != OwnBuildId())
			return false;
		return HashBytes(img.data() + sizeof(ImageHeader), img.size() - sizeof(ImageHeader)) == hdr.payloadHash;
	}

	// Scan the cache dir at startup and verify every vu<index>_*.vuprog. Counts
	// loaded/rejected for the stats. Called once per VU from mVUinit.
	static void LoadDiskCache(u32 vuIndex)
	{
		Init();
		if (!s_enabled || !CacheDir())
			return;
		DIR* d = opendir(CacheDir());
		if (!d)
			return;
		char prefix[16];
		std::snprintf(prefix, sizeof(prefix), "vu%u_", vuIndex);
		struct dirent* e;
		while ((e = readdir(d)) != nullptr)
		{
			if (std::strncmp(e->d_name, prefix, std::strlen(prefix)) != 0)
				continue;
			char path[512];
			std::snprintf(path, sizeof(path), "%s/%s", CacheDir(), e->d_name);
			if (LoadAndVerifyImage(path))
				s_diskLoaded[vuIndex]++;
			else
				s_diskRejected[vuIndex]++;
		}
		closedir(d);
	}

	//------------------------------------------------------------------
	// Rebase primitives (phase 3b-2): rewrite a baked address in a copy of the
	// chunk so it points at the target's new location. The canonical forms make
	// each an in-place edit of a fixed instruction slot.
	//------------------------------------------------------------------

	// Rewrite the canonical movz+movk*3 quartet at `off` to materialize `newVal`
	// into the same Rd the original used.
	static void PatchMovImm(u8* code, u32 off, u64 newVal)
	{
		u32 first;
		std::memcpy(&first, code + off, 4);
		const u32 rd = first & 0x1F;
		const u32 q[4] = {
			0xD2800000u | (0u << 21) | ((u32)((newVal >> 0) & 0xFFFF) << 5) | rd,  // movz
			0xF2800000u | (1u << 21) | ((u32)((newVal >> 16) & 0xFFFF) << 5) | rd, // movk 16
			0xF2800000u | (2u << 21) | ((u32)((newVal >> 32) & 0xFFFF) << 5) | rd, // movk 32
			0xF2800000u | (3u << 21) | ((u32)((newVal >> 48) & 0xFFFF) << 5) | rd, // movk 48
		};
		std::memcpy(code + off, q, sizeof(q));
	}

	// Re-page the ADRP at `off` (whose runtime pc is `pcAtOff`) to `newTarget`.
	// The low-12 offset in the paired add/ldr is invariant. False if the new page
	// displacement does not fit ADRP's signed 21-bit range.
	static bool PatchAdrp(u8* code, u32 off, uptr pcAtOff, uptr newTarget)
	{
		const s64 disp = ((s64)(newTarget & ~(uptr)0xFFF) - (s64)(pcAtOff & ~(uptr)0xFFF)) >> 12;
		if (disp < -(1LL << 20) || disp >= (1LL << 20))
			return false;
		u32 insn;
		std::memcpy(&insn, code + off, 4);
		const u32 immlo = (u32)(disp & 0x3);
		const u32 immhi = (u32)((disp >> 2) & 0x7FFFF);
		insn = (insn & 0x9F00001Fu) | (immlo << 29) | (immhi << 5);
		std::memcpy(code + off, &insn, 4);
		return true;
	}

	// Re-target the B/BL at `off` (runtime pc `pcAtOff`) to `newTarget`. False if
	// out of imm26 (+/-128 MB) range.
	static bool PatchBranch(u8* code, u32 off, uptr pcAtOff, uptr newTarget)
	{
		const s64 disp = ((s64)newTarget - (s64)pcAtOff) >> 2;
		if (disp < -(1LL << 25) || disp >= (1LL << 25))
			return false;
		u32 insn;
		std::memcpy(&insn, code + off, 4);
		insn = (insn & 0xFC000000u) | ((u32)disp & 0x03FFFFFFu);
		std::memcpy(code + off, &insn, 4);
		return true;
	}

	// Re-target a conditional branch at `off` (runtime pc `pcAtOff`). `imm19` picks
	// the field: B.cond/CBZ/CBNZ carry imm19 at bits 23:5 (+/-1 MB), TBZ/TBNZ carry
	// imm14 at bits 18:5 (+/-32 KB). False if the new displacement does not fit --
	// hydration then falls back to a normal recompile.
	static bool PatchCondBranch(u8* code, u32 off, uptr pcAtOff, uptr newTarget, bool imm19)
	{
		const s64 disp = ((s64)newTarget - (s64)pcAtOff) >> 2;
		const s64 lim = imm19 ? (1LL << 18) : (1LL << 13);
		if (disp < -lim || disp >= lim)
			return false;
		const u32 bits = imm19 ? 19u : 14u;
		const u32 field = (u32)((1u << bits) - 1u);
		u32 insn;
		std::memcpy(&insn, code + off, 4);
		insn = (insn & ~(field << 5)) | (((u32)disp & field) << 5);
		std::memcpy(code + off, &insn, 4);
		return true;
	}

	// Decode helpers to read a materialized value back for verification.
	static u64 DecodeMovImm(const u8* code, u32 off)
	{
		u64 v = 0;
		for (u32 k = 0; k < 4; k++)
		{
			u32 insn;
			std::memcpy(&insn, code + off + k * 4, 4);
			const u32 hw = (insn >> 21) & 0x3;
			const u64 imm = (insn >> 5) & 0xFFFF;
			v |= imm << (hw * 16);
		}
		return v;
	}
	static uptr DecodeAdrpPage(const u8* code, u32 off, uptr pcAtOff)
	{
		u32 insn;
		std::memcpy(&insn, code + off, 4);
		const s64 immlo = (insn >> 29) & 0x3;
		s64 immhi = (insn >> 5) & 0x7FFFF;
		if (immhi & 0x40000) immhi |= ~0x7FFFFll;
		const s64 page = (immhi << 2) | immlo;
		return (pcAtOff & ~(uptr)0xFFF) + (uptr)(page << 12);
	}
	static uptr DecodeBranch(const u8* code, u32 off, uptr pcAtOff)
	{
		u32 insn;
		std::memcpy(&insn, code + off, 4);
		s64 imm26 = insn & 0x03FFFFFF;
		if (imm26 & 0x02000000) imm26 |= ~0x03FFFFFFll;
		return pcAtOff + (uptr)(imm26 << 2);
	}
	static uptr DecodeCondBranch(const u8* code, u32 off, uptr pcAtOff, bool imm19)
	{
		u32 insn;
		std::memcpy(&insn, code + off, 4);
		const u32 bits = imm19 ? 19u : 14u;
		s64 imm = (insn >> 5) & ((1u << bits) - 1u);
		if (imm & (1ll << (bits - 1))) imm |= ~((1ll << bits) - 1ll);
		return pcAtOff + (uptr)(imm << 2);
	}

	// Copy a chunk's code into `dst` (placed at runtime address `dstBase`) and
	// rewrite each fixup to the target `resolve` returns for it. `resolve`
	// receives the fixup and yields the address the fixup should now point at.
	template <typename Resolve>
	static bool RebaseChunkInto(u8* dst, uptr dstBase, const PersistChunk& chunk, Resolve resolve)
	{
		std::memcpy(dst, chunk.code.data(), chunk.code.size());
		for (const PersistFixup& f : chunk.fixups)
		{
			const uptr newTarget = resolve(f);
			const uptr pcAtOff = dstBase + f.codeOffset;
			switch (f.form)
			{
				case kFormMovzMovk: PatchMovImm(dst, f.codeOffset, newTarget); break;
				case kFormAdrp: if (!PatchAdrp(dst, f.codeOffset, pcAtOff, newTarget)) return false; break;
				case kFormBranch: if (!PatchBranch(dst, f.codeOffset, pcAtOff, newTarget)) return false; break;
				case kFormCondBranch:
				case kFormTestBranch:
					if (!PatchCondBranch(dst, f.codeOffset, pcAtOff, newTarget, f.form == kFormCondBranch))
						return false;
					break;
				default: return false;
			}
		}
		return true;
	}

	// In-process rebase self-test: copy each chunk into a scratch buffer at a
	// *different* address than it was recorded at, rewrite every fixup to its
	// unchanged target (delta 0, but a fresh placement), then decode the patched
	// instructions and confirm they resolve to that target. This proves the
	// re-encode math is placement-independent — the property hydration relies on
	// — without touching the live code cache. Returns true on full agreement.
	static bool RebaseSelfTest(const MvuPersistLog& log)
	{
		for (const PersistChunk& c : log.chunks)
		{
			// Rewrite into a heap copy but reckon addresses as if the chunk sat at
			// a page-shifted-but-still-in-range placement (the code moved, the
			// targets did not). The ADRP/branch operands are recomputed from this
			// virtual base; decoding them must recover the unchanged targets — the
			// placement-independence hydration needs. A small shift keeps every
			// PC-relative operand inside its encodable range.
			const uptr virtBase = c.recordBase + 0x10000;
			std::vector<u8> dst = c.code;
			if (!RebaseChunkInto(dst.data(), virtBase, c, [](const PersistFixup& f) { return (uptr)f.target; }))
				return false;
			for (const PersistFixup& f : c.fixups)
			{
				const uptr pcAtOff = virtBase + f.codeOffset;
				uptr got = 0;
				switch (f.form)
				{
					case kFormMovzMovk: got = (uptr)DecodeMovImm(dst.data(), f.codeOffset); break;
					case kFormAdrp: got = DecodeAdrpPage(dst.data(), f.codeOffset, pcAtOff); break;
					case kFormBranch: got = DecodeBranch(dst.data(), f.codeOffset, pcAtOff); break;
					case kFormCondBranch:
					case kFormTestBranch:
						got = DecodeCondBranch(dst.data(), f.codeOffset, pcAtOff, f.form == kFormCondBranch);
						break;
				}
				const uptr want = (f.form == kFormAdrp) ? (f.target & ~(uptr)0xFFF) : (uptr)f.target;
				if (got != want)
					return false;
			}
		}
		return true;
	}

	//------------------------------------------------------------------
	// Hydration resolve (phase 3b-2b): map a fixup's recorded target to the
	// address it must point at in this run, given where everything now lives.
	//------------------------------------------------------------------
	struct HydrationPlan
	{
		s64 imageDelta = 0;   // newImageBase - record imageBase
		u64 recMemBase = 0, newMemBase = 0, memSize = 0;
		u64 recMicroBase = 0, newMicroBase = 0, microSize = 0;
		u64 recArenaBase = 0, newArenaBase = 0;
		std::vector<uptr> chunkRecordBase; // per chunk, record-run address
		std::vector<uptr> chunkNewBase;    // per chunk, this-run address
		std::vector<uptr> chunkSize;       // per chunk, code size in bytes
		std::vector<std::pair<uptr, uptr>> blockRemap; // old microBlock base -> new
	};

	// Resolve one fixup's target under a plan.
	static uptr ResolveFixup(const HydrationPlan& p, const PersistFixup& f)
	{
		const uptr V = static_cast<uptr>(f.target);
		switch (f.targetClass)
		{
			case kClassImage:
				return static_cast<uptr>(static_cast<s64>(V) + p.imageDelta);
			case kClassVuMem:
				if (V >= p.recMemBase && V < p.recMemBase + p.memSize)
					return p.newMemBase + (V - p.recMemBase);
				return p.newMicroBase + (V - p.recMicroBase);
			case kClassBlock:
				for (const auto& m : p.blockRemap)
					if (V >= m.first && V < m.first + sizeof(microBlock))
						return m.second + (V - m.first);
				return V; // unreachable: recorded block absolutes are always known
			case kClassArena:
			default:
				// Cross-chunk code of this program moves to its new placement;
				// a dispatcher stub elsewhere in the arena shifts by the arena delta.
				for (size_t i = 0; i < p.chunkRecordBase.size(); i++)
				{
					const uptr rb = p.chunkRecordBase[i];
					if (V >= rb && V < rb + p.chunkSize[i])
						return p.chunkNewBase[i] + (V - rb);
				}
				return V + (p.newArenaBase - p.recArenaBase);
		}
	}

	// Walk every recorded program and round-trip serialize/verify it. Called
	// from mVUclose before programs are freed. Gated by
	// LRPS2_VU_PROGCACHE_SELFTEST; counts pass/fail into the stats.
	static u64 s_selftestPass[2] = {};
	static u64 s_selftestFail[2] = {};
	static u64 s_rebasePass[2] = {};
	static u64 s_rebaseFail[2] = {};
	static u64 s_hydratePass[2] = {};
	static u64 s_hydrateFail[2] = {};

	// End-to-end hydration self-test exercising the full resolve. Builds a plan
	// that moves the program's chunks to a fresh (shifted, in-range) placement
	// and remaps its blocks to fresh addresses — so cross-chunk arena refs and
	// block absolutes take *non-zero* rebases even in-process — then rewrites
	// each chunk through ResolveFixup and decodes every fixup to confirm it
	// resolves to the planned target. Image/VuMem deltas are zero in-process
	// (same bases), so this isolates the placement/remap logic; the disk path
	// reuses the same ResolveFixup with real cross-process deltas.
	static bool HydrateSelfTest(microVU& mVU, const MvuPersistLog& log)
	{
		HydrationPlan p;
		p.imageDelta = 0;
		p.recMemBase = p.newMemBase = reinterpret_cast<u64>(mVU.regs().Mem);
		p.memSize = mVU.vuMemSize;
		p.recMicroBase = p.newMicroBase = reinterpret_cast<u64>(mVU.regs().Micro);
		p.microSize = mVU.microMemSize;
		p.recArenaBase = p.newArenaBase = reinterpret_cast<u64>(mVU.cache);

		// Place the chunks contiguously at a shifted-but-in-range virtual base.
		uptr cursor = (log.chunks.empty() ? 0 : log.chunks[0].recordBase) + 0x20000;
		for (const PersistChunk& c : log.chunks)
		{
			p.chunkRecordBase.push_back(c.recordBase);
			p.chunkNewBase.push_back(cursor);
			p.chunkSize.push_back(c.code.size());
			cursor += (c.code.size() + 15) & ~(uptr)15;
		}
		// Remap each recorded block to a fresh synthetic address (a distinct
		// region so block absolutes visibly differ from their originals).
		std::vector<std::vector<u8>> newBlockStorage;
		newBlockStorage.reserve(log.blocks.size());
		for (const PersistBlockRec& b : log.blocks)
		{
			newBlockStorage.emplace_back(sizeof(microBlock));
			p.blockRemap.emplace_back(reinterpret_cast<uptr>(b.live),
				reinterpret_cast<uptr>(newBlockStorage.back().data()));
		}

		for (size_t ci = 0; ci < log.chunks.size(); ci++)
		{
			const PersistChunk& c = log.chunks[ci];
			const uptr newBase = p.chunkNewBase[ci];
			std::vector<u8> dst = c.code;
			if (!RebaseChunkInto(dst.data(), newBase, c, [&](const PersistFixup& f) { return ResolveFixup(p, f); }))
				return false;
			for (const PersistFixup& f : c.fixups)
			{
				const uptr pcAtOff = newBase + f.codeOffset;
				const uptr want = ResolveFixup(p, f);
				uptr got = 0;
				switch (f.form)
				{
					case kFormMovzMovk: got = (uptr)DecodeMovImm(dst.data(), f.codeOffset); break;
					case kFormAdrp: got = DecodeAdrpPage(dst.data(), f.codeOffset, pcAtOff); break;
					case kFormBranch: got = DecodeBranch(dst.data(), f.codeOffset, pcAtOff); break;
					case kFormCondBranch:
					case kFormTestBranch:
						got = DecodeCondBranch(dst.data(), f.codeOffset, pcAtOff, f.form == kFormCondBranch);
						break;
				}
				const uptr wantCmp = (f.form == kFormAdrp) ? (want & ~(uptr)0xFFF) : want;
				if (got != wantCmp)
					return false;
			}
		}
		return true;
	}

	static u64 s_residualHits[2] = {};

	// Independent missed-relocation detector. Unlike HydrateSelfTest (which only
	// checks that RECORDED fixups re-encode correctly), this rebases each chunk
	// with *nonzero* deltas for every class and then re-decodes EVERY address the
	// code materializes, flagging any that still points into a record-time region.
	// A correctly-relocated address moves out of the record range into its shifted
	// new range; one that stays put is an address the scanner never turned into a
	// fixup (misclassified as a constant — e.g. via the stale IsMapped snapshot —
	// or emitted in a form the scanner does not recognize). Those are exactly the
	// pointers that fault after a real cross-process hydration. Writes findings to
	// `out` (may be null). Returns the number of residuals found.
	static u32 ResidualScan(microVU& mVU, const microProgram& prog, const MvuPersistLog& log, std::FILE* out)
	{
		ResolveRanges();
		const uptr imgB = s_ranges.imgBegin, imgE = s_ranges.imgEnd;
		const uptr arB = reinterpret_cast<uptr>(mVU.cache);
		const uptr arE = reinterpret_cast<uptr>(mVU.prog.codeReserveEnd);
		const uptr memB = reinterpret_cast<uptr>(mVU.regs().Mem), memE = memB + mVU.vuMemSize;
		const uptr micB = reinterpret_cast<uptr>(mVU.regs().Micro), micE = micB + mVU.microMemSize;

		// Deltas large enough that a relocated address never lands back in its own
		// record range (each span is << its shift), and within ADRP/branch reach.
		HydrationPlan p;
		p.imageDelta   = 0x40000000;                       // +1 GiB (image refs: movz/adrp)
		p.recMemBase   = memB; p.newMemBase   = memB + 0x02000000; p.memSize   = mVU.vuMemSize;
		p.recMicroBase = micB; p.newMicroBase = micB + 0x04000000; p.microSize = mVU.microMemSize;
		p.recArenaBase = arB;  p.newArenaBase = arB + 0x00400000;  // +4 MiB (dispatcher stubs)
		for (const PersistChunk& c : log.chunks)
		{
			p.chunkRecordBase.push_back(c.recordBase);
			p.chunkNewBase.push_back(c.recordBase + 0x00400000);
			p.chunkSize.push_back(c.code.size());
		}
		std::vector<std::vector<u8>> blockStore;
		blockStore.reserve(log.blocks.size());
		for (const PersistBlockRec& b : log.blocks)
		{
			blockStore.emplace_back(sizeof(microBlock));
			p.blockRemap.emplace_back(reinterpret_cast<uptr>(b.live),
				reinterpret_cast<uptr>(blockStore.back().data()));
		}
		auto inRec = [&](uptr v) -> const char* {
			if (imgB && v >= imgB && v < imgE) return "IMAGE";
			if (v >= arB && v < arE) return "ARENA";
			if (v >= memB && v < memE) return "VUMEM";
			if (v >= micB && v < micE) return "VUMICRO";
			for (const PersistBlockRec& b : log.blocks)
			{ const uptr lb = reinterpret_cast<uptr>(b.live);
			  if (v >= lb && v < lb + sizeof(microBlock)) return "BLOCK"; }
			return nullptr;
		};
		// A FRESH read of the live mappings (not the stale ResolveRanges snapshot):
		// any unchanged pointer into a live mapping is a baked address that never got
		// relocated, even into a region no fixup class tracks (e.g. a GS/MTGS buffer
		// mmap'd after the snapshot) — exactly the misclassified-as-constant bug.
		std::vector<std::pair<uptr, uptr>> live;
		if (std::FILE* mf = std::fopen("/proc/self/maps", "r"))
		{
			char ln[512];
			while (std::fgets(ln, sizeof(ln), mf))
			{
				unsigned long long b, e;
				if (std::sscanf(ln, "%llx-%llx", &b, &e) == 2)
					live.emplace_back((uptr)b, (uptr)e);
			}
			std::fclose(mf);
			std::sort(live.begin(), live.end());
		}
		auto liveMapped = [&](uptr v) -> bool {
			size_t lo = 0, hi = live.size();
			while (lo < hi) { size_t m = (lo + hi) / 2; if (live[m].first <= v) lo = m + 1; else hi = m; }
			return lo > 0 && v < live[lo - 1].second;
		};

		u32 hits = 0;
		for (size_t ci = 0; ci < log.chunks.size(); ci++)
		{
			const PersistChunk& c = log.chunks[ci];
			const uptr newBase = p.chunkNewBase[ci];
			std::vector<u8> dst = c.code;
			if (!RebaseChunkInto(dst.data(), newBase, c, [&](const PersistFixup& f) { return ResolveFixup(p, f); }))
				continue;
			const u8* code = dst.data();          // rebased
			const u8* orig = c.code.data();       // record-time
			const size_t n = c.code.size() / 4;
			int curReg = -1; u32 curOff = 0;
			// Flag only when the rebased address is UNCHANGED from record — i.e. no
			// fixup moved it — AND it still points into a record region. A correctly
			// relocated arena ref changes value (even if it lands back inside the
			// arena span), so comparing new-vs-orig avoids the false positive of a
			// small arena shift keeping a moved address in range.
			auto check = [&](u32 off, uptr vnew, uptr vorig) {
				if (vnew != vorig) return;        // relocated -> fine
				if (vorig >= c.recordBase && vorig < c.recordBase + c.code.size()) return; // intra-chunk
				const char* reg = inRec(vorig);
				if (!reg)
				{
					// Not in a tracked class. If it is a live-mapped, pointer-like value
					// it is a missed relocation into an untracked region; a small/unmapped
					// value is a genuine constant (mask/immediate) — ignore those.
					if (vorig < 0x10000 || !liveMapped(vorig)) return;
					reg = "UNTRACKED";
				}
				hits++;
				if (out)
					std::fprintf(out, "  VU%u prog@%x chunk%zu +%#x -> %#llx (stale %s)\n",
						mVU.index, (unsigned)prog.startPC * 8u, ci, off, (unsigned long long)vorig, reg);
			};
			for (size_t i = 0; i < n; i++)
			{
				u32 insn; std::memcpy(&insn, code + i * 4, 4);
				const u32 off = static_cast<u32>(i * 4);
				const int rd = insn & 0x1F;
				const bool isMovz = (insn & 0x7F800000u) == 0x52800000u;
				const bool isMovk = (insn & 0x7F800000u) == 0x72800000u;
				if (isMovz)
				{
					if (curReg >= 0) check(curOff, DecodeMovImm(code, curOff), DecodeMovImm(orig, curOff));
					curReg = rd; curOff = off; continue;
				}
				if (isMovk && curReg == rd)
					continue;
				if (curReg >= 0) { check(curOff, DecodeMovImm(code, curOff), DecodeMovImm(orig, curOff)); curReg = -1; }
				if ((insn & 0x9F000000u) == 0x90000000u) // ADRP
					check(off, DecodeAdrpPage(code, off, newBase + off), DecodeAdrpPage(orig, off, c.recordBase + off));
				else if ((insn & 0x7C000000u) == 0x14000000u) // B/BL
					check(off, DecodeBranch(code, off, newBase + off), DecodeBranch(orig, off, c.recordBase + off));
			}
			if (curReg >= 0) check(curOff, DecodeMovImm(code, curOff), DecodeMovImm(orig, curOff));
		}
		s_residualHits[mVU.index] += hits;
		return hits;
	}

	static void RoundTripSelfTest(microVU& mVU)
	{
		if (!s_enabled)
			return;
		static const bool run = getenv("LRPS2_VU_PROGCACHE_SELFTEST") != nullptr;
		if (!run)
			return;
		// Findings file for the nonzero-delta residual detector (headless-friendly:
		// survives even when the frontend swallows Console output).
		std::FILE* residualOut = nullptr;
		if (CacheDir())
		{
			char rp[600];
			std::snprintf(rp, sizeof(rp), "%s/selftest_residuals_vu%u.txt", CacheDir(), mVU.index);
			residualOut = std::fopen(rp, "a");
		}
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
				if (RebaseSelfTest(*p->persist))
					s_rebasePass[mVU.index]++;
				else
					s_rebaseFail[mVU.index]++;
				if (HydrateSelfTest(mVU, *p->persist))
					s_hydratePass[mVU.index]++;
				else
					s_hydrateFail[mVU.index]++;
				ResidualScan(mVU, *p, *p->persist, residualOut);
			}
		}
		if (residualOut)
		{
			std::fprintf(residualOut, "== VU%u total residual (missed-relocation) hits: %llu ==\n",
				mVU.index, (unsigned long long)s_residualHits[mVU.index]);
			std::fclose(residualOut);
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
		Console.WriteLn("aVUPersist[VU%u]: fixups img=%llu arena=%llu block=%llu vumem=%llu adrp=%llu bra=%llu bimg=%llu bcond=%llu UNCLASS=%llu",
			vuIndex, (unsigned long long)st.fixImage, (unsigned long long)st.fixArena,
			(unsigned long long)st.fixBlock, (unsigned long long)st.fixVuMem, (unsigned long long)st.fixAdrp,
			(unsigned long long)st.fixBranchArena, (unsigned long long)st.fixBranchImage,
			(unsigned long long)st.fixCondBranch, (unsigned long long)st.fixUnclassifiable);
		if (s_diskSaved[vuIndex] || s_diskLoaded[vuIndex] || s_diskRejected[vuIndex] || s_hydrated[vuIndex])
			Console.WriteLn("aVUPersist[VU%u]: disk saved=%llu loaded=%llu rejected=%llu hydrated=%llu",
				vuIndex, (unsigned long long)s_diskSaved[vuIndex],
				(unsigned long long)s_diskLoaded[vuIndex],
				(unsigned long long)s_diskRejected[vuIndex],
				(unsigned long long)s_hydrated[vuIndex]);
		if (s_selftestPass[vuIndex] || s_selftestFail[vuIndex])
			Console.WriteLn("aVUPersist[VU%u]: selftest roundtrip=%llu/%llu rebase=%llu/%llu hydrate=%llu/%llu (pass/fail)",
				vuIndex, (unsigned long long)s_selftestPass[vuIndex],
				(unsigned long long)s_selftestFail[vuIndex],
				(unsigned long long)s_rebasePass[vuIndex],
				(unsigned long long)s_rebaseFail[vuIndex],
				(unsigned long long)s_hydratePass[vuIndex],
				(unsigned long long)s_hydrateFail[vuIndex]);
		if (s_residualHits[vuIndex])
			Console.WriteLn("aVUPersist[VU%u]: selftest MISSED-RELOCATION residuals=%llu (see selftest_residuals_vu%u.txt)",
				vuIndex, (unsigned long long)s_residualHits[vuIndex], vuIndex);
	}
} // namespace aVUPersist

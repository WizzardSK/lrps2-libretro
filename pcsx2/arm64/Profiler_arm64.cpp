// SPDX-License-Identifier: LGPL-3.0+
//
// See Profiler_arm64.h. The sampling side is deliberately tiny and
// async-signal-safe: the handler reads the interrupted PC out of the ucontext
// and appends it to a preallocated array with one atomic increment. All the
// expensive work (region lookup, dladdr, sorting) happens in Report().

#include "Profiler_arm64.h"

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <cxxabi.h>
#include <dlfcn.h>
#include <elf.h>
#include <signal.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

namespace ArmProf
{
	namespace
	{
		constexpr int kMaxRegions = 8;
		constexpr int kMaxThreads = 8;

		struct Region
		{
			const char* name;
			uintptr_t   base;
			uintptr_t   end;
		};

		Region              s_regions[kMaxRegions];
		std::atomic<int>    s_region_count{0};

		const char*         s_thread_name[kMaxThreads];
		std::atomic<int>    s_thread_count{0};
		thread_local int    s_tls_slot = -1;

		// Sample storage. Fixed size, filled once: dropping the tail of a very
		// long run is better than wrapping (a wrapped buffer silently reweights
		// the profile toward whatever ran last).
		size_t                   s_cap = 0;
		uintptr_t*               s_pc = nullptr;
		uint8_t*                 s_slot = nullptr;
		std::atomic<size_t>      s_n{0};
		std::atomic<size_t>      s_dropped{0};

		struct BlockNote
		{
			uintptr_t   start;
			uintptr_t   end;
			unsigned    guest_pc;
			const char* region;
		};

		std::vector<BlockNote> s_notes;
		std::mutex             s_notes_lock;

		bool EnabledInit()
		{
			static const bool on = getenv("LRPS2_PROF") != nullptr;
			return on;
		}

		void Handler(int, siginfo_t*, void* ctx)
		{
			const int slot = s_tls_slot;
			if (slot < 0)
				return;
			const ucontext_t* uc = static_cast<const ucontext_t*>(ctx);
			const size_t i = s_n.fetch_add(1, std::memory_order_relaxed);
			if (i >= s_cap)
			{
				s_dropped.fetch_add(1, std::memory_order_relaxed);
				return;
			}
			s_pc[i] = static_cast<uintptr_t>(uc->uc_mcontext.pc);
			s_slot[i] = static_cast<uint8_t>(slot);
		}

		void EnsureInit()
		{
			static bool done = false;
			if (done)
				return;
			done = true;

			const char* cap_env = getenv("LRPS2_PROF_MAX");
			s_cap = cap_env ? strtoul(cap_env, nullptr, 0) : (4u << 20);
			s_pc = static_cast<uintptr_t*>(calloc(s_cap, sizeof(uintptr_t)));
			s_slot = static_cast<uint8_t*>(calloc(s_cap, 1));
			if (!s_pc || !s_slot)
			{
				s_cap = 0;
				return;
			}

			struct sigaction sa = {};
			sa.sa_sigaction = Handler;
			sa.sa_flags = SA_SIGINFO | SA_RESTART;
			sigemptyset(&sa.sa_mask);
			sigaction(SIGPROF, &sa, nullptr);

			atexit(Report);
		}

		// ---- symbolization ----
		//
		// dladdr() only sees the dynamic symbol table, and the core exports just
		// retro_*, so essentially every interesting function (the interpreter,
		// vtlb, the GS, the JIT helpers) comes back nameless. The static symbols
		// are all still there in .symtab, so read them straight out of the ELF
		// file on disk and resolve against the object's load base instead.

		struct Sym
		{
			uintptr_t   addr; // relative to the object's load base
			size_t      size;
			std::string name;
		};

		std::vector<Sym> LoadSymtab(const char* path)
		{
			std::vector<Sym> syms;
			FILE* f = fopen(path, "rb");
			if (!f)
				return syms;
			fseek(f, 0, SEEK_END);
			const long len = ftell(f);
			fseek(f, 0, SEEK_SET);
			std::vector<char> buf(static_cast<size_t>(len > 0 ? len : 0));
			const bool ok = len > 0 && fread(buf.data(), 1, buf.size(), f) == buf.size();
			fclose(f);
			if (!ok || buf.size() < sizeof(Elf64_Ehdr))
				return syms;

			const auto* eh = reinterpret_cast<const Elf64_Ehdr*>(buf.data());
			if (memcmp(eh->e_ident, ELFMAG, SELFMAG) != 0 || eh->e_ident[EI_CLASS] != ELFCLASS64)
				return syms;
			if (eh->e_shoff == 0 || eh->e_shoff + eh->e_shnum * sizeof(Elf64_Shdr) > buf.size())
				return syms;

			const auto* sh = reinterpret_cast<const Elf64_Shdr*>(buf.data() + eh->e_shoff);
			for (int i = 0; i < eh->e_shnum; i++)
			{
				if (sh[i].sh_type != SHT_SYMTAB || sh[i].sh_link >= eh->e_shnum)
					continue;
				const Elf64_Shdr& str = sh[sh[i].sh_link];
				if (sh[i].sh_offset + sh[i].sh_size > buf.size() ||
					str.sh_offset + str.sh_size > buf.size())
					continue;

				const auto* st = reinterpret_cast<const Elf64_Sym*>(buf.data() + sh[i].sh_offset);
				const size_t count = sh[i].sh_size / sizeof(Elf64_Sym);
				const char* strs = buf.data() + str.sh_offset;
				for (size_t k = 0; k < count; k++)
				{
					if (ELF64_ST_TYPE(st[k].st_info) != STT_FUNC || !st[k].st_value || !st[k].st_size)
						continue;
					if (st[k].st_name >= str.sh_size)
						continue;
					syms.push_back({static_cast<uintptr_t>(st[k].st_value),
						static_cast<size_t>(st[k].st_size), std::string(strs + st[k].st_name)});
				}
			}
			std::sort(syms.begin(), syms.end(),
				[](const Sym& a, const Sym& b) { return a.addr < b.addr; });
			return syms;
		}

		// Resolve pc within the object loaded at `fbase` from file `fname`.
		// Returns an empty string when it is not covered by any function symbol.
		std::string SymbolFor(const char* fname, uintptr_t fbase, uintptr_t pc)
		{
			static std::unordered_map<std::string, std::vector<Sym>> cache;
			auto it = cache.find(fname);
			if (it == cache.end())
				it = cache.emplace(fname, LoadSymtab(fname)).first;

			const std::vector<Sym>& syms = it->second;
			if (syms.empty() || pc < fbase)
				return {};

			const uintptr_t off = pc - fbase;
			// First symbol starting after off, then step back one.
			auto up = std::upper_bound(syms.begin(), syms.end(), off,
				[](uintptr_t v, const Sym& s) { return v < s.addr; });
			if (up == syms.begin())
				return {};
			--up;
			if (off >= up->addr + up->size)
				return {};
			return up->name;
		}

		std::string Demangle(const std::string& name)
		{
			int status = 0;
			char* d = abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
			if (status != 0 || !d)
			{
				free(d);
				return name;
			}
			std::string out(d);
			free(d);
			// Drop the argument list: the caller wants to scan a list of hot
			// functions, and mangled C++ signatures make that unreadable.
			const size_t paren = out.find('(');
			if (paren != std::string::npos)
				out.resize(paren);
			return out;
		}

		const char* RegionOf(uintptr_t pc)
		{
			const int n = s_region_count.load(std::memory_order_acquire);
			for (int i = 0; i < n; i++)
			{
				if (pc >= s_regions[i].base && pc < s_regions[i].end)
					return s_regions[i].name;
			}
			return nullptr;
		}
	} // namespace

	bool Enabled()
	{
		return EnabledInit();
	}

	void RegisterRegion(const char* name, const void* base, size_t size)
	{
		if (!EnabledInit() || !base || !size)
			return;
		EnsureInit();
		const int i = s_region_count.load(std::memory_order_relaxed);
		if (i >= kMaxRegions)
			return;
		s_regions[i].name = name;
		s_regions[i].base = reinterpret_cast<uintptr_t>(base);
		s_regions[i].end = s_regions[i].base + size;
		s_region_count.store(i + 1, std::memory_order_release);
		fprintf(stderr, "[prof] region %-8s %p..%p\n", name, base,
			reinterpret_cast<const void*>(s_regions[i].end));
	}

	void AttachThread(const char* name)
	{
		if (!EnabledInit() || s_tls_slot >= 0)
			return;
		EnsureInit();
		if (!s_cap)
			return;

		const int slot = s_thread_count.fetch_add(1, std::memory_order_relaxed);
		if (slot >= kMaxThreads)
			return;
		s_thread_name[slot] = name;

		// A per-thread CPU-time timer: samples land on this thread only, and a
		// blocked thread stops accumulating them (so the sample count per thread
		// is its CPU time, which is exactly the comparison we want between the
		// EE, MTVU and GS threads).
		struct sigevent sev = {};
		sev.sigev_notify = SIGEV_THREAD_ID;
		sev.sigev_signo = SIGPROF;
		sev.sigev_notify_thread_id = static_cast<pid_t>(syscall(SYS_gettid));

		timer_t timer;
		if (timer_create(CLOCK_THREAD_CPUTIME_ID, &sev, &timer) != 0)
		{
			fprintf(stderr, "[prof] timer_create failed for %s\n", name);
			return;
		}

		const char* hz_env = getenv("LRPS2_PROF_HZ");
		const long hz = hz_env ? strtol(hz_env, nullptr, 0) : 1000;
		const long ns = 1000000000L / (hz > 0 ? hz : 1000);

		struct itimerspec its = {};
		its.it_interval.tv_nsec = ns;
		its.it_value.tv_nsec = ns;
		timer_settime(timer, 0, &its, nullptr);

		// Published last: the handler ignores any thread that has not finished
		// attaching.
		s_tls_slot = slot;
		fprintf(stderr, "[prof] sampling thread %s (slot %d) at %ld Hz\n", name, slot, hz);
	}

	void NoteBlock(const char* region, const void* host_start, size_t host_size, unsigned guest_pc)
	{
		if (!EnabledInit() || !host_start || !host_size)
			return;
		const uintptr_t start = reinterpret_cast<uintptr_t>(host_start);
		std::lock_guard<std::mutex> lock(s_notes_lock);
		s_notes.push_back({start, start + host_size, guest_pc, region});
	}

	namespace
	{
		// Hottest guest blocks inside the JIT code caches. Blocks are matched by
		// host PC; when the cache has been reused, the most recent note covering
		// an address wins (s_notes is in compile order, so a stable sort by start
		// address keeps the later note last among equals).
		void ReportHotBlocks(const std::vector<uintptr_t>& pcs, const char* thread_name)
		{
			std::vector<BlockNote> notes;
			{
				std::lock_guard<std::mutex> lock(s_notes_lock);
				notes = s_notes;
			}
			if (notes.empty() || pcs.empty())
				return;

			std::stable_sort(notes.begin(), notes.end(),
				[](const BlockNote& a, const BlockNote& b) { return a.start < b.start; });

			struct Hot
			{
				size_t      samples = 0;
				size_t      host_bytes = 0;
				const char* region = nullptr;
			};
			std::unordered_map<unsigned, Hot> hot;
			size_t matched = 0;

			for (const uintptr_t pc : pcs)
			{
				auto up = std::upper_bound(notes.begin(), notes.end(), pc,
					[](uintptr_t v, const BlockNote& b) { return v < b.start; });
				// Walk back over the candidates starting at or below pc; take the
				// last (most recently compiled) one that actually covers pc.
				const BlockNote* found = nullptr;
				for (auto it = up; it != notes.begin();)
				{
					--it;
					if (pc >= it->start && pc < it->end)
					{
						found = &*it;
						break;
					}
					// Notes are sorted by start; once we are past a note whose end
					// is far below pc there is still the chance of an earlier long
					// block, so keep scanning a bounded window.
					if (it->start + (1u << 20) < pc)
						break;
				}
				if (!found)
					continue;
				matched++;
				Hot& h = hot[found->guest_pc];
				h.samples++;
				h.host_bytes = found->end - found->start;
				h.region = found->region;
			}
			if (!matched)
				return;

			std::vector<std::pair<unsigned, Hot>> v(hot.begin(), hot.end());
			std::sort(v.begin(), v.end(),
				[](const auto& a, const auto& b) { return a.second.samples > b.second.samples; });

			fprintf(stderr, "\n[prof] hottest guest blocks on %s (%zu/%zu JIT samples matched, %zu blocks)\n",
				thread_name, matched, pcs.size(), v.size());
			const size_t show = std::min<size_t>(v.size(), 20);
			for (size_t i = 0; i < show; i++)
			{
				fprintf(stderr, "[prof]   %-7s pc=0x%08x  %6zu samples  %5.2f%% of JIT  (%zu host bytes)\n",
					v[i].second.region, v[i].first, v[i].second.samples,
					100.0 * (double)v[i].second.samples / (double)pcs.size(),
					v[i].second.host_bytes);
			}
		}
	} // namespace

	void Report()
	{
		static bool reported = false;
		if (!EnabledInit() || reported)
			return;
		reported = true;

		const size_t n = std::min(s_n.load(std::memory_order_relaxed), s_cap);
		if (!n)
		{
			fprintf(stderr, "[prof] no samples\n");
			return;
		}

		const int nthreads = std::min(s_thread_count.load(std::memory_order_relaxed), kMaxThreads);
		const int nregions = s_region_count.load(std::memory_order_relaxed);

		std::vector<size_t> per_thread(kMaxThreads, 0);
		// [thread][region], with the last column for "not in any code cache".
		std::vector<std::vector<size_t>> per_region(
			kMaxThreads, std::vector<size_t>(kMaxRegions + 1, 0));
		// Native (non-JIT) samples, keyed by symbol, per thread.
		std::vector<std::unordered_map<std::string, size_t>> native(kMaxThreads);
		// JIT-cache samples per thread, kept for the per-guest-block breakdown.
		std::vector<std::vector<uintptr_t>> jit_pcs(kMaxThreads);

		for (size_t i = 0; i < n; i++)
		{
			const int slot = s_slot[i];
			if (slot >= kMaxThreads)
				continue;
			per_thread[slot]++;

			const uintptr_t pc = s_pc[i];
			int r = -1;
			for (int k = 0; k < nregions; k++)
			{
				if (pc >= s_regions[k].base && pc < s_regions[k].end)
				{
					r = k;
					break;
				}
			}
			if (r >= 0)
			{
				per_region[slot][r]++;
				jit_pcs[slot].push_back(pc);
				continue;
			}
			per_region[slot][kMaxRegions]++;

			Dl_info info;
			std::string sym;
			if (dladdr(reinterpret_cast<void*>(pc), &info))
			{
				if (info.dli_sname)
				{
					sym = Demangle(info.dli_sname);
				}
				else if (info.dli_fname)
				{
					sym = Demangle(SymbolFor(
						info.dli_fname, reinterpret_cast<uintptr_t>(info.dli_fbase), pc));
					if (sym.empty())
					{
						// Not covered by a function symbol: name the object at
						// least, so unaccounted time is visibly attributed.
						const char* slash = strrchr(info.dli_fname, '/');
						sym = std::string("[") + (slash ? slash + 1 : info.dli_fname) + "]";
					}
				}
			}
			if (sym.empty())
				sym = "??";
			native[slot][sym]++;
		}

		fprintf(stderr, "\n[prof] ===== %zu samples", n);
		if (const size_t dropped = s_dropped.load(std::memory_order_relaxed))
			fprintf(stderr, " (+%zu dropped, buffer full)", dropped);
		fprintf(stderr, " =====\n");

		size_t total = 0;
		for (int t = 0; t < nthreads; t++)
			total += per_thread[t];
		if (!total)
			return;

		for (int t = 0; t < nthreads; t++)
		{
			const size_t tn = per_thread[t];
			if (!tn)
				continue;
			fprintf(stderr, "\n[prof] thread %-6s %8zu samples (%5.1f%% of all CPU time)\n",
				s_thread_name[t], tn, 100.0 * (double)tn / (double)total);

			for (int k = 0; k < nregions; k++)
			{
				if (!per_region[t][k])
					continue;
				fprintf(stderr, "[prof]   %-10s %8zu  %5.1f%%\n", s_regions[k].name,
					per_region[t][k], 100.0 * (double)per_region[t][k] / (double)tn);
			}
			const size_t nat = per_region[t][kMaxRegions];
			if (!nat)
			{
				ReportHotBlocks(jit_pcs[t], s_thread_name[t]);
				continue;
			}
			fprintf(stderr, "[prof]   %-10s %8zu  %5.1f%%\n", "native C++", nat,
				100.0 * (double)nat / (double)tn);

			std::vector<std::pair<std::string, size_t>> syms(native[t].begin(), native[t].end());
			std::sort(syms.begin(), syms.end(),
				[](const auto& a, const auto& b) { return a.second > b.second; });
			const size_t show = std::min<size_t>(syms.size(), 25);
			for (size_t i = 0; i < show; i++)
			{
				fprintf(stderr, "[prof]     %6.2f%% of thread  %8zu  %s\n",
					100.0 * (double)syms[i].second / (double)tn, syms[i].second,
					syms[i].first.c_str());
			}

			ReportHotBlocks(jit_pcs[t], s_thread_name[t]);
		}
		fprintf(stderr, "\n");
	}
} // namespace ArmProf

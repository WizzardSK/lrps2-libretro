# LRPS2

PlayStation 2 emulation core for [libretro](https://www.libretro.com/) (RetroArch),
based on PCSX2. This fork's headline work is a native **AArch64 (arm64) port**:
upstream PCSX2 emulation relies on x86-64 recompilers, which do not exist on ARM —
this tree adds arm64 recompilers for the EE and IOP plus the supporting
infrastructure to run correctly (and reasonably fast) on 64-bit ARM Linux devices.

## Status (arm64)

Verified booting to real in-game content (Mega Man X7 gameplay, Gran Turismo 3
intro/FMV) on a Snapdragon-class device (Adreno 618, 4K-page Linux, glibc
RetroArch flatpak + musl test host).

**Profile from a save state, not from a boot.** A headless run can only reach
content the emulator walks to by itself, and for Gran Turismo 3 that is its
opening movie -- 9000 frames (150 s of emulated time) with Start auto-pressed
still land in the intro. Profiles taken that way put the IPU (`IPUWorker` +
`yuv2rgb`, ~14 %) on top and leave the MTVU thread at 0.2 % of CPU, i.e. they
measure the video decoder rather than the game, and they sent two optimisation
attempts down blind alleys before that was noticed. Resuming from an in-race
save state instead:

| Thread | CPU | Inside it |
|---|---|---|
| EE | 39 % | 48 % in its own JIT, 44 % native C++ (SPU `Mix` 7.6 %, `memcpy` 4.4 %, event test 2.3 %), 7 % IOP JIT |
| GS | 37 % | Vulkan renderer |
| MTVU | 24 % | 77 % inside microVU1's generated code |

No IPU at all, and the event test that looked like a 6-7 % problem is 2.3 %.
Any performance claim here should say which of the two it came from.

Three findings that came out of getting that profile:

* **Loading a save state used to drop the EE onto the interpreter for the rest
  of the session** (3.5x slower: GT3 in-race 92 s -> 26 s per 600 frames). The
  interpreter's execute loop always restarted in its boot stage, whose
  "interpret until the pc reaches EELOAD" condition can never be satisfied
  mid-game, so the JIT stage was never reached. Fixed: the loop resumes in
  GAME_RUNNING when `g_GameStarted` (which is part of the save state) says the
  entry point is behind us.
* **The EE thread is the critical path, and it is not waiting.** Wall-clock
  timing of the sync points (`LRPS2_SYNC_STATS=1`) puts the EE's blocking at
  0.97 s on the GS and 0.00 s on MTVU over a 600-frame in-race run. Letting the
  EE run frames ahead of the GS, or turning MTVU off, both measure as nothing.
  (The sampling profiler's cross-thread percentages are undercounts -- SIGPROF
  does not queue -- so only within-thread splits are trustworthy.)
* **The EE thread is not instruction-count-bound either.** The in-race handoff
  work (C.63-C.65) removed 63 % of the interpreter handoffs, and the code-quality
  work (C.66-C.67) shrank the hottest block 35 % (3068 -> 1984 host bytes:
  dead COP2 flag pipelines dropped, unaligned pairs fused to single accesses) --
  and wall time did not move for any of them. On this core (A76/A55) the OoO
  front end absorbs the removed instructions; what remains is bound by memory
  and dependency chains, so further emitted-code polishing has a low ceiling.
  These changes are kept for coverage, interpreter-fidelity, code density and
  x86 parity, not for speed.

Overall recompiler-suite progress: roughly **~87 %** (weighted by remaining
work; dynamic instruction coverage on the tested titles is much higher — the
vast majority of EE+IOP instructions already execute natively, and the
remaining interpreter handoffs are mostly untranslatable exceptions). Opcode
coverage is essentially done; the current work is the quality of the emitted
code, guided by the in-tree sampling profiler (`LRPS2_PROF=1`), which buckets
PC samples by JIT code cache and resolves the rest against the core's symbol
table.

| Component | ~Done | State |
|---|---|---|
| EE (R5900) JIT | 88 % | Native ALU, loads/stores incl. LQ/SQ (inline vtlb fast path) + unaligned family, branches + likely branches + BC0x/BC1x, block linking/chaining, MULT/DIV/MADD + HI/LO (both pipelines), **complete COP1 (FPU)** — data moves + all S-format arithmetic + CVT + C.cond, interpreter-exact clamp/flag semantics, **CFC1/CTC1 native** (C.65: `fs` is a compile-time constant, so CFC1 specializes to FCR31 load / revision constant / zero, and CTC1 to a non-FCR31 register is a nop), bit-exact MMI subset → NEON, MFSA/MTSA(B/H), kernel idle-loop skip, block revalidation cache, write-back GPR register cache with direct-to-cache emission, event tests gated on `cycle >= nextEventCycle` in block tails (the upstream-rec dispatcher rule; ~23 % faster wall clock on GT3, `LRPS2_NO_EVTGATE=1` restores every-branch tests), default-rate cycle bookkeeping inlined into block tails (`LRPS2_NO_INLINE_UPD=1` restores the helper call), block chaining keeps the frame live across hops (no per-block frame pop/push or x19 rematerialization; same on the IOP side). ADDI/DADDI (overflow trap dropped, exactly like the x86 rec), COP0 EI/DI, LQC2/SQC2 (128-bit VU0-register load/store, native since C.59: vu0Sync plus a 128-bit access straight into vuRegs[0].VF[ft], so the EE register cache survives an op MMX7 runs ~827K times; `LRPS2_NO_EE_VUQMEM=1` restores the interpreter call). **Memory goes through the vtlb fastmem area** (C.50): the 4 GB host mapping in which every memory-backed guest page sits at its own guest address, so a load/store is one instruction (`ldr/str <data>, [x28, w<addr>, UXTW]`) with no vmap indirection. Hardware registers and unmapped pages are absent from the area, so touching one faults; the faulting instruction is rewritten into a branch to a slow stub emitted next to it at compile time, and the guest pc is blacklisted so later compiles of that op skip fastmem (`LRPS2_NO_FASTMEM=1` restores the old inline vmap path, `LRPS2_FASTMEM_LOG=1` traces the patches). Emitted-code quality: cpuRegs fields are reached through the x19 guest-reg base instead of a 4-instruction absolute address per access (C.46); load/store displacements fold into the address add (C.48); the vmap/fastmem base is pinned in x28 for the life of the block frame (C.49); the backpatch stubs (C.51), the misalign cancel paths (C.53), the event-due tails and the frame pop/ret (C.54) all live out of line past the epilogue, so the hot instruction stream carries no cold code; the chain epilogue folds the mirror mask and RAM range check into one test (C.52); an exit whose successor is known at compile time chains through a constant LUT slot with no pc read at all, on the event gate's not-due path where pc provably has not been redirected (C.54); the tail's cycle bookkeeping and cycle update are fused into one sequence (C.55); the FP loads/stores use fastmem too (C.56); and ADDIU/DADDIU fold their immediate into the add (C.57); the canonical unaligned pairs (LDL/LDR, LWL/LWR, SDL/SDR, SWL/SWR -- adjacent, left-then-right, matching registers and offsets) fuse into ONE host unaligned access through fastmem instead of two helper calls each, with a fault stub that replays the exact two-access guest-order semantics for hw pages (C.67, `LRPS2_NO_EE_UPAIR=1` restores the helpers). **Handoff coverage (C.63-C.65), measured from an in-race GT3 save state rather than an intro movie:** ADD/SUB (trap dropped like ADDI), PLZCW (= `CLS` per 32-bit half), PREVH (= `REV64 .8H`) and CFC1/CTC1 are native, and a **likely branch is no longer refused by an untranslatable delay slot** -- GT3 is full of `beql ..., panic; break` asserts whose slot only runs when taken, i.e. never; the not-taken path is emitted natively and the taken path hands control to the interpreter at the branch (C.64). Interpreter handoffs in-race: 887k -> 332k per 600 frames (-63 %), and what remains is COP0 (196k) + SYSCALL (123k), i.e. exception paths. Translating PLZCW also FIXED a timing deviation: a block that breaks at an untranslatable op flushes its cycle count there, a rounding point the interpreter does not have -- with PLZCW native, MMX7's frame-3000 framebuffer now matches the pure interpreter (88190948), which the old JIT did not. The bit-exactness reference for MMX7 moved accordingly. `LRPS2_DUMP_RANGE=lo:hi` dumps the guest ops a pc range compiles from with their translatability, and `LRPS2_DUMP_HOST=<pc>` writes a block's emitted host code out for `objdump -D -b binary -m aarch64`. Missing: SYSCALL/ERET/TLB (interpreter forever); marginal MMI stragglers (PPAC5, PMFHI/PMTHL) |
| IOP (R3000A) JIT | 98 % | Native ALU incl. the trapping forms ADDI/ADD/SUB (this interpreter drops the overflow trap, so they are the U ops — same call the EE made in C.42) and RFE (pure Status bit shuffle) (C.68, `LRPS2_NO_IOP_C68=1` restores the interpreter), aligned + unaligned (LWL/LWR/SWL/SWR) loads/stores, branches + delay slots, block linking, MULT/DIV + HI/LO moves, COP0 moves (MFC0/CFC0/MTC0/CTC0), write-back GPR register cache (w22–w27, `LRPS2_NO_IOP_REGCACHE=1` to disable), inline RAM fastmem for loads (`LRPS2_NO_IOP_FASTMEM=1` to disable; the remaining helper-call traffic is genuine HW-register polling), event tests gated on `cycle >= iopNextEventCycle` in branch tails (`LRPS2_NO_IOP_EVTGATE=1` to disable); J native (only the module-import stub form `j ...; li $zero, fn` stays interpreted for the HLE hook — and that hook's resolution is cached per stub pc (C.71): the up-to-0x2000-byte backward magic scan, the `std::string` heap allocation and the library-name compare chain ran on EVERY stub execution, millions of times a minute; now a hit revalidates its inputs with three RAM reads (`LRPS2_NO_IOP_JSTUB_CACHE=1` restores the uncached path — `iopMemReadString` left the profile, ~1.5-2 % of the EE thread). A straight-line run longer than the block cap chains to the next block instead of handing its fully-translatable tail to the interpreter (C.69, the EE's C.44 rule; `LRPS2_NO_IOP_CAP_CHAIN=1` to disable), and an interpreter-tail block also covers its breaker word for SMC invalidation, so code loaded OVER what was data at compile time (IRX module loads) recompiles instead of re-handing off forever (C.70 — one stale MMX7 block was re-interpreting `lw ra` thousands of times a run). **Handoff coverage is closed:** `LRPS2_IOP_HANDOFF_STATS=1` (histogram of interpreted ops, EE-style) shows exactly two breaker classes left on both test titles — the J import stub (HLE hook, by design) and SYSCALL (exception, interpreter forever); GTE/COP2 is PS1-only and never fires. The kernel idle spin (`j <self>` + nop) is fast-forwarded to the next event or the end of the cycle budget rather than emulated an iteration at a time (C.47, `LRPS2_NO_IOP_IDLE=1` to disable) — it was 18 % of all JIT time; psxRegs fields go through the x19 base (C.46) |
| VU1 recompiler | 65 % | **microVU1 (armsx2 aVU transplant) is the default provider**: native AArch64 VU codegen, MVU_DIFF-verified register-exact against the interpreter on both test titles, byte-identical framebuffers through 12000-frame runs. Instant-VU1 and MTVU are back to default-on with it. **In-race profile facts (GT3 save state)**: EE never waits on MTVU (SyncStats 0.00 s) so VU1 speed does not gate wall time; mvu1 is ~75 % of the MTVU thread and FLAT (150 blocks, hottest 5.6 % of JIT) — no per-block target exists, only systematic emitter quality. First such passes done: C.72 folds the page offset of absolute-address accesses (flag spills, clamp constants) into the load/store instead of a separate `add` after `adrp` (`armAbsMemOperand`), and C.73 replaces the naive copy+4×Ins `mVUshufflePS` permute with the cheap NEON form where one exists (Dup/Rev64/Ext/identity; 2-lane swaps insert only the moved lanes) — the hottest block dropped 1776 → 1472 host bytes (−17 %), and the register-exact MVU_DIFF shadow run from the in-race save state produces a byte-identical divergence log to the pre-C.72 baseline (the 9 logged programs are the known pre-existing interp-vs-microVU last-bit FP differences). LRPS2_PROF now attributes mvu0/mvu1 samples per guest block and `LRPS2_DUMP_HOST_MVU=0x<pc>` dumps a block's host code. Fallbacks: `LRPS2_VU1REC_PAIR=1` (interp-pair rec), `LRPS2_NO_VU1REC=1` (interpreter; both restore conservative non-instant/opt-in-MTVU behavior) |
| VU0 / COP2 | 75 % | **microVU0 runs VU0 micro programs** (VCALLMS) natively; **COP2 macro ALU ops emit native NEON directly into EE JIT blocks** (aVU_Macro single-op emitters) with **real flag liveness** (C.66): the analysis runs over a run of consecutive macro ALU ops, capped at the block end, and only the last writer of each flag category computes it -- the same flag-hack the x86 rec ships by default (its `COP2FlagHackPass`, here run-local because this builder interleaves analysis with emission). Dead intermediate MAC/status pipelines (~40 host instructions per op) vanish: the hottest in-race GT3 block, VU0-macro matrix code, dropped 3068 -> 2004 host bytes. Wall-time flat (OoO absorbed the ALU); the win is code size, I-cache and x86 parity. `LRPS2_NO_COP2_LIVENESS=1` restores all-live; BC2 branches native; **the COP2 transfers (QMFC2/CFC2/QMTC2/CTC2) are native too** (C.58) -- the interlock bit is known at compile time, so the `_vu0FinishMicro`/`_vu0WaitMicro` call is only emitted when set, and CFC2's REG_R quirk (writes UL[0], leaves UL[1]) is reproduced exactly. CTC2 to FBRST/CMSAR1 (VU reset / VU1 microprogram kick) and CALLMS/CALLMSR stay on the interpreter call, faithful to x86. `LRPS2_NO_VU0REC=1` / `LRPS2_NO_EE_COP2MACRO=1` / `LRPS2_NO_EE_COP2XFER=1` fall back |
| VIF unpack dynarec | 100 % | NEON unpack kernels (armsx2 transplant); portable C fallback (`LRPS2_NO_VIF_DYNAREC=1`) |
| SMC / overlays | Compiled pages are write-protected; faults invalidate stale blocks (vtlb `mmap_MarkCountedRamPage` flow). A page the game keeps *writing* would otherwise ping-pong forever — fault, unprotect, drop blocks, revive them, re-protect, fault again — at a SIGSEGV plus four `mprotect` calls a cycle (the RAM page and its alias in the fastmem area). After `kManualClears` clears a page is therefore left writable and its blocks check their own source words at entry instead (C.60), which is where the chained entry point sits, so a chained jump validates too; the check is handed its block record at compile time rather than looking itself up (C.61). On GT3 that took the run from 48006 protects / 47803 clears to 251 / 49 — two pages had been ping-ponging ~23k times each — and `mprotect` left the profile entirely. `LRPS2_NO_EE_MANUAL=1` restores protect-on-every-revive, `LRPS2_SMC_STATS=1` dumps the per-page protect/clear counts |
| MTVU (VU1 thread) | **Default-on** with microVU1 (≥3 cores; `LRPS2_NO_MTVU=1` disables). Partial-packet flush protocol prevents the continuous-microprogram livelock; with an interp-style VU1 provider MTVU stays opt-in (`LRPS2_MTVU=1`) |
| GS renderer | Standard **Vulkan** renderer (`pcsx2_renderer = "Vulkan"`). paraLLEl-GS requires GPU features (8/16-bit storage, small subgroups) that e.g. Adreno 618 lacks |
| MTGS | Works (GS thread active) |

Correctness methodology: headless libretro harness runs are compared for
byte-identical framebuffer output against the interpreter (`LRPS2_NO_EEREC=1`)
and the mem-interpreted JIT (`LRPS2_NO_EE_MEM=1`) baselines; JIT-vs-baseline
lockstep is verified with per-frame RAM+scratchpad hashes, per-op GPR-hash
traces, and interrupt/syscall vector logs (see debug toggles below).

## Building

CMake options common to all builds:

```
-DLIBRETRO=ON -DDISABLE_ADVANCE_SIMD=ON -DUSE_VULKAN=ON -DUSE_OPENGL=ON
```

The build produces `pcsx2_libretro.so`; install it into RetroArch's `cores/`
directory. A PS2 BIOS image is required in `<system>/pcsx2/bios/`.

Notes for arm64:
- Build against the same libc your frontend uses (a musl-built core will not
  load into a glibc RetroArch flatpak; build inside the matching SDK).
- 4K host pages are assumed on non-Apple aarch64 (16K remains for Apple
  Silicon); the vtlb page-protection code depends on this matching the kernel.

## Debug/diagnostic environment toggles (arm64, all off by default)

Bisection switches:

| Variable | Effect |
|---|---|
| `LRPS2_NO_EEREC=1` | EE runs on the pure interpreter |
| `LRPS2_NO_IOPREC=1` | IOP runs on the pure interpreter |
| `LRPS2_NO_EE_MEM/LOAD/STORE/BRANCH/MMI/MULDIV/COP1/LD64=1` | Route the given EE op class to the interpreter |
| `LRPS2_NO_EE_FPU_ARITH=1` | Route COP1 S-format arithmetic to the interpreter (data moves stay native) |
| `LRPS2_NO_EE_DIVS/SQRTS/RSQRTS=1` | Route just DIV.S / SQRT.S / RSQRT.S to the interpreter |
| `LRPS2_EE_SPLIT_MEM=1` | End the block after every native mem op |
| `LRPS2_NO_INLINE_MEM=1` | Disable the inline vtlb fast path (wrapper calls only) |
| `LRPS2_NO_EE_MANUAL=1` | Re-protect a code page on every revive instead of letting it go manual (C.60) |
| `LRPS2_MTVU=1` | Opt in to the MTVU (VU1 worker thread) |
| `LRPS2_VU1_INSTANT=1` | Restore instant VU1 (off on arm64; see VU0/VU1 row) |
| `LRPS2_MTVU_LOG=1` | MTVU worker/GS packet + per-path GS byte counters |

Tracing/logging:

| Variable | Effect |
|---|---|
| `LRPS2_RAMCRC=1` | Per-frame FNV hash of EE RAM + scratchpad (+ cycle) |
| `LRPS2_DUMP=<path>` + `LRPS2_DUMP_FRAME=<n>` | Full RAM dump at frame n |
| `LRPS2_TRACE=<path>` + `LRPS2_TRACE_LO/HI=<hex>` (+`_STEP`, `_FRAME`) | Binary (pc, GPR-hash) execution trace |
| `LRPS2_EXCLOG=<path>` | COP0 state at every 0x180/0x200 vector entry (+ syscall number/args) |
| `LRPS2_EVTLOG=<path>` + `LRPS2_EVT_LO/HI=<frame>` | Log every EE event test |
| `LRPS2_WLOG=1` + `LRPS2_WLO/WHI=<hex>` + `LRPS2_WFRAME=<n>` | Watch EE memory accesses in a physical range |
| `LRPS2_IPU_LOG_FRAME=<n>` | Log IPU FDEC results + IPU1 DMA feed from frame n |
| `LRPS2_FAULT_LOG=1` | Log vtlb page-fault handler activity; on an unhandled fault, locate the faulting pc's JIT block (guest pc, emitted-code hexdump, guest MIPS source) |
| `LRPS2_HANDOFF_STATS=1` | Histogram of ops handed to the interpreter + first-op breaker table |
| `LRPS2_JIT_STATS=1` | JIT compile statistics (e.g. likely-branch counts) |
| `LRPS2_SMC_STATS=1` | Per-page write-protect/clear counts (which pages ping-pong between code and data) |
| `LRPS2_EVT_STATS=1` | EE event-test rate: entries, how many run the IOP or the interrupt scan, and which clamp set `nextEventCycle` |

## Measured and rejected

Kept here because each looks obviously worth doing until you measure it. The
common thread: on this core (A76/A55) scalar integer ALU work is close to free,
so anything that trades it for wider-but-longer-latency code, or for fewer
instructions at the cost of a memory access, tends to lose.

| Idea | Why it was dropped |
|---|---|
| NEON `IDCT_Block` (IPU) | Written and **bit-exact on 602k blocks** — but slower at every block density: 169 vs 73 ns/block for the full vector version, and even a hybrid keeping the scalar row pass (whose DC shortcut skips most rows of a real MPEG block) with a NEON column pass that needs no transpose is 78 vs 74. The scalar code is not naive; it is well matched to the data. `yuv2rgb` already has a NEON path, so IPU is closed |
| SPU mixer micro-optimisation | The mixer is the biggest native C++ item on the EE thread in-game (`Mix` 7.6 %, reverb 1.2 %), but every cheap lever measures empty: the ceiling is ~6 % wall (muting the whole mixer: 17 -> 16 s medians); 47.1 of 48 voices are genuinely active in GT3 (engine sounds on every car) and none are silent, so the stopped-voice and silence shortcuts have nothing to skip; noise/modulation/volume-slides are 0.0 per sample; `TimeUpdate` mixes 1.01 samples per call, so a voice-outer reorder has no batch; and all 46 sustain voices have a nonzero envelope step, so an exact static-sustain ADSR skip has zero candidates. What remains is the price of 47 live voices at 48 kHz in already-tight scalar code (3-cache-line voice layout, prefetch, cached ADSR phases). The only lever left is an SPU worker thread -- a synchronisation project (games read ENDX/IRQA against mixing progress), not an optimisation. `LRPS2_SPU_STATS=1` dumps the voice-shape numbers, `LRPS2_SPU_MUTE=1` re-measures the ceiling |
| Coarsening the EE↔IOP slice | The event test runs 102M times on a 3000-frame MMX7 run and is 5–7 % of the EE thread, but it is not slow — it is asked for that often. The 48-cycle re-test at the end of the test wins the clamp 663 times out of 76M; sweeping it 48→384 changed neither the call count, nor the output (bit-identical), nor the time. `CPU_INT`/`cpuTestINTCInts` keep pulling `nextEventCycle` back to 4–8 cycles, so the rate is the density of the DMA/INT schedule. Thinning that moves interrupt delivery, for a ceiling measured at 2–4 % (inside the noise) |

## Project Details

Upstream PCSX2: https://pcsx2.net/ — this core inherits its GPL/LGPL licensing;
see the source headers. The arm64 AArch64 emitter is [VIXL](https://github.com/Linaro/vixl)
(vendored under `3rdparty/vixl`).

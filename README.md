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
| EE | 41 % | 50 % in its own JIT, 43 % native C++ (SPU `Mix` 8.9 %, `memcpy` 4.0 %, event test 2.1 %), 7 % IOP JIT |
| GS | 37 % | Vulkan renderer |
| MTVU | 23 % | 75 % inside microVU1's generated code |

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
* **The remaining native-C++ "plumbing" on the EE thread is protocol cost, not
  waste.** Single-level caller attribution for leaf symbols
  (`LRPS2_PROF_LR=memcpy,memcmp,__aarch64_` -- the SIGPROF handler records LR
  next to PC, and for a leaf function LR is the caller) pins it down: `memcpy`
  (4 %) is 59 % the MTGS ring write (`WRITERING_DMA`) and 31 % the MTVU ring
  write (`VU_Thread::VifUnpack`); the outlined atomics (2.2 %) are 84 % the
  `WorkSema::NotifyOfWork` fetch_add, one per VIF unpack packet; `memcmp`
  (1.2 %) is entirely the C.60 block-revalidation check. The WorkSema RMW in
  particular must stay *per notify*: an RMW always observes the latest state,
  which is what makes the sleep/wake handshake sound -- the tempting
  load-then-skip variant can read a stale RUNNING_N while the worker is going
  to sleep and miss the wakeup. What C.80 does instead is issue **fewer
  notifies**: a VIF unpack packet only publishes its ring data (the release
  store of the write pointer stays per-packet, so an awake worker drains it
  immediately) and defers the wakeup RMW to the next flush point -- any other
  MTVU command (they all notify), `WaitVU` (mandatory: `WaitForEmpty` trusts
  the sema state machine), `Close`, or the VIF1 DMA/MFIFO end-of-transfer
  tails (latency bound). In-race GT3 that removes **82 %** of the notifies
  (3.9M unpacks -> 0.7M notifies per 600 frames) and the fetch_add leaves the
  EE-thread profile (2.2 % -> below threshold); wall time is flat, as with
  every plumbing change since C.49 (`LRPS2_NO_VIF_LAZYKICK=1` restores the
  per-packet notify, `LRPS2_MTVU_STATS=1` reports the deferred/issued counts).
  An SPU worker thread was measured and rejected: both test titles run with an
  SPU IRQ armed 93-98 % of all sample ticks and GT3's in-race IRQs are 100 %
  mixer-voice IRQA crossings, discovered tick-exactly during mixing -- async
  mixing cannot deliver them at the bit-identical IOP cycle, and the only
  sound design (speculative mixing with rollback) is out of proportion to the
  ~5-6 % ceiling (`LRPS2_SPU_SYNC_STATS=1` collects the evidence).

Overall recompiler-suite progress: roughly **~88 %** (weighted by remaining
work; dynamic instruction coverage on the tested titles is much higher — the
vast majority of EE+IOP instructions already execute natively, and the
remaining interpreter handoffs are mostly untranslatable exceptions). Opcode
coverage is essentially done; the current work is the quality of the emitted
code, guided by the in-tree sampling profiler (`LRPS2_PROF=1`), which buckets
PC samples by JIT code cache and resolves the rest against the core's symbol
table.

### EE (R5900) JIT — ~89 %

Everything GT3 and Mega Man X7 execute is translated except SYSCALL and the
TLB/exception path. Interpreter handoffs in-race fell 887k → 136k per 600 frames
across C.62-C.79, and what is left is SYSCALL (123k).

- **Integer/memory**: ALU, loads/stores incl. LQ/SQ and the unaligned family,
  branches incl. likely branches and BC0x/BC1x, MULT/DIV/MADD + HI/LO on both
  pipelines, MFSA/MTSA, ADDI/DADDI and ADD/SUB with the overflow trap dropped
  exactly as the x86 rec does. A likely branch is no longer refused by an
  untranslatable delay slot (C.64): GT3 is full of `beql ..., panic; break`
  asserts whose slot only runs when taken, i.e. never — the not-taken path is
  native, the taken path hands off at the branch.
- **COP1 (FPU) complete**: data moves, all S-format arithmetic, CVT, C.cond with
  interpreter-exact clamp/flag semantics, and native CFC1/CTC1 (C.65: `fs` is a
  compile-time constant, so CFC1 specializes to an FCR31 load / revision constant
  / zero and CTC1 to a non-FCR31 register is a nop).
- **MMI → NEON**, bit-exact, now including PPAC5 (per-word RGB1555 pack) and the
  full-128-bit PMFHI/PMFLO/PMTHI/PMTLO (C.79, `LRPS2_NO_EE_MMIHL=1` restores the
  handoff), plus PLZCW (`CLS` per half) and PREVH (`REV64 .8H`). PMFHL/PMTHL stay
  with the interpreter deliberately — they never fire in the reference titles, so
  a native path could not be golden-verified.
- **LQC2/SQC2 native** (C.59): vu0Sync plus a 128-bit access straight into
  `vuRegs[0].VF[ft]`, so the EE register cache survives an op MMX7 runs ~827K
  times (`LRPS2_NO_EE_VUQMEM=1`).
- **ERET native** (C.77) — the single biggest remaining breaker at 196k per 600
  in-race frames, since every interrupt return broke its block. It is a branch
  with no delay slot (pc from ErrorEPC/EPC by `Status.ERL`, clear that bit, pull
  `nextEventCycle`), and its tail deliberately does no cycle flush and no event
  test, mirroring the interpreter-handoff runner it replaces; the next block's
  gated tail delivers the pending interrupt. Adding a flush+test instead shifted
  MMX7's FMV frame phase. `LRPS2_NO_EE_ERET=1` restores the handoff.
- **Memory goes through the vtlb fastmem area** (C.50): the 4 GB host mapping in
  which every memory-backed guest page sits at its own guest address, so a
  load/store is one `ldr/str <data>, [x28, w<addr>, UXTW]` with no vmap
  indirection. Hardware registers and unmapped pages are absent and therefore
  fault; the faulting instruction is rewritten into a branch to a slow stub
  emitted beside it, and the guest pc is blacklisted so later compiles skip
  fastmem (`LRPS2_NO_FASTMEM=1`, `LRPS2_FASTMEM_LOG=1`). The canonical unaligned
  pairs (LDL/LDR, LWL/LWR, SDL/SDR, SWL/SWR — adjacent, left-then-right, matching
  registers and offsets) fuse into one host unaligned access, with a fault stub
  that replays the exact two-access guest-order semantics for hw pages (C.67,
  `LRPS2_NO_EE_UPAIR=1`).
- **Block plumbing**: block linking/chaining that keeps the frame live across
  hops, a block revalidation cache, a write-back GPR register cache with
  direct-to-cache emission, kernel idle-loop skip, and event tests gated on
  `cycle >= nextEventCycle` in block tails rather than at every branch — the
  upstream-rec dispatcher rule, ~23 % faster wall clock on GT3
  (`LRPS2_NO_EVTGATE=1`, `LRPS2_NO_INLINE_UPD=1`).
- **Emitted-code quality** (C.46-C.57): cpuRegs through the x19 guest-reg base
  instead of a 4-instruction absolute per access; displacements folded into the
  address add; the fastmem base pinned in x28 for the life of the frame;
  backpatch stubs, misalign cancels, event-due tails and the frame pop all moved
  out of line past the epilogue; the chain epilogue's mirror mask and RAM range
  check fused into one test; an exit whose successor is known at compile time
  chains through a constant LUT slot with no pc read at all.

Translating PLZCW also fixed a timing deviation: a block that breaks at an
untranslatable op flushes its cycle count there, a rounding point the interpreter
does not have. `LRPS2_DUMP_RANGE=lo:hi` dumps a pc range's guest ops with their
translatability, `LRPS2_DUMP_HOST=<pc>` writes a block's host code for
`objdump -D -b binary -m aarch64`.

### IOP (R3000A) JIT — ~98 %

Handoff coverage is closed: `LRPS2_IOP_HANDOFF_STATS=1` shows exactly two
breaker classes left on both test titles — the J import stub (HLE hook, by
design) and SYSCALL. GTE/COP2 is PS1-only and never fires.

Native ALU including the trapping forms ADDI/ADD/SUB (this interpreter drops the
overflow trap, so they are the U ops) and RFE (C.68, `LRPS2_NO_IOP_C68=1`),
aligned and unaligned loads/stores, branches + delay slots, block linking,
MULT/DIV + HI/LO, COP0 moves, a write-back GPR register cache in w22-w27
(`LRPS2_NO_IOP_REGCACHE=1`), inline RAM fastmem for loads
(`LRPS2_NO_IOP_FASTMEM=1`; the remaining helper traffic is genuine HW-register
polling), and tail-gated event tests (`LRPS2_NO_IOP_EVTGATE=1`). Only the module
-import stub form `j ...; li $zero, fn` stays interpreted, and its HLE resolution
is cached per stub pc (C.71) — the backward magic scan, `std::string` allocation
and library-name compare chain used to run on every stub execution, millions of
times a minute (`LRPS2_NO_IOP_JSTUB_CACHE=1`). A straight-line run longer than
the block cap chains to the next block instead of handing its tail to the
interpreter (C.69, `LRPS2_NO_IOP_CAP_CHAIN=1`), and an interpreter-tail block
covers its breaker word for SMC invalidation, so code loaded over what was data
at compile time (IRX module loads) recompiles instead of handing off forever
(C.70). The kernel idle spin is fast-forwarded to the next event or the end of
the cycle budget rather than emulated an iteration at a time (C.47, was 18 % of
all JIT time, `LRPS2_NO_IOP_IDLE=1`).

### VU1 recompiler — ~70 %

**microVU1 (armsx2 aVU transplant) is the default provider**: native AArch64 VU
codegen, MVU_DIFF-verified register-exact against the interpreter on both test
titles, byte-identical framebuffers through 12000-frame runs. Instant-VU1 and
MTVU are default-on with it.

In-race profile facts (GT3 save state): the EE never waits on MTVU (SyncStats
0.00 s), so VU1 speed does not gate wall time, and mvu1 is ~75 % of the MTVU
thread but FLAT — 150 blocks, hottest 5.6 % of JIT. There is no per-block target,
only systematic emitter quality, and the passes done so far shrank the hottest
block 1776 → 1292 host bytes (−27 %, adrp count 54 → 9): the page offset of
absolute accesses folds into the load/store (C.72 `armAbsMemOperand`), the naive
copy+4×Ins `mVUshufflePS` permute became the cheap NEON form where one exists
(C.73), `&mVU` is pinned in x27 with the hot per-op scalars moved ahead of the
~290KB `prog` member so they encode as scaled immediates (C.74), and the emit
-constant tables (clamp bounds, FTOI/ITOF scales, EFU polynomials) are embedded
in `microVU` itself so a clamp constant is one `[x27 + imm]` (C.75). The
register-exact MVU_DIFF shadow run is byte-identical to the pre-C.72 baseline.
`LRPS2_PROF` attributes mvu0/mvu1 samples per guest block and
`LRPS2_DUMP_HOST_MVU=0x<pc>` dumps a block's host code. Fallbacks:
`LRPS2_VU1REC_PAIR=1` (interp-pair rec), `LRPS2_NO_VU1REC=1` (interpreter).

### VU0 / COP2 — ~78 %

**microVU0 runs VU0 micro programs** (VCALLMS) natively, and **COP2 macro ALU
ops emit native NEON directly into EE JIT blocks** (aVU_Macro single-op emitters)
with real flag liveness (C.66): the analysis runs over a run of consecutive macro
ALU ops, capped at the block end, and only the last writer of each flag category
computes it — the x86 rec's `COP2FlagHackPass`, here run-local because this
builder interleaves analysis with emission. Dead intermediate MAC/status
pipelines (~40 host instructions per op) vanish (`LRPS2_NO_COP2_LIVENESS=1`).
BC2 branches are native, and so are the COP2 transfers QMFC2/CFC2/QMTC2/CTC2
(C.58): the interlock bit is known at compile time, so the
`_vu0FinishMicro`/`_vu0WaitMicro` call is only emitted when set, and CFC2's REG_R
quirk (writes UL[0], leaves UL[1]) is reproduced exactly. CTC2 to FBRST/CMSAR1
and CALLMS/CALLMSR stay on the interpreter call, faithful to x86.

C.78 is macro emission quality, which sits on the EE critical path (VU0 macro
runs in the EE thread): vuRegs is addressed through an adrp+pageoff fold instead
of a 4-instruction absolute per access (`LRPS2_NO_COP2_ABSFOLD=1`); the per-op
macro bracket — VU0-busy check, x19 repoint to `&vuRegs[0]`, x19 restore, ~11
instructions — is hoisted to once per run of consecutive native macro ALU ops,
which is sound because nothing else is emitted between them, events only fire at
block tails, and no run op can start a VU0 microprogram
(`LRPS2_NO_COP2_RUNHOIST=1`); and x27 = `&microVU0` is materialized inside the
bracket so mvuAbsMem's one-instruction addressing and the C.75 embedded tables
apply in macro mode too (`LRPS2_NO_COP2_MX27=1`). Hottest in-race block 1860 →
1540 host bytes (−17 %). **Footgun captured in the run predicate**: VNOP/VWAITQ
have empty emitters, so a hoisted bracket ending on one never emits its x19
restore — GT3's VU0→GS upload loop ends `VSQI, VSQI, VNOP, VNOP` and the
following branch then read its GPRs out of VF registers (instant wedge); they are
excluded from runs. `LRPS2_NO_VU0REC=1` / `LRPS2_NO_EE_COP2MACRO=1` /
`LRPS2_NO_EE_COP2XFER=1` fall back.

### Persisted VU program cache — ~97 %

A JIT warm cache for microVU, aimed at the per-launch VU recompilation stall
(the emitted code is in-memory only, so every launch re-JITs from cold). Off by
default; the **VU JIT Cache** core option (arm64, restart) turns the whole
feature on — record what is missing *and* hydrate what is already there, so the
store fills itself over a few sessions, under `<frontend cache>/vujit`. The env
vars stay for debugging and override the option: `LRPS2_VU_PROGCACHE=1` records,
`_HYDRATE=1` hydrates (VU0 only unless `_VU1=1`), `_DIR=` picks the store,
`_STATS=1`/`_SELFTEST=1` dump counters and run the verifiers. Every stage is verified bit-identical against the
goldens (GT3 1500f 106138289/860124, MMX7 1500f 111415017/800725).

A recorder captures each aVU emission episode (code chunk + the blocks entered
inside it), a classifier decodes the chunk's ARM64 stream and turns every baked
address into a relocation form (movz/movk quartet, ADRP page, B/BL, conditional
branch) plus a target class — the core `.so` image (bss statics included), the
code-cache arena, this program's `microBlock`s, or the VU's Mem/Micro buffers.
A value in no process mapping is a constant, not an address; recording forces
canonical fixed-length movz/movk so every absolute sits in a re-encodable slot.
Programs serialize to content-addressed `.vuprog` images keyed on the guest
microprogram, and `mVUtryHydrate` (hooked ahead of the recompile in
`mVUsearchProg`) loads one in a later process: chunks are placed at the code
cursor, `microBlock`s re-created in the block managers, every fixup rebased by
its per-class ASLR delta. Rebasing by delta rather than refusing on a base
mismatch is what makes it work at all in a libretro `.so`. Round-trip,
placement-independence and hydration-resolve self-tests cover the format and the
rebase math before either touches the live cache.

Four bugs stood between that and real gameplay, all found from a deterministic
in-race GT3 save state:

- `mVUcompileJIT`'s epilogue rewound `codePtr` onto chunks a nested hydration had
  just written, so the next compile aliased live code.
- The entry block was chosen by `startPC` alone, handing callers a block baked
  for a different pipeline state.
- Both are fixed by closing and reopening the caller's emit session at the
  post-hydration cursor (a pure move — hydration emits nothing through the
  assembler), which makes compiling *after* hydrating safe; the entry then goes
  through the normal `mVUblockFetch`, matching pState and compiling variants the
  recording never produced.
- Cross-chunk **conditional** branches were not modelled: the classifier knew
  only B/BL imm26, so a `B.cond` reaching out of its chunk survived unrelocated.
  Harmless while chunks were hydrated one at a time, wrong once several are
  packed contiguously — which is why multi-chunk programs used to render
  incorrectly and stall for seconds. They are fixups now (imm19 and imm14 forms),
  and ADR / LDR-literal, which have no patch, drop the episode rather than be
  persisted silently.

The header also carries the writing core's GNU build-id and hydration rejects
any other build's images — the image-relative rebase only holds for the exact
layout that recorded them, so a rebuild invalidates the cache by design. That
makes housekeeping mandatory rather than optional: the first time a run touches
the store it drops every image this build can never load (otherwise each core
update would orphan the whole store and leave it there forever) and evicts
oldest-first while the total exceeds a soft cap, 128 MB by default
(`LRPS2_VU_PROGCACHE_MAXMB`). Only the 88-byte header is read per file. A
program re-saved during a session gets a fresh timestamp, so the working set of
whatever is being played survives eviction; the cap is checked at first use, so
a single long session can still overshoot it.

Warm starts *replay*, they do not *extend*: do not enable recording and
hydration together, or a run re-records episodes whose branches reach into
foreign chunks.

Measured on GT3 in-race (600 frames): total wall time is unchanged — emulation,
not VU compilation, dominates — while the compile hitches show up in the tail:
stalls ≥50 ms 10 → 5, p99 55.7 → 51.4 ms, first frame 370 → 214-312 ms. All 63
in-race VU1 programs hydrate per warm start, bit-exact across repeated runs.

Still to come: validation across more titles, and turning the option on by
default once that holds. Longer term the relocation layer should record its
fixups from the emitter as it emits, instead of decoding the finished byte
stream — the scanner is why cross-chunk conditional branches were missed, and
why forms it cannot patch (ADR, LDR-literal) have to drop the episode.

### VIF unpack dynarec — ~100 %

NEON unpack kernels (armsx2 transplant); portable C fallback (`LRPS2_NO_VIF_DYNAREC=1`)

### SMC / overlays

Compiled pages are write-protected; faults invalidate stale blocks (vtlb `mmap_MarkCountedRamPage` flow). A page the game keeps *writing* would otherwise ping-pong forever — fault, unprotect, drop blocks, revive them, re-protect, fault again — at a SIGSEGV plus four `mprotect` calls a cycle (the RAM page and its alias in the fastmem area). After `kManualClears` clears a page is therefore left writable and its blocks check their own source words at entry instead (C.60), which is where the chained entry point sits, so a chained jump validates too; the check is handed its block record at compile time rather than looking itself up (C.61). On GT3 that took the run from 48006 protects / 47803 clears to 251 / 49 — two pages had been ping-ponging ~23k times each — and `mprotect` left the profile entirely. `LRPS2_NO_EE_MANUAL=1` restores protect-on-every-revive, `LRPS2_SMC_STATS=1` dumps the per-page protect/clear counts

### MTVU (VU1 thread)

**Default-on** with microVU1 (≥3 cores; `LRPS2_NO_MTVU=1` disables). Partial-packet flush protocol prevents the continuous-microprogram livelock; with an interp-style VU1 provider MTVU stays opt-in (`LRPS2_MTVU=1`). **Lazy VIF-unpack kick (C.80)**: an unpack packet publishes its ring data but defers the WorkSema wakeup RMW to the next flush point (any other MTVU command, `WaitVU`, `Close`, or the VIF1 DMA/MFIFO end-of-transfer tails) — in-race GT3 82 % of the notifies disappear (`LRPS2_NO_VIF_LAZYKICK=1` restores per-packet notify, `LRPS2_MTVU_STATS=1` reports the counts)

### GS renderer

Standard **Vulkan** renderer (`pcsx2_renderer = "Vulkan"`). paraLLEl-GS requires GPU features (8/16-bit storage, small subgroups) that e.g. Adreno 618 lacks

### MTGS

Works (GS thread active)


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
| `LRPS2_NO_EE_MMIHL=1` | Route PPAC5 + PMFHI/PMFLO/PMTHI/PMTLO (C.79) to the interpreter |
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
| `LRPS2_IOP_HANDOFF_STATS=1` (+`LRPS2_IOP_HANDOFF_PC=<key>`) | IOP-side interpreted-op histogram + first-op breaker table (+ pinpoint the pcs behind one breaker key) |
| `LRPS2_PROF=1` (+`_HZ`, `_MAX`) | In-tree sampling profiler: per-thread CPU-time SIGPROF sampling, bucketed by JIT code cache, symbolized against `.symtab`, with per-guest-block hot lists |
| `LRPS2_PROF_LR=sub1,sub2` | Caller attribution for leaf symbols (memcpy/memcmp/`__aarch64_*`): samples matching a substring get their LR symbolized into a per-thread caller histogram |
| `LRPS2_SYNC_STATS=1` | Wall-clock blocking at the EE↔GS/MTVU sync points (who actually waits on whom) |
| `LRPS2_SPU_STATS=1` / `LRPS2_SPU_MUTE=1` | SPU voice-shape statistics / mute the mixer to re-measure its wall-time ceiling |
| `LRPS2_SPU_SYNC_STATS=1` | SPU worker-thread feasibility: register read/write/DMA rates, armed-IRQ tick fraction, and which site (mixer/reverb/ADMA-input/DMA/register) each IRQA match came from |
| `LRPS2_MTVU_STATS=1` | C.80 lazy-kick effect: VIF unpack packets deferred vs MTVU notifies issued |
| `LRPS2_VU_PROGCACHE=1` (+`_HYDRATE=1`, `_VU1=1`, `_DIR=<path>`, `_STATS=1`, `_DUMP=1`, `_SELFTEST=1`) | Persisted VU program cache: enable emit-time recording (+ load the on-disk cache instead of recompiling; + include VU1, off by default; + cache directory, **required** — no default, so a missing `_DIR` silently makes a "warm" run cold; + per-VU episode/fixup counters; + per-hydration tracing; + serializer round-trip self-test). Record and hydrate are separate on purpose: populate with `_PROGCACHE=1`, warm-start with `_HYDRATE=1` alone |
| `LRPS2_DUMP_HOST=<pc>` / `LRPS2_DUMP_HOST_IOP=<pc>` / `LRPS2_DUMP_HOST_MVU=<pc>` | Write a compiled block's host code (+ guest words) for offline `objdump -D -b binary -m aarch64` |
| `MVU_DIFF=1` | microVU1-vs-interpreter register-exact shadow differential (needs instant VU1 — even an empty `LRPS2_NO_VU1_INSTANT=` disables it and poisons the log) |

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

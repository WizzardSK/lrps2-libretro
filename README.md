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

Overall recompiler-suite progress: roughly **~76 %** (weighted by remaining
work; dynamic instruction coverage on the tested titles is much higher — the
vast majority of EE+IOP instructions already execute natively, and the
remaining interpreter handoffs are mostly untranslatable exceptions).

| Component | ~Done | State |
|---|---|---|
| EE (R5900) JIT | 82 % | Native ALU, loads/stores incl. LQ/SQ (inline vtlb fast path) + unaligned family, branches + likely branches + BC0x/BC1x, block linking/chaining, MULT/DIV/MADD + HI/LO (both pipelines), **complete COP1 (FPU)** — data moves + all S-format arithmetic + CVT + C.cond, interpreter-exact clamp/flag semantics (only CFC1/CTC1 interpreted), bit-exact MMI subset → NEON, MFSA/MTSA(B/H), kernel idle-loop skip, block revalidation cache, write-back GPR register cache with direct-to-cache emission, event tests gated on `cycle >= nextEventCycle` in block tails (the upstream-rec dispatcher rule; ~23 % faster wall clock on GT3, `LRPS2_NO_EVTGATE=1` restores every-branch tests), default-rate cycle bookkeeping inlined into block tails (`LRPS2_NO_INLINE_UPD=1` restores the helper call), block chaining keeps the frame live across hops (no per-block frame pop/push or x19 rematerialization; same on the IOP side). ADDI/DADDI (overflow trap dropped, exactly like the x86 rec). Missing: SYSCALL/ERET/TLB/COP0 CO-format (interpreter forever) |
| IOP (R3000A) JIT | 95 % | Native ALU, aligned + unaligned (LWL/LWR/SWL/SWR) loads/stores, branches + delay slots, block linking, MULT/DIV + HI/LO moves, COP0 moves (MFC0/CFC0/MTC0/CTC0), write-back GPR register cache (w22–w27, `LRPS2_NO_IOP_REGCACHE=1` to disable), inline RAM fastmem for loads (`LRPS2_NO_IOP_FASTMEM=1` to disable; the remaining helper-call traffic is genuine HW-register polling), event tests gated on `cycle >= iopNextEventCycle` in branch tails (`LRPS2_NO_IOP_EVTGATE=1` to disable); J native (only the module-import stub form `j ...; li $zero, fn` stays interpreted for the HLE hook); RFE/syscall via interpreter |
| VU1 recompiler | 60 % | **microVU1 (armsx2 aVU transplant) is the default provider**: native AArch64 VU codegen, MVU_DIFF-verified register-exact against the interpreter on both test titles, byte-identical framebuffers through 12000-frame runs. Instant-VU1 and MTVU are back to default-on with it. Fallbacks: `LRPS2_VU1REC_PAIR=1` (interp-pair rec), `LRPS2_NO_VU1REC=1` (interpreter; both restore conservative non-instant/opt-in-MTVU behavior) |
| VU0 / COP2 | 60 % | **microVU0 runs VU0 micro programs** (VCALLMS) natively; **COP2 macro ALU ops emit native NEON directly into EE JIT blocks** (aVU_Macro single-op emitters, conservative all-flags-live gating); BC2 branches native. CALLMS/transfer ops via inline interpreter calls (faithful to x86). `LRPS2_NO_VU0REC=1` / `LRPS2_NO_EE_COP2MACRO=1` fall back |
| VIF unpack dynarec | 100 % | NEON unpack kernels (armsx2 transplant); portable C fallback (`LRPS2_NO_VIF_DYNAREC=1`) |
| SMC / overlays | Compiled pages are write-protected; faults invalidate stale blocks (vtlb `mmap_MarkCountedRamPage` flow) |
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

## Project Details

Upstream PCSX2: https://pcsx2.net/ — this core inherits its GPL/LGPL licensing;
see the source headers. The arm64 AArch64 emitter is [VIXL](https://github.com/Linaro/vixl)
(vendored under `3rdparty/vixl`).

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

| Component | State |
|---|---|
| EE (R5900) JIT | Native ALU, loads/stores incl. LQ/SQ (inline vtlb fast path), branches + likely branches, block linking/chaining, MULT/DIV + HI/LO, COP1 data moves, bit-exact MMI subset → NEON, kernel idle-loop skip |
| IOP (R3000A) JIT | Native ALU, aligned loads/stores, branches + delay slots, block linking; mult/div/COP0/unaligned via interpreter |
| VU0/VU1 | Interpreter (VU recompilers are future work). Instant-VU1 is disabled on arm64 — continuous VU1 microprograms would spin the 3M-cycle budget per kick; VU1 runs in interleaved slices (`LRPS2_VU1_INSTANT=1` restores instant mode). VU1-render-heavy scenes (e.g. GT3's attract demo) are still interpreter-bound and run far below full speed |
| VIF unpack | Portable C path (x86 SSE dynarec tables are bypassed) |
| SMC / overlays | Compiled pages are write-protected; faults invalidate stale blocks (vtlb `mmap_MarkCountedRamPage` flow) |
| MTVU (VU1 thread) | **Opt-in** (`LRPS2_MTVU=1`, ≥3 cores). Works and is pixel-verified on VU-light titles; games driving a continuous VU1 microprogram (e.g. GT3's attract) deadlock the one-packet-per-program handoff with the VU1 interpreter — needs a partial-flush protocol |
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
| `LRPS2_FAULT_LOG=1` | Log vtlb page-fault handler activity |
| `LRPS2_JIT_STATS=1` | JIT compile statistics (e.g. likely-branch counts) |

## Project Details

Upstream PCSX2: https://pcsx2.net/ — this core inherits its GPL/LGPL licensing;
see the source headers. The arm64 AArch64 emitter is [VIXL](https://github.com/Linaro/vixl)
(vendored under `3rdparty/vixl`).

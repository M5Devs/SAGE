# Project memory (SAGE)

Repo: `M5Devs/SAGE` — "Sega Arcade Golden-era Emulator", modern C++20 + CMake.
Targets **Dead or Alive** (Sega **Model 2B**) and, in progress, **Virtua
Fighter 2 / vf2** (**Model 2A**). `README.md`, `LICENSE` (GPL) pre-existing.

**Index of durable facts. The full narrative of every root cause below (with the
diagnostic that found it and the exact verification numbers) is archived in
`archive-sage-history.md` — read it before re-deriving any of these.**

## Build & run
- `cmake`/`ninja` are **not** preinstalled and are **not** on PATH after a
  session/container reset. `pip install --user cmake ninja` → `~/.local/bin`
  (add to PATH). **A build with cmake missing is a silent no-op that leaves a
  stale `build/sage`** — always `export PATH="$HOME/.local/bin:$PATH"` and check
  the binary timestamp / `strings build/sage` for a marker you just added.
- g++ 14.2, 4 cores. `file`, `ffmpeg`, `Pillow` absent by default (`sudo -n
  apt-get update && apt-get install -y ffmpeg`; `pip3 install Pillow numpy`).
  miniz via FetchContent tag **3.1.2** (`MINIZ_STANDALONE_PROJECT OFF`).
- Run: `./build/sage roms/<set>.zip <game> [instructions]`. Env hooks:
  `SAGE_TRACE`, `SAGE_WATCH=lo:hi` (+`SAGE_WATCH_R`), `SAGE_DISASM=hex[:n],...`,
  `SAGE_PPM` (composited frame), `SAGE_PPM3D` (isolated 3D layer),
  `SAGE_RAWVIDEO=<path>` (RGB24 496x384/frame), `SAGE_HEADLESS`, `SAGE_WAV`.
- Release builds are **deterministic** (byte-identical frames across rebuilds).
- `.gitignore`: `build/`, `references/`, `roms/`, `*.zip`.

## Git / push
- `origin` is `https://github.com/M5Devs/SAGE.git` (public, branch `main`). The
  embedded credential is **stale**: `git push origin main` hangs on a password
  prompt. Use `git push "https://${GITHUB_TOKEN}@github.com/M5Devs/SAGE.git"
  main`. `git config` user is `openhands` / `openhands@all-hands.dev`.
- Screenshots live in `docs/screenshots/` (PPM→PNG, 2× NEAREST).

## ROM loading
- `roms/doa.zip` (archive.org item `m2emu1.1a`, nested space-containing path —
  percent-encode) is actually the MAME **`doab`** set (differs from `doa` only in
  the two main-CPU ROMs). `vf2.zip` is the same item's Model 2 set.
- **Resolve ROMs by CRC32 first, then filename.** Zip filenames can differ from
  MAME's. Known alt dumps are reported "OK (alt dump)", not failures.
- MAME macro semantics (`ROM_LOAD*`): see `archive-sage-history.md`; placement is
  `dst = off + k*(group+skip) + (rev ? group-1-j : j)`. Post-load, non-native
  regions are byte-swapped (`samples` only here).

## Environment quirks that cost time
- `caches`/SDL/apt/ffmpeg/Pillow reset with the container; reinstall as above.
- A large `cat > f <<'EOF'` heredoc once **silently kept the old file content**
  (tool said success). For big writes use `file_editor create` (after `rm`) and
  `grep -c` a marker before building.
- `.openhands/memory/legacy-openrom-memory.md` is stale content from an unrelated
  OpenROM Marketplace project — ignore it.

## Root causes already solved (details in `archive-sage-history.md`)
Model 2B / DOA (all fixed, verified):
- **i960 `sext()` must mask before sign-extending** (corrupts 13-bit COBR
  displacement otherwise). Same class of bug hit the SHARC core.
- **i960 BURST flag**: `ldq/ldt/ldl/stq` only advance the address when the target
  reports BURST. Must be true for ROM/RAM/geo_prg/copro_function/luma, **false**
  for the CPU wait-state block `0x00e00000`.
- **`0x06000000` aliases `main_data[0x1000000]`** (not `[0]`) — caused the DOA
  "Motion Initialize Err".
- **317-0229 protection registers** (`0x01d87ff0…`, status `0x01d8400c`) overlay
  the protection RAM tail and must be decoded *before* the `map()` fallback.
- **SHARC must start HALTED**; cleared only by `copro_ctl1_w`.
- **SHARC/i960 FIFO flow control**: MAME's `generic_fifo_u32_device` couples the
  CPUs (empty-out → `i960_stall()`, full-in → HALT). `copro_fifo_sync()` mirrors
  this with an early-out when the SHARC is parked on an unsent command.
- **GEO windows** (`0x00800000-0x00803fff`, `0x00804000-0x00807fff`) must route to
  the register handlers, not plain RAM, or `geo_write`/`geo_prg_w` are dead code.
- **Luma RAM** is `umask16(0x00ff)` on 2B (one byte per half-word; index
  `(addr-base)>>1`, 0x8000 bytes). Treating it as a byte window flattens shading.
- **System 24 tile decode**: byte index XOR 1 (`code*32 + ((y*4+(x>>1))^1)`,
  high nibble = even x). Omitting it swaps pixels in pairs ("almost right").
- **3D compositing**: sprite pass draws cells whose tile-RAM `0x6000` priority bit
  is set, background the clear cells; `opaque` false on the sprite pass. H/V sync
  regs `0x01040000`/`0x01060000` must reach the video CRTC offsets.
- **Scanline streaking** was a row-pitch mismatch: `m_dest` is 512-wide
  (`kDestStride`), readers walked 496.
- **SHARC `sext()` masking bug** was the copro FIFO over-read (17,164 → 0).

## Model 2A (vf2) — current state
- **ROM loader** (`src/rom_loader.cpp`, table-driven, commit `9d50d34`): vf2 table
  transcribed from MAME `ROM_START( vf2 )` + `MODEL2A_VID_BOARD`; **36/36 ICs
  verified**. doa/doab tables preserved verbatim.
- **TGP coprocessor** (`include/sage/cpu/tgp.h`, `src/cpu/tgp.cpp`, commit
  `84fb1de`): `mb86233::Tgp` core + `TgpCoprocessor` (host FIFOs, bank reg,
  sincos/atan/inv/isqrt units). `Bus` selects it via `is_model2a()` (presence of
  the `copro_tgp_tables` region). Runs at 2/3 the i960 rate.
- **Model 2A map differs from 2B**: luma `0x12800000` (0x20000, one byte/word),
  texture RAM0/1 `0x12000000`/`0x12400000` (0x200000 region, `.mirror`), UART
  `0x01c80000` (umask16), `0x01c00040-43` is `.nopw()` (write-only nop).
- **GEO function-port READS must be wired** (commit `e4acd68`): `geo_r` returns
  the display-list write pointer at `0x00802008` and read pointer at `0x00803008`.
  Only the writes were decoded, so the i960 read open bus and vf2 spun forever at
  PC `0x00002EFC`. Fix: decode `0x00800000-0x00803fff` in `read_register` (byte
  lanes), `read16`, `read32` → `Geometry::geo_read`. Effect (1500 frames):
  unmapped reads 285699 → 9235, polygons rendered 9052 → 111113, i960 reaches its
  idle loop and runs attract mode. 36/36 ICs still OK.
- `0x01c00040` (CRX cabinet link) is **written but never read** by vf2 — a
  fabricated "link ready" read there is dead code; don't add it.
- Remaining vf2 unmapped reads are 16-bit accesses to `0x00884000` (copro FIFO);
  only `read32` decodes that window. Low volume, not a stall.
- **Tilemap scroll sign (vf2 sky bar)**: `draw_tile_layers` walks the source map
  and adds the scroll (`screen = source + scroll`), but segaic24 blits the map at
  `(-hscr, +vscr)` (`source = screen + scroll`). So the loop must **negate both**:
  `hscr = +hscr_reg`, `vscr = -vscr_reg`. HEAD had `hscr` negated and `vscr`
  **not**, so vf2's title backdrop (layer 2, `vscr=0x2056`) was sampled 0x56 rows
  low → olive `#41412e` bar across the top 86 rows instead of sky `#1d3576`.
  DOA never sets these regs (`hscr=vscr=0`), so the bug was invisible there.
  (`ctrl & 0x6000` mode-1 split path is a separate, still-unported gap; for vf2 it
  degenerates — `boundary=(-vscr)&0x1ff=426 > 384` — to a plain full-screen draw.)
- **Open**: `SAGE_PPM3D` still rasterises alpha-only black (`0xFF000000`) although
  polygons reach the rasteriser (~95k dest px at frame 1200). Solid shading
  `draw_scanline_solid` looks up `colorxlat`/`palram` (colorbase) that vf2 has not
  populated — the next 3D-render gate.

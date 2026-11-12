# Project memory (SAGE)

Repo: `M5Devs/SAGE` — "Sega Arcade Golden-era Emulator", modern C++20 + CMake.
Target game: **Dead or Alive**, Sega **Model 2B** (i960 main CPU + 315-5881 GPU +
SCSP sound). `README.md`, `LICENSE` (GPL), and `.openhands/` were pre-existing.

## Final Graphics Polish audit (2026-11-08)
The "four visual artifacts" task (colored/solid shadows, untextured background,
face/hair alpha, missing TECMO/DOA logos) does **not** reproduce. `draw_scanline_solid`,
`draw_scanline_tex`, `check_culling` and `process_polygon` are byte-faithful to MAME
(references/i960/* verified identical to upstream master). Evidence: feet strip is
dark grey (#4a4a4a), 3D layer @1500 = 703 colours, TECMO(red)+DOA title render at
f1070-1140. Do not "fix" these by inventing changes. Real latent gap:
`Video::draw_tile_layers` omits MAME's `ctrl & 0x6000` window/line-scroll modes and
the `0x6800` mask base; DOA never uses them, so no effect.

## Clean-capture screenshots (2026-11-09, commit `11fd7cb`)
- `doa_sky_fixed.png` = frame **1200**, `doa_kasumi_fixed.png` = frame **1500**,
  native 496x384 via plain `PIL Image.open(ppm).save(png)` (NO resize). Exporting
  this way reproduces the canonical prior `doa_sky_fixed.png` byte-for-byte
  (md5 `469e6c64df3f5f773419dc564f446522`) — see `2026-11-09.md`.
- The **Release build is deterministic**: a fresh `cmake -B build
  -DCMAKE_BUILD_TYPE=Release && cmake --build build -j4` reproduced prior
  sessions' `/tmp/frame{1200,1500}.ppm` md5 exactly.
- af56890 fix effect (before = parent `f94c9d5`): first differing frame is
  **1144**; change is ~0.2-0.4% of pixels, turning near-black bleed/alpha blocks
  into clean sky/skin. **Green/red pixel masks are identical before vs after** —
  the fix removes dark blocks, not green trees / a red bar.
- `doa_3d_title.png` == `doa_combat_verified.png` == frame 1500 upscaled 2x
  nearest (both md5 `35dd5dc4…`); the latter is the *pre-fix* f1500.

## Build environment
- `cmake` is **not** preinstalled. `python3 -m pip install --user cmake` (4.4.4,
  lands in `~/.local/bin`, add to PATH) works; `pip install --user numpy` too.
  (Older note: `sudo -n apt-get update && sudo -n apt-get install -y cmake
  ninja-build unzip`.)
- g++ 14.2, CMake 3.31, 4 cores. `file` command is absent.
- miniz via FetchContent tag **3.1.2**; set `MINIZ_STANDALONE_PROJECT OFF` before
  `FetchContent_MakeAvailable` so it doesn't pull in its examples/tests.

## ROM source + quirks (learned the hard way)
- `roms/doa.zip` from Internet Archive item `m2emu1.1a` lives at a **nested,
  space-containing path**:
  `.../m2emu1.1a/Sega Model 2 Emu 1.1a and Full Romset/Model 2 Romset (Merged)/doa.zip`.
  Must be percent-encoded or the download 404s. md5 `1fcf21db...6bfd`.
- The archive is actually the MAME **`doab` (Rev B, Dec 6 1996)** set, *not* the
  default `doa` (Rev C). `doa` and `doab` differ **only** in the two main-CPU
  program ROMs (`epr-19379*.15`, `epr-19380*.16`). Loader auto-detects by CRC.
- **Resolve ROMs by CRC32 first, then filename.** The zip stores the sound ROM
  as `epr-19328.30` (MAME: `.31`) and the sample ROMs as `mpr-19329.31` ... `.34`
  (MAME: `.32` ... `.35`).
- `mpr-19324.19` in the zip is `D972201F`, which MAME does **not** list (MAME's
  `doab` wants `0D6BF454`). The m2emu 1.1a DAT documents `D972201F`, so treat it
  as a known alternate dump (data row `alt_crc`), not a loader failure.
- MAME load macro semantics (from `src/emu/romentry.h`, not romload.h):
  `ROM_LOAD32_WORD` = group 2 / skip 2 / no reverse;
  `ROM_LOAD16_WORD_SWAP` = group 2 / reverse;
  `ROM_LOAD32_WORD_SWAP` = group 2 / skip 2 / reverse.
  Placement: `dst = off + k*(group+skip) + (rev ? group-1-j : j)`.
- After loading, MAME `region_post_process()` **byte-swaps non-native regions**
  (host is LE): only `samples` (`ROM_REGION16_BE`) is affected here.
- Region sizes: maincpu 2MiB, main_data 32MiB, polygons 32MiB (ERASEFF),
  textures 8MiB, copro_data 8MiB (ERASE00, all-zero), audiocpu 512KiB,
  samples 8MiB. `ROM_COPY` duplicates main_data 0x1800000->0x1C00000 (4MiB).
- Golden region CRC32s (doab semantics, archive files): maincpu `E64DCBF3`,
  main_data `3EFB6628`, polygons `939D5D9E`, textures `B17541BD`,
  copro_data `1AD2BC45`, audiocpu `31D118A5`, samples `DADF9335`.
- Supermodel (`references/supermodel`) is **Model 3**, so it has no DOA ROM
  tables — it's an architecture reference only.

## Conventions
- `.gitignore`: `build/`, `references/`, `roms/`, `*.zip`.
- `scripts/fetch_assets.sh` downloads references only (MAME sources + Supermodel)
  as of 2026-11-08. It used to auto-download `roms/doa.zip` from archive.org;
  that was removed for legal hygiene — users supply their own ROM set. Still
  exposed as the optional CMake target `fetch-assets` (not in the default build).
- `.openhands/memory/legacy-openrom-memory.md` is stale content from an unrelated
  OpenROM Marketplace project that previously occupied this path; ignore it.

## i960 CPU core + Bus (Task 2, 2026-11-05)
- `src/cpu/i960.cpp` + `i960dis.cpp` are standalone ports of MAME
  `src/devices/cpu/i960/{i960,i960dis}.cpp` (BSD-3-Clause, Farfetch'd / R.
  Belmont). Framework deps removed: `memory_access<>` -> `cpu::BusInterface`,
  `fatalerror` -> `cpu::Fault` exception, `save_item`/`state_add` dropped,
  `util::stream_format` -> local `fmt()`.
- **Critical porting bug to remember**: MAME's `util::sext(v, bits)` must MASK to
  the low `bits` bits before sign-extending. Writing it as `(v ^ m) - m` without
  the mask silently corrupts `get_disp_s` (13-bit COBR branch displacement) and
  makes the CPU jump to garbage (IP became 0x3DA44420 instead of 0x420).
- MAME's `m_icount` is a *remaining budget*; `execute_op` DECREMENTS it. The
  standalone `step()` sets `m_icount = 1` then returns `1 - m_icount` (so a
  `-9` call yields 10 cycles, matching `execute_min_cycles()==1`).
- `BusInterface` reads are `const` (the disassembler takes a `const BusInterface&`);
  the Bus diagnostic counters are therefore `mutable`.
- `BusInterface::burst(addr)` mirrors MAME's per-map-entry
  `flags(i960_cpu_device::BURST)`: ROM/RAM true, register/FIFO apertures false.
  Multi-word loads (`ldl`/`ldt`/`ldq`) only advance the address on BURST.
- Model 2B map implemented (subset of `model2b_0229_mem`): ROM 0x0, work RAM
  0x00200000 & 0x00500000, geo 0x00800000, buffer RAM 0x00900000
  (0x80000-mirror), CPU ctl 0x00E00000 (0x38), IRQ ctl 0x00E80000, timers
  0x00F00000, palette 0x01800000, backup 0x01D00000, main_data ROM
  0x02000000 & 0x06000000, texture RAM 0x11000000/0x11200000
  (0x100000-mirror), luma 0x11400000.
- **Boot is real**: reset reads SAT=0, PRCB=0xB0, entry IP=0x410. The first
  block (0x410-0x458) copies a 26-entry dword table from ROM 0x3D0 to the CPU
  control block at 0xE00000; IP then walks 0x634 -> 0x668 (clearing 32MiB of
  work RAM) -> 0x724C. 6M instructions reaches 0x724C with a few hundred
  thousand unmapped reads (SHARC/uart/comm apertures not yet modelled).
- `main.cpp` takes an optional 3rd arg = instruction count (default 1000) for
  longer soak runs: `./build/sage roms/doa.zip doa 5000000`.

## Task 3: video/boot findings (2026-11-05)
- **doa is a `model2b_0229` set** (317-0229 protection), NOT the SHARC `5881`
  variant. The apparent boot hang is the intended per-frame VBLANK wait, not a
  stall: the game is alive and rendering.
- **The game boots to its own diagnostic screen**: "Motion Initialize Err!!
  ERR ANIME = 0x00B0 / real anime name exit on ACODE.DEF / PUSH H or P or K!!
  then NEXT ANIME Initialize!!". This is because the **SHARC/ADSP-21062 copro is
  stubbed** (the 3D/geometry + motion engine never responds), so this is a
  correct-looking, honest emulator state, not a video bug.
- The tile layer decodes as ASCII in char RAM: `tilemap word = charcode |
  (color<<7) | (0x8000 = category)`, so a rendered row of tile values is
  literally the on-screen text. Decoding layer 0 (0x0000 offset) as 64x64
  `chr(v&0xff)` is the fastest way to read what the game is printing.
- **colorxlat stride is 0x200 bytes** (MAME `m_colorxlat` is `u16[]`; byte
  offsets 0x80/0x4080/0x8080 are the >>1 bases doubled). The earlier 0x100
  stride collapsed the palette. Palette resolves through
  `m_colorxlat[base + ((pal>>shift)&0x1f)*0x200]` then a gamma LUT
  `max((i-64)*255/191, 0)`.
- `Video::map()` mirrors MAME: tile RAM 0x01000000 (mirror 0x110000), char RAM
  0x01080000 (mirror 0x100000). Palette 0x01800000, colorxlat 0x01810000.
- doa's loader verifies 22/23 ICs; `mpr-19324.19` is a known alternate dump
  (D972201F vs MAME's 0D6BF454) — reported as "OK (alt dump)", not a failure.
- Debug env hooks that proved useful: `SAGE_TRACE`, `SAGE_WATCH=lo:hi`
  (+`SAGE_WATCH_R`), `SAGE_DISASM=hex:count`, `SAGE_DUMPVID`, `SAGE_DUMPRAW`,
  `SAGE_PCORDER`, `SAGE_INTTAB`, `SAGE_PPM`, `SAGE_HEADLESS`, `SAGE_REALTIME`.
- `build-rel`/`build` are git-ignored; the SDL build needs
  `LD_LIBRARY_PATH=/opt/sdl2/usr/lib/x86_64-linux-gnu`.

## SAGE emulator bring-up (2026-11-05)
- `scripts/fetch_assets.sh` pulls the MAME `model2.cpp/.h`, a shallow Supermodel
  clone, and `doa.zip` (the archive.org path is nested + space-containing, so the
  URL must be percent-encoded). `roms/`, `references/`, `build/`, `*.zip` are all
  git-ignored.
- **SHARC must start HALTED.** MAME's `model2b_state::machine_reset()` asserts
  HALT on the ADSP21062 and only clears it from `copro_ctl1_w()`. Starting the
  SHARC un-halted makes it fetch garbage before the i960 uploads the program
  (PC ran off to 0x5AE24FEA). With the halt the boot lands exactly on PC=0x20004
  and then sits in its idle loop at 0x20123 — correct.
- MAME i960 device handlers receive **word offsets** (`byte >> 2`), so the copro
  IOP window needs `(addr - kCoproIopBase) >> 2`.
- The i960 main loop at 0x27E0 spins on a VBLANK semaphore at 0x54B150; the
  IRQ0 handler at 0x7700/0x7838 sets it and the main loop clears it — this is
  normal frame sync, NOT a hang. IRQ taken count tracks VBLANK count.
- **SDL2 was absent and there is no `/opt/sdl2`.** It can be installed with
  `sudo apt-get install libsdl2-dev` after `apt-get update` (the snapshot.debian.org
  mirror 503/504s intermittently — retry with `--fix-missing`). `Display::open()`
  now falls back to `SDL_RENDERER_SOFTWARE` when no GPU is present, so
  `SDL_VIDEODRIVER=dummy` opens a window.
- Video is still essentially black (2 distinct colors: black bg + a white
  overlay) — tile/char/colorxlat/palette path is a separate, unfinished task.

## DOA "Motion Initialize Err" root cause (2026-11-05, fixed)
- **Root cause**: SAGE aliased i960 window `0x06000000` to `main_data[0]`.
  MAME's `model2_base_mem` maps it to `main_data[0x1000000]` (the "extra data"
  ROMs). DOA reads its animation tables from `0x06000000`, so the wrong offset
  made the game abort with `Motion Initialize Err!! ERR ANIME = 0x00B0` and park
  on the diagnostic screen forever (SHARC idle loop at `0x020123` was a red herring
  — that is the correct wait-for-command spin; `0x20122` is a `DO ... UNTIL` loop).
  Fixed in `Bus::map()`; the diagnostic error text disappeared immediately and the
  game now reads `0x06000000` (34k+ reads) and renders its font/logo tiles.
- **Second bug**: 317-0229 protection registers (`0x01d87ff0-0x01d87ffb`, status
  `0x01d8400c`) overlay the tail of the protection source-RAM window
  (`0x01d80000-0x01d87fff`). `read16/32` and `write16/32` must decode the register
  block **before** the generic `map()` RAM fallback, else the registers are dead
  code (reads return raw RAM). Matches MAME `model2_0229_mem` (specific handler wins).
- DOA never actually polls the protection status during boot, so that fix alone did
  not change the symptom — but it is still correct for MAME parity.
- Remaining gap to visible gameplay: the 3D polygon rasteriser is not implemented
  (only System 24 tile layers + direct framebuffer render), and the game still
  does not issue SHARC commands (0 non-upload `fifo_w`) — copro command path is the
  next milestone. Screen shows the game's own tile/logo output now, not the error.
- Debug hooks added: `SAGE_SHARC_PC` (PM PC histogram), `SAGE_SHARC_DUMP=base:count`
  (PM 48-bit words around PC), `SAGE_SHARC_EXT`, `SAGE_PROT_TRACE` (removed).

## TASK 6 root cause: no 3D because the GEO path is entirely unwired (2026-11-05)
- DOA pushes ~55k dwords/frame into the GEO **program FIFO** at `0x00804000`
  (`model2b_state::geo_prg_w`) and toggles `m_geo_write/read_start` between 0x0 and
  0x10000 via `0x00801008`/`0x00803008` (double buffering). But `Bus::map()`
  mapped `0x00800000-0x00803fff` and `0x00804000-0x00807fff` to plain RAM
  (`m_geo`, `m_geo_prg`) **before** the register handlers, so `geo_write()`
  and `geo_prg_w()` were dead code: `m_bufferram` was never filled and
  `geo_parse()` was never called. SAGE's `trigger_vblank()` only raised IRQ0.
- Fix: unshadow the two GEO windows (route them to a new `Geometry` module),
  port the geometry engine + rasterizer, and call `geo_parse()` at vblank.
- `pz`/`pu`/`pv` are macros: `p[0]`/`p[1]`/`p[2]` of `poly_manager::vertex_t`
  (MaxParams=4). `raster_state` is heap-allocated in MAME (make_unique) — the
  poly_list is 32768 * ~180B, so keep it off the stack (use vectors).
- MAME screen_update order: tiles {3,2} opaque -> tiles {1,0} transparent ->
  3D polygons -> tiles {(layer<<1)|1} sprites. Tilemap layer L reads map words at
  L*0x1000; scroll regs tile_ram[0x5000+L]/[0x5004+L] (word offsets).
- DOA H/V sync: `0x01040000`=0xFFA4 -> crtc_xoffset = 84-92 = -8;
  `0x01060000`=0xFFD8 -> crtc_yoffset = 130-40 = 90. Renderer defaults are
  xoffs=90, yoffs=-8 (differ from crtc until the game writes them).

## TASK 6 / T4 root cause: 3D is black because the GEO engine is never booted (2026-11-05)
- Instrumented trace of `geo_prg_write()` (0x00804000 FIFO) shows the i960 only ever
  pushes **zero dwords** with `geoctl == 0x80000000` (geo *upload* mode). `set_geoctl`
  never sees the high bit clear, so `geo_prg_write` never calls `push_geo_data`, and
  `m_bufferram` keeps its reset pattern `0x07800F0F` (= GEO "end" opcode, function 0x0F).
- Consequence: `geo_parse()` runs but immediately hits the init pattern and ends with
  0 polygons. The earlier "407k polygons rendered" figure came from `render_one()`
  being invoked on stale/garbage polygon entries, not real DOA geometry.
- So the black 3D is **not** a viewport/clip bug. It is the same root cause already
  recorded for the copro: the i960 stalls before it can hand the geometry list to the
  SHARC, which is the Model 2B path that actually feeds the GEO engine. Fixing it means
  making the SHARC copro command path run (upload program, exchange FIFO commands) so
  the i960 boots geo (`geoctl` high bit cleared) and writes the real display list.
- `m_xoffs=90 / m_yoffs=-8` renderer defaults and `crtc_xoffset`/`crtc_yoffset` from
  `0x01040000`/`0x01060000` are correct; `poly.viewport[]` parsed from window data
  (0x0003) is correct (`vpraw=[8,90,504,474]`).

## T2 root cause + fix: i960/SHARC FIFO flow control (2026-11-05)
- **Symptom**: i960 raced ~100k words ahead of the SHARC; SHARC PC diverged to
  `0x817FCFF1` (bogus dispatch from empty-FIFO garbage), then a hard hang/timeout
  around frame 545 (600 frames never finished).
- **Cause**: SAGE's copro FIFOs were unbounded deques returning 0 on empty reads.
  MAME's `generic_fifo_u32_device` instead *couples the two CPUs*:
  - output FIFO: `pop()` on empty → `m_on_fifo_empty_pre_sync` = `i960_stall()`
    (stalls the i960 for one instruction, retried next timeslice);
  - input FIFO (cap 16): `push()` when full → `m_on_fifo_full_post_sync` =
    `set_input_line(HALT, ASSERT)` on the i960.
  Model2B `machine_start()` (`references/model2.cpp` ~line 241) wires exactly this
  with `setup(16, ...)` for both FIFOs.
- **Fix** (`src/bus.cpp`): `copro_fifo_sync(bool need_out)` runs the SHARC forward
  (64-cycle slices, 4000-iter cap) until the threshold is satisfied. Called from
  `copro_fifo_r` (empty out), `copro_fifo_w` (full in) and `copro_function_port_w`.
  Key detail: **early-out when `need_out && fifo_in.empty() && sharc.pc()` is
  unchanged** — the SHARC is parked on an input command that has not been pushed,
  so spinning can never produce output. Without this the loop burned the full
  4000 iterations and the run took 10+ min for 600 frames.
- **Verified**: SHARC idles correctly at `PC=0x020123` (its real wait loop), 0
  invalid dispatches, 0 empty-FIFO output reads, 1800 frames (30 s emulated) clean.
- Remaining (separate issue, NOT flow control): the i960 parks in its delay loop
  at `0x27E0`/`0x27E8` (`ld 0x54b150,g4; cmpibne`) after ~frame 150. `0x54b150` is
  written by `0x7838` and `0x27FC`, so it is being serviced; 3D stays black because
  `geoctl` is never cleared (GEO never booted) — see TASK 6 note above.
- Debug env gates used while chasing this (kept in tree): `SAGE_SHARC_TRACE`,
  `SAGE_WAITDBG`, `SAGE_SHARC_RING`, `SAGE_SHARC_WIN`, `SAGE_SHARC_BADPC`,
  `SAGE_SHARC_LOOPDBG`, `SAGE_SHARC_CMD`, `SAGE_SHARC_DM`, `SAGE_SHARC_UNMAP`,
  `SAGE_DISASM=<hex[:n],...>` (prints live disassembly at boot).

## 0-polygon root cause: i960 BURST flag on ROM (2026-11-05)
- **Symptom**: `GEO parses: N polygons rendered: 0` — the display list parsed but
  every `object_data` (fn 01) submitted the SAME word 4× as tpa/tha/oba/obc, so
  `obp` pointed at garbage and no polygon ever reached the rasteriser.
- **Root cause**: `Bus::burst()` returned true only for *writable* memory
  (`map(...) && w`). The i960 core's `ldq`/`ldt`/`ldl`/`stq` advance the address
  only when the target reports BURST (`if (flags & kBurst) t1 += 4;`). MAME flags
  the `main_data` ROM region (and maincpu ROM, workram, bufferram, geo_prg,
  copro_function, texture/luma/palette/backup RAM) with
  `i960_cpu_device::BURST`. Without it, `ldq (g5),g4` at PC 0x65CD4 read
  main_data 0x0207A470 four times instead of the real quad
  `002142D8 00944350 00004929 0021000F` → 4 identical operands.
- **Fix** (`src/bus.cpp::burst`): return true for every region MAME flags BURST.
  Practically: `map(addr,4,w) != nullptr` (drop the `&& w`), plus explicit true
  for geo_prg (0x00804000) and copro_function (0x00880000) which have no `map`
  entry, and explicit **false** for the CPU wait-state block 0x00e00000
  (`kCpuCtlBase`), the one plain-RAM aperture MAME leaves non-BURST.
- **Result**: 80677 polygons rendered over 1300 frames; frame dumps show the DOA
  title screen + 3D content (36 unique colours, no longer all-black).
- **Second fix** (`Bus::map`): MAME maps 0x00200000-0x0021ffff as RAM and
  0x00220000-0x0023ffff as **maincpu ROM at +0x20000** (`model2_5881_mem`); SAGE
  had the whole 256 KiB as RAM. Now split accordingly (read-only upper half).
- **How to verify**: `SAGE_HEADLESS=1 ./build/sage roms/doa.zip doa 1300` → expect
  `polygons rendered` in the tens of thousands, not 0. (Polygons start rendering
  ~frame 1050 when the game first uploads its display list; earlier frames are
  legitimately empty because geoctl/boot hasn't happened yet.)
- **Method that found it**: diff SAGE's memory map + `burst()` semantics against
  MAME (`references/model2.cpp` map + `references/i960/i960.cpp` ldq/stq). No
  opcode hunting — the reference cores had the answer.

## Git push auth quirk (2026-11-05)
- The remote's embedded credential (`https://ghu_...@github.com/M5Devs/SAGE.git`)
  is **stale** — `git push origin main` hangs on an interactive password prompt
  (and there is no TTY, so it just times out).
- Working push: `git -c credential.helper= push
  "https://x-access-token:${GITHUB_TOKEN}@github.com/M5Devs/SAGE.git" main`
  (inline, no persisted credential). `GITHUB_TOKEN` is injected from secrets.
- `M5Devs/SAGE` is **public**, default branch `main`; raw files are reachable at
  `https://raw.githubusercontent.com/M5Devs/SAGE/main/<path>`.
- Screenshots live in `docs/screenshots/` (PPM→PNG via Pillow, 2× NEAREST).

## System 24 tile decode: byte-pair XOR (2026-11-05)
- **Symptom**: text glyphs and cloud backgrounds had horizontal combing /
  interleaving — each pair of pixels looked swapped ("almost right").
- **Root cause**: SAGE's `tile_pixel()` read 4 bitplanes LSB-first straight from
  the char-RAM byte array (`code*256 + p + y*32 + x*4`, bit `b&7`). MAME's
  `gfx_element` builds the tile over a **uint16_t** array, so the byte index is
  XORed with **1** (`NATIVE_ENDIAN_VALUE_LE_BE(8,0)` → xormask 0x08 on LE hosts).
  Without that XOR the two bytes of each 16-bit char word are read swapped.
- **Fix** (`src/video.cpp`): decode as 2 pixels/byte, high nibble first:
  `index = code*32 + ((y*4 + (x>>1)) ^ 1)`; even x → `byte>>4`, odd x → `byte&0xf`.
  This is byte-for-byte the validated `sm2_emu` `decode_tile_pixel()`.
- **Ground truth**: `references/sm2_emu/src/hw/segaic24.cpp` has an explicit
  comment on exactly this ("Omitting that swaps pixels in pairs, which looks
  almost right"). `references/segaic24.cpp` `char_layout` = 8x8, planes {0,1,2,3},
  xoffset STEP8(0,4), yoffset STEP8(0,32), charincrement 32 bytes.
- **Addressing was already correct** (do not touch): name tables at word
  0x0000/0x1000/0x2000/0x3000 (= byte 0/0x2000/0x4000/0x6000); hscr/vscr regs at
  `tile_ram[0x5000/0x5004 + (layer>>1)]` → SAGE's `(0x5000+layer)` matches.
- **Method**: simulated MAME's `decode()` (readbit MSB-first, planebit 1<<(3-p),
  xormask 8) vs sm2 vs SAGE in Python — sm2 == MAME exactly, SAGE differed.
- **Env quirk**: `cmake`/`ninja` are pip-installed under `~/.local`; a container
  reset wiped them → `pip install cmake ninja` and re-add `~/.local/bin` to PATH.
  A stale `build/` from a different cmake just reconfigured cleanly.

## SHARC `sext()` masking bug — root cause of copro FIFO over-read (2026-11-07)
- **Symptom**: SHARC read the i960→SHARC input FIFO far more often than the i960
  pushed (17,164 empty-FIFO reads at `pc=0x20D97`), with bogus external read
  addresses (`0x4811D1`, `0x55E192`, …). Polygon output was garbage (177,677
  "polygons" that never changed) yet the emulator looked healthy.
- **Root cause**: `sext()` in `include/sage/cpu/sharc.h` did **not** mask its
  input to `bits` before sign-extending. `op_get_reladdr()` calls
  `sext(opcode >> 27, 6)` — the arg is 20+ bits, so the high bits leaked into the
  result. The opcode `0x6E7E090B0800` (`DM(I7,M1)` post-modify, mod field = 1)
  produced `0xDD9C1` instead of `1`, so `I7` walked out of internal DM
  (`0x3030C`) into the FIFO window after a handful of iterations.
- **Fix**: `raw = value & ((1<<bits)-1); return (raw ^ m) - m;` — byte-for-byte
  MAME's `util::sext` and the `references/sm2_emu/src/cpu/sharc/sharc.cpp` port.
  **One-line-class change; the scheduler was never the problem.**
- **Verification**: empty-FIFO reads 17,164 → **0**; polygons now grow with frames
  (3000f=16,985, 6000f=54,168) and frames render real attract-mode content.
  The old 177,677 figure was garbage from corrupted SHARC state — do not treat it
  as a baseline.
- **Lesson**: when a DAG/index register "wanders", check field-extraction helpers
  (`sext`, `BIT`) against MAME before suspecting the FIFO protocol or scheduler.
  ElSemi's `m2emu` (and MAME) run i960 & SHARC in **separate scheduler
  timeslices**; SAGE's lockstep `tick_copro()` is fine once decoding is correct.
- **Debug env flags used**: `SAGE_SHARC_WIN=lo:hi` (DAG/reg snapshot at a PC),
  `SAGE_SHARC_DUMP=pc:count` (PM48 dump + disasm via `/tmp/sharcdis/sharcdis`),
  `SAGE_SHARC_FIFO`, `SAGE_FIFO_EMPTY`, `SAGE_RAWVIDEO=<path>` (RGB24 frames).

## Video capture recipe (2026-11-07)
- `SAGE_RAWVIDEO=<path>` appends each rendered frame as packed **RGB24 496x384**
  (`src/main.cpp` `write_rgb24()`), so a run feeds ffmpeg without stdout mixing.
  Add `SAGE_HEADLESS=1` to skip the SDL window; the build here is headless anyway
  (SDL2 not found).
- Full capture: `SAGE_HEADLESS=1 SAGE_RAWVIDEO=/tmp/sage_raw.rgb ./build/sage
  roms/doa.zip doa 3600` → exactly `3600*496*384*3` = 2,057,011,200 bytes, ~4 min.
- Encode: `ffmpeg -y -f rawvideo -pix_fmt rgb24 -s 496x384 -r 60 -i /tmp/sage_raw.rgb
  -vf "scale=992:768:flags=neighbor" -c:v libx264 -pix_fmt yuv420p -b:v 2500k
  -movflags +faststart docs/videos/doa_intro.mp4` → 60.0 s, ~10 MB (GitHub-safe).
- **Sequence timing**: black boot ≈0–9 s → logo ≈10 s → full 3D attract mode from
  ≈20 s onward (frame ~1200). Motion continues past frame 1833 (the old stall
  point) with no hang.
- `ffmpeg`/`ffprobe`/`Pillow` are **not** preinstalled → `sudo -n apt-get install -y
  ffmpeg`; `pip3 install Pillow`. Pillow's `Image.getdata` is deprecated (warns).

## 3D layer compositing fix (2026-11-05)
- **Two independent bugs kept Model 2B polygons out of the final frame.**
- **Geometry**: `Geometry::push()` bottom clip plane had operands swapped vs
  MAME `model2_v.cpp`: SAGE had `m_viewport[1] - m_center[i][1]` (→ -192,
  `normal.pz = -0.99999`) instead of `m_center[i][1] - m_viewport[1]` (→ +192).
  That culled **every** polygon with `pz > 0`, leaving the 3D buffer empty.
  One-line operand swap; 3D buffer went 0 → ~19k–23k non-black pixels.
- **Compositing**: `Video::draw_tile_layers(group)` is called twice around the 3D
  draw. The sprite pass (`group != 0`) must draw only cells whose System 24
  screen-space priority bit (tile RAM `0x6000`, 4 words/row, MSB-first per
  8-px cell) is **set**; the background pass draws the clear cells. Condition:
  `pri_bit != sprite_pass → skip`. DoA never writes the mask, so the whole
  background ends up behind the 3D layer — matching MAME `screen_update()`.
  Also: `opaque` must be false on the sprite pass, else pass 1 repaints.
- **bus.cpp**: System 24 H/V sync outputs at `0x01040000`/`0x01060000` must be
  routed to the video CRTC offsets (`write_hsync`/`write_vsync`) **before**
  `is_nopw()`. MAME maps them as 16-bit handlers on the tile device
  (`xhout_w`/`xvout_w` → `horizontal_sync_w`/`vertical_sync_w`). Without this the
  viewport offset never reached the 3D renderer and every polygon projected
  off-screen. Note: `bus.cpp` handles these in both `write8` and `write32`.
- **Verification**: `frame==3D` is now 23233/23233 (frame 1200) and
  18938/18938 (frame 2000); 3D bbox y varies 65→424 across frames (dynamic
  geometry, no longer a static/blank layer). Branch `fix/3d-layer-compositing`.
- **Do NOT** use tile-word bit 15 (category) as the 3D/sprite split: empirically
  it hides the 3D entirely (frame uniq dropped to 25). The screen-space priority
  mask at `0x6000` is the correct discriminator.

## Pushed to origin/main (2026-11-05)
- `fix/3d-layer-compositing` fast-forwarded into `main` and pushed:
  `981f882` (compositing fix) + `c4e5ce1` (memory notes). Remote head then
  advanced by `bc46c1f` (refreshed video + screenshot).
- **Push auth**: the repo's embedded `origin` URL token (`ghu_…`) no longer
  authenticates — `git push` prompts for a password and hangs. Fix:
  `git remote set-url origin https://${GITHUB_TOKEN}@github.com/M5Devs/SAGE.git`
  then `GIT_TERMINAL_PROMPT=0 git push origin main`. (Same quirk noted earlier
  in memory; the old token in the remote is stale.)
- **Video refresh recipe** (post-fix): `SAGE_HEADLESS=1 SAGE_RAWVIDEO=/tmp/sage_raw.rgb
  ./build/sage roms/doa.zip doa 3600` → exactly 2,057,011,200 bytes. Encode with
  **CRF 30 / preset slow** (not the 2500k bitrate from the old recipe — the new
  3D content is much busier and 2500k produced 33 MB). CRF 30 → 14.8 MB,
  mean abs pixel diff 2.31 vs source; CRF 33 → 11.8 MB / 3.0 diff.
  Command: `ffmpeg -y -f rawvideo -pix_fmt rgb24 -s 496x384 -r 60 -i /tmp/sage_raw.rgb
  -vf "scale=992:768:flags=neighbor" -c:v libx264 -crf 30 -preset slow
  -pix_fmt yuv420p -movflags +faststart docs/videos/doa_intro.mp4`.
- **Best screenshot frame**: 2700 (64,844 3D px = highest of the run; frame 1500
  peaks ~33k, 2760 ~44k). `SAGE_PPM3D` dumps the raw 3D layer for measuring
  coverage. Screenshot upscaled 496x384 → 992x768 with NEAREST.
- **Tools**: ffmpeg/ffprobe not preinstalled; `sudo -n apt-get update && sudo -n
  apt-get install -y ffmpeg` (the first install failed before `apt-get update`).
  Pillow 13.0.0 + numpy installed via pip3.

## Scanline streaking fix (2026-11-05)
- **Root cause was NOT the edge-walking maths** — SAGE's `fill_triangle`/
  `fill_polygon` already matched MAME poly.h and `references/sm2_emu`'s
  `render_triangle`/`render_convex` almost line-for-line. The bug was a **row-pitch
  mismatch**: `geometry.cpp` rasterises into `m_dest` at pitch `kStride = 512`
  (MAME's 512x512 destmap), but the *readers* walked the surface at the 496-pixel
  display width. Reading row `y` at `y*496` while it was written at `y*512` skips
  16 px per row → diagonal shear / horizontal smearing across the screen.
- Fix: added `Geometry::kDestStride = 512` as the single source of truth;
  `kStride` aliases it, and `video.cpp` (compositor) + `main.cpp` (PPM3D dump)
  now index with `kDestStride`. Committed `7b4608d`.
- **Diagnostic**: dump the raw 3D surface with `SAGE_PPM3D` and count rows whose
  nonzero span is >80% of width — 55 streak rows before, 0 after at frame 1500.
  The *composited* frame is a poor metric (full-width tile background dominates);
  always measure the isolated 3D layer.
- **Environment resets**: a mid-session restart wiped `/tmp`, pip packages, apt
  ffmpeg, and the pip cmake. Reinstall cmake via `pip3 install cmake` (binary lands
  at `~/.local/bin/cmake`), ffmpeg via `sudo -n apt-get update && apt-get install
  -y ffmpeg`, numpy/pillow via pip. Also reset git identity (`git config user.name
  openhands` / `user.email openhands@all-hands.dev`) and the `origin` remote URL
  token (use `${GITHUB_TOKEN}`).
- Fixed video is **smaller** (6.16 MB vs 14.85 MB) at the same CRF 30 — the
  streaking was high-frequency noise that h264 was spending bits on.

## Milestone 7: SCSP sound board (2026-11-05)
- New subsystem: 68EC000 (`third_party/musashi` v4.60, 68000-only) + Yamaha
  YMF292 SCSP + i8251 link. Files: `include/sage/cpu/m68000.h`,
  `include/sage/hw/{compat,scsp,scsp_dsp,i8251,sound_board}.h` and matching
  `src/`. Wired into `Bus` via `tick_sound()` from the main loop.
- **Musashi opcodes are generated at build time** by the `m68kmake` host tool
  (CMake `add_custom_command` → `${CMAKE_BINARY_DIR}/musashi`); do NOT commit
  `m68kops.c/h`. The `musashi` lib must also compile `softfloat/softfloat.c`
  even for 68000-only, because `m68kcpu.c` references `m68kfpu.c` symbols.
- Host UART at `0x009c0000` is `umask32(0x000000ff)`: data +0, status +4 (one
  dword apart — different from the 2A/2C boards' half stride). Sound IRQ = bit
  10 of intreq/intena; the i8251 ready handler is level-based, not edge.
- Sound clocks kept exact: 68000 = host×7056/15625, SCSP = 22.5792 MHz/512 =
  44100 Hz, 256 68000 cycles per sample; remainder carried between calls.
- `Bus::attach(roms, game)` takes the set name now (drives SCSP flat gain).
- Optional `SAGE_WAV` dumps SCSP output as 16-bit stereo for audio checks.
- Verified: sound program runs, SCSP keys slots and produces structured audio;
  rendered video is byte-identical to before (cmp of SAGE_RAWVIDEO).
- Details in `.openhands/memory/2026-11-05.md`. Branch `feat/scsp-sound-board`.

## 3D texture fix + SCSP merge (2026-11-08)
- Merged `feat/scsp-sound-board` into `main` (`--no-ff`, merge commit `78fd437`);
  the branch existed only in a sibling clone, not on the remote. Clean Release
  build, 0 errors (only miniz + sharc warnings). Pushed main + the feature branch.
- **Root cause of flat-shaded 3D (10 colours): luma RAM addressing.**
  `0x11400000-0x1140ffff` is mapped `umask16(0x00ff)` on Model 2B — one byte per
  **16-bit half-word**, array index `(addr - base) >> 1`, packed array 0x8000 bytes
  (MAME: `make_unique_clear<u8[]>(0x8000)`). SAGE had mapped it as a plain byte
  window, so the i960's word writes landed on even offsets with zeros interleaved
  and the renderer's contiguous read `lumaram[lumabase + (t >> 1)]` always
  returned 0 → one flat colour per polygon. Fix: decode luma explicitly in
  `read8/16/32` + `write8/16/32`, keep it BURST-flagged (MAME does), and remove it
  from `Bus::map()`. See `references/sm2_emu/src/hw/model2b.cpp` (validated port).
- **Diagnostic that found it**: instrument `poly.luma`/`poly.lumabase` (both were
  sane: luma 176-255, lumabase 12544-12800) vs the computed `luma` (always 0) →
  the luma *curve* lookup was the zero, not the polygon fields. Counting nonzero
  bytes showed values at even offsets only (12204 even / 0 odd).
- **Metric**: 3D layer (`SAGE_PPM3D`) unique colours 10 → 703; composited title
  frame 25 → 718. Non-black 3D px 33.7k → 117k. Frame 1500 still matches the
  title screen (lumcorr 0.72 vs the old screenshot).
- Texel pipeline itself (`get_texel`, `fetch_bilinear_texel`, texsheet ROM/RAM
  select, colorxlat, gamma) already matched MAME — only luma was broken.
- Debug hooks used: temporary static counters in `render_one` and tracing in
  `fetch_bilinear_texel`; reverted before committing. PPM analysis with Pillow +
  numpy (pip3), ASCII-art dump for frame selection.

## Model 2A Virtua Fighter 2 (vf2) ROM loader (2026-11-11, commit 9d50d34)
- `src/rom_loader.cpp` is now **table-driven**: a game selects a
  `RegionDef` table + a program-entry list + a shared-entry list. Region enum
  `R_MAINCPU..R_VIDEO_UNK` is shared; doa's `kRegions`->`kDoaRegions`,
  `kSharedEntries`->`kDoaSharedEntries` (tables preserved verbatim).
- `vf2` = Model 2A, Virtua Fighter 2 Version 2.1, **36 ICs**:
  4 maincpu (epr-18385..18388), 8 main_data (mpr-17560/61,17558/59,17566/67,17564/65),
  6 polygons (mpr-17554/48,17555/49,17556/50), 4 textures (mpr-17553/52,17547/46, ERASEFF),
  1 audiocpu (epr-17574.30, WORD_SWAP), 4 samples (mpr-17573/72/71/70, WORD_SWAP+BE),
  2 copro_tgp_tables (opr-14742a/43a), 4 other_data (opr-14744..14747),
  3 video_unk (mpr-16310/11/12, ROM_LOAD32_BYTE = group 1/skip 3).
- vf2 region sizes: main_data 0x2400000, textures 0x1000000 (not doa's 0x800000),
  polygons 0x2000000. **polygons ERASEFF is doa-only** — vf2 polygons erase 0x00.
- `opr-14742a`/`opr-14743a` are the CPU-board TGP sin/cos tables; `opr-14744..14747`
  the 1/x + 1/sqrt tables (`other_data`); both board-level, shared by every TGP Model 2.
- Verified 36/36 ICs, 10 regions, program 0x00000000-0x001FFFFF. Region CRCs
  (independent Python replication must include `reverse` for WORD_SWAP ROMs, else
  audiocpu/samples differ): maincpu 6DA7DAD2, main_data F3E7804E, polygons D129AB66,
  textures 2E3F2B56, copro_data 1AD2BC45, audiocpu 85FD940E, samples 1D7F3304,
  copro_tgp_tables BDEA5049, other_data B79EA65B, video_unk 9745FC42.
- Source of truth: MAME `references/model2.cpp` `ROM_START( vf2 )` + board macros
  `MODEL2_CPU_BOARD` / `MODEL2A_VID_BOARD`; cross-checked against
  `references/sm2_emu/data/games.xml`.
- `main.cpp` banner/display-title/final line now derive from the game arg
  (`vf2` -> "Virtua Fighter 2 - Sega Model 2A"), not hardcoded DOA.
- **Writing a big C++ file via a bash heredoc silently truncated to the old
  content** (the tool reported success but the file kept its 325 lines). Use the
  file_editor tool (`create` after `rm`) for large source writes, then verify
  `grep -c <marker>` before building.
- vf2.zip (IA item `m2emu1.1a`, merged Model 2 set) is a **single-game archive**
  of 48 entries; it contains all four vf2 program revisions (2.1/a/b/original) so
  CRC-based identification picks the right 4. Only the 36 v2.1 ICs are loaded.

## Model 2A TGP coprocessor (2026-11-11)
- `include/sage/cpu/tgp.h` + `src/cpu/tgp.cpp`: `mb86233::Tgp` (Fujitsu MB86233/
  86234 core: 4096-word microcode RAM, two 0x200-word data banks, ALU, EA/reg
  file) + `TgpCoprocessor` (TgpBus impl, host FIFOs, bank reg, sincos/atan/inv/
  isqrt lookup units). Ported from MAME `cpu/mb86233/mb86233.{cpp,h}` and
  model2.cpp; `SM2_DEBUG` macro added to `hw/compat.h`.
- `Bus` selects the coprocessor via `Board` (`is_model2a()`), decided by whether
  the set has a `copro_tgp_tables` region. 2A routes ctl1/FIFO-ctl/FIFO-data/
  function-port windows to the TGP; `tick_copro` runs it at 2/3 the i960 rate
  (50 MHz/3 vs 25 MHz); `tgp_sync()` runs it forward on an empty-FIFO read.
- **Model 2A video/IO map differs from 2B**: luma RAM 0x12800000 (0x20000, umask32
  one byte/word; 2B = 0x11400000 umask16 one byte/half-word) via `Bus::luma_index`;
  texture RAM0/1 at 0x12000000/0x12400000 (0x200000 region `.mirror(0x200000)`,
  window spans 0x400000); UART at 0x01c80000 (umask16); 0x01c00040-43 is `.nopw()`
  (handle in `write_register` so all widths are caught).
- vf2 600-frame run: TGP uploads 3802 words, boots, 30662 insns, PC=0x0035;
  activity 6227 cmds in / 1572 results out / 532 table reads. Unmapped accesses
  758663 -> 6 writes + 25 reads (remainder genuinely unmapped in MAME too).
- Verify with: `/tmp/tgp_selftest.cpp` (links `src/cpu/tgp.cpp` alone) +
  `./build/sage roms/vf2.zip vf2` (36/36 ICs). Repo has no C++ test target.

## Model 2A GEO read wiring + vf2 attract mode (2026-11-11, commit e4acd68)
- **The GEO function-port read path was the vf2 init-stall gate.** MAME `geo_r`
  returns the display-list write pointer at 0x00802008 and the read pointer at
  0x00803008; SAGE only decoded the GEO *writes*, so the i960 read open bus there
  (PC 0x00002EFC loop) and never saw the TGP fill the list. Fix in `src/bus.cpp`:
  decode 0x00800000-0x00803fff in `read_register` (byte lanes), `read16`, `read32`
  → `Geometry::geo_read(offset)`. Effect (1500 frames): unmapped reads
  285699 → 9235, polygons rendered 9052 → 111113, i960 reaches its idle loop and
  runs attract mode. 36/36 ICs still OK.
- `0x01c00040` (CRX cabinet link) is **written but never read** by vf2 — a
  fabricated "link ready" read there is dead code; don't add it.
- Remaining vf2 unmapped reads are 16-bit accesses to 0x00884000 (copro FIFO);
  only `read32` decodes that window. Low volume, not a stall.
- `SAGE_PPM3D` still rasterises to alpha-only black (0xFF000000) even though
  polygons reach the rasteriser (95k dest pixels): solid shading path
  `draw_scanline_solid` uses `colorxlat`/`palram` (colorbase) that vf2 hasn't
  populated yet. Open follow-up.
- **`cmake`/`ninja` vanish from PATH after a session reset** (they live in
  `~/.local/bin`). `pip install --user cmake ninja` restores them; a build run
  without them is a *silent no-op* that leaves a stale `build/sage` — always
  `export PATH="$HOME/.local/bin:$PATH"` and check the binary timestamp/strings.

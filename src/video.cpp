// Sega Model 2 video: palette, colour translation, System 24 tiles, the direct
// framebuffer and the 3D polygon layer.
//
// Ported from MAME's src/mame/sega/model2_v.cpp and src/mame/sega/segaic24.cpp
// (BSD-3-Clause, copyright-holders: Olivier Galibert, R. Belmont, ElSemi,
// Angelo Salese, David Haywood).
//
// screen_update() order (model2_v.cpp): tilemaps {3,2} opaque, tilemaps {1,0}
// transparent, the 3D polygons, then the sprite tilemaps {(layer<<1)|1}. The 3D
// layer is produced by the Geometry module.

#include "sage/video.h"

#include <algorithm>
#include <cstring>

#include "sage/geometry.h"

namespace sage {
namespace {

// System 24 tile generator geometry (segas24_tile_device::char_layout): 8x8
// tiles, 4 planes, two pixels per byte, 32 bytes per tile. Tile RAM is 64x64
// cells; MAME indexes it as 16-bit words at 0x0000/0x1000/0x2000/0x3000.
constexpr int kTilemapDim = 64;  // 64x64 tiles

// Palette / colour-translation layout (all byte offsets from 0x01800000).
constexpr std::size_t kPaletteOff = 0x0000;
constexpr std::size_t kColorXlatOff = 0x10000;

std::uint8_t gamma_lut(std::uint8_t v) {
    // model2_v.cpp: max((i - 64) * 255 / 191, 0)
    const int raw = (static_cast<int>(v) - 64) * 255 / 191;
    return static_cast<std::uint8_t>(std::max(raw, 0));
}

std::uint16_t rd16(const std::vector<std::uint8_t>& b, std::size_t off) {
    if (off + 1 >= b.size())
        return 0;
    return static_cast<std::uint16_t>(b[off] | (b[off + 1] << 8));
}

}  // namespace

Video::Video() { reset(); }

void Video::reset() {
    m_palram.assign(0x4000, 0);
    m_colorxlat.assign(0xc000, 0);
    m_char.assign(0x80000, 0);
    m_tile.assign(0x10000, 0);
    m_fb[0].assign(0x80000, 0);
    m_fb[1].assign(0x80000, 0);
    m_palette.assign(0x2000, 0xff000000u);
    m_rgba.assign(kWidth * kHeight, 0xff000000u);
    m_fb_bank = 0;
    m_palette_dirty = true;
    m_render_test_mode = false;
    m_videoctl = 0;
    m_xhout = 0;
    m_xvout = 0;
}

std::uint8_t* Video::map(std::uint32_t addr, std::size_t len) const {
    // System 24 tile generator, with MAME's mirroring: tile RAM is 64 KiB at
    // 0x01000000 mirrored by 0x110000; char RAM is 512 KiB at 0x01080000
    // mirrored by 0x100000.
    if (addr >= 0x01000000 && addr < 0x01020000) {
        const std::uint32_t off = addr - 0x01000000;
        if (off + len <= m_tile.size())
            return const_cast<std::uint8_t*>(m_tile.data()) + off;
    }
    if (addr >= 0x01100000 && addr < 0x01120000) {
        const std::uint32_t off = addr - 0x01100000;
        if (off + len <= m_tile.size())
            return const_cast<std::uint8_t*>(m_tile.data()) + off;
    }
    if (addr >= 0x01080000 && addr < 0x01100000) {
        const std::uint32_t off = addr - 0x01080000;
        if (off + len <= m_char.size())
            return const_cast<std::uint8_t*>(m_char.data()) + off;
    }
    if (addr >= 0x01180000 && addr < 0x01200000) {
        const std::uint32_t off = addr - 0x01180000;
        if (off + len <= m_char.size())
            return const_cast<std::uint8_t*>(m_char.data()) + off;
    }

    const auto region = [&](const std::vector<std::uint8_t>& buf, std::uint32_t base) -> std::uint8_t* {
        if (addr < base || addr - base + len > buf.size())
            return nullptr;
        return const_cast<std::uint8_t*>(buf.data()) + (addr - base);
    };
    if (auto* p = region(m_palram, 0x01800000)) return p;
    if (auto* p = region(m_colorxlat, 0x01810000)) return p;
    if (auto* p = region(m_fb[0], 0x11600000)) return p;
    if (auto* p = region(m_fb[1], 0x11680000)) return p;
    return nullptr;
}

bool Video::read(std::uint32_t addr, std::uint8_t& out) const {
    if (const std::uint8_t* p = map(addr, 1))
        out = *p;
    else
        out = 0;
    return true;
}

bool Video::write(std::uint32_t addr, std::uint8_t val) {
    std::uint8_t* p = map(addr, 1);
    if (!p)
        return false;
    *p = val;
    if (addr >= 0x01800000 && addr < 0x01804000)
        m_palette_dirty = true;
    return true;
}

void Video::write_hsync(unsigned lane, std::uint8_t val) {
    const std::uint32_t mask = 0xffu << (lane * 8);
    m_xhout = static_cast<std::uint16_t>((m_xhout & ~mask) |
                                         (static_cast<std::uint32_t>(val) << (lane * 8)));
    if (m_geometry)
        m_geometry->set_hsync(m_xhout);
}

void Video::write_vsync(unsigned lane, std::uint8_t val) {
    const std::uint32_t mask = 0xffu << (lane * 8);
    m_xvout = static_cast<std::uint16_t>((m_xvout & ~mask) |
                                         (static_cast<std::uint32_t>(val) << (lane * 8)));
    if (m_geometry)
        m_geometry->set_vsync(m_xvout);
}

std::uint32_t Video::palette_entry(int index) const {
    const std::uint16_t palcolor = rd16(m_palram, static_cast<std::size_t>(index) * 2);
    // MAME indexes m_colorxlat as a u16 array, so one intensity step is two
    // bytes; the byte offsets 0x80/0x4080/0x8080 are the >>1 bases doubled.
    const auto lookup = [&](std::size_t base, int shift) {
        const std::size_t off = base + static_cast<std::size_t>((palcolor >> shift) & 0x1f) * 0x200;
        return off < m_colorxlat.size() ? m_colorxlat[off] : 0;
    };
    const std::uint8_t r = gamma_lut(lookup(0x0080, 0));
    const std::uint8_t g = gamma_lut(lookup(0x4080, 5));
    const std::uint8_t b = gamma_lut(lookup(0x8080, 10));
    return 0xff000000u | (static_cast<std::uint32_t>(r) << 16) |
           (static_cast<std::uint32_t>(g) << 8) | b;
}

void Video::rebuild_palette() {
    for (int i = 0; i < 0x1000; ++i)
        m_palette[i] = palette_entry(i);
    m_palette_dirty = false;
}

// Decodes one tile pixel: 8x8, 4 planes, two pixels per byte, high nibble
// first (segas24_tile_device::char_layout, planes {0,1,2,3}, xoffset STEP8(0,4),
// yoffset STEP8(0,32), charincrement 32 bytes). The byte index is XORed with
// one because MAME builds the graphics element over a 16-bit array with a
// little-endian byte swap (NATIVE_ENDIAN_VALUE_LE_BE(8,0)); without it each
// pair of bytes is read swapped and the image combs. Mirrors sm2_emu
// decode_tile_pixel().
int tile_pixel(const std::vector<std::uint8_t>& chram, int code, int x, int y) {
    const int index = (code & 0x3fff) * 32 + ((y * 4 + (x >> 1)) ^ 1);
    if (index < 0 || static_cast<std::size_t>(index) >= chram.size())
        return 0;
    const std::uint8_t byte = chram[static_cast<std::size_t>(index)];
    return (x & 1) == 0 ? (byte >> 4) : (byte & 0x0f);
}

void Video::draw_tile_layers(int group) {
    // MAME screen_update() draws the tilemaps twice: before the 3D layer with
    // priority 0 (tilemaps 3,2 opaque, 1,0 transparent) and after it with
    // priority 1. draw_rect hides an 8-pixel cell when its screen-space
    // priority bit (tile RAM 0x6000) is set; Dead or Alive never writes that
    // mask, so the whole background sits behind the 3D polygons.
    static constexpr int kOrder[4] = {3, 2, 1, 0};
    const bool sprite_pass = (group != 0);
    for (int idx = 0; idx < 4; ++idx) {
        const int layer = kOrder[idx];
        const bool opaque = (layer >= 2) && !sprite_pass;
        const int mask_base = 0x6000;  // MAME: tile_ram + (layer & 4 ? 0x6800 : 0x6000)

        const std::uint16_t vscr_reg = rd16(m_tile, (0x5004 + layer) * 2);
        if (vscr_reg & 0x8000)  // layer disabled
            continue;
        const std::uint16_t hscr_reg = rd16(m_tile, (0x5000 + layer) * 2);
        // segaic24 blits each tilemap at (-hscr, +vscr), i.e. source = screen +
        // scroll. This loop runs the other way (screen = source + scroll), so
        // both registers are negated relative to MAME here. Leaving vscr
        // un-negated sampled Virtua Fighter 2's title backdrop (layer 2,
        // vscr=0x56) 0x56 rows too low, showing a grey bar of bare ground tiles
        // across the top 86 rows where the sky should be.
        const int hscr = static_cast<int>(hscr_reg) & 0x1ff;
        const int vscr = (-static_cast<int>(vscr_reg)) & 0x1ff;

        for (int ty = 0; ty < kTilemapDim; ++ty) {
            for (int tx = 0; tx < kTilemapDim; ++tx) {
                const std::uint16_t val =
                    rd16(m_tile, (static_cast<std::size_t>(layer) * 0x1000 +
                                  ty * kTilemapDim + tx) * 2);
                const int code = val & 0x3fff;
                const int color = (val >> 7) & 0xff;

                for (int py = 0; py < 8; ++py) {
                    const int my = (ty * 8 + py + vscr) & 0x1ff;
                    if (my >= kHeight)
                        continue;
                    for (int px = 0; px < 8; ++px) {
                        const int mx = (tx * 8 + px + hscr) & 0x1ff;
                        if (mx >= kWidth)
                            continue;
                        const std::uint16_t mw =
                            rd16(m_tile, (mask_base + my * 4 + (mx >> 7)) * 2);
                        const bool pri_bit = (mw & (0x8000 >> ((mx & 127) >> 3))) != 0;
                        if (pri_bit != sprite_pass)
                            continue;
                        const int pen = tile_pixel(m_char, code, px, py);
                        if (pen == 0 && !opaque)
                            continue;
                        const int index = ((color << 4) | pen) & 0xfff;
                        m_rgba[static_cast<std::size_t>(my) * kWidth + mx] = m_palette[index];
                    }
                }
            }
        }
    }
}

void Video::draw_framebuffer() {
    // model2_v.cpp draw_framebuffer(): 512x400, colour 0 is transparent.
    const std::vector<std::uint8_t>& fb = m_fb[m_fb_bank];
    const int yoffs = (512 - 384);
    for (int y = 0; y < kHeight; ++y) {
        for (int x = 0; x < kWidth; ++x) {
            const std::size_t off =
                static_cast<std::size_t>(x + (y + yoffs) * 512) * 2;
            const std::uint16_t v = static_cast<std::uint16_t>(fb[off] | (fb[off + 1] << 8));
            const std::uint32_t r = gamma_lut(m_colorxlat[0x0080 + ((v >> 5) & 0x1f) * 0x200]);
            const std::uint32_t g = gamma_lut(m_colorxlat[0x4080 + ((v >> 10) & 0x1f) * 0x200]);
            const std::uint32_t b = gamma_lut(m_colorxlat[0x8080 + ((v >> 0) & 0x1f) * 0x200]);
            m_rgba[static_cast<std::size_t>(y) * kWidth + x] =
                0xff000000u | (r << 16) | (g << 8) | b;
        }
    }
}

void Video::render() {
    if (m_palette_dirty)
        rebuild_palette();

    std::fill(m_rgba.begin(), m_rgba.end(), m_palette[0]);
    draw_tile_layers(0);
    // 3D polygons sit between the background tiles and the sprite tiles.
    if (m_geometry && !m_render_test_mode) {
        const std::uint32_t* poly = m_geometry->render();
        if (poly) {
            for (int y = 0; y < kHeight; ++y) {
                const std::uint32_t* src =
                    poly + static_cast<std::size_t>(y) * Geometry::kDestStride;
                std::uint32_t* dst = m_rgba.data() + static_cast<std::size_t>(y) * kWidth;
                for (int x = 0; x < kWidth; ++x) {
                    if (src[x] != 0)  // 0 = transparent in the geometry surface
                        dst[x] = src[x];
                }
            }
        }
    }

    draw_tile_layers(1);
    if (m_render_test_mode)
        draw_framebuffer();
}

}  // namespace sage

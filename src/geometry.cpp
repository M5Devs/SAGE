// Sega Model 2B 3D geometry pipeline and polygon rasteriser.
//
// Ported from MAME's src/mame/sega/model2_v.cpp, src/mame/sega/model2rd.ipp and
// src/devices/video/poly.h (BSD-3-Clause, copyright-holders: R. Belmont,
// Olivier Galibert, ElSemi, Angelo Salese, Matthew Daniels, David Haywood).
//
// The i960 builds a display list in the GEO program FIFO / buffer RAM; parse()
// walks it (the geometry engine) and pushes transformed vertices into push()
// (the hardware rasteriser). push() assembles polygons, which are clipped,
// z-sorted and filled into a 496x384 ARGB surface by render().

#include "sage/geometry.h"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace sage {
namespace {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using s16 = std::int16_t;
using s32 = std::int32_t;

constexpr int kStride = Geometry::kDestStride;  // destmap / fillmap row pitch
constexpr int kClipLeft = 0;
constexpr int kClipTop = 0;
constexpr int kClipRight = Geometry::kWidth - 1;    // 495
constexpr int kClipBottom = Geometry::kHeight - 1;  // 383

constexpr float u2f(u32 v) { return std::bit_cast<float>(v); }
constexpr u32 f2u(float f) { return std::bit_cast<u32>(f); }

// sign-extend the low `bits` bits of `value`.
constexpr s32 sext(u32 value, int bits) {
    const u32 m = 1u << (bits - 1);
    return static_cast<s32>((value ^ m) - m);
}

// model2_v.cpp gamma table: max((i - 64) * 255 / 191, 0).
u8 gamma_lut(u8 v) {
    const int raw = (static_cast<int>(v) - 64) * 255 / 191;
    return static_cast<u8>(std::max(raw, 0));
}

// model2rd.ipp LERP: interpolate two packed 8-bit channels at once.
constexpr u32 LERP(u32 x, u32 y, unsigned a) {
    return (x + (((y - x) * a) >> 8)) & 0x00ff00ff;
}

// model2rd.ipp fast_log2 (from video/voodoo_render.cpp).
s32 fast_log2(float value) {
    if (value < 0.0f)
        return 0;
    const u32 ival = f2u(value) >> 16;
    const s32 exp = static_cast<s32>(ival >> 7) - 127;
    static const u8 s_log2_table[128] = {
        0,   2,   5,   8,   11,  14,  16,  19,  22,  25,  27,  30,  33,  35,  38,  40,
        43,  46,  48,  51,  53,  56,  58,  61,  63,  65,  68,  70,  73,  75,  77,  80,
        82,  84,  87,  89,  91,  93,  96,  98,  100, 102, 104, 106, 109, 111, 113, 115,
        117, 119, 121, 123, 125, 127, 129, 132, 134, 136, 138, 140, 141, 143, 145, 147,
        149, 151, 153, 155, 157, 159, 161, 162, 164, 166, 168, 170, 172, 173, 175, 177,
        179, 181, 182, 184, 186, 188, 189, 191, 193, 194, 196, 198, 200, 201, 203, 205,
        206, 208, 209, 211, 213, 214, 216, 218, 219, 221, 222, 224, 225, 227, 229, 230,
        232, 233, 235, 236, 238, 239, 241, 242, 244, 245, 247, 248, 250, 251, 253, 254};
    return (exp << 8) | s_log2_table[ival & 127];
}

// ---- generic 3D math (model2_v.cpp) --------------------------------------

void transform_point(Geometry::Vertex* point, const float* m) {
    const float tx = (point->x * m[0]) + (point->y * m[3]) + (point->pz() * m[6]) + m[9];
    const float ty = (point->x * m[1]) + (point->y * m[4]) + (point->pz() * m[7]) + m[10];
    const float tz = (point->x * m[2]) + (point->y * m[5]) + (point->pz() * m[8]) + m[11];
    point->x = tx;
    point->y = ty;
    point->pz() = tz;
}

void transform_vector(Geometry::Vertex* v, const float* m) {
    const float tx = (v->x * m[0]) + (v->y * m[3]) + (v->pz() * m[6]);
    const float ty = (v->x * m[1]) + (v->y * m[4]) + (v->pz() * m[7]);
    const float tz = (v->x * m[2]) + (v->y * m[5]) + (v->pz() * m[8]);
    v->x = tx;
    v->y = ty;
    v->pz() = tz;
}

float dot_product(const Geometry::Vertex& a, const Geometry::Vertex& b) {
    return (a.x * b.x) + (a.y * b.y) + (a.pz() * b.pz());
}

void normalize_vector(Geometry::Vertex* v) {
    const float n = std::sqrt((v->x * v->x) + (v->y * v->y) + (v->pz() * v->pz()));
    if (n) {
        const float oon = 1.0f / n;
        v->x *= oon;
        v->y *= oon;
        v->pz() *= oon;
    }
}

void vector_cross3(Geometry::Vertex* dst, const Geometry::Vertex* v0,
                   const Geometry::Vertex* v1, const Geometry::Vertex* v2) {
    Geometry::Vertex p1, p2;
    p1.x = v1->x - v0->x; p1.y = v1->y - v0->y; p1.pz() = v1->pz() - v0->pz();
    p2.x = v2->x - v0->x; p2.y = v2->y - v0->y; p2.pz() = v2->pz() - v0->pz();
    dst->x = (p1.y * p2.pz()) - (p1.pz() * p2.y);
    dst->y = (p1.pz() * p2.x) - (p1.x * p2.pz());
    dst->pz() = (p1.x * p2.y) - (p1.y * p2.x);
}

}  // namespace

Geometry::Geometry() { reset(); }

void Geometry::reset() {
    m_polygon_ram0.assign(0x8000, 0);
    m_polygon_ram1.assign(0x8000, 0);
    m_texture_ram.assign(0x10000, 0);
    m_log_ram.assign(0x8000, 0);
    m_poly_list.assign(32768, Polygon{});
    m_poly_sorted_list.assign(0x10000, nullptr);
    m_dest.assign(static_cast<std::size_t>(kStride) * kStride, 0);
    m_fill.assign(static_cast<std::size_t>(kStride) * kStride, 0);
    m_mode = 0;
    m_mesh_mode = 0;
    std::memset(m_matrix, 0, sizeof(m_matrix));
    m_focus = Vertex{};
    m_light = Vertex{};
    m_lod = 0;
    std::memset(m_coef_table, 0, sizeof(m_coef_table));
    for (auto& tp : m_texture_parameters) tp = TextureParam{};
    std::memset(m_viewport, 0, sizeof(m_viewport));
    for (auto& c : m_center) { c[0] = 0; c[1] = 0; }
    m_center_sel = 0;
    m_reverse = 0;
    m_z_adjust = 0;
    m_polygon_z = 1e10f;
    m_master_z_clip = 0xff;
    m_cur_command = 0;
    std::memset(m_command_buffer, 0, sizeof(m_command_buffer));
    m_command_index = 0;
    m_poly_list_index = 0;
    std::fill(m_poly_sorted_list.begin(), m_poly_sorted_list.end(), nullptr);
    m_min_z = 0xffff;
    m_max_z = 0;
    m_cur_window = 0;
    m_render_done = false;
    m_geo_write_addr = 0;
    m_geo_read_addr = 0;
    m_geoctl = 0;
    m_geocnt = 0;
    m_xoffs = 90;
    m_yoffs = -8;
    m_crtc_xoffset = 0;
    m_crtc_yoffset = 0;
}

// ---------------------------------------------------------------------------
// Shared resources
// ---------------------------------------------------------------------------

void Geometry::set_texture_rom(const std::uint8_t* data, std::size_t bytes) {
    m_texture_rom = reinterpret_cast<const u16*>(data);
    m_texture_rom_mask = static_cast<u32>(bytes / 2) - 1;
}

void Geometry::set_polygon_rom(const std::uint8_t* data, std::size_t bytes) {
    m_polygon_rom = reinterpret_cast<const u32*>(data);
    m_polygon_rom_mask = static_cast<u32>(bytes / 4) - 1;
}

void Geometry::set_texture_ram(std::uint32_t* ram0, std::uint32_t* ram1, std::size_t words) {
    m_textureram0 = ram0;
    m_textureram1 = ram1;
    m_texture_words = words;
}

void Geometry::set_luma_ram(const std::uint8_t* data) { m_lumaram = data; }

void Geometry::set_palette_ram(const std::uint8_t* palram, const std::uint8_t* colorxlat) {
    m_palram = palram;
    m_colorxlat = colorxlat;
}

void Geometry::set_buffer_ram(std::uint32_t* words, std::size_t word_count) {
    m_bufferram = words;
    m_bufferram_words = word_count;
}

// ---------------------------------------------------------------------------
// Register interface (MAME model2_state::geo_r/geo_w/geo_prg_r/geo_prg_w)
// ---------------------------------------------------------------------------

u32 Geometry::geo_read(u32 offset) const {
    if (offset == 0x2008)
        return m_geo_write_addr;
    if (offset == 0x3008)
        return m_geo_read_addr;
    return 0;
}

void Geometry::geo_write(u32 offset, u32 data) {
    if (offset < 0x1000) {
        u32 r;
        if (data & 0x80000000) {
            r = (data & 0x800fffff) | (((offset >> 4) & 0x3f) << 23);
        } else if ((offset & 0xf) == 0) {
            r = (data & 0x000fffff) | (((offset >> 4) & 0x3f) << 23);
            if ((offset >> 4) & 0xc0) {
                if (((offset >> 4) & 0x3f) == 1)
                    r |= ((offset >> 10) & 3) << 29;  // eye mode (Sega Rally)
            }
        } else {
            return;
        }
        push_geo_data(r);
    } else if (offset == 0x1008) {
        m_geo_write_addr = data & 0xfffff;
    } else if (offset == 0x3008) {
        m_geo_read_addr = data & 0xfffff;
    }
}

u32 Geometry::geo_prg_read(u32) const { return 0xffffffffu; }

void Geometry::geo_prg_write(u32 data) {
    if (m_geoctl & 0x80000000) {
        m_geocnt++;
    } else {
        push_geo_data(data);
    }
}

void Geometry::set_geoctl(u32 value) {
    if ((value ^ m_geoctl) == 0x80000000) {
        if (value & 0x80000000)
            m_geocnt = 0;
    }
    m_geoctl = value;
}

void Geometry::set_hsync(std::uint16_t data) {
    m_crtc_xoffset = static_cast<std::int16_t>(84 + static_cast<s16>(data));
    m_xoffs = m_crtc_xoffset;
}

void Geometry::set_vsync(std::uint16_t data) {
    m_crtc_yoffset = static_cast<std::int16_t>(130 + static_cast<s16>(data));
    m_yoffs = m_crtc_yoffset;
}

void Geometry::push_geo_data(u32 data) {
    if (!m_bufferram)
        return;
    const std::size_t idx = (m_geo_write_addr / 4) % m_bufferram_words;
    m_bufferram[idx] = data;
    m_geo_write_addr += 4;
}
std::size_t Geometry::geo_words_left(const u32* p) const {
    if (!m_bufferram || !p || p < m_bufferram || p > m_bufferram + m_bufferram_words)
        return 0;
    return static_cast<std::size_t>((m_bufferram + m_bufferram_words) - p);
}
const u16* Geometry::tex_source(u32 sel, std::size_t& words) const {
    if (sel & 0x800000) {
        words = m_texture_ram.size();
        return m_texture_ram.data();
    }
    words = m_texture_rom ? (m_texture_rom_mask + 1) : 0;
    return m_texture_rom;
}
u16 Geometry::tex_read(const u16* base, std::size_t words, std::size_t index) const {
    return (base && index < words) ? base[index] : 0u;
}

// ---------------------------------------------------------------------------
// Geometry engine: display-list commands (model2_v.cpp)
// ---------------------------------------------------------------------------

const u32* Geometry::cmd_nop(u32 opcode, const u32* input) {
    push(opcode >> 23);
    return input;
}

const u32* Geometry::cmd_object_data(u32 opcode, const u32* input) {
    const u32 tpa = *input++;
    const u32 tha = *input++;
    const u32 oba = *input++;
    u32 obc = *input++;

    const u32* obp;
    std::size_t obp_words;
    push(opcode >> 23);
    push(tpa);
    push(tha);

    if (oba & 0x01000000) {
        const std::size_t off = oba & 0x7fff;
        obp = m_polygon_ram1.data() + off;
        obp_words = m_polygon_ram1.size() - off;
    } else if (oba & 0x00800000) {
        const std::size_t off = oba & m_polygon_rom_mask;
        obp = m_polygon_rom + off;
        obp_words = m_polygon_rom ? (m_polygon_rom_mask + 1) - off : 0;
    } else {
        const std::size_t off = oba & 0x7fff;
        obp = m_polygon_ram0.data() + off;
        obp_words = m_polygon_ram0.size() - off;
    }

    if (obc == 0)
        obc = 0xfffff;

    // The parsers read a 6-word header then up to 10 words per vertex with no
    // bound of their own (matching the reference). A desynced display list can
    // otherwise drive them past the end of the polygon RAM/ROM, so cap the
    // vertex count to what the region can actually hold.
    constexpr std::size_t kHeaderWords = 6;
    constexpr std::size_t kMaxVertexWords = 10;
    if (obp_words >= kHeaderWords) {
        const std::size_t max_vertices = (obp_words - kHeaderWords) / kMaxVertexWords;
        if (obc > max_vertices)
            obc = static_cast<u32>(max_vertices);

        switch (m_mode & 3) {
            case 0: parse_np_ns(obp, obc); break;
            case 1: parse_np_s(obp, obc); break;
            case 2: parse_nn_ns(obp, obc); break;
            case 3: parse_nn_s(obp, obc); break;
        }
    }
    return input;
}

const u32* Geometry::cmd_direct_data(u32 opcode, const u32* input) {
    const u32 tpa = *input++;
    const u32 tha = *input++;
    push((opcode >> 23) - 1);
    push(tpa);
    push(tha);
    push((*input++) >> 8);
    push((*input++) >> 8);
    push((*input++) >> 8);
    push((*input++) >> 8);
    push((*input++) >> 8);
    push((*input++) >> 8);

    u32 attr;
    const u32* const end = m_bufferram + m_bufferram_words;
    while (input < end && ((attr = *input++) & 3) != 0) {
        push(attr & 0x00ffffff);
        push((*input++) >> 8);
        push((*input++) >> 8);
        push((*input++) >> 8);
        push((*input++) >> 8);
        push((*input++) >> 8);
        if (attr & 1) {
            push((*input++) >> 8);
            push((*input++) >> 8);
            push((*input++) >> 8);
        }
    }
    push(0);
    return input;
}

const u32* Geometry::cmd_window_data(u32 opcode, const u32* input) {
    push(opcode >> 23);
    m_cur_window++;
    for (u32 i = 0; i < 6; i++) {
        u32 y = *input++;
        const u32 x = (y & 0x0fff0000) >> 4;
        y &= 0xfff;
        push(x | y);
    }
    return input;
}

const u32* Geometry::cmd_texture_data(u32 opcode, const u32* input) {
    push(opcode >> 23);
    push(*input++);
    u32 count = *input++;
    const std::size_t left = geo_words_left(input);
    if (count > left)
        count = static_cast<u32>(left);
    push(count);
    for (u32 i = 0; i < count; i++)
        push(*input++);
    return input;
}

const u32* Geometry::cmd_polygon_data(u32 opcode, const u32* input) {
    (void)opcode;
    const u32 address = *input++;
    const bool fast = (address & 0x01000000) != 0;
    const std::size_t off = address & 0x7fff;
    u32* const ram = fast ? m_polygon_ram1.data() : m_polygon_ram0.data();
    const std::size_t ram_words = fast ? m_polygon_ram1.size() : m_polygon_ram0.size();
    u32* p = ram + (off < ram_words ? off : 0);
    u32 count = *input++;
    // Bound by both the source (buffer RAM) and the destination (polygon RAM);
    // a desynced list can otherwise write past the end of the RAM vectors.
    const std::size_t left = std::min(geo_words_left(input), ram_words - (off < ram_words ? off : 0));
    if (count > left)
        count = static_cast<u32>(left);
    for (u32 i = 0; i < count; i++)
        *p++ = *input++;
    return input;
}

const u32* Geometry::cmd_texture_parameters(u32 opcode, const u32* input) {
    (void)opcode;
    u32 index = ((*input++) >> 2) & 0x1f;  // 32-entry tables; a desynced list must not over-index
    u32 count = *input++;
    const std::size_t left = geo_words_left(input) / 2;  // two words per entry
    if (count > left)
        count = static_cast<u32>(left);
    for (u32 i = 0; i < count; i++) {
        const u32 param = *input++;
        m_texture_parameters[index].diffuse = static_cast<float>(param & 0xff);
        m_texture_parameters[index].ambient = static_cast<float>((param >> 8) & 0xff);
        m_texture_parameters[index].specular_control = (param >> 24) & 0xff;
        m_texture_parameters[index].specular_scale = static_cast<float>((param >> 16) & 0xff);
        m_coef_table[index] = u2f(*input++);
        index = (index + 1) & 0x1f;
    }
    return input;
}

const u32* Geometry::cmd_geo_mode(u32 opcode, const u32* input) {
    (void)opcode;
    m_mode = *input++;
    return input;
}

const u32* Geometry::cmd_zsort_mode(u32 opcode, const u32* input) {
    push(opcode >> 23);
    push((*input++) >> 8);
    return input;
}

const u32* Geometry::cmd_focal_distance(u32 opcode, const u32* input) {
    (void)opcode;
    m_focus.x = u2f(*input++);
    m_focus.y = u2f(*input++);
    return input;
}

const u32* Geometry::cmd_light_source(u32 opcode, const u32* input) {
    (void)opcode;
    m_light.x = u2f(*input++);
    m_light.y = u2f(*input++);
    m_light.pz() = u2f(*input++);
    return input;
}

const u32* Geometry::cmd_matrix_write(u32 opcode, const u32* input) {
    (void)opcode;
    for (u32 i = 0; i < 12; i++)
        m_matrix[i] = u2f(*input++);
    return input;
}

const u32* Geometry::cmd_translate_write(u32 opcode, const u32* input) {
    (void)opcode;
    for (u32 i = 0; i < 3; i++)
        m_matrix[i + 9] = u2f(*input++);
    return input;
}

const u32* Geometry::cmd_data_mem_push(u32 opcode, const u32* input) {
    (void)opcode;
    input++;  // address
    input++;  // count
    return input;
}

const u32* Geometry::cmd_geo_test(u32 opcode, const u32* input) {
    (void)opcode;
    for (u32 i = 0; i < 32; i++)
        input++;
    const u32 blocks = *input++;
    for (u32 i = 0; i < blocks; i++) {
        input++;  // address
        const u32 count = *input++;
        input++;  // checksum
        (void)count;
    }
    return input;
}

const u32* Geometry::cmd_geo_end(u32 opcode, const u32* input) {
    (void)opcode;
    push(0xff000000);
    return nullptr;
}

const u32* Geometry::cmd_dummy(u32 opcode, const u32* input) {
    (void)opcode;
    input++;
    return input;
}

const u32* Geometry::cmd_log_data(u32 opcode, const u32* input) {
    push(opcode >> 23);
    push(*input++);
    u32 count = *input++;
    const std::size_t left = geo_words_left(input);
    if (count > left)
        count = static_cast<u32>(left);
    push(count << 2);
    for (u32 i = 0; i < count; i++) {
        const u32 data = *input++;
        push(data & 0xff);
        push((data >> 8) & 0xff);
        push((data >> 16) & 0xff);
        push((data >> 24) & 0xff);
    }
    return input;
}

const u32* Geometry::cmd_lod(u32 opcode, const u32* input) {
    (void)opcode;
    m_lod = u2f(*input++);
    return input;
}

const u32* Geometry::cmd_code_upload(u32 opcode, const u32* input) {
    (void)opcode;
    input++;  // flags
    u32 count = *input++;
    const std::size_t left = geo_words_left(input) / 3;  // three words per entry
    if (count > left)
        count = static_cast<u32>(left);
    for (u32 i = 0; i < count; i++)
        input += 3;
    return input;
}

const u32* Geometry::cmd_code_jump(u32 opcode, const u32* input) {
    (void)opcode;
    input++;
    return input;
}

const u32* Geometry::process_command(u32 opcode, const u32* input, bool* end_code) {
    switch ((opcode >> 23) & 0x1f) {
        case 0x00: input = cmd_nop(opcode, input); break;
        case 0x01: input = cmd_object_data(opcode, input); break;
        case 0x02: input = cmd_direct_data(opcode, input); break;
        case 0x03: input = cmd_window_data(opcode, input); break;
        case 0x04: input = cmd_texture_data(opcode, input); break;
        case 0x05: input = cmd_polygon_data(opcode, input); break;
        case 0x06: input = cmd_texture_parameters(opcode, input); break;
        case 0x07: input = cmd_geo_mode(opcode, input); break;
        case 0x08: input = cmd_zsort_mode(opcode, input); break;
        case 0x09: input = cmd_focal_distance(opcode, input); break;
        case 0x0a: input = cmd_light_source(opcode, input); break;
        case 0x0b: input = cmd_matrix_write(opcode, input); break;
        case 0x0c: input = cmd_translate_write(opcode, input); break;
        case 0x0d: input = cmd_data_mem_push(opcode, input); break;
        case 0x0e: input = cmd_geo_test(opcode, input); break;
        case 0x0f: input = cmd_geo_end(opcode, input); *end_code = true; break;
        case 0x10: input = cmd_dummy(opcode, input); break;
        case 0x11: input = cmd_object_data(opcode, input); break;
        case 0x12: input = cmd_direct_data(opcode, input); break;
        case 0x13: input = cmd_window_data(opcode, input); break;
        case 0x14: input = cmd_log_data(opcode, input); break;
        case 0x15: input = cmd_polygon_data(opcode, input); break;
        case 0x16: input = cmd_lod(opcode, input); break;
        case 0x17: input = cmd_geo_mode(opcode, input); break;
        case 0x18: input = cmd_zsort_mode(opcode, input); break;
        case 0x19: input = cmd_focal_distance(opcode, input); break;
        case 0x1a: input = cmd_light_source(opcode, input); break;   // no mesh games
        case 0x1b: input = cmd_matrix_write(opcode, input); break;
        case 0x1c: input = cmd_translate_write(opcode, input); break;
        case 0x1d: input = cmd_code_upload(opcode, input); break;
        case 0x1e: input = cmd_code_jump(opcode, input); break;
        case 0x1f: input = cmd_geo_end(opcode, input); *end_code = true; break;
    }
    return input;
}

void Geometry::parse() {
    m_parse_count++;
    render_frame_start();
    if (!m_bufferram || m_bufferram_words == 0)
        return;

    u32 address = (m_geo_read_addr & 0x1ffff) / 4;
    const u32* base = m_bufferram;
    const u32* input = base + (address % m_bufferram_words);
    u32 op_count = 0;
    bool end_code = false;

    while (!end_code && static_cast<std::size_t>(input - base) < m_bufferram_words &&
           op_count++ < 0x8000) {
        const u32 opcode = *input++;
        if (opcode & 0x80000000) {
            address = (opcode & 0x1ffff) / 4;
            input = base + (address % m_bufferram_words);
            continue;
        }
        input = process_command(opcode, input, &end_code);
        if (!input)
            break;
    }
}

// ---------------------------------------------------------------------------
// Geometry engine: polygon parsers (model2_v.cpp geo_parse_*)
// ---------------------------------------------------------------------------

void Geometry::parse_np_ns(const u32* input, u32 count) {
    Vertex point, normal;
    u32 attr, i;

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    for (i = 0; i < count; i++) {
        attr = *input++;
        push(attr & 0x0003ffff);
        normal.x = u2f(*input++);
        normal.y = u2f(*input++);
        normal.pz() = u2f(*input++);
        transform_vector(&normal, m_matrix);

        if ((attr & 3) != 0) {
            float dotl, dotp, luminance, distance, coef, face;
            s32 luma;
            TextureParam* texparam;

            point.x = u2f(*input++);
            point.y = u2f(*input++);
            point.pz() = u2f(*input++);
            transform_point(&point, m_matrix);

            dotl = dot_product(normal, m_light);
            dotp = dot_product(normal, point);
            point.x *= m_focus.x;
            point.y *= m_focus.y;

            face = 0x100;
            if (dotp >= 0) face = 0;

            texparam = &m_texture_parameters[(attr >> 18) & 0x1f];

            if ((dotl * dotp) < 0) luminance = 0;
            else luminance = std::fabs(dotl);

            luminance = (luminance * texparam->diffuse) + texparam->ambient;
            luminance = std::clamp(luminance, 0.0f, 255.0f);
            luma = static_cast<s32>(luminance);
            luma += static_cast<s32>(face);

            coef = m_coef_table[attr >> 27];
            distance = coef * std::fabs(dotp) * m_lod;

            push(static_cast<u32>(luma) << 15);
            push(f2u(distance) >> 8);
            push(f2u(point.x) >> 8);
            push(f2u(point.y) >> 8);
            push(f2u(point.pz()) >> 8);

            if (attr & 1) {
                point.x = u2f(*input++);
                point.y = u2f(*input++);
                point.pz() = u2f(*input++);
                transform_point(&point, m_matrix);
                point.x *= m_focus.x;
                point.y *= m_focus.y;
                push(f2u(point.x) >> 8);
                push(f2u(point.y) >> 8);
                push(f2u(point.pz()) >> 8);
            } else {
                input += 3;
            }
        } else {
            break;
        }
    }
    push(0);
}

void Geometry::parse_np_s(const u32* input, u32 count) {
    Vertex point, normal;
    u32 attr, i;

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    for (i = 0; i < count; i++) {
        attr = *input++;
        push(attr & 0x0003ffff);
        normal.x = u2f(*input++);
        normal.y = u2f(*input++);
        normal.pz() = u2f(*input++);
        transform_vector(&normal, m_matrix);

        if ((attr & 3) != 0) {
            float dotl, dotp, luminance, distance, coef, face, specular;
            s32 luma;
            TextureParam* texparam;

            point.x = u2f(*input++);
            point.y = u2f(*input++);
            point.pz() = u2f(*input++);
            transform_point(&point, m_matrix);

            dotl = dot_product(normal, m_light);
            dotp = dot_product(normal, point);
            point.x *= m_focus.x;
            point.y *= m_focus.y;

            face = 0x100;
            if (dotp >= 0) face = 0;

            texparam = &m_texture_parameters[(attr >> 18) & 0x1f];

            if ((dotl * dotp) < 0) luminance = 0;
            else luminance = std::fabs(dotl);

            // Specular (MAME geo_parse_np_s).
            specular = ((2 * dotl) * normal.pz()) - m_light.pz();
            if (specular < 0) specular = 0;
            if (texparam->specular_control == 0) specular = 0;
            if ((texparam->specular_control >> 1) != 0) specular *= specular;
            if ((texparam->specular_control >> 2) != 0) specular *= specular;
            if (((texparam->specular_control + 1) >> 3) != 0) specular *= specular;
            specular *= texparam->specular_scale;

            luminance = (luminance * texparam->diffuse) + texparam->ambient + specular;
            luminance = std::clamp(luminance, 0.0f, 255.0f);
            luma = static_cast<s32>(luminance);
            luma += static_cast<s32>(face);

            coef = m_coef_table[attr >> 27];
            distance = coef * std::fabs(dotp) * m_lod;

            push(static_cast<u32>(luma) << 15);
            push(f2u(distance) >> 8);
            push(f2u(point.x) >> 8);
            push(f2u(point.y) >> 8);
            push(f2u(point.pz()) >> 8);

            if (attr & 1) {
                point.x = u2f(*input++);
                point.y = u2f(*input++);
                point.pz() = u2f(*input++);
                transform_point(&point, m_matrix);
                point.x *= m_focus.x;
                point.y *= m_focus.y;
                push(f2u(point.x) >> 8);
                push(f2u(point.y) >> 8);
                push(f2u(point.pz()) >> 8);
            } else {
                input += 3;
            }
        } else {
            break;
        }
    }
    push(0);
}

void Geometry::parse_nn_ns(const u32* input, u32 count) {
    Vertex point, normal, p0, p1, p2, p3;
    u32 attr, i;

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    p0 = point;
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    p1 = point;
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    for (i = 0; i < count; i++) {
        attr = *input++;
        push(attr & 0x0003ffff);

        if ((attr & 3) != 0) {
            float dotl, dotp, luminance, distance, coef, face;
            s32 luma;
            TextureParam* texparam;

            input += 3;  // skip normal

            point.x = u2f(*input++);
            point.y = u2f(*input++);
            point.pz() = u2f(*input++);
            transform_point(&point, m_matrix);
            p2 = point;

            vector_cross3(&normal, &p0, &p1, &p2);
            normalize_vector(&normal);

            dotl = dot_product(normal, m_light);
            dotp = dot_product(normal, point);
            point.x *= m_focus.x;
            point.y *= m_focus.y;

            face = 0x100;
            if (dotp >= 0) face = 0;

            texparam = &m_texture_parameters[(attr >> 18) & 0x1f];

            if ((dotl * dotp) < 0) luminance = 0;
            else luminance = std::fabs(dotl);

            luminance = (luminance * texparam->diffuse) + texparam->ambient;
            luminance = std::clamp(luminance, 0.0f, 255.0f);
            luma = static_cast<s32>(luminance);
            luma += static_cast<s32>(face);

            coef = m_coef_table[attr >> 27];
            distance = coef * std::fabs(dotp) * m_lod;

            push(static_cast<u32>(luma) << 15);
            push(f2u(distance) >> 8);
            push(f2u(point.x) >> 8);
            push(f2u(point.y) >> 8);
            push(f2u(point.pz()) >> 8);

            if (attr & 1) {
                point.x = u2f(*input++);
                point.y = u2f(*input++);
                point.pz() = u2f(*input++);
                transform_point(&point, m_matrix);
                p3 = point;
                point.x *= m_focus.x;
                point.y *= m_focus.y;
                push(f2u(point.x) >> 8);
                push(f2u(point.y) >> 8);
                push(f2u(point.pz()) >> 8);
            } else {
                input += 3;
                p3 = p2;
            }
        } else {
            break;
        }

        switch ((attr >> 8) & 3) {
            case 0:
            case 2: p0 = p2; p1 = p3; break;
            case 1: p1 = p2; break;
            case 3: p0 = p3; break;
        }
    }
    push(0);
}

void Geometry::parse_nn_s(const u32* input, u32 count) {
    Vertex point, normal, p0, p1, p2, p3;
    u32 attr, i;

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    p0 = point;
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    point.x = u2f(*input++);
    point.y = u2f(*input++);
    point.pz() = u2f(*input++);
    transform_point(&point, m_matrix);
    p1 = point;
    point.x *= m_focus.x;
    point.y *= m_focus.y;
    push(f2u(point.x) >> 8);
    push(f2u(point.y) >> 8);
    push(f2u(point.pz()) >> 8);

    for (i = 0; i < count; i++) {
        attr = *input++;
        push(attr & 0x0003ffff);

        if ((attr & 3) != 0) {
            float dotl, dotp, luminance, distance, coef, face, specular;
            s32 luma;
            TextureParam* texparam;

            input += 3;  // skip normal

            point.x = u2f(*input++);
            point.y = u2f(*input++);
            point.pz() = u2f(*input++);
            transform_point(&point, m_matrix);
            p2 = point;

            vector_cross3(&normal, &p0, &p1, &p2);
            normalize_vector(&normal);

            dotl = dot_product(normal, m_light);
            dotp = dot_product(normal, point);
            point.x *= m_focus.x;
            point.y *= m_focus.y;

            face = 0x100;
            if (dotp >= 0) face = 0;

            texparam = &m_texture_parameters[(attr >> 18) & 0x1f];

            if ((dotl * dotp) < 0) luminance = 0;
            else luminance = std::fabs(dotl);

            specular = ((2 * dotl) * normal.pz()) - m_light.pz();
            if (specular < 0) specular = 0;
            if (texparam->specular_control == 0) specular = 0;
            if ((texparam->specular_control >> 1) != 0) specular *= specular;
            if ((texparam->specular_control >> 2) != 0) specular *= specular;
            if (((texparam->specular_control + 1) >> 3) != 0) specular *= specular;
            specular *= texparam->specular_scale;

            luminance = (luminance * texparam->diffuse) + texparam->ambient + specular;
            luminance = std::clamp(luminance, 0.0f, 255.0f);
            luma = static_cast<s32>(luminance);
            luma += static_cast<s32>(face);

            coef = m_coef_table[attr >> 27];
            distance = coef * std::fabs(dotp) * m_lod;

            push(static_cast<u32>(luma) << 15);
            push(f2u(distance) >> 8);
            push(f2u(point.x) >> 8);
            push(f2u(point.y) >> 8);
            push(f2u(point.pz()) >> 8);

            if (attr & 1) {
                point.x = u2f(*input++);
                point.y = u2f(*input++);
                point.pz() = u2f(*input++);
                transform_point(&point, m_matrix);
                p3 = point;
                point.x *= m_focus.x;
                point.y *= m_focus.y;
                push(f2u(point.x) >> 8);
                push(f2u(point.y) >> 8);
                push(f2u(point.pz()) >> 8);
            } else {
                input += 3;
                p3 = p2;
            }
        } else {
            break;
        }

        switch ((attr >> 8) & 3) {
            case 0:
            case 2: p0 = p2; p1 = p3; break;
            case 1: p1 = p2; break;
            case 3: p0 = p3; break;
        }
    }
    push(0);
}

// ---------------------------------------------------------------------------
// Hardware rasteriser: command decoder (model2_v.cpp model2_3d_push)
// ---------------------------------------------------------------------------

void Geometry::push(u32 input) {
    if (m_cur_command != 0) {
        if (m_command_index < 32)
            m_command_buffer[m_command_index++] = input;

        switch (m_cur_command) {
            case 0x00:  // NOP
                break;

            case 0x01: {  // Polygon Data
                if (m_command_index < 9)
                    return;
                const u32 attr = m_command_buffer[8];
                if ((attr & 3) == 0) {
                    m_cur_command = 0;
                    return;
                }
                if (attr & 1) {
                    if (m_command_index < 17)
                        return;
                    process_polygon<4>(attr);
                    m_command_index = 8;
                } else {
                    if (m_command_index < 14)
                        return;
                    process_polygon<3>(attr);
                    m_command_index = 8;
                }
            } break;

            case 0x03: {  // Window Data
                if (m_command_index < 6)
                    return;
                auto sext12 = [](u32 v) -> s16 {
                    return static_cast<s16>(v & 0x800 ? -(0x800 - (v & 0x7ff)) : v);
                };
                m_viewport[0] = sext12((m_command_buffer[0] >> 12) & 0xfff);
                m_viewport[1] = sext12(m_command_buffer[0] & 0xfff);
                m_viewport[2] = sext12((m_command_buffer[1] >> 12) & 0xfff);
                m_viewport[3] = sext12(m_command_buffer[1] & 0xfff);

                for (u32 i = 0; i < 4; i++) {
                    m_center[i][0] = sext12((m_command_buffer[2 + i] >> 12) & 0xfff);
                    m_center[i][1] = sext12(m_command_buffer[2 + i] & 0xfff);

                    const float left_plane = static_cast<float>(m_center[i][0] - m_viewport[0]);
                    const float right_plane = static_cast<float>(m_viewport[2] - m_center[i][0]);
                    const float top_plane = static_cast<float>(m_viewport[3] - m_center[i][1]);
                    const float bottom_plane = static_cast<float>(m_center[i][1] - m_viewport[1]);

                    m_clip_plane[i][0].normal.x = 1.0f / std::hypot(1.0f, left_plane);
                    m_clip_plane[i][0].normal.y = 0.0f;
                    m_clip_plane[i][0].normal.pz() = left_plane / std::hypot(1.0f, left_plane);

                    m_clip_plane[i][1].normal.x = -1.0f / std::hypot(-1.0f, right_plane);
                    m_clip_plane[i][1].normal.y = 0.0f;
                    m_clip_plane[i][1].normal.pz() = right_plane / std::hypot(-1.0f, right_plane);

                    m_clip_plane[i][2].normal.x = 0.0f;
                    m_clip_plane[i][2].normal.y = -1.0f / std::hypot(-1.0f, top_plane);
                    m_clip_plane[i][2].normal.pz() = top_plane / std::hypot(-1.0f, top_plane);

                    m_clip_plane[i][3].normal.x = 0.0f;
                    m_clip_plane[i][3].normal.y = 1.0f / std::hypot(1.0f, bottom_plane);
                    m_clip_plane[i][3].normal.pz() = bottom_plane / std::hypot(1.0f, bottom_plane);
                }
                m_cur_command = 0;
            } break;

            case 0x04: {  // Texture / Log Data write
                if (m_command_index < 2)
                    return;
                if (m_command_buffer[1] > 0) {
                    if (m_command_index >= 3) {
                        const u32 address = m_command_buffer[0];
                        if (address & 0x800000)
                            m_texture_ram[address & 0xffff] = m_command_buffer[2];
                        else
                            m_log_ram[address & 0x7fff] = static_cast<u8>(m_command_buffer[2]);
                        m_command_buffer[0]++;
                        m_command_buffer[1]--;
                        m_command_index--;
                    }
                }
                if (m_command_buffer[1] == 0)
                    m_cur_command = 0;
            } break;

            case 0x08:  // ZSort mode
                m_z_adjust = static_cast<s32>(m_command_buffer[0] << 8);
                m_cur_command = 0;
                break;

            default:
                // Unknown command: resync instead of aborting the whole frame.
                m_cur_command = 0;
                break;
        }
    } else {
        m_cur_command = input & 0x0f;
        m_command_index = 0;
        if (m_cur_command == 1) {
            m_reverse = (input >> 4) & 1;
            m_center_sel = (input >> 6) & 3;
        }
    }
}

// ---------------------------------------------------------------------------
// Hardware rasteriser: polygon assembly (model2_v.cpp process_polygon)
// ---------------------------------------------------------------------------

std::int32_t Geometry::round_coord(float value) {
    if (value >= static_cast<float>(std::numeric_limits<s32>::max()))
        return std::numeric_limits<s32>::max();
    const float ipart = std::floor(value);
    if (ipart < static_cast<float>(std::numeric_limits<s32>::min()))
        return std::numeric_limits<s32>::min();
    const float fpart = value - ipart;
    return static_cast<s32>(ipart) + ((fpart > 0.5f) ? 1 : 0);
}

u16 Geometry::float_to_zval(float value, s32 z_adjust) {
    const s32 fpint = static_cast<s32>(f2u(value));
    s32 exponent = ((fpint >> 23) & 0xff) - ((z_adjust >> 23) & 0xff);
    u32 mantissa = static_cast<u32>(fpint) & 0x7fffff;

    mantissa += 0x400;
    if (mantissa > 0x7fffff) {
        exponent++;
        mantissa = (mantissa & 0x7fffff) >> 1;
    }
    mantissa >>= 11;

    if (fpint < 0)
        return 0x0000;
    if (exponent < -12)
        return 0x0000;
    if (exponent < 0)
        return static_cast<u16>((mantissa | 0x1000) >> -exponent);
    if (exponent < 15)
        return static_cast<u16>(((exponent + 1) << 12) | mantissa);
    return 0xffff;
}

bool Geometry::check_culling(u32 attr, float min_z, float max_z) const {
    if (((attr >> 17) & 1) == 0) {
        if (m_command_buffer[9] & 0x00800000)
            return true;
    }
    if (((attr >> 8) & 3) == 0)
        return true;
    if (m_master_z_clip != 0xff && static_cast<s32>(1.0 / min_z) > m_master_z_clip)
        return true;
    if (max_z < 0)
        return true;
    return false;
}

template <unsigned NumVerts>
void Geometry::process_polygon(u32 attr) {
    Polygon object;


    object.v[1].x = u2f(m_command_buffer[2] << 8);
    object.v[1].y = u2f(m_command_buffer[3] << 8);
    object.v[1].pz() = u2f(m_command_buffer[4] << 8);

    object.v[0].x = u2f(m_command_buffer[5] << 8);
    object.v[0].y = u2f(m_command_buffer[6] << 8);
    object.v[0].pz() = u2f(m_command_buffer[7] << 8);

    object.v[2].x = u2f(m_command_buffer[11] << 8);
    object.v[2].y = u2f(m_command_buffer[12] << 8);
    object.v[2].pz() = u2f(m_command_buffer[13] << 8);

    if constexpr (NumVerts == 4) {
        object.v[3].x = u2f(m_command_buffer[14] << 8);
        object.v[3].y = u2f(m_command_buffer[15] << 8);
        object.v[3].pz() = u2f(m_command_buffer[16] << 8);
    } else {
        m_command_buffer[14] = m_command_buffer[11];
        m_command_buffer[15] = m_command_buffer[12];
        m_command_buffer[16] = m_command_buffer[13];
    }

    float min_z = object.v[0].pz();
    if (object.v[1].pz() < min_z) min_z = object.v[1].pz();
    if (object.v[2].pz() < min_z) min_z = object.v[2].pz();
    if constexpr (NumVerts == 4)
        if (object.v[3].pz() < min_z) min_z = object.v[3].pz();

    float max_z = object.v[0].pz();
    if (object.v[1].pz() > max_z) max_z = object.v[1].pz();
    if (object.v[2].pz() > max_z) max_z = object.v[2].pz();
    if constexpr (NumVerts == 4)
        if (object.v[3].pz() > max_z) max_z = object.v[3].pz();

    std::size_t tp_words = 0;
    const u16* tp_base = tex_source(m_command_buffer[0], tp_words);
    std::size_t tp_index = m_command_buffer[0] & 0xffff;

    object.v[0].pv() = tex_read(tp_base, tp_words, tp_index++);
    object.v[0].pu() = tex_read(tp_base, tp_words, tp_index++);
    object.v[1].pv() = tex_read(tp_base, tp_words, tp_index++);
    object.v[1].pu() = tex_read(tp_base, tp_words, tp_index++);
    object.v[2].pv() = tex_read(tp_base, tp_words, tp_index++);
    object.v[2].pu() = tex_read(tp_base, tp_words, tp_index++);
    if constexpr (NumVerts == 4) {
        object.v[3].pv() = tex_read(tp_base, tp_words, tp_index++);
        object.v[3].pu() = tex_read(tp_base, tp_words, tp_index++);
    }

    m_command_buffer[0] += NumVerts * 2;

    std::size_t th_words = 0;
    const u16* th_base = tex_source(m_command_buffer[1], th_words);
    std::size_t th_index = m_command_buffer[1] & 0xffff;

    object.texheader[0] = tex_read(th_base, th_words, th_index++);
    object.texheader[1] = tex_read(th_base, th_words, th_index++);
    object.texheader[2] = tex_read(th_base, th_words, th_index++);
    object.texheader[3] = tex_read(th_base, th_words, th_index++);

    const s32 tho = sext(attr >> 12, 5);
    m_command_buffer[1] += static_cast<u32>(tho * 4);

    object.luma = static_cast<u8>((m_command_buffer[9] >> 15) & 0xff);
    object.texlod = static_cast<s32>(((m_command_buffer[10] >> 8) & 0x7f80)) - 0x3f80;
    object.texlod += m_log_ram[m_command_buffer[10] & 0x7fff];

    const bool cull = check_culling(attr, min_z, max_z);

    float zvalue;
    switch ((attr >> 10) & 3) {
        case 0: zvalue = m_polygon_z; break;
        case 1: zvalue = min_z; break;
        case 2: zvalue = max_z; break;
        default: zvalue = 1e10f; break;
    }
    m_polygon_z = zvalue;

    if (!cull) {
        Vertex verts_in[8], verts_out[8];
        for (unsigned i = 0; i < NumVerts; i++)
            verts_in[i] = object.v[i];

        s32 clipped_verts = NumVerts;
        for (int i = 0; i < 4; i++) {
            clipped_verts = clip_polygon(verts_in, clipped_verts, verts_out,
                                         m_clip_plane[m_center_sel][i]);
            for (s32 j = 0; j < clipped_verts; j++)
                verts_in[j] = verts_out[j];
        }

        // MAME aborts the frame when the polygon pool is exhausted; the reference
        // drops the remaining polygons instead. Either way the slot must NOT be
        // reused: wrapping the index relinks a polygon that is still in
        // m_poly_sorted_list, which builds a cycle the renderer never escapes.
        if (clipped_verts > 2 && m_poly_list_index < m_poly_list.size()) {
            object.z = float_to_zval(zvalue, m_z_adjust);
            Polygon* const zpoly = m_poly_sorted_list[object.z];
            Polygon& poly = m_poly_list[m_poly_list_index++];

            poly.z = object.z;
            for (int i = 0; i < 4; i++)
                poly.texheader[i] = object.texheader[i];
            poly.luma = object.luma;
            poly.texlod = object.texlod;
            for (int i = 0; i < 4; i++)
                poly.viewport[i] = m_viewport[i];
            poly.center[0] = m_center[m_center_sel][0];
            poly.center[1] = m_center[m_center_sel][1];
            poly.window = m_cur_window;
            poly.num_vertices = static_cast<u8>(clipped_verts);
            for (s32 i = 0; i < clipped_verts; i++)
                poly.v[i] = verts_out[i];

            m_poly_sorted_list[object.z] = &poly;
            poly.next = zpoly;

            if (object.z < m_min_z) m_min_z = object.z;
            if (object.z > m_max_z) m_max_z = object.z;
        }
    }

    switch ((attr >> 8) & 3) {
        case 0:
        case 2:
            for (u32 i = 0; i < 6; i++)
                m_command_buffer[2 + i] = m_command_buffer[11 + i];
            break;
        case 1:
            for (u32 i = 0; i < 3; i++)
                m_command_buffer[5 + i] = m_command_buffer[11 + i];
            break;
        case 3:
            for (u32 i = 0; i < 3; i++)
                m_command_buffer[2 + i] = m_command_buffer[14 + i];
            break;
    }
}

// ---------------------------------------------------------------------------
// Polygon clipping (model2_v.cpp clip_polygon)
// ---------------------------------------------------------------------------

s32 Geometry::clip_polygon(Vertex* v, s32 num_vertices, Vertex* vout, Plane clip_plane) {
    s32 outcount = 0;
    const Vertex* cur = v;
    Vertex* out = vout;

    float curdot = dot_product(*cur, clip_plane.normal);
    s32 curin = (curdot >= clip_plane.distance) ? 1 : 0;

    for (s32 i = 0; i < num_vertices; i++) {
        const s32 nextvert = (i + 1) % num_vertices;
        if (curin)
            out[outcount++] = *cur;

        const float nextdot = dot_product(v[nextvert], clip_plane.normal);
        const s32 nextin = (nextdot >= clip_plane.distance) ? 1 : 0;

        if ((curin != nextin) && !std::isnan(curdot) && !std::isnan(nextdot)) {
            const float scale = (clip_plane.distance - curdot) / (nextdot - curdot);
            out[outcount].x = cur->x + ((v[nextvert].x - cur->x) * scale);
            out[outcount].y = cur->y + ((v[nextvert].y - cur->y) * scale);
            out[outcount].pz() = cur->pz() + ((v[nextvert].pz() - cur->pz()) * scale);
            out[outcount].pu() = cur->pu() + ((v[nextvert].pu() - cur->pu()) * scale);
            out[outcount].pv() = cur->pv() + ((v[nextvert].pv() - cur->pv()) * scale);
            outcount++;
        }
        curdot = nextdot;
        curin = nextin;
        cur++;
    }
    return outcount;
}

// ---------------------------------------------------------------------------
// Frame setup and rendering
// ---------------------------------------------------------------------------

void Geometry::render_frame_start() {
    m_poly_list_index = 0;
    std::fill(m_poly_sorted_list.begin(), m_poly_sorted_list.end(), nullptr);
    m_min_z = 0xffff;
    m_max_z = 0;
    m_polygon_z = 1e10f;
    m_cur_window = 0;
    m_render_done = false;
}

void Geometry::project(Polygon& poly) {
    for (int i = 0; i < poly.num_vertices; i++) {
        poly.v[i].x = static_cast<float>(m_crtc_xoffset + poly.center[0]) +
                      (poly.v[i].x / (poly.v[i].pz() + std::numeric_limits<float>::min()));
        poly.v[i].y = static_cast<float>((384 - poly.center[1]) + m_crtc_yoffset) -
                      (poly.v[i].y / (poly.v[i].pz() + std::numeric_limits<float>::min()));
    }
}

void Geometry::render_one(Polygon& poly) {
    const int renderer = (poly.texheader[0] >> 13) & 3;
    ++m_polygons_rendered;

    const int vp_left = std::max(poly.viewport[0] + m_xoffs, kClipLeft);
    const int vp_top = std::max((384 - poly.viewport[3]) + m_yoffs, kClipTop);
    const int vp_right = std::min(poly.viewport[2] + m_xoffs, kClipRight);
    const int vp_bottom = std::min((384 - poly.viewport[1]) + m_yoffs, kClipBottom);
    if (vp_left > vp_right || vp_top > vp_bottom)
        return;

    poly.checker = (poly.texheader[0] >> 15) & 1;
    poly.lumabase = (poly.texheader[1] & 0xff) << 7;
    poly.colorbase = (poly.texheader[3] >> 6) & 0x3ff;

    if (renderer & 2) {
        poly.texmirrorx = (poly.texheader[0] >> 8) & 1;
        poly.texmirrory = (poly.texheader[0] >> 9) & 1;
        poly.texwrapx = ((poly.texheader[0] >> 6) & 1) & ~poly.texmirrorx;
        poly.texwrapy = ((poly.texheader[0] >> 7) & 1) & ~poly.texmirrory;

        poly.texsheet[0] = (poly.texheader[2] & 0x1000) ? m_textureram1 : m_textureram0;
        poly.texsheet[1] = (poly.texheader[2] & 0x1000) ? m_textureram0 : m_textureram1;
        poly.texwidth = 32u << ((poly.texheader[0] >> 0) & 0x7);
        poly.texheight = 32u << ((poly.texheader[0] >> 3) & 0x7);
        poly.texx = 32 * ((poly.texheader[2] >> 0) & 0x3f);
        poly.texy = 32 * ((poly.texheader[2] >> 6) & 0x1f);
        poly.utex = (poly.texheader[0] >> 12) & 1;
        poly.utexminlod = (poly.texheader[0] >> 10) & 3;
        poly.utexx = ((poly.texheader[2] >> 13) & 1) * 128;
        poly.utexy = ((poly.texheader[2] >> 14) & 3) * 128;

        for (int i = 0; i < poly.num_vertices; i++) {
            poly.v[i].pz() = 1.0f / (poly.v[i].pz() + std::numeric_limits<float>::min());
            poly.v[i].pu() = poly.v[i].pu() * poly.v[i].pz() * (1.0f / 8.0f);
            poly.v[i].pv() = poly.v[i].pv() * poly.v[i].pz() * (1.0f / 8.0f);
        }
    }

    // Discard textured polygons with an invalid texture sheet — prevents rogue
    // neon green/cyan triangles from garbled texture headers. Solid polygons
    // (renderer & 2 == 0) intentionally leave texsheet/texwidth/texheight at
    // their defaults and are unaffected.
    if ((renderer & 2) &&
        (poly.texsheet[0] == nullptr || poly.texwidth == 0 || poly.texheight == 0))
        return;

    // The 3D rasteriser clips to the viewport rectangle only (the polygon
    // clipping already happened in hardware space).
    m_clip_l = vp_left;
    m_clip_t = vp_top;
    m_clip_r = vp_right;
    m_clip_b = vp_bottom;

    switch (poly.num_vertices) {
        case 3: fill_triangle(poly.v[0], poly.v[1], poly.v[2], poly, renderer); break;
        case 4: fill_polygon<4>(poly.v, poly, renderer); break;
        case 5: fill_polygon<5>(poly.v, poly, renderer); break;
        case 6: fill_polygon<6>(poly.v, poly, renderer); break;
        case 7: fill_polygon<7>(poly.v, poly, renderer); break;
        case 8: fill_polygon<8>(poly.v, poly, renderer); break;
        default: break;
    }
}

void Geometry::render_polygons() {
    if (m_render_done)
        return;
    if (m_poly_list_index == 0)
        return;

    std::fill(m_dest.begin(), m_dest.end(), 0u);
    std::fill(m_fill.begin(), m_fill.end(), static_cast<u8>(0));

    for (int window = m_cur_window; window >= 0; window--) {
        for (s32 z = m_min_z; z <= m_max_z; z++) {
            Polygon* poly = m_poly_sorted_list[z];
            while (poly != nullptr) {
                if (poly->window == window) {
                    project(*poly);
                    render_one(*poly);
                }
                poly = poly->next;
            }
        }
    }
    m_render_done = true;

}

const std::uint32_t* Geometry::render() {
    render_polygons();
    return m_dest.data();
}

// ---------------------------------------------------------------------------
// Polygon fill (poly.h poly_manager::render_triangle / render_polygon)
// ---------------------------------------------------------------------------

void Geometry::fill_triangle(const Vertex& _v1, const Vertex& _v2, const Vertex& _v3,
                             const Polygon& poly, int renderer) {
    const Vertex* v1 = &_v1;
    const Vertex* v2 = &_v2;
    const Vertex* v3 = &_v3;

    if (v2->y < v1->y) std::swap(v1, v2);
    if (v3->y < v2->y) {
        std::swap(v2, v3);
        if (v2->y < v1->y) std::swap(v1, v2);
    }

    const s32 v1y = round_coord(v1->y);
    const s32 v3y = round_coord(v3->y);
    const s32 v1yclip = std::max(v1y, m_clip_t);
    const s32 v3yclip = std::min(v3y, m_clip_b + 1);
    if (v3yclip - v1yclip <= 0)
        return;

    const float dxdy_v1v2 = (v2->y == v1->y) ? 0.0f : (v2->x - v1->x) / (v2->y - v1->y);
    const float dxdy_v1v3 = (v3->y == v1->y) ? 0.0f : (v3->x - v1->x) / (v3->y - v1->y);
    const float dxdy_v2v3 = (v3->y == v2->y) ? 0.0f : (v3->x - v2->x) / (v3->y - v2->y);

    float param_start[3], param_dpdx[3], param_dpdy[3];
    {
        const float a00 = v2->y - v3->y;
        const float a01 = v3->x - v2->x;
        const float a02 = v2->x * v3->y - v3->x * v2->y;
        const float a10 = v3->y - v1->y;
        const float a11 = v1->x - v3->x;
        const float a12 = v3->x * v1->y - v1->x * v3->y;
        const float a20 = v1->y - v2->y;
        const float a21 = v2->x - v1->x;
        const float a22 = v1->x * v2->y - v2->x * v1->y;
        const float det = a02 + a12 + a22;
        if (std::fabs(det) < 0.00001f) {
            for (int p = 0; p < 3; p++) {
                param_dpdx[p] = 0.0f;
                param_dpdy[p] = 0.0f;
                param_start[p] = v1->p[p];
            }
        } else {
            const float idet = 1.0f / det;
            for (int p = 0; p < 3; p++) {
                param_dpdx[p] = idet * (v1->p[p] * a00 + v2->p[p] * a10 + v3->p[p] * a20);
                param_dpdy[p] = idet * (v1->p[p] * a01 + v2->p[p] * a11 + v3->p[p] * a21);
                param_start[p] = idet * (v1->p[p] * a02 + v2->p[p] * a12 + v3->p[p] * a22);
            }
        }
    }

    for (s32 curscan = v1yclip; curscan < v3yclip; curscan++) {
        const float fully = static_cast<float>(curscan) + 0.5f;
        const float startx = v1->x + (fully - v1->y) * dxdy_v1v3;
        const float stopx = (fully < v2->y) ? (v1->x + (fully - v1->y) * dxdy_v1v2)
                                            : (v2->x + (fully - v2->y) * dxdy_v2v3);

        s32 istartx = round_coord(startx);
        s32 istopx = round_coord(stopx);
        if (istartx > istopx) std::swap(istartx, istopx);
        istartx = std::max(istartx, m_clip_l);
        istopx = std::min(istopx, m_clip_r + 1);
        if (istartx >= istopx)
            continue;

        Extent extent;
        extent.startx = istartx;
        extent.stopx = istopx;
        const float fullstartx = static_cast<float>(istartx) + 0.5f;
        for (int p = 0; p < 3; p++) {
            extent.p_start[p] =
                param_start[p] + fullstartx * param_dpdx[p] + fully * param_dpdy[p];
            extent.p_dpdx[p] = param_dpdx[p];
        }
        draw_span(curscan, extent, poly, renderer);
    }
}

template <unsigned NumVerts>
void Geometry::fill_polygon(const Vertex* v, const Polygon& poly, int renderer) {
    float minx = v[0].x, maxx = v[0].x;
    int minv = 0, maxv = 0;
    for (unsigned n = 1; n < NumVerts; n++) {
        if (v[n].y < v[minv].y) minv = static_cast<int>(n);
        else if (v[n].y > v[maxv].y) maxv = static_cast<int>(n);
        minx = std::min(minx, v[n].x);
        maxx = std::max(maxx, v[n].x);
    }
    (void)minx;
    (void)maxx;

    const s32 miny = round_coord(v[minv].y);
    const s32 maxy = round_coord(v[maxv].y);
    const s32 minyclip = std::max(miny, m_clip_t);
    const s32 maxyclip = std::min(maxy, m_clip_b + 1);
    if (maxyclip - minyclip <= 0)
        return;

    struct Edge {
        const Vertex* v1;
        const Vertex* v2;
        float dxdy;
        float dpdy[3];
    };
    Edge fed[NumVerts > 1 ? NumVerts - 1 : 1];
    int fedn = 0;
    for (int curv = minv; curv != maxv; curv = (curv == static_cast<int>(NumVerts) - 1) ? 0 : curv + 1) {
        Edge& e = fed[fedn];
        e.v1 = &v[curv];
        e.v2 = &v[(curv == static_cast<int>(NumVerts) - 1) ? 0 : curv + 1];
        if (e.v1->y == e.v2->y)
            continue;
        const float ooy = 1.0f / (e.v2->y - e.v1->y);
        e.dxdy = (e.v2->x - e.v1->x) * ooy;
        for (int p = 0; p < 3; p++)
            e.dpdy[p] = (e.v2->p[p] - e.v1->p[p]) * ooy;
        fedn++;
    }

    Edge bed[NumVerts > 1 ? NumVerts - 1 : 1];
    int bedn = 0;
    for (int curv = minv; curv != maxv; curv = (curv == 0) ? static_cast<int>(NumVerts) - 1 : curv - 1) {
        Edge& e = bed[bedn];
        e.v1 = &v[curv];
        e.v2 = &v[(curv == 0) ? static_cast<int>(NumVerts) - 1 : curv - 1];
        if (e.v1->y == e.v2->y)
            continue;
        const float ooy = 1.0f / (e.v2->y - e.v1->y);
        e.dxdy = (e.v2->x - e.v1->x) * ooy;
        for (int p = 0; p < 3; p++)
            e.dpdy[p] = (e.v2->p[p] - e.v1->p[p]) * ooy;
        bedn++;
    }
    if (fedn == 0 || bedn == 0)
        return;

    const Edge* ledge;
    const Edge* redge;
    if ((fed[0].v1 == bed[0].v1 && fed[0].dxdy < bed[0].dxdy) ||
        (fed[0].v1 != bed[0].v1 && fed[0].v1->x < bed[0].v1->x)) {
        ledge = fed;
        redge = bed;
    } else {
        ledge = bed;
        redge = fed;
    }
    const Edge* ledge_end = ledge + (ledge == fed ? fedn : bedn) - 1;
    const Edge* redge_end = redge + (redge == fed ? fedn : bedn) - 1;

    for (s32 curscan = minyclip; curscan < maxyclip; curscan++) {
        const float fully = static_cast<float>(curscan) + 0.5f;
        while (fully > ledge->v2->y && fully < v[maxv].y && ledge < ledge_end) ++ledge;
        while (fully > redge->v2->y && fully < v[maxv].y && redge < redge_end) ++redge;

        const float startx = ledge->v1->x + (fully - ledge->v1->y) * ledge->dxdy;
        const float stopx = redge->v1->x + (fully - redge->v1->y) * redge->dxdy;

        s32 istartx = round_coord(startx);
        s32 istopx = round_coord(stopx);
        if (istartx > istopx) std::swap(istartx, istopx);
        istartx = std::max(istartx, m_clip_l);
        istopx = std::min(istopx, m_clip_r + 1);
        if (istartx >= istopx)
            continue;

        Extent extent;
        extent.startx = istartx;
        extent.stopx = istopx;

        const float ldy = fully - ledge->v1->y;
        const float rdy = fully - redge->v1->y;
        const float oox = 1.0f / (stopx - startx);
        for (int p = 0; p < 3; p++) {
            const float lparam = ledge->v1->p[p] + ldy * ledge->dpdy[p];
            const float rparam = redge->v1->p[p] + rdy * redge->dpdy[p];
            const float dpdx = (rparam - lparam) * oox;
            extent.p_start[p] = lparam + (static_cast<float>(istartx) + 0.5f - startx) * dpdx;
            extent.p_dpdx[p] = dpdx;
        }
        draw_span(curscan, extent, poly, renderer);
    }
}

// ---------------------------------------------------------------------------
// Scanline renderers (model2rd.ipp)
// ---------------------------------------------------------------------------

void Geometry::draw_span(int scanline, const Extent& extent, const Polygon& poly, int renderer) {
    switch (renderer) {
        case 0: draw_scanline_solid<false>(scanline, extent, poly); break;
        case 1: draw_scanline_solid<true>(scanline, extent, poly); break;
        case 2: draw_scanline_tex<false>(scanline, extent, poly); break;
        case 3: draw_scanline_tex<true>(scanline, extent, poly); break;
    }
}

u16 Geometry::get_texel(u32 base_x, u32 base_y, int x, int y, const u32* sheet) const {
    int x2 = static_cast<int>(base_x) + x;
    int y2 = static_cast<int>(base_y) + y;
    if (x2 >= 1024) {
        x2 -= 1024;
        y2 ^= 1024;
    }
    const u32 offset = static_cast<u32>((y2 / 2) * 512 + (x2 / 2));
    const u32 index = offset >> 1;
    u32 texel = index < m_texture_words ? sheet[index] : 0u;
    if (offset & 1)
        texel >>= 16;
    if ((y & 1) == 0)
        texel >>= 8;
    if ((x & 1) == 0)
        texel >>= 4;
    return static_cast<u16>(texel & 0x0f);
}

template <bool Translucent>
u32 Geometry::fetch_bilinear_texel(const Polygon& poly, s32 miplevel, s32 u, s32 v) const {
    const u32 tex_wrap_x = poly.texwrapx;
    const u32 tex_wrap_y = poly.texwrapy;
    const u32 tex_mirr_x = poly.texmirrorx;
    const u32 tex_mirr_y = poly.texmirrory;
    u32 tex_width, tex_height, tex_x, tex_y;
    const u32* sheet;

    if (miplevel == -1) {
        tex_width = 128;
        tex_height = 128;
        tex_x = poly.utexx;
        tex_y = poly.utexy;
        sheet = poly.texsheet[1];
        u <<= 1 << poly.utexminlod;
        v <<= 1 << poly.utexminlod;
    } else {
        tex_width = poly.texwidth >> miplevel;
        tex_height = poly.texheight >> miplevel;
        tex_x = (static_cast<u32>(poly.texx - 2048) >> miplevel) & 2047;
        tex_y = (static_cast<u32>(poly.texy - 1024) >> miplevel) & 1023;
        sheet = poly.texsheet[miplevel & 1];
        u >>= miplevel;
        v >>= miplevel;
    }

    if (tex_mirr_x && (u & static_cast<s32>(tex_width << 8)))
        u = ~u;
    if (tex_mirr_y && (v & static_cast<s32>(tex_height << 8)))
        v = ~v;

    u -= 0x80;
    v -= 0x80;

    u32 ufrac = static_cast<u32>(u) & 0xff;
    u32 vfrac = static_cast<u32>(v) & 0xff;
    s32 u0 = static_cast<s32>(u) >> 8;
    s32 u1 = u0 + 1;
    s32 v0 = static_cast<s32>(v) >> 8;
    s32 v1 = v0 + 1;

    if (tex_wrap_x) {
        u0 = static_cast<s32>(static_cast<u32>(u0) & (tex_width - 1));
        u1 = static_cast<s32>(static_cast<u32>(u1) & (tex_width - 1));
    } else {
        if (u0 < 0) u0 = 0;
        if (u0 >= static_cast<s32>(tex_width)) u0 = static_cast<s32>(tex_width) - 1;
        if (u1 < 0) u1 = 0;
        if (u1 >= static_cast<s32>(tex_width)) u1 = static_cast<s32>(tex_width) - 1;
    }
    if (tex_wrap_y) {
        v0 = static_cast<s32>(static_cast<u32>(v0) & (tex_height - 1));
        v1 = static_cast<s32>(static_cast<u32>(v1) & (tex_height - 1));
    } else {
        if (v0 < 0) v0 = 0;
        if (v0 >= static_cast<s32>(tex_height)) v0 = static_cast<s32>(tex_height) - 1;
        if (v1 < 0) v1 = 0;
        if (v1 >= static_cast<s32>(tex_height)) v1 = static_cast<s32>(tex_height) - 1;
    }

    u32 tex00 = static_cast<u32>(get_texel(tex_x, tex_y, static_cast<int>(u0), static_cast<int>(v0), sheet)) << 4;
    u32 tex01 = static_cast<u32>(get_texel(tex_x, tex_y, static_cast<int>(u1), static_cast<int>(v0), sheet)) << 4;
    u32 tex10 = static_cast<u32>(get_texel(tex_x, tex_y, static_cast<int>(u0), static_cast<int>(v1), sheet)) << 4;
    u32 tex11 = static_cast<u32>(get_texel(tex_x, tex_y, static_cast<int>(u1), static_cast<int>(v1), sheet)) << 4;

    if (Translucent) {
        if (tex00 != 0xf0) tex00 |= 0x00800000;
        if (tex01 != 0xf0) tex01 |= 0x00800000;
        if (tex10 != 0xf0) tex10 |= 0x00800000;
        if (tex11 != 0xf0) tex11 |= 0x00800000;

        if (tex00 == 0x000000f0) tex00 = tex01 & 0xff;
        if (tex01 == 0x000000f0) tex01 = tex00 & 0xff;
        if (tex10 == 0x000000f0) tex10 = tex11 & 0xff;
        if (tex11 == 0x000000f0) tex11 = tex10 & 0xff;
    }

    u32 tex0x = LERP(tex00, tex01, ufrac);
    u32 tex1x = LERP(tex10, tex11, ufrac);

    if (Translucent) {
        if (tex0x == 0x000000f0) tex0x = tex1x & 0xff;
        if (tex1x == 0x000000f0) tex1x = tex0x & 0xff;
    }

    return LERP(tex0x, tex1x, vfrac);
}

template <bool Translucent>
void Geometry::draw_scanline_solid(int scanline, const Extent& extent, const Polygon& poly) {
    if (Translucent)
        return;

    u32* const p = &m_dest[static_cast<std::size_t>(scanline) * kStride];
    u8* const fill = &m_fill[static_cast<std::size_t>(scanline) * kStride];

    // MAME: color = m_palram[object.colorbase + 0x1000] & 0xffff.
    const u32 color = pal_lookup(poly.colorbase, 0xffff);
    const u32 luma = poly.luma >> 2;

    const auto xlat = [&](std::size_t base, int shift, u32 idx) -> u8 {
        const std::size_t off = base +
            (static_cast<std::size_t>((color >> shift) & 0x1f) << 8) * 2 +
            static_cast<std::size_t>(idx) * 2;
        return off < 0xc000 ? m_colorxlat[off] : 0;
    };

    const u8 tr = gamma_lut(xlat(0x0000, 0, luma));
    const u8 tg = gamma_lut(xlat(0x4000, 5, luma));
    const u8 tb = gamma_lut(xlat(0x8000, 10, luma));
    const u32 out = 0xff000000u | (static_cast<u32>(tr) << 16) | (static_cast<u32>(tg) << 8) | tb;

    int x = extent.startx;
    int dx = poly.checker ? 2 : 1;
    if (poly.checker && !((x ^ scanline) & 1))
        x++;
    for (; x < extent.stopx; x += dx) {
        if (fill[x] == 0) {
            p[x] = out;
            fill[x] = 0xff;
        }
    }
}

template <bool Translucent>
void Geometry::draw_scanline_tex(int scanline, const Extent& extent, const Polygon& poly) {
    u32* const p = &m_dest[static_cast<std::size_t>(scanline) * kStride];
    u8* const fill = &m_fill[static_cast<std::size_t>(scanline) * kStride];

    float ooz = extent.p_start[0];
    float uoz = extent.p_start[1];
    float voz = extent.p_start[2];
    float dooz = extent.p_dpdx[0];
    float duoz = extent.p_dpdx[1];
    float dvoz = extent.p_dpdx[2];

    const s32 max_level = 30 - std::countl_zero(std::min(poly.texwidth, poly.texheight));
    // MAME: colorbase = m_palram[object.colorbase + 0x1000] & 0x7fff.
    const u32 colorbase = pal_lookup(poly.colorbase, 0x7fff);

    const auto xlat = [&](std::size_t base, int shift, u32 idx) -> u8 {
        const std::size_t off = base +
            (static_cast<std::size_t>((colorbase >> shift) & 0x1f) << 8) * 2 +
            static_cast<std::size_t>(idx) * 2;
        return off < 0xc000 ? m_colorxlat[off] : 0;
    };

    int x = extent.startx;
    int dx = 1;
    if (poly.checker) {
        if (!((x ^ scanline) & 1)) {
            x++;
            ooz += dooz;
            uoz += duoz;
            voz += dvoz;
        }
        dx = 2;
        dooz *= 2.0f;
        duoz *= 2.0f;
        dvoz *= 2.0f;
    }

    for (; x < extent.stopx; x += dx, ooz += dooz, uoz += duoz, voz += dvoz) {
        if (fill[x] > 0)
            continue;

        const float z = 1.0f / ooz;
        const s32 mml = -poly.texlod + fast_log2(z);
        const s32 level = std::clamp(mml >> 7, 0, max_level);

        const s32 u = static_cast<s32>(uoz * z * 256.0f);
        const s32 v = static_cast<s32>(voz * z * 256.0f);

        u32 t = fetch_bilinear_texel<Translucent>(poly, level, u, v);

        // Alpha discard: pen 0 (transparent texel on decals) never writes.
        if (t == 0)
            continue;

        if (mml > 0 && level < max_level) {
            const u32 t2 = fetch_bilinear_texel<Translucent>(poly, level + 1, u, v);
            const s32 frac = (mml & 127) << 1;
            t = LERP(t, t2, static_cast<unsigned>(frac));
        } else if (poly.utex && mml < 0) {
            const u32 t2 = fetch_bilinear_texel<Translucent>(poly, -1, u, v);
            const s32 frac = std::min(-mml >> poly.utexminlod, 127);
            t = LERP(t, t2, static_cast<unsigned>(frac));
        }

        if (Translucent) {
            if (t < 0x00400000)
                continue;
            t &= 0xff;
        }

        u8 luma = static_cast<u8>(
            static_cast<u32>(m_lumaram[poly.lumabase + (t >> 1)]) * poly.luma / 256);
        luma = std::min<u8>(luma, 0x3f);

        const u8 tr = gamma_lut(xlat(0x0000, 0, luma));
        const u8 tg = gamma_lut(xlat(0x4000, 5, luma));
        const u8 tb = gamma_lut(xlat(0x8000, 10, luma));

        p[x] = 0xff000000u | (static_cast<u32>(tr) << 16) | (static_cast<u32>(tg) << 8) | tb;
        fill[x] = 0xff;
    }
}

u32 Geometry::pal_lookup(u32 colorbase, u32 mask) const {
    if (!m_palram)
        return 0;
    const std::size_t off = (static_cast<std::size_t>(colorbase) + 0x1000) * 2;
    if (off + 1 >= 0x4000)
        return 0;
    return (static_cast<u32>(m_palram[off]) | (static_cast<u32>(m_palram[off + 1]) << 8)) & mask;
}

// Explicit instantiations.
template void Geometry::process_polygon<3>(u32);
template void Geometry::process_polygon<4>(u32);
template void Geometry::fill_polygon<4>(const Vertex*, const Polygon&, int);
template void Geometry::fill_polygon<5>(const Vertex*, const Polygon&, int);
template void Geometry::fill_polygon<6>(const Vertex*, const Polygon&, int);
template void Geometry::fill_polygon<7>(const Vertex*, const Polygon&, int);
template void Geometry::fill_polygon<8>(const Vertex*, const Polygon&, int);

}  // namespace sage

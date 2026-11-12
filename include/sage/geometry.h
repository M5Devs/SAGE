#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sage {

// Sega Model 2B 3D geometry pipeline and polygon rasteriser.
//
// A faithful port of MAME's src/mame/sega/model2_v.cpp (the geometry engine
// that walks the display list in buffer RAM, and the hardware rasteriser
// command decoder), src/mame/sega/model2rd.ipp (the textured/solid scanline
// renderers) and src/devices/video/poly.h (the polygon fill used by
// model2_renderer).
//
// The geometry engine (geo_state + geo_* command handlers + parse()) walks the
// display list the i960 builds in the GEO FIFO / buffer RAM and pushes a linear
// stream of transformed vertices into the hardware rasteriser (raster_state +
// push() + process_polygon()), which assembles polygons, clips them, z-sorts
// them into buckets and fills them into a 496x384 ARGB surface.
//
// BSD-3-Clause; copyright-holders: R. Belmont, Olivier Galibert, ElSemi,
// Angelo Salese, Matthew Daniels, David Haywood.
class Geometry {
public:
    static constexpr int kWidth = 496;
    static constexpr int kHeight = 384;

    // The 3D surface is allocated and rasterised at a 512-pixel row pitch (the
    // hardware destmap is 512x512), then composited as a 496x384 cliprect.
    // Writers and readers MUST agree on this pitch or every scanline shears.
    static constexpr int kDestStride = 512;

    // In MAME, `pz`/`pu`/`pv` are macros over the first three of the four
    // iterated parameters of poly_manager::vertex_t.
    struct Vertex {
        float x = 0, y = 0;
        float p[4] = {0, 0, 0, 0};
        float& pz() { return p[0]; }
        float& pu() { return p[1]; }
        float& pv() { return p[2]; }
        float pz() const { return p[0]; }
        float pu() const { return p[1]; }
        float pv() const { return p[2]; }
    };

    struct Plane {
        Vertex normal;
        float distance = 0;
    };

    struct TextureParam {
        float diffuse = 0;
        float ambient = 0;
        std::uint32_t specular_control = 0;
        float specular_scale = 0;
    };

    // Per-polygon data. The first block mirrors MAME's model2_state::polygon;
    // the second mirrors m2_poly_extra_data (derived from texheader in
    // model2_3d_render()).
    struct Polygon {
        Polygon* next = nullptr;
        Vertex v[8];
        std::uint8_t num_vertices = 3;
        std::uint16_t z = 0;
        std::uint16_t texheader[4] = {0, 0, 0, 0};
        std::uint8_t luma = 0;
        std::int32_t texlod = 0;
        std::int16_t viewport[4] = {0, 0, 0, 0};
        std::int16_t center[2] = {0, 0};
        std::uint8_t window = 0;

        // m2_poly_extra_data
        std::uint32_t lumabase = 0;
        std::uint32_t colorbase = 0;
        std::uint8_t checker = 0;
        std::uint32_t* texsheet[2] = {nullptr, nullptr};
        std::uint32_t texwidth = 0;
        std::uint32_t texheight = 0;
        std::uint32_t texx = 0;
        std::uint32_t texy = 0;
        std::uint8_t texwrapx = 0;
        std::uint8_t texwrapy = 0;
        std::uint8_t texmirrorx = 0;
        std::uint8_t texmirrory = 0;
        std::uint8_t utex = 0;
        std::uint8_t utexminlod = 0;
        std::uint32_t utexx = 0;
        std::uint32_t utexy = 0;
    };

    // A scanline span with 3 iterated parameters (ooZ, uoZ, voZ).
    struct Extent {
        int startx = 0, stopx = 0;
        float p_start[3] = {0, 0, 0};
        float p_dpdx[3] = {0, 0, 0};
    };

    Geometry();
    void reset();

    // ---- Register interface (MAME model2b_crx_mem) -----------------------
    // 0x00800000-0x00803fff: geo_r / geo_w (offset = byte address - base).
    std::uint32_t geo_read(std::uint32_t offset) const;
    void geo_write(std::uint32_t offset, std::uint32_t data);
    // 0x00804000-0x00807fff: geo_prg_r / geo_prg_w (the direct FIFO).
    std::uint32_t geo_prg_read(std::uint32_t offset) const;
    void geo_prg_write(std::uint32_t data);

    // GEO control register (0x00980008), MAME geo_ctl1_w.
    void set_geoctl(std::uint32_t value);
    std::uint32_t geoctl() const { return m_geoctl; }
    std::uint32_t geocnt() const { return m_geocnt; }

    // Called from the vblank handler: parses the current display list.
    void parse();

    // System 24 H/V sync outputs (0x01040000 / 0x01060000). MAME's
    // horizontal_sync_w()/vertical_sync_w() update both the CRTC offset used by
    // project() and the renderer's viewport offset.
    void set_hsync(std::uint16_t data);
    void set_vsync(std::uint16_t data);
    std::int16_t crtc_xoffset() const { return m_crtc_xoffset; }
    std::int16_t crtc_yoffset() const { return m_crtc_yoffset; }

    // Composites the current polygon frame into an internal 496x384 ARGB
    // surface and returns it (never null; an empty frame is fully transparent).
    const std::uint32_t* render();

    // ---- Shared resources (wired by the bus once the ROMs are loaded) -----
    void set_texture_rom(const std::uint8_t* data, std::size_t bytes);
    void set_polygon_rom(const std::uint8_t* data, std::size_t bytes);
    void set_texture_ram(std::uint32_t* ram0, std::uint32_t* ram1, std::size_t words);
    void set_luma_ram(const std::uint8_t* data);
    void set_palette_ram(const std::uint8_t* palram, const std::uint8_t* colorxlat);
    void set_buffer_ram(std::uint32_t* words, std::size_t word_count);

    // Diagnostic counters.
    std::uint64_t polygons_rendered() const { return m_polygons_rendered; }
    std::uint64_t parse_count() const { return m_parse_count; }

private:
    // ---- Geometry engine -------------------------------------------------
    const std::uint32_t* process_command(std::uint32_t opcode, const std::uint32_t* input,
                                         bool* end_code);
    const std::uint32_t* cmd_nop(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_object_data(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_direct_data(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_window_data(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_texture_data(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_polygon_data(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_texture_parameters(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_geo_mode(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_zsort_mode(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_focal_distance(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_light_source(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_matrix_write(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_translate_write(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_data_mem_push(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_geo_test(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_geo_end(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_dummy(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_log_data(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_lod(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_code_upload(std::uint32_t opcode, const std::uint32_t* input);
    const std::uint32_t* cmd_code_jump(std::uint32_t opcode, const std::uint32_t* input);

    void parse_np_ns(const std::uint32_t* input, std::uint32_t count);
    void parse_np_s(const std::uint32_t* input, std::uint32_t count);
    void parse_nn_ns(const std::uint32_t* input, std::uint32_t count);
    void parse_nn_s(const std::uint32_t* input, std::uint32_t count);

    // ---- Hardware rasteriser --------------------------------------------
    void push(std::uint32_t input);
    template <unsigned NumVerts>
    void process_polygon(std::uint32_t attr);
    void render_frame_start();
    void render_polygons();
    void render_one(Polygon& poly);
    void project(Polygon& poly);

    // Scanline renderers (model2rd.ipp). `Translucent` selects the alpha path.
    template <bool Translucent>
    void draw_scanline_solid(int scanline, const Extent& extent, const Polygon& poly);
    template <bool Translucent>
    void draw_scanline_tex(int scanline, const Extent& extent, const Polygon& poly);
    // Dispatch one span to the right renderer (renderer = (texheader[0]>>13)&3).
    void draw_span(int scanline, const Extent& extent, const Polygon& poly, int renderer);

    // ---- Polygon fill (poly.h) ------------------------------------------
    template <unsigned NumVerts>
    void fill_polygon(const Vertex* v, const Polygon& poly, int renderer);
    void fill_triangle(const Vertex& a, const Vertex& b, const Vertex& c,
                       const Polygon& poly, int renderer);

    // ---- Helpers ---------------------------------------------------------
    static std::int32_t round_coord(float value);
    static std::uint16_t float_to_zval(float value, std::int32_t z_adjust);
    static std::int32_t clip_polygon(Vertex* v, std::int32_t num_vertices, Vertex* vout,
                                     Plane clip_plane);
    void push_geo_data(std::uint32_t data);
    // Words left in buffer RAM from `p` to the end (0 if `p` is out of range).
    // A desynced display list can otherwise drive a count-driven reader past the
    // buffer; the parse loop bound alone does not cover the readers themselves.
    std::size_t geo_words_left(const std::uint32_t* p) const;
    bool check_culling(std::uint32_t attr, float min_z, float max_z) const;
    std::uint16_t get_texel(std::uint32_t base_x, std::uint32_t base_y, int x, int y,
                            const std::uint32_t* sheet) const;
    // Selects the texture source for a command-buffer selector and reports its
    // word count. `tex_read` returns 0 past the end, mirroring the reference's
    // guarded get_texel: a desynced display list must not read past the sheet.
    const std::uint16_t* tex_source(std::uint32_t sel, std::size_t& words) const;
    std::uint16_t tex_read(const std::uint16_t* base, std::size_t words, std::size_t index) const;
    template <bool Translucent>
    std::uint32_t fetch_bilinear_texel(const Polygon& poly, std::int32_t miplevel,
                                       std::int32_t u, std::int32_t v) const;
    // Reads the 16-bit palette entry that colour-translates `colorbase`.
    std::uint32_t pal_lookup(std::uint32_t colorbase, std::uint32_t mask) const;

    // Geometry-engine state (MAME geo_state).
    std::uint32_t m_mode = 0;
    std::uint32_t m_mesh_mode = 0;
    float m_matrix[12] = {0};
    Vertex m_focus;
    Vertex m_light;
    float m_lod = 0;
    float m_coef_table[32] = {0};
    TextureParam m_texture_parameters[32];
    const std::uint32_t* m_polygon_rom = nullptr;
    std::uint32_t m_polygon_rom_mask = 0;
    std::vector<std::uint32_t> m_polygon_ram0;  // 0x8000 words
    std::vector<std::uint32_t> m_polygon_ram1;  // 0x8000 words

    // Rasteriser state (MAME raster_state).
    const std::uint16_t* m_texture_rom = nullptr;
    std::uint32_t m_texture_rom_mask = 0;
    std::int16_t m_viewport[4] = {0, 0, 0, 0};
    std::int16_t m_center[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
    std::uint16_t m_center_sel = 0;
    std::uint32_t m_reverse = 0;
    std::int32_t m_z_adjust = 0;
    float m_polygon_z = 0;
    std::uint8_t m_master_z_clip = 0xff;
    std::uint32_t m_cur_command = 0;
    std::uint32_t m_command_buffer[32] = {0};
    std::uint32_t m_command_index = 0;
    std::vector<Polygon> m_poly_list;
    std::uint32_t m_poly_list_index = 0;
    std::vector<Polygon*> m_poly_sorted_list;
    std::uint16_t m_min_z = 0xffff;
    std::uint16_t m_max_z = 0;
    std::vector<std::uint16_t> m_texture_ram;  // 0x10000 words
    std::vector<std::uint8_t> m_log_ram;       // 0x8000 bytes
    std::uint8_t m_cur_window = 0;
    Plane m_clip_plane[4][4];
    bool m_render_done = false;

    // Buffer RAM (shared with the bus, 0x8000 words) and GEO FIFO pointers.
    std::uint32_t* m_bufferram = nullptr;
    std::size_t m_bufferram_words = 0;
    std::uint32_t m_geo_write_addr = 0;
    std::uint32_t m_geo_read_addr = 0;
    std::uint32_t m_geoctl = 0;
    std::uint32_t m_geocnt = 0;

    // Shared video RAM.
    std::uint32_t* m_textureram0 = nullptr;
    std::uint32_t* m_textureram1 = nullptr;
    std::size_t m_texture_words = 0;
    const std::uint8_t* m_lumaram = nullptr;
    const std::uint8_t* m_palram = nullptr;
    const std::uint8_t* m_colorxlat = nullptr;

    // Output surface and the per-pixel "filled" mask (MAME destmap/fillmap).
    std::vector<std::uint32_t> m_dest;
    std::vector<std::uint8_t> m_fill;

    std::int16_t m_xoffs = 90;
    std::int16_t m_yoffs = -8;
    std::int16_t m_crtc_xoffset = 0;
    std::int16_t m_crtc_yoffset = 0;

    // Per-polygon viewport clip rectangle (set in render_one()).
    int m_clip_l = 0;
    int m_clip_t = 0;
    int m_clip_r = kWidth - 1;
    int m_clip_b = kHeight - 1;

    std::uint64_t m_polygons_rendered = 0;
    std::uint64_t m_parse_count = 0;
};

}  // namespace sage

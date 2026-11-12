// Sega Model 2B memory map and peripheral integration.
//
// Ported from MAME's src/mame/sega/model2.cpp (model2_base_mem,
// model2b_crx_mem, model2_0229_mem), src/mame/sega/315_5649.cpp and
// src/mame/sega/315-5838_317-0229_comp.cpp (BSD-3-Clause).
//
// Register windows use the same decoding MAME does: the handler offset is the
// word index `(addr - base) >> 2` and `umask32(0x00ff00ff)` connects only byte
// lanes 0 and 2, so a 32-bit access at register N covers registers N and N+1.

#include "sage/bus.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string_view>

namespace sage {
namespace {

constexpr std::uint32_t kRomBase = 0x00000000;
constexpr std::uint32_t kRam0020Base = 0x00200000;
constexpr std::uint32_t kWorkRamBase = 0x00500000;
constexpr std::uint32_t kGeoBase = 0x00800000;
constexpr std::uint32_t kGeoPrgBase = 0x00804000;
constexpr std::uint32_t kCoproFunctionBase = 0x00880000;
constexpr std::uint32_t kCoproFifoBase = 0x00884000;
constexpr std::uint32_t kCoproIopBase = 0x008c0000;
constexpr std::uint32_t kBufferRamBase = 0x00900000;
constexpr std::uint32_t kCoproCtlBase = 0x00980000;
constexpr std::uint32_t kUartBase = 0x009c0000;
// Model 2A moves the i8251 UART to 0x01c80000 and narrows it to umask16(0x00ff),
// so the two registers are a half-word apart instead of a dword.
constexpr std::uint32_t kUartBase2a = 0x01c80000;
constexpr std::uint32_t kCpuCtlBase = 0x00e00000;
constexpr std::uint32_t kIrqBase = 0x00e80000;
constexpr std::uint32_t kTimersBase = 0x00f00000;
constexpr std::uint32_t kRenderModeBase = 0x10000000;
constexpr std::uint32_t kPolygonCountBase = 0x10400000;
constexpr std::uint32_t kTexture0Base = 0x11000000;
constexpr std::uint32_t kTexture1Base = 0x11200000;
constexpr std::uint32_t kLumaBase = 0x11400000;
// Model 2A moves the video apertures: texture RAM 0/1 sit at 0x12000000 and
// 0x12400000 (each a 0x200000 region mirrored, so the window spans 0x400000) and
// the polygon luma RAM at 0x12800000 (0x20000, umask32(0x000000ff): one byte per
// 32-bit word, against the Model 2B's one byte per 16-bit half-word).
constexpr std::uint32_t kTexture0Base2a = 0x12000000;
constexpr std::uint32_t kTexture1Base2a = 0x12400000;
constexpr std::uint32_t kLumaBase2a = 0x12800000;
constexpr std::size_t kTexture2aSpan = 0x00400000;
constexpr std::size_t kLumaSpan2a = 0x00020000;
constexpr std::uint32_t kIoBase = 0x01c00000;
constexpr std::uint32_t kBackupBase = 0x01d00000;
constexpr std::uint32_t kProtBase = 0x01d80000;
constexpr std::uint32_t kMainDataBase = 0x02000000;
constexpr std::uint32_t kMainDataBase2 = 0x06000000;

constexpr std::size_t kRam0020Size = 0x00040000;
constexpr std::size_t kWorkRamSize = 0x00100000;
constexpr std::size_t kBufferRamSize = 0x00020000;  // mirrored across 0x80000
constexpr std::size_t kBackupSize = 0x00004000;
constexpr std::size_t kCpuCtlSize = 0x00000038;
constexpr std::size_t kProtSize = 0x00008000;
constexpr std::size_t kTextureSize = 0x00100000;  // mirrored across 0x100000
// Luma RAM: the 0x11400000-0x1140ffff aperture is mapped umask16(0x00ff), so one
// byte is stored per 16-bit half-word and the array index is the byte offset >> 1.
// The renderer indexes it as `lumaram[lumabase + (t >> 1)]`, so the packed array
// is 0x8000 bytes (MAME: `make_unique_clear<u8[]>(0x8000)`).
constexpr std::size_t kLumaSpan = 0x00010000;

// The i960 on the Model 2 CPU board runs from a 50 MHz crystal divided by two.
constexpr std::uint32_t kCpuClock = 25'000'000;
// The host serial link: 8-N-1 at 31250 baud, ten bits per byte including the
// start and stop bits.
constexpr std::uint32_t kUartBitRate = 31'250;
constexpr std::uint32_t kUartBitsPerByte = 10;
// The host sound interrupt is bit 10 of the interrupt request/enable registers
// (MAME's `const u32 line = 1 << 10` in sound_ready_w).
constexpr std::uint32_t kSoundIrqLine = 1u << 10;

// Write-only model2b registers that MAME maps as `.nopw()`: the ABSEL register,
// the System 24 H/V sync outputs, the video sync switch and the 3D z-clip port.
bool is_nopw(std::uint32_t addr) {
    return (addr >= 0x01020000 && addr < 0x01080000) ||
           (addr >= 0x01120000 && addr < 0x01180000) ||
           (addr >= 0x0181c000 && addr < 0x0181c004);
}

bool in_range(std::uint32_t addr, std::size_t len, std::uint32_t base, std::size_t size) {
    return addr >= base && static_cast<std::uint64_t>(addr - base) + len <= size;
}

// `span` is the mirror period; the mapped store is `size` bytes repeated across it.
bool in_mirror(std::uint32_t addr, std::size_t len, std::uint32_t base, std::size_t size,
               std::size_t span) {
    if (addr < base || addr - base >= span)
        return false;
    return (addr - base) % span + len <= size;
}

std::uint8_t byte_lane(std::uint32_t v, unsigned lane) {
    return static_cast<std::uint8_t>(v >> (lane * 8));
}

// A ROM region as a byte span, empty when the region is absent.
std::span<const std::uint8_t> region_span(const RomLoader& roms, std::string_view name) {
    if (const Region* r = roms.region(name))
        return r->data;
    return {};
}

// A ROM region reinterpreted as 32-bit words. The TGP addresses its ROMs a word
// at a time, and MAME's ROM_REGION32_LE already stores the region little-endian
// with the ROM_LOAD32_WORD entries interleaved, so a straight reinterpretation
// is the same view MAME's as_u32() takes.
std::vector<std::uint32_t> region_words(const RomLoader& roms, std::string_view name) {
    std::vector<std::uint32_t> words;
    if (const Region* r = roms.region(name)) {
        const std::size_t count = r->data.size() / 4;
        words.resize(count);
        for (std::size_t i = 0; i < count; ++i) {
            const std::size_t o = i * 4;
            words[i] = std::uint32_t(r->data[o]) | (std::uint32_t(r->data[o + 1]) << 8) |
                       (std::uint32_t(r->data[o + 2]) << 16) |
                       (std::uint32_t(r->data[o + 3]) << 24);
        }
    }
    return words;
}

}  // namespace

void Bus::attach(const RomLoader& roms, std::string_view game) {
    m_roms = &roms;

    if (const Region* cpu = roms.region("maincpu")) {
        m_rom = cpu->data.data();
        m_rom_size = cpu->data.size();
    }

    m_workram.assign(kWorkRamSize, 0);
    m_ram_0020.assign(kRam0020Size, 0);
    m_backup.assign(kBackupSize, 0xff);  // NVRAM DEFAULT_ALL_1
    m_bufferram.assign(kBufferRamSize, 0);
    // MAME's model2_state::machine_reset() pre-fills buffer RAM with this
    // pattern; the geometry engine parses it before the game uploads a list.
    for (std::size_t i = 0; i + 3 < m_bufferram.size(); i += 4) {
        m_bufferram[i + 0] = 0x0f;
        m_bufferram[i + 1] = 0x0f;
        m_bufferram[i + 2] = 0x80;
        m_bufferram[i + 3] = 0x07;
    }
    m_cpu_ctl.assign(kCpuCtlSize, 0);
    m_prot_ram.assign(kProtSize, 0);
    m_luma.assign(0x8000, 0);
    m_textureram0.assign(kTextureSize, 0);
    m_textureram1.assign(kTextureSize, 0);

    // 0229 protection reads encrypted words from the shared source RAM.
    m_protection.reset();
    m_protection.set_source([this](std::uint32_t word_index) {
        const std::size_t off = (word_index & 0x3fff) * 2;
        if (off + 1 >= m_prot_ram.size())
            return static_cast<std::uint16_t>(0);
        return static_cast<std::uint16_t>(m_prot_ram[off] | (m_prot_ram[off + 1] << 8));
    });

    m_io.reset();
    m_io.set_in_port([this](int port) -> std::uint8_t {
        switch (port) {
            case 1: return in0_read();
            case 2: return m_inputs.p1;
            case 3: return m_inputs.p2;
            case 6: return m_inputs.dipsw;
            default: return 0xff;
        }
    });
    m_io.set_out_port([this](int port, std::uint8_t data) {
        if (port == 0)
            eeprom_write(data);
    });

    m_eeprom.reset();
    m_video.reset();

    // SHARC coprocessor: reset into the host boot state and wire its external
    // data port to the Model 2B coprocessor map (FIFOs, buffer RAM, copro ROM).
    m_sharc.set_boot_mode(cpu::Sharc::BootMode::Host);
    m_sharc.reset();
    m_sharc.set_external_port(
        [this](std::uint32_t addr, std::uint32_t& data) -> bool {
            if (std::getenv("SAGE_SHARC_EXT") && m_sharc_ext_log++ < 80)
                std::fprintf(stderr, "[sharc-ext] R pc=%06X addr=%08X\n", m_sharc.pc(), addr);
            if (addr >= 0x00400000 && addr < 0x00c00000) {  // copro FIFO in
                const unsigned ser = m_fifo_in.serials.empty() ? 0 : m_fifo_in.serials.front();
                data = m_fifo_in.pop();
                if (std::getenv("SAGE_SHARC_FIFO"))
                    std::fprintf(stderr, "[fifo] R pc=%06X ser=%u -> %08X (in.q=%zu)\n",
                                 m_sharc.pc(), ser, data, m_fifo_in.q.size());
                copro_update_flags();
                return true;
            }
            if (addr >= 0x01400000 && addr < 0x01c00000) {  // copro buffer RAM
                const std::size_t i = (addr - 0x01400000) >> 2;
                std::uint32_t v = 0;
                const std::size_t o = (i & 0x7fff) * 4;
                if (o + 3 < m_bufferram.size())
                    v = std::uint32_t(m_bufferram[o]) |
                        (std::uint32_t(m_bufferram[o + 1]) << 8) |
                        (std::uint32_t(m_bufferram[o + 2]) << 16) |
                        (std::uint32_t(m_bufferram[o + 3]) << 24);
                data = v;
                return true;
            }
            if (addr >= 0x01c00000 && addr < 0x01e00000) {  // copro_data ROM
                if (!m_copro_data.empty()) {
                    const std::size_t i = (addr - 0x01c00000) >> 2;
                    const std::size_t o = (i & 0x1fffff) * 4;
                    if (o + 3 < m_copro_data.size())
                        data = std::uint32_t(m_copro_data[o]) |
                               (std::uint32_t(m_copro_data[o + 1]) << 8) |
                               (std::uint32_t(m_copro_data[o + 2]) << 16) |
                               (std::uint32_t(m_copro_data[o + 3]) << 24);
                    else
                        data = 0;
                    return true;
                }
                return false;
            }
            return false;
        },
        [this](std::uint32_t addr, std::uint32_t data) {
            if (addr >= 0x00c00000 && addr < 0x01400000) {  // copro FIFO out
                m_fifo_out.push(data);
                if (std::getenv("SAGE_SHARC_FIFOOUT"))
                    std::fprintf(stderr, "[fifoout] W pc=%06X v=%08X (out.q=%zu)\n",
                                 m_sharc.pc(), data, m_fifo_out.q.size());
                copro_update_flags();
                return;
            }
            if (addr >= 0x01400000 && addr < 0x01c00000) {  // copro buffer RAM
                const std::size_t o = ((addr - 0x01400000) >> 2 & 0x7fff) * 4;
                if (o + 3 < m_bufferram.size()) {
                    m_bufferram[o] = static_cast<std::uint8_t>(data);
                    m_bufferram[o + 1] = static_cast<std::uint8_t>(data >> 8);
                    m_bufferram[o + 2] = static_cast<std::uint8_t>(data >> 16);
                    m_bufferram[o + 3] = static_cast<std::uint8_t>(data >> 24);
                }
            }
        });

    m_fifo_in.cap = 16;
    m_fifo_out.cap = 16;
    m_fifo_in.clear();
    m_fifo_out.clear();
    m_coproctl = 0;
    m_coprocnt = 0;
    // MAME's model2b_state::machine_reset() asserts HALT on the ADSP and only
    // clears it from copro_ctl1_w(), so the SHARC must not execute until the
    // i960 has uploaded its program.
    m_copro_halted = true;
    m_copro_program_upload = false;
    copro_update_flags();

    if (const Region* cd = roms.region("copro_data"))
        m_copro_data = cd->data;

    // Board selection: the Model 2A sets carry a `copro_tgp_tables` region and
    // use the MB86234 TGP; everything else uses the SHARC. Choosing from the
    // loaded regions keeps the decision with the ROM set rather than the caller.
    m_board = roms.region("copro_tgp_tables") != nullptr ? Board::Model2A : Board::Model2B;

    if (m_board == Board::Model2A) {
        // The coprocessor reads the CPU board's mathematical tables and the
        // game's own data ROM, and writes its results into the display list
        // buffer. The regions are byte-oriented; the TGP wants 32-bit words, so
        // word views are materialised here and outlive the attach().
        m_tgp_tables_words = region_words(roms, "copro_tgp_tables");
        m_tgp_data_words   = region_words(roms, "copro_data");
        m_tgp_other_words  = region_words(roms, "other_data");

        auto* buffer_words = reinterpret_cast<std::uint32_t*>(m_bufferram.data());
        const std::size_t buffer_count = m_bufferram.size() / 4;
        m_tgp.attach(m_tgp_tables_words, m_tgp_data_words, m_tgp_other_words,
                     std::span<std::uint32_t>(buffer_words, buffer_count));
        m_tgp.reset();

        // Host side of the FIFO flow control, the same handshake MAME wires for
        // the SHARC: a read of an empty output FIFO stalls the i960 until the
        // TGP produces a value; a write into a full input FIFO stalls it until
        // the TGP drains one. Only the machine can stall the host CPU, which is
        // why this is here and not inside the coprocessor.
        m_tgp.fifo_out().set_on_empty_retry([this] { tgp_host_stall(); });
        m_tgp.fifo_out().set_on_empty_halt([this] { tgp_host_halt(); });
        m_tgp.fifo_out().set_on_unempty([this] { tgp_host_resume(); });
        m_tgp.fifo_in().set_on_full([this] { tgp_host_halt(); });
        m_tgp.fifo_in().set_on_unfull([this] { tgp_host_resume(); });
        m_tgp_debt    = 0;
        m_tgp_in_sync = false;
    }

    // 3D geometry: share buffer RAM, the texture/polygon ROMs and the video RAM
    // the rasteriser reads (MAME wires these as device tags on the renderer).
    m_geometry.reset();
    m_geometry.set_buffer_ram(reinterpret_cast<std::uint32_t*>(m_bufferram.data()),
                              m_bufferram.size() / 4);
    if (const Region* tex = roms.region("textures"))
        m_geometry.set_texture_rom(tex->data.data(), tex->data.size());
    if (const Region* poly = roms.region("polygons"))
        m_geometry.set_polygon_rom(poly->data.data(), poly->data.size());
    m_geometry.set_texture_ram(reinterpret_cast<std::uint32_t*>(m_textureram0.data()),
                               reinterpret_cast<std::uint32_t*>(m_textureram1.data()));
    m_geometry.set_luma_ram(m_luma.data());
    m_geometry.set_palette_ram(m_video.palram().data(), m_video.colorxlat().data());
    m_video.set_geometry(&m_geometry);

    // Sound board: the 68EC000 + SCSP with the "audiocpu" program ROM and the
    // "samples" PCM ROM. The serial link to the host is an i8251 whose transmit
    // handler feeds the SCSP's MIDI port and whose receiver takes the SCSP's
    // replies. The ready handler raises the host sound interrupt.
    m_sound.attach(region_span(roms, "audiocpu"), region_span(roms, "samples"));
    m_sound.configure_balance(std::string(game));
    m_uart.set_tx_handler([this](std::uint8_t value) { m_sound.midi_in(value); });
    m_sound.set_midi_out_handler([this](std::uint8_t value) { m_uart.write_rxd(value); });
    m_uart.set_ready_handler([this] { sound_ready_w(); });
    m_uart.configure(kCpuClock, kUartBitRate, kUartBitsPerByte);
    m_sound.reset();
    m_uart.reset();
}

std::uint8_t* Bus::map(std::uint32_t addr, std::size_t len, bool& writable) const {
    writable = false;

    if (in_range(addr, len, kRomBase, m_rom_size))
        return const_cast<std::uint8_t*>(m_rom) + addr;

    if (in_range(addr, len, kRam0020Base, m_ram_0020.size())) {
        // MAME maps 0x00200000-0x0021ffff as RAM and 0x00220000-0x0023ffff as
        // maincpu ROM at offset 0x20000.
        if (addr - kRam0020Base < kRam0020Size) {
            writable = true;
            return const_cast<std::uint8_t*>(m_ram_0020.data()) + (addr - kRam0020Base);
        }
        const std::uint32_t off = 0x20000 + (addr - (kRam0020Base + kRam0020Size));
        if (off + len <= m_rom_size)
            return const_cast<std::uint8_t*>(m_rom) + off;
    }
    if (in_range(addr, len, kWorkRamBase, m_workram.size())) {
        writable = true;
        return const_cast<std::uint8_t*>(m_workram.data()) + (addr - kWorkRamBase);
    }
    // The GEO windows (0x00800000-0x00807fff) are decoded by the Geometry
    // module through the register path, not as plain RAM (MAME maps them to
    // geo_r/geo_w and geo_prg_r/geo_prg_w).
    if (in_mirror(addr, len, kBufferRamBase, m_bufferram.size(), 0x80000)) {
        writable = true;
        return const_cast<std::uint8_t*>(m_bufferram.data()) +
               ((addr - kBufferRamBase) % m_bufferram.size());
    }
    if (in_range(addr, len, kCpuCtlBase, m_cpu_ctl.size())) {
        writable = true;
        return const_cast<std::uint8_t*>(m_cpu_ctl.data()) + (addr - kCpuCtlBase);
    }
    if (in_range(addr, len, kProtBase, m_prot_ram.size())) {
        writable = true;
        return const_cast<std::uint8_t*>(m_prot_ram.data()) + (addr - kProtBase);
    }
    if (in_range(addr, len, kBackupBase, m_backup.size())) {
        writable = true;
        return const_cast<std::uint8_t*>(m_backup.data()) + (addr - kBackupBase);
    }
    if (in_mirror(addr, len, kTexture0Base, m_textureram0.size(), 0x100000)) {
        writable = true;
        return const_cast<std::uint8_t*>(m_textureram0.data()) +
               ((addr - kTexture0Base) % m_textureram0.size());
    }
    if (in_mirror(addr, len, kTexture1Base, m_textureram1.size(), 0x100000)) {
        writable = true;
        return const_cast<std::uint8_t*>(m_textureram1.data()) +
               ((addr - kTexture1Base) % m_textureram1.size());
    }
    // Model 2A texture RAM. MAME maps a 0x200000 region with `.mirror(0x200000)`,
    // so tex0 repeats across 0x12000000-0x123fffff and tex1 across
    // 0x12400000-0x127fffff, each window indexing the 0x100000 array modulo its
    // size.
    if (is_model2a()) {
        auto tex = [&](std::uint32_t base, const std::vector<std::uint8_t>& ram) -> std::uint8_t* {
            if (addr < base || addr - base >= kTexture2aSpan)
                return nullptr;
            const std::size_t off = (addr - base) % ram.size();
            if (off + len > ram.size())
                return nullptr;
            return const_cast<std::uint8_t*>(ram.data()) + off;
        };
        if (std::uint8_t* p = tex(kTexture0Base2a, m_textureram0)) {
            writable = true;
            return p;
        }
        if (std::uint8_t* p = tex(kTexture1Base2a, m_textureram1)) {
            writable = true;
            return p;
        }
    }

    // Luma RAM is deliberately not mapped here: its 16-bit aperture packs one
    // byte per half-word (umask16(0x00ff)), so a plain byte window would put the
    // game's word writes at every other offset and the renderer would read
    // zeros. It is decoded explicitly in the read/write entry points instead.

    if (std::uint8_t* p = m_video.map(addr, len)) {
        writable = true;  // all video apertures are RAM
        return p;
    }

    if (m_roms) {
        if (const Region* md = m_roms->region("main_data")) {
            if (in_range(addr, len, kMainDataBase, md->data.size()))
                return const_cast<std::uint8_t*>(md->data.data()) + (addr - kMainDataBase);
            // MAME maps 0x06000000 to main_data[0x1000000] (the "extra data"
            // ROMs), not to the start of the region.
            if (addr >= kMainDataBase2 && addr - kMainDataBase2 + len <= md->data.size() - 0x1000000u)
                return const_cast<std::uint8_t*>(md->data.data()) + 0x1000000u +
                       (addr - kMainDataBase2);
        }
    }

    return nullptr;
}

// ---------------------------------------------------------------------------
// Register windows
// ---------------------------------------------------------------------------

bool Bus::read_register(std::uint32_t addr, unsigned lane, std::uint8_t& out) const {
    // 315-5649 I/O controller: registers 0..7, byte lanes 0 and 2 only.
    if (addr >= kIoBase && addr < kIoBase + 0x20) {
        if (lane != 0 && lane != 2) {
            out = 0xff;
            return true;
        }
        out = m_io.read((addr - kIoBase) >> 2);
        return true;
    }

    // GEO function ports (0x00800000-0x00803fff, MAME geo_r/geo_w). Only the two
    // address registers read back: 0x2008 is the write pointer and 0x3008 the
    // read pointer. The i960 polls the write pointer to see how far the TGP has
    // filled the display list, so this has to be wired for the geometry to
    // advance. The byte lanes are little-endian, as the i960 sees them.
    if (addr >= kGeoBase && addr < kGeoBase + 0x4000) {
        out = byte_lane(m_geometry.geo_read(addr - kGeoBase), lane);
        return true;
    }

    // 0x00980000: coprocessor (SHARC or TGP) and GEO control/status.
    if (addr >= kCoproCtlBase && addr < kCoproCtlBase + 0x1000) {
        switch ((addr - kCoproCtlBase) >> 2) {
            case 0:  // copro control 1
                out = byte_lane(is_model2a() ? m_tgp.control_read() : m_coproctl, lane);
                return true;
            case 1: {  // copro FIFO control: bit 0 set while the output FIFO is empty
                bool empty;
                if (is_model2a()) {
                    // A game polls this before reading a result, so give the TGP
                    // a chance to produce one first.
                    if (m_tgp.output_empty())
                        tgp_sync();
                    empty = m_tgp.output_empty();
                } else {
                    empty = m_fifo_out.empty();
                }
                out = byte_lane(empty ? 1u : 0u, lane);
                return true;
            }
            case 2:  // GEO control 1
                out = byte_lane(m_geoctl, lane);
                return true;
            case 3: {  // video control: frame parity used for scanout sync
                const std::uint32_t framenum = render_mode() ? ((m_framenum & 1) << 2)
                                                             : ((m_framenum & 2) << 1);
                out = byte_lane(framenum | (m_videoctl & 3), lane);
                return true;
            }
            case 5:  // copro status: 0xffffffff once a program has been uploaded
                if (is_model2a())
                    out = byte_lane(m_tgp.uploaded_words() == 0 ? 0xffffffffu : 0u, lane);
                else
                    out = byte_lane(m_coprocnt == 0 ? 0xffffffffu : 0u, lane);
                return true;
            default:
                out = 0xff;
                return true;
        }
    }

    // 0x009c0000 i8251 UART (0x01c80000 on Model 2A). MAME maps this range
    // umask32(0x000000ff) on Model 2B and umask16(0x00ff) on Model 2A, so the two
    // registers sit one dword apart on 2B and one half-word apart on 2A. The value
    // returned here is the low byte lane, which is what the mask leaves visible.
    {
        const std::uint32_t uart_base  = is_model2a() ? kUartBase2a : kUartBase;
        const unsigned      uart_shift = is_model2a() ? 1u : 2u;
        if (addr >= uart_base && addr < uart_base + 8) {
            // Only lane 0 is connected; the others read back 0.
            out = (lane == 0) ? m_uart.read((addr - uart_base) >> uart_shift) : 0x00;
            return true;
        }
    }

    // IRQ controller: request / enable.
    if (addr >= kIrqBase && addr < kIrqBase + 8) {
        out = byte_lane((addr - kIrqBase < 4) ? m_intreq : m_intena, lane);
        return true;
    }

    // 25 MHz countdown timers.
    if (addr >= kTimersBase && addr < kTimersBase + 0x10) {
        out = byte_lane(m_timer_val[(addr - kTimersBase) >> 2], lane);
        return true;
    }

    // Renderer mode register.
    if (addr >= kRenderModeBase && addr < kRenderModeBase + 0x200000) {
        out = byte_lane(m_render_mode, lane);
        return true;
    }
    if (addr >= kPolygonCountBase && addr < kPolygonCountBase + 0x200000) {
        out = 0;
        return true;
    }

    return false;
}

bool Bus::write_register(std::uint32_t addr, unsigned lane, std::uint8_t data) {
    if (addr >= kIoBase && addr < kIoBase + 0x20) {
        if (lane == 0 || lane == 2)
            m_io.write((addr - kIoBase) >> 2, data);
        return true;
    }

    // Model 2A maps 0x01c00040-0x01c00043 as a write-only nop (MAME:
    // model2a_crx_mem's `.nopw()`), for every access width.
    if (is_model2a() && addr >= 0x01c00040 && addr < 0x01c00044)
        return true;

    if (addr >= kCoproCtlBase && addr < kCoproCtlBase + 0x1000) {
        const std::uint32_t mask = 0xffu << (lane * 8);
        switch ((addr - kCoproCtlBase) >> 2) {
            case 0: {  // copro control 1: high-bit edge halts / boots the copro
                if (is_model2a()) {
                    const std::uint32_t nv =
                        (m_tgp.control_read() & ~mask) |
                        (static_cast<std::uint32_t>(data) << (lane * 8));
                    tgp_ctl1_write(nv);
                    return true;
                }
                const std::uint32_t nv =
                    (m_coproctl & ~mask) | (static_cast<std::uint32_t>(data) << (lane * 8));
                copro_ctl1_write(nv);
                return true;
            }
            case 2:  // GEO control 1
                m_geoctl = (m_geoctl & ~mask) | (static_cast<std::uint32_t>(data) << (lane * 8));
                m_geometry.set_geoctl(m_geoctl);
                return true;
            case 3:  // video control
                m_videoctl = (m_videoctl & ~mask) | (static_cast<std::uint32_t>(data) << (lane * 8));
                return true;
            default:  // bank control register and others are write-only
                return true;
        }
    }

    if (addr >= kCoproFunctionBase && addr < kCoproFunctionBase + 0x4000)
        return true;  // SHARC function port is write-only
    if (addr >= kCoproFifoBase && addr < kCoproFifoBase + 0x4000)
        return true;
    if (addr >= kCoproIopBase && addr < kCoproIopBase + 0x1000)
        return true;  // SHARC IOP port is write-only

    {
        const std::uint32_t uart_base  = is_model2a() ? kUartBase2a : kUartBase;
        const unsigned      uart_shift = is_model2a() ? 1u : 2u;
        if (addr >= uart_base && addr < uart_base + 8) {
            // Only lane 0 is connected.
            if (lane == 0)
                m_uart.write((addr - uart_base) >> uart_shift, data);
            return true;
        }
    }

    if (addr >= kIrqBase && addr < kIrqBase + 8) {
        const std::uint32_t mask = 0xffu << (lane * 8);
        if (addr - kIrqBase < 4) {
            // Write-one-to-clear acknowledge: the bits written as 1 are cleared
            // in the request register (MAME `m_intreq &= data`).
            const std::uint32_t ack = (static_cast<std::uint32_t>(data) << (lane * 8)) |
                                      ~mask;
            m_intreq &= ack;
            irq_update();
        } else {
            // Enable updates are delayed by two cycles; vcop2 relies on this.
            m_intena_pending = (m_intena_pending & ~mask) |
                               (static_cast<std::uint32_t>(data) << (lane * 8));
            m_intena_delay = 2;
            if (m_trace_irq)
                std::fprintf(stderr, "[irq] enable write addr=%08X data=%02X pending=%08X\n", addr,
                             data, m_intena_pending);
        }
        return true;
    }

    if (addr >= kTimersBase && addr < kTimersBase + 0x10) {
        const unsigned t = (addr - kTimersBase) >> 2;
        const std::uint32_t mask = 0xffu << (lane * 8);
        m_timer_val[t] = (m_timer_val[t] & ~mask) | (static_cast<std::uint32_t>(data) << (lane * 8));
        m_timer_orig[t] = m_timer_val[t];
        m_timer_run[t] = true;
        return true;
    }

    // 317-0229 protection source index and table upload port.
    if (addr >= kProtBase + 0x7ff0 && addr < kProtBase + 0x7ff4) {
        m_prot_source = (m_prot_source & ~(0xffu << (lane * 8))) |
                        (static_cast<std::uint32_t>(data) << (lane * 8));
        m_protection.source_w(m_prot_source);
        m_doa_dummy = true;
        return true;
    }
    if (addr >= kProtBase + 0x7ff4 && addr < kProtBase + 0x7ff8) {
        const unsigned offset = (addr - (kProtBase + 0x7ff4)) >> 1;
        m_protection.table_w(offset, static_cast<std::uint16_t>(data) << ((lane & 1) * 8));
        return true;
    }

    if (addr >= kRenderModeBase && addr < kRenderModeBase + 0x200000) {
        const std::uint32_t mask = 0xffu << (lane * 8);
        m_render_mode = (m_render_mode & ~mask) | (static_cast<std::uint32_t>(data) << (lane * 8));
        m_video.set_render_test_mode((m_render_mode & 1) != 0);
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Byte / word / dword access
// ---------------------------------------------------------------------------

void Bus::watch_log(char op, std::uint32_t addr, std::uint32_t data) const {
    if (m_watch_limit == 0 || addr < m_watch_lo || addr > m_watch_hi)
        return;
    if (op == 'R' && !m_watch_reads)
        return;
    if (m_watch_count++ >= m_watch_limit)
        return;
    const std::uint32_t pc = m_cpu ? m_cpu->get_ip() : 0;
    std::fprintf(stderr, "[watch] %c pc=%08X addr=%08X data=%08X\n", op, pc, addr, data);
}

bool Bus::luma_index(std::uint32_t addr, std::size_t& out, std::size_t& entries) const {
    if (is_model2a()) {
        if (addr < kLumaBase2a || addr >= kLumaBase2a + kLumaSpan2a)
            return false;
        out     = (addr - kLumaBase2a) >> 2;  // one byte per 32-bit word
        entries = 1;
        return true;
    }
    if (addr < kLumaBase || addr >= kLumaBase + kLumaSpan)
        return false;
    out     = (addr - kLumaBase) >> 1;  // one byte per 16-bit half-word
    entries = 2;
    return true;
}

std::uint8_t Bus::raw_read8(std::uint32_t addr) const {
    // Luma RAM: the packed index is one byte per address step (a half-word on
    // Model 2B, a word on Model 2A).
    std::size_t luma_idx = 0, luma_entries = 0;
    if (luma_index(addr, luma_idx, luma_entries))
        return luma_idx < m_luma.size() ? m_luma[luma_idx] : 0;

    bool w = false;
    if (const std::uint8_t* p = map(addr, 1, w))
        return *p;

    std::uint8_t out = 0xff;
    if (read_register(addr, addr & 3, out))
        return out;

    ++unmapped_reads;
    if (m_trace) ++m_unmapped_read_hist[addr & 0xfffc0000];
    if (m_trace && m_unmapped_read_log.size() < 40)
        m_unmapped_read_log.push_back({addr, m_cpu ? m_cpu->get_ip() : 0});
    return 0;
}

std::uint8_t Bus::read8(std::uint32_t addr) const {
    if (m_trace) ++m_read_hist[addr & 0xfffc0000];
    const std::uint8_t v = raw_read8(addr);
    watch_log('R', addr, v);
    return v;
}

std::uint16_t Bus::read16(std::uint32_t addr) const {
    watch_log('R', addr, 0);

    // GEO function ports (MAME geo_r): the two address registers read back at
    // their dword offsets. Handled here so a half-word access is decoded once
    // rather than per byte lane.
    if (addr >= kGeoBase && addr < kGeoBase + 0x4000)
        return static_cast<std::uint16_t>(m_geometry.geo_read(addr - kGeoBase) & 0xffff);

    // Luma RAM: one byte per address step, returned in the low lane.
    std::size_t luma_idx = 0, luma_entries = 0;
    if (luma_index(addr, luma_idx, luma_entries))
        return luma_idx < m_luma.size() ? m_luma[luma_idx] : 0;

    // 317-0229 registers overlay the tail of the protection source RAM window,
    // so they must be decoded before the RAM fallback (matches MAME's
    // model2_0229_mem, where the more specific handler wins).
    // Data port: two decompressed bytes, high byte first. The first halfword
    // after a source write is discarded by Dead or Alive.
    if (addr >= kProtBase + 0x7ff8 && addr < kProtBase + 0x7ffc) {
        if (m_doa_dummy) {
            m_doa_dummy = false;
            return 0;
        }
        return m_protection.data_r();
    }
    // Dead or Alive status location: alternates per access.
    if (addr >= 0x01d8400c && addr < 0x01d84010) {
        m_prot_a = !m_prot_a;
        return m_prot_a ? 0xffff : 0xfff0;
    }

    bool w = false;
    if (const std::uint8_t* p = map(addr, 2, w))
        return static_cast<std::uint16_t>(p[0] | (p[1] << 8));

    return static_cast<std::uint16_t>(read8(addr) | (read8(addr + 1) << 8));
}

std::uint32_t Bus::read32(std::uint32_t addr) const {
    watch_log('R', addr, 0);
    if (m_trace) ++m_read_hist[addr & 0xfffc0000];

    // GEO function ports (MAME geo_r): 0x2008 is the write pointer the i960
    // polls to track display-list progress; 0x3008 is the read pointer.
    if (addr >= kGeoBase && addr < kGeoBase + 0x4000)
        return m_geometry.geo_read(addr - kGeoBase);

    // Luma RAM: a 32-bit access covers `entries` packed bytes, one per 16-bit
    // half-word on Model 2B or one per 32-bit word on Model 2A.
    {
        std::size_t luma_idx = 0, luma_entries = 0;
        if (luma_index(addr, luma_idx, luma_entries)) {
            std::uint32_t v = luma_idx < m_luma.size() ? m_luma[luma_idx] : 0;
            if (luma_entries >= 2)
                v |= std::uint32_t(luma_idx + 1 < m_luma.size() ? m_luma[luma_idx + 1] : 0) << 16;
            return v;
        }
    }

    if (addr >= kCoproFifoBase && addr < kCoproFifoBase + 0x4000)
        return is_model2a() ? tgp_fifo_r() : copro_fifo_r();

    if (addr >= kProtBase + 0x7ff8 && addr < kProtBase + 0x7ffc) {
        const std::uint32_t lo = read16(addr);
        const std::uint32_t hi = read16(addr + 2);
        return lo | (hi << 16);
    }
    if (addr >= 0x01d8400c && addr < 0x01d84010) {
        m_prot_a = !m_prot_a;
        return m_prot_a ? 0xffff : 0xfff0;
    }

    bool w = false;
    if (const std::uint8_t* p = map(addr, 4, w))
        return static_cast<std::uint32_t>(p[0] | (p[1] << 8) | (p[2] << 16) |
                                          (static_cast<std::uint32_t>(p[3]) << 24));

    std::uint32_t v = 0;
    for (unsigned i = 0; i < 4; ++i)
        v |= static_cast<std::uint32_t>(raw_read8(addr + i)) << (i * 8);
    return v;
}

void Bus::write8(std::uint32_t addr, std::uint8_t val) {
    if (m_trace) ++m_write_hist[addr & 0xfffc0000];
    watch_log('W', addr, val);
    std::size_t luma_idx = 0, luma_entries = 0;
    if (luma_index(addr, luma_idx, luma_entries)) {
        if (luma_idx < m_luma.size())
            m_luma[luma_idx] = val;
        return;
    }
    bool w = false;
    if (std::uint8_t* p = map(addr, 1, w)) {
        if (w) {
            *p = val;
            if (addr >= 0x01800000 && addr < 0x01804000)
                m_video.mark_palette_dirty();
            return;
        }
        ++rom_writes;
        ++m_rom_write_pages[addr & 0xfffc0000];
        return;
    }
    if (write_register(addr, addr & 3, val))
        return;

    // System 24 H/V sync outputs (0x01040000/0x01060000). MAME registers these
    // as 16-bit handlers on the tile device, so the i960 writes them as words;
    // decode them here (before is_nopw()) or the CRTC viewport offset never
    // reaches the 3D renderer and every polygon projects off-screen.
    if (addr >= 0x01040000 && addr < 0x01060000) {
        m_video.write_hsync(addr & 3, val);
        return;
    }
    if (addr >= 0x01060000 && addr < 0x01080000) {
        m_video.write_vsync(addr & 3, val);
        return;
    }

    if (is_nopw(addr))
        return;
    ++unmapped_writes;
    ++m_unmapped_write_hist[addr & 0xfffc0000];
}

void Bus::write16(std::uint32_t addr, std::uint16_t val) {
    watch_log('W', addr, val);

    // Luma RAM: one byte per address step.
    std::size_t luma_idx = 0, luma_entries = 0;
    if (luma_index(addr, luma_idx, luma_entries)) {
        if (luma_idx < m_luma.size())
            m_luma[luma_idx] = static_cast<std::uint8_t>(val);
        return;
    }

    // Protection registers overlay the source RAM window; decode them first.
    if (addr >= kProtBase + 0x7ff4 && addr < kProtBase + 0x7ff8) {
        m_protection.table_w((addr - (kProtBase + 0x7ff4)) >> 1, val);
        return;
    }

    bool w = false;
    if (std::uint8_t* p = map(addr, 2, w)) {
        if (w) {
            p[0] = static_cast<std::uint8_t>(val);
            p[1] = static_cast<std::uint8_t>(val >> 8);
            if (addr >= 0x01800000 && addr < 0x01804000)
                m_video.mark_palette_dirty();
            return;
        }
        ++rom_writes;
        return;
    }

    write8(addr, static_cast<std::uint8_t>(val));
    write8(addr + 1, static_cast<std::uint8_t>(val >> 8));
}

void Bus::write32(std::uint32_t addr, std::uint32_t val) {
    watch_log('W', addr, val);

    // Luma RAM: a 32-bit store covers `entries` packed bytes, one per 16-bit
    // half-word on Model 2B or one per 32-bit word on Model 2A.
    {
        std::size_t luma_idx = 0, luma_entries = 0;
        if (luma_index(addr, luma_idx, luma_entries)) {
            if (luma_idx < m_luma.size())
                m_luma[luma_idx] = static_cast<std::uint8_t>(val);
            if (luma_entries >= 2 && luma_idx + 1 < m_luma.size())
                m_luma[luma_idx + 1] = static_cast<std::uint8_t>(val >> 16);
            return;
        }
    }

    // Protection registers overlay the source RAM window; decode them first.
    if (addr >= kProtBase + 0x7ff0 && addr < kProtBase + 0x7ff4) {
        m_prot_source = val;
        m_protection.source_w(val);
        m_doa_dummy = true;
        return;
    }
    if (addr >= kProtBase + 0x7ff4 && addr < kProtBase + 0x7ff8) {
        m_protection.table_w(0, static_cast<std::uint16_t>(val));
        m_protection.table_w(1, static_cast<std::uint16_t>(val >> 16));
        return;
    }

    bool w = false;
    if (std::uint8_t* p = map(addr, 4, w)) {
        if (w) {
            p[0] = static_cast<std::uint8_t>(val);
            p[1] = static_cast<std::uint8_t>(val >> 8);
            p[2] = static_cast<std::uint8_t>(val >> 16);
            p[3] = static_cast<std::uint8_t>(val >> 24);
            if (addr >= 0x01800000 && addr < 0x01804000)
                m_video.mark_palette_dirty();
            return;
        }
        ++rom_writes;
        return;
    }

    if (addr >= kGeoBase && addr < kGeoBase + 0x4000) {
        m_geometry.geo_write(addr - kGeoBase, val);
        return;
    }
    if (addr >= kGeoPrgBase && addr < kGeoPrgBase + 0x4000) {
        m_geometry.geo_prg_write(val);
        return;
    }

    // System 24 H/V sync outputs. MAME maps these `.nopw()`, but the 3D renderer
    // derives its viewport offset from them (horizontal_sync_w/vertical_sync_w).
    if (addr >= 0x01040000 && addr < 0x01060000) {
        m_video.write_hsync(addr & 3, val);
        return;
    }
    if (addr >= 0x01060000 && addr < 0x01080000) {
        m_video.write_vsync(addr & 3, val);
        return;
    }

    if (is_nopw(addr))
        return;

    if (addr >= kCoproFunctionBase && addr < kCoproFunctionBase + 0x4000) {
        if (is_model2a())
            tgp_function_port_w(addr - kCoproFunctionBase, val);
        else
            copro_function_port_w(addr - kCoproFunctionBase, val);
        return;
    }
    if (addr >= kCoproFifoBase && addr < kCoproFifoBase + 0x4000) {
        if (is_model2a())
            tgp_fifo_w(val);
        else
            copro_fifo_w(val);
        return;
    }
    if (addr >= kCoproIopBase && addr < kCoproIopBase + 0x1000) {
        // The SHARC's IOP port. The TGP has no equivalent window; MAME leaves it
        // unmapped on the Model 2A map, so a write is ignored.
        if (is_model2a())
            return;
        try {
            // MAME registers this window on the i960's 32-bit space, so the
            // device handler receives a word index (byte address >> 2).
            m_sharc.external_iop_write((addr - kCoproIopBase) >> 2, val);
        } catch (const std::exception& e) {
            std::fprintf(stderr, "IOP write fault addr=%08X off=%03X val=%08X pc=%08X: %s\n",
                         addr, (addr - kCoproIopBase) >> 2, val,
                         m_cpu ? m_cpu->get_ip() : 0, e.what());
            m_copro_halted = true;
        }
        return;
    }

    if (addr >= kProtBase + 0x7ff0 && addr < kProtBase + 0x7ff4) {
        m_prot_source = val;
        m_protection.source_w(val);
        m_doa_dummy = true;
        return;
    }
    if (addr >= kProtBase + 0x7ff4 && addr < kProtBase + 0x7ff8) {
        m_protection.table_w(0, static_cast<std::uint16_t>(val));
        m_protection.table_w(1, static_cast<std::uint16_t>(val >> 16));
        return;
    }

    for (unsigned i = 0; i < 4; ++i)
        write8(addr + i, static_cast<std::uint8_t>(val >> (i * 8)));
}

bool Bus::burst(std::uint32_t addr) const {
    // MAME flags these i960 regions i960_cpu_device::BURST (model2_base_mem,
    // model2_5881_mem, model2_tgp_mem). The i960 only advances the address of a
    // multi-word load/store when the target reports BURST, so ROM and RAM must
    // both qualify; otherwise ldq/stq fold onto a single word.
    if (addr >= kGeoPrgBase && addr < kGeoPrgBase + 0x4000) return true;
    if (addr >= kCoproFunctionBase && addr < kCoproFunctionBase + 0x4000) return true;
    // Luma RAM is flagged BURST in MAME; it is decoded outside map(), so it must
    // be listed here explicitly (both boards' apertures).
    {
        std::size_t luma_idx = 0, luma_entries = 0;
        if (luma_index(addr, luma_idx, luma_entries)) return true;
    }
    // The CPU wait-state block is the one plain-RAM aperture MAME leaves non-BURST.
    if (addr >= kCpuCtlBase && addr < kCpuCtlBase + kCpuCtlSize) return false;
    bool w = false;
    return map(addr, 4, w) != nullptr;
}

std::string Bus::describe(std::uint32_t addr) const {
    bool w = false;
    if (map(addr, 1, w))
        return w ? "RAM" : "ROM";
    std::uint8_t dummy = 0;
    return read_register(addr, addr & 3, dummy) ? "register" : "unmapped";
}

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------

void Bus::eeprom_write(std::uint8_t data) {
    m_ctrlmode = data & 1;
    m_eeprom.di_write((data >> 5) & 1);
    m_eeprom.clk_write((data >> 7) & 1);
    m_eeprom.cs_write((data >> 6) & 1);
}

std::uint8_t Bus::in0_read() const {
    std::uint8_t data = 0xff;
    if (m_inputs.coin1) data &= ~0x01;
    if (m_inputs.coin2) data &= ~0x02;
    if (m_inputs.test) data &= ~0x04;
    if (m_inputs.service) data &= ~0x08;
    if (m_inputs.start1) data &= ~0x10;
    if (m_inputs.start2) data &= ~0x20;

    if (m_ctrlmode)
        return static_cast<std::uint8_t>(0xc0 | (m_eeprom.do_read() << 5) | 0x10 | (data & 0x0f));
    return data;
}

void Bus::irq_update() {
    if (!m_cpu)
        return;
    m_cpu->set_input(0, (m_intreq & 0x001) ? 1 : 0);
    m_cpu->set_input(1, (m_intreq & 0x002) ? 1 : 0);
    m_cpu->set_input(2, (m_intreq & 0x3fc) ? 1 : 0);
    m_cpu->set_input(3, (m_intreq & 0xc00) ? 1 : 0);
}

void Bus::sound_ready_w() {
    // The host's sound interrupt is asserted while the i8251 has a byte ready to
    // be read or room to accept one; MAME's sound_ready_w does the same.
    if ((m_uart.txrdy() || m_uart.rxrdy()) && (m_intena & kSoundIrqLine) != 0) {
        m_intreq |= kSoundIrqLine;
        irq_update();
    }
}

void Bus::tick_sound(int cycles) {
    // The i8251 shifts a byte out over `cycles` of the host clock, so it is
    // advanced with the same cycle count the i960 was granted.
    m_uart.run(static_cast<std::uint32_t>(cycles > 0 ? cycles : 0));
    m_sound.run(static_cast<std::uint32_t>(cycles > 0 ? cycles : 0));
    // The board's own activity can make a byte ready, which is the other half of
    // the interrupt's sources.
    sound_ready_w();
}

void Bus::tick(int cycles) {
    if (m_intena_delay > 0) {
        m_intena_delay -= cycles;
        if (m_intena_delay <= 0) {
            m_intena_delay = 0;
            m_intena = m_intena_pending;
        }
    }

    for (int i = 0; i < 4; ++i) {
        if (!m_timer_run[i])
            continue;
        const std::uint32_t c = static_cast<std::uint32_t>(cycles);
        if (m_timer_val[i] > c) {
            m_timer_val[i] -= c;
            continue;
        }
        m_timer_val[i] = 0xfffff;
        m_timer_run[i] = false;
        ++m_timer_irq_count;
        const std::uint32_t line = 1u << (i + 2);
        if (m_intena & line) {
            m_intreq |= line;
            irq_update();
        }
    }
}

void Bus::trigger_vblank() {
    m_framenum++;
    ++m_vblank_count;

    // MAME's screen_vblank(): "if 60 Hz mode or frame number is even, trigger
    // geometrizer to start new frame". Video control bit 0 selects 30 Hz
    // geometry, which runs on even frames only; the renderer then presents the
    // previous result again.
    if ((m_videoctl & 1) == 0 || (m_framenum & 1) == 0)
        m_geometry.parse();

    const std::uint32_t line = 1u << 0;
    if (m_intena & line) {
        m_intreq |= line;
        irq_update();
    }
}

// ---- SHARC coprocessor -----------------------------------------------------

void Bus::copro_fifo_sync(bool need_out) const {
    // Mirror MAME's generic_fifo cross-CPU synchronization: a pop on an empty
    // output FIFO stalls the i960 until the SHARC has produced a value, and a
    // push into a full input FIFO halts it until the SHARC has drained one.
    // Without this the i960 races arbitrarily far ahead and consumes results
    // that were never written.
    if (m_copro_halted || !m_copro_fault.empty() || m_copro_in_sync) return;
    m_copro_in_sync = true;
    std::uint32_t last_pc = m_sharc.pc();
    for (int i = 0; i < 4000; ++i) {
        if (need_out ? !m_fifo_out.empty() : !m_fifo_in.full()) break;
        if (m_copro_halted || !m_copro_fault.empty()) break;
        try {
            m_sharc.run_cycles(64);
        } catch (const std::exception& e) {
            m_copro_fault = e.what();
            m_copro_halted = true;
            break;
        }
        // The SHARC is parked with an empty input FIFO: no amount of further
        // execution can produce the output value, so stop rather than spin.
        if (need_out && m_fifo_in.empty() && m_sharc.pc() == last_pc) break;
        last_pc = m_sharc.pc();
    }
    m_copro_in_sync = false;
}

void Bus::copro_update_flags() const {
    // FLAG0 mirrors the input FIFO empty state, FLAG1 the output FIFO full state.
    m_sharc.set_flag_input(0, m_fifo_in.empty());
    m_sharc.set_flag_input(1, m_fifo_out.full());
    if (std::getenv("SAGE_FIFODBG"))
        std::fprintf(stderr, "[fifo] in=%zu out=%zu flag0=%d flag1=%d shpc=%06X\n",
                     m_fifo_in.q.size(), m_fifo_out.q.size(), m_sharc.flag_input(0),
                     m_sharc.flag_input(1), m_sharc.pc());
}

void Bus::copro_halt() {
    m_copro_halted = true;
    m_copro_program_upload = true;
    m_coprocnt = 0;
}

void Bus::copro_boot() {
    m_copro_halted = false;
    m_copro_program_upload = false;
    copro_update_flags();
    if (std::getenv("SAGE_COPRO_TRACE")) {
        std::fprintf(stderr, "[copro] boot: %u halfwords uploaded, PC=0x%06X\n",
                     m_coprocnt, m_sharc.pc());
        for (unsigned i = 0; i < 12; ++i)
            std::fprintf(stderr, "[copro]   PM[%06X] = %012llX\n", 0x20004u + i,
                         (unsigned long long)m_sharc.debug_pm48(0x20004u + i));
    }
}

void Bus::copro_ctl1_write(std::uint32_t value) {
    if (std::getenv("SAGE_COPRO_TRACE"))
        std::fprintf(stderr, "[copro] ctl1 write %08X (old %08X) pc=%08X\n", value,
                     m_coproctl, m_cpu ? m_cpu->get_ip() : 0);
    // The high bit toggling selects halt (set) / boot (clear), matching
    // model2_state::copro_ctl1_w.
    if ((value ^ m_coproctl) == 0x80000000u) {
        if (value & 0x80000000u)
            copro_halt();
        else
            copro_boot();
    }
    m_coproctl = value;
}

void Bus::copro_fifo_w(std::uint32_t data) {
    if (std::getenv("SAGE_COPRO_TRACE"))
        std::fprintf(stderr, "[copro] fifo_w %08X (ctl=%08X cnt=%u) pc=%08X\n", data, m_coproctl,
                     m_coprocnt, m_cpu ? m_cpu->get_ip() : 0);
    if (m_coproctl & 0x80000000u) {
        // Program upload phase: each 32-bit write carries a 16-bit halfword that
        // external_dma_write() packs into 48-bit PM words.
        m_sharc.external_dma_write(m_coprocnt, data & 0xffff);
        ++m_coprocnt;
    } else {
        if (m_fifo_in.full()) copro_fifo_sync(false);
        const unsigned ser = m_fifo_in.push(data);
        if (std::getenv("SAGE_SHARC_PUSH"))
            std::fprintf(stderr, "[push-fifo] ser=%u pc=%08X v=%08X (q=%zu)\n",
                         ser, m_cpu ? m_cpu->get_ip() : 0, data, m_fifo_in.q.size());
        copro_update_flags();
    }
}

std::uint32_t Bus::copro_fifo_r() const {
    if (m_fifo_out.empty()) copro_fifo_sync(true);
    const bool was_empty = m_fifo_out.empty();
    const std::uint32_t v = m_fifo_out.pop();
    if (std::getenv("SAGE_SHARC_FIFOOUT"))
        std::fprintf(stderr, "[fifoout] R pc=%08X -> %08X (was_empty=%d out.q=%zu)\n",
                     m_cpu ? m_cpu->get_ip() : 0, v, was_empty ? 1 : 0, m_fifo_out.q.size());
    if (std::getenv("SAGE_COPRO_TRACE"))
        std::fprintf(stderr, "[copro] fifo_r %08X pc=%08X\n", v, m_cpu ? m_cpu->get_ip() : 0);
    copro_update_flags();
    return v;
}

void Bus::copro_function_port_w(std::uint32_t byte_offset, std::uint32_t data) {
    // The i960 handler sees a word index; MAME then takes bits 2..9 of that as
    // the 8-bit function selector.
    const std::uint32_t a = ((byte_offset >> 2) >> 2) & 0xff;
    const std::uint32_t d = (data & 0x800fffff) | (a << 23);
    if (m_fifo_in.full()) copro_fifo_sync(false);
    const unsigned ser = m_fifo_in.push(d);
    if (std::getenv("SAGE_SHARC_PUSH"))
        std::fprintf(stderr, "[push-func] ser=%u pc=%08X a=%02X data=%08X d=%08X (q=%zu)\n",
                     ser, m_cpu ? m_cpu->get_ip() : 0, a, data, d, m_fifo_in.q.size());
    copro_update_flags();
}

void Bus::tick_copro(int cycles) {
    if (is_model2a()) {
        // TGP 50 MHz at three clocks per instruction vs i960 25 MHz: two TGP
        // instructions per three host cycles. The remainder is kept so the ratio
        // is exact across slices.
        m_tgp_debt += static_cast<std::uint32_t>(cycles) * 2;
        const std::int32_t tgp_cycles = static_cast<std::int32_t>(m_tgp_debt / 3);
        m_tgp_debt %= 3;
        if (tgp_cycles > 0)
            static_cast<void>(m_tgp.run(tgp_cycles));
        return;
    }

    if (m_copro_halted || !m_copro_fault.empty()) return;
    // SHARC 32 MHz vs i960 25 MHz: accumulate the fractional remainder so the
    // two stay in step over long runs.
    m_copro_phase += std::uint64_t(cycles) * 32;
    const std::uint64_t sharc_cycles = m_copro_phase / 25;
    m_copro_phase -= sharc_cycles * 25;
    if (sharc_cycles) {
        // A coprocessor fault must not take down the i960: record it, halt the
        // SHARC and let the main CPU keep running (MAME would fatal-error here).
        try {
            m_sharc.run_cycles(static_cast<int>(sharc_cycles));
        } catch (const std::exception& e) {
            m_copro_fault = e.what();
            m_copro_halted = true;
        }
    }
}

// ---- TGP coprocessor (Model 2A) --------------------------------------------

void Bus::tgp_host_stall() const {
    // The TGP read an empty output FIFO. MAME stalls the i960 so it re-executes
    // the load; SAGE's i960 has no cheap re-execute for a plain load, so the
    // sync path below already ran the TGP forward before the read. Nothing more
    // is needed here beyond keeping the hook (the halt variant is the hard stop).
}

void Bus::tgp_host_halt() const {
    // A full input FIFO (or an empty output one) stops the host until the TGP
    // makes progress. SAGE advances the TGP every tick_copro(), so the halt is
    // recorded for diagnostics only; halting the i960 outright would deadlock
    // the single-threaded loop, which is why the SHARC path uses a bounded sync
    // instead. Kept as a no-op so the wiring matches MAME's.
}

void Bus::tgp_host_resume() const {}

void Bus::tgp_sync() const {
    if (m_tgp_in_sync)
        return;
    m_tgp_in_sync = true;

    // Run in small steps and stop as soon as a result appears, so a program that
    // polls for one value does not let the coprocessor race ahead of the host.
    std::int32_t remaining = 4096;
    while (remaining > 0 && m_tgp.output_empty() && !m_tgp.cpu().halted()) {
        const std::int32_t slice = std::min<std::int32_t>(remaining, 32);
        const std::int32_t used  = m_tgp.run(slice);
        if (used <= 0)
            break;
        remaining -= used;
        // Borrow from the coprocessor's future share so this time is not spent
        // twice. The debt is in thirds of a host cycle, times two.
        const std::uint32_t owed = static_cast<std::uint32_t>(used) * 3;
        m_tgp_debt = m_tgp_debt > owed ? m_tgp_debt - owed : 0;
    }

    m_tgp_in_sync = false;
}

void Bus::tgp_ctl1_write(std::uint32_t value) {
    if (std::getenv("SAGE_TGP_TRACE"))
        std::fprintf(stderr, "[tgp] ctl1 write %08X pc=%08X\n", value,
                     m_cpu ? m_cpu->get_ip() : 0);
    m_tgp.control_write(value);
}

void Bus::tgp_fifo_w(std::uint32_t data) {
    if (std::getenv("SAGE_TGP_TRACE"))
        std::fprintf(stderr, "[tgp] fifo_w %08X (ctl=%08X cnt=%u) pc=%08X\n", data,
                     m_tgp.control_read(), m_tgp.uploaded_words(),
                     m_cpu ? m_cpu->get_ip() : 0);
    m_tgp.host_fifo_write(data);
}

std::uint32_t Bus::tgp_fifo_r() const {
    if (m_tgp.output_empty())
        tgp_sync();
    const std::uint32_t v = m_tgp.host_fifo_read();
    if (std::getenv("SAGE_TGP_TRACE"))
        std::fprintf(stderr, "[tgp] fifo_r %08X pc=%08X\n", v, m_cpu ? m_cpu->get_ip() : 0);
    return v;
}

void Bus::tgp_function_port_w(std::uint32_t byte_offset, std::uint32_t data) {
    if (std::getenv("SAGE_TGP_TRACE"))
        std::fprintf(stderr, "[tgp] func port %04X <- %08X pc=%08X\n", byte_offset, data,
                     m_cpu ? m_cpu->get_ip() : 0);
    m_tgp.function_port_write(byte_offset, data);
}

}  // namespace sage

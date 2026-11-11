#include "sage/rom_loader.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <limits>
#include <optional>
#include <span>
#include <unordered_map>

#include <miniz.h>

namespace sage {
namespace {

// ---------------------------------------------------------------------------
// CRC-32 (IEEE 802.3, as used by MAME/ZIP). Bit-reflected, poly 0xEDB88320.
// ---------------------------------------------------------------------------
struct Crc32Table {
    std::array<std::uint32_t, 256> v{};
    Crc32Table() {
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k)
                c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            v[i] = c;
        }
    }
};

std::uint32_t crc32(const std::uint8_t* data, std::size_t len) {
    static const Crc32Table table;
    std::uint32_t c = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; ++i)
        c = table.v[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

std::uint32_t crc32(const std::vector<std::uint8_t>& v) { return crc32(v.data(), v.size()); }

// ---------------------------------------------------------------------------
// ROM definition tables, transcribed from MAME's model2.cpp ROM_START blocks:
//
//   doa / doab  Sega Model 2B, Dead or Alive (Rev C / Rev B)
//   vf2         Sega Model 2A, Virtua Fighter 2 (Version 2.1)
//
// Each entry mirrors one ROM_LOAD macro; a game selects one region table and
// two entry lists (the program ROMs plus everything shared).
// ---------------------------------------------------------------------------

struct RegionDef {
    const char* tag;
    std::uint32_t length;
    std::uint32_t bus_width;  // bits
    bool big_endian;
    std::uint8_t erase_value;
};

enum class EntryKind { Load, Copy };

struct Entry {
    std::uint32_t region;
    EntryKind kind;
    const char* name;   // filename for Load entries
    std::uint32_t offset;
    std::uint32_t length;
    std::uint32_t crc;  // expected CRC32 for Load entries
    std::uint32_t alt_crc;  // known alternate dump CRC32 (0 = none)
    std::uint32_t group;    // ROM_GROUPWORD -> 2
    std::uint32_t skip;     // ROM_SKIP(n) -> n
    bool reverse;           // ROM_REVERSE
    std::uint32_t copy_dst; // destination offset for Copy entries
};

// Region indices. Both games order their region table the same way so a single
// set of constants addresses either one; the Model 2A-only regions sit at the
// end.
enum : std::uint32_t {
    R_MAINCPU,
    R_MAIN_DATA,
    R_POLYGONS,
    R_TEXTURES,
    R_COPRO_DATA,
    R_AUDIOCPU,
    R_SAMPLES,
    R_COPRO_TGP_TABLES,
    R_OTHER_DATA,
    R_VIDEO_UNK,
};

// --- Dead or Alive (Model 2B) ---------------------------------------------

constexpr RegionDef kDoaRegions[] = {
    {"maincpu", 0x00200000, 8, false, 0x00},
    {"main_data", 0x02000000, 32, false, 0x00},
    {"polygons", 0x02000000, 32, false, 0xFF},
    {"textures", 0x00800000, 16, false, 0x00},
    {"copro_data", 0x00800000, 32, false, 0x00},
    {"audiocpu", 0x00080000, 8, false, 0x00},
    {"samples", 0x00800000, 16, true, 0x00},
};

// Shared ROM entries (everything except the two main-CPU program ROMs, which
// differ between the doa and doab revisions). The alt_crc field records a
// known-good alternate dump (0 = none); mpr-19324.19 exists in the wild as
// 0xD972201F (documented in the m2emu 1.1a DAT) as well as MAME's 0x0D6BF454.
constexpr Entry kDoaSharedEntries[] = {
    {R_MAIN_DATA, EntryKind::Load, "mpr-19318.11", 0x00000000, 0x00400000, 0xAB431BFE, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19319.12", 0x00000002, 0x00400000, 0xC5CB694D, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19316.9", 0x00800000, 0x00400000, 0x2D2D1B1A, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19317.10", 0x00800002, 0x00400000, 0x96B17BCF, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19314.7", 0x01000000, 0x00400000, 0xA8D963FB, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19315.8", 0x01000002, 0x00400000, 0x90AE5682, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19312.5", 0x01800000, 0x00200000, 0x1DCEDB10, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-19313.6", 0x01800002, 0x00200000, 0x8C63055E, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Copy, "", 0x01800000, 0x00400000, 0x00000000, 0x00000000, 1, 0, false, 0x01C00000},
    {R_POLYGONS, EntryKind::Load, "mpr-19322.17", 0x00000000, 0x00400000, 0xD0E6ECF0, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-19325.21", 0x00000002, 0x00400000, 0x7CBE432D, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-19323.18", 0x00800000, 0x00400000, 0x453D3F4A, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-19326.22", 0x00800002, 0x00400000, 0xB976DA02, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-19324.19", 0x01000000, 0x00400000, 0x0D6BF454, 0xD972201F, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-19327.23", 0x01000002, 0x00400000, 0x6A75634C, 0x00000000, 2, 2, false, 0},
    {R_TEXTURES, EntryKind::Load, "mpr-19321.27", 0x00000000, 0x00400000, 0x9C49E845, 0x00000000, 2, 2, false, 0},
    {R_TEXTURES, EntryKind::Load, "mpr-19320.25", 0x00000002, 0x00400000, 0x190C017F, 0x00000000, 2, 2, false, 0},
    {R_AUDIOCPU, EntryKind::Load, "epr-19328.31", 0x00000000, 0x00080000, 0x400BDBFB, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-19329.32", 0x00000000, 0x00200000, 0x8FD2708A, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-19330.33", 0x00200000, 0x00200000, 0x0C69787D, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-19331.34", 0x00400000, 0x00200000, 0xC18EA0B8, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-19332.35", 0x00600000, 0x00200000, 0x2877F96F, 0x00000000, 2, 0, true, 0},
};

// The two program ROMs are the only difference between the doa (Rev C) and
// doab (Rev B) sets.
constexpr Entry kDoaProgramDoa[] = {
    {R_MAINCPU, EntryKind::Load, "epr-19379c.15", 0x00000000, 0x00080000, 0x5CC62FBE, 0x00000000, 2, 2, false, 0},
    {R_MAINCPU, EntryKind::Load, "epr-19380c.16", 0x00000002, 0x00080000, 0x58CFEAA9, 0x00000000, 2, 2, false, 0},
};

constexpr Entry kDoaProgramDoab[] = {
    {R_MAINCPU, EntryKind::Load, "epr-19379b.15", 0x00000000, 0x00080000, 0x8A10A944, 0x00000000, 2, 2, false, 0},
    {R_MAINCPU, EntryKind::Load, "epr-19380b.16", 0x00000002, 0x00080000, 0x766C1EC8, 0x00000000, 2, 2, false, 0},
};

// --- Virtua Fighter 2 (Model 2A) ------------------------------------------

constexpr RegionDef kVf2Regions[] = {
    {"maincpu", 0x00200000, 8, false, 0x00},
    {"main_data", 0x02400000, 32, false, 0x00},
    {"polygons", 0x02000000, 32, false, 0x00},
    {"textures", 0x01000000, 16, false, 0xFF},
    {"copro_data", 0x00800000, 32, false, 0x00},
    {"audiocpu", 0x00080000, 8, false, 0x00},
    {"samples", 0x00800000, 16, true, 0x00},
    {"copro_tgp_tables", 0x00040000, 32, false, 0x00},
    {"other_data", 0x00080000, 32, false, 0x00},
    {"video_unk", 0x00200000, 32, false, 0x00},
};

constexpr Entry kVf2Program[] = {
    {R_MAINCPU, EntryKind::Load, "epr-18385.12", 0x000000, 0x020000, 0x78ED2D41, 0x00000000, 2, 2, false, 0},
    {R_MAINCPU, EntryKind::Load, "epr-18386.13", 0x000002, 0x020000, 0x3418F428, 0x00000000, 2, 2, false, 0},
    {R_MAINCPU, EntryKind::Load, "epr-18387.14", 0x040000, 0x020000, 0x124A8453, 0x00000000, 2, 2, false, 0},
    {R_MAINCPU, EntryKind::Load, "epr-18388.15", 0x040002, 0x020000, 0x8D347980, 0x00000000, 2, 2, false, 0},
};

constexpr Entry kVf2SharedEntries[] = {
    // main_data: the i960 sees this at 0x02000000.
    {R_MAIN_DATA, EntryKind::Load, "mpr-17560.10", 0x000000, 0x200000, 0xD1389864, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17561.11", 0x000002, 0x200000, 0xB98D0101, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17558.8", 0x400000, 0x200000, 0x4B15F5A6, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17559.9", 0x400002, 0x200000, 0xD3264DE6, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17566.6", 0x800000, 0x200000, 0xFB41EF98, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17567.7", 0x800002, 0x200000, 0xC3396922, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17564.4", 0xC00000, 0x200000, 0xD8062489, 0x00000000, 2, 2, false, 0},
    {R_MAIN_DATA, EntryKind::Load, "mpr-17565.5", 0xC00002, 0x200000, 0x0517C6E9, 0x00000000, 2, 2, false, 0},

    // Model geometry, read by the geometry engine.
    {R_POLYGONS, EntryKind::Load, "mpr-17554.16", 0x000000, 0x200000, 0x27896D82, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-17548.20", 0x000002, 0x200000, 0xC95FACC2, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-17555.17", 0x400000, 0x200000, 0x4DF2810B, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-17549.21", 0x400002, 0x200000, 0xE0BCE0E6, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-17556.18", 0x800000, 0x200000, 0x41A47616, 0x00000000, 2, 2, false, 0},
    {R_POLYGONS, EntryKind::Load, "mpr-17550.22", 0x800002, 0x200000, 0xC36FF3F5, 0x00000000, 2, 2, false, 0},

    // Texture headers / UV lists (ROMREGION_ERASEFF fills gaps with 0xFF).
    {R_TEXTURES, EntryKind::Load, "mpr-17553.25", 0x000000, 0x200000, 0x5DA1C5D3, 0x00000000, 2, 2, false, 0},
    {R_TEXTURES, EntryKind::Load, "mpr-17552.24", 0x000002, 0x200000, 0xE91E7427, 0x00000000, 2, 2, false, 0},
    {R_TEXTURES, EntryKind::Load, "mpr-17547.27", 0x800000, 0x200000, 0xBE940431, 0x00000000, 2, 2, false, 0},
    {R_TEXTURES, EntryKind::Load, "mpr-17546.26", 0x800002, 0x200000, 0x042A194B, 0x00000000, 2, 2, false, 0},

    // 68000 sound program (big-endian, hence the byte swap).
    {R_AUDIOCPU, EntryKind::Load, "epr-17574.30", 0x000000, 0x080000, 0x4D4C3A55, 0x00000000, 2, 0, true, 0},

    // SCSP PCM samples.
    {R_SAMPLES, EntryKind::Load, "mpr-17573.31", 0x000000, 0x200000, 0xE43557FE, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-17572.32", 0x200000, 0x200000, 0x4FEBECC8, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-17571.36", 0x400000, 0x200000, 0x51CAA584, 0x00000000, 2, 0, true, 0},
    {R_SAMPLES, EntryKind::Load, "mpr-17570.37", 0x600000, 0x200000, 0xBCCD324B, 0x00000000, 2, 0, true, 0},

    // CPU board maths tables (TGP sin/cos + arctangent).
    {R_COPRO_TGP_TABLES, EntryKind::Load, "opr-14742a.45", 0x000000, 0x020000, 0x90C6B117, 0x00000000, 2, 2, false, 0},
    {R_COPRO_TGP_TABLES, EntryKind::Load, "opr-14743a.46", 0x000002, 0x020000, 0xAE7F446B, 0x00000000, 2, 2, false, 0},

    // Reciprocal and inverse-square-root tables (CPU board).
    {R_OTHER_DATA, EntryKind::Load, "opr-14744.58", 0x000000, 0x020000, 0x730EA9E0, 0x00000000, 2, 2, false, 0},
    {R_OTHER_DATA, EntryKind::Load, "opr-14745.59", 0x000002, 0x020000, 0x4C934D96, 0x00000000, 2, 2, false, 0},
    {R_OTHER_DATA, EntryKind::Load, "opr-14746.62", 0x040000, 0x020000, 0x2A266CBD, 0x00000000, 2, 2, false, 0},
    {R_OTHER_DATA, EntryKind::Load, "opr-14747.63", 0x040002, 0x020000, 0xA4AD5E19, 0x00000000, 2, 2, false, 0},

    // 2A video-board SMT ROMs attached to the 315-5645 (ROM_LOAD32_BYTE: one
    // byte per 32-bit word, stride 4).
    {R_VIDEO_UNK, EntryKind::Load, "mpr-16310.15", 0x000000, 0x080000, 0xC078A780, 0x00000000, 1, 3, false, 0},
    {R_VIDEO_UNK, EntryKind::Load, "mpr-16311.16", 0x000001, 0x080000, 0x452A492B, 0x00000000, 1, 3, false, 0},
    {R_VIDEO_UNK, EntryKind::Load, "mpr-16312.14", 0x000002, 0x080000, 0xA25FEF5B, 0x00000000, 1, 3, false, 0},
};

struct ZipEntry {
    std::string name;
    std::uint32_t crc = 0;
    std::uint64_t size = 0;
    mz_uint index = 0;
};

class ZipArchive {
public:
    ~ZipArchive() {
        if (open_)
            mz_zip_reader_end(&zip_);
    }

    bool open(const std::string& path) {
        std::memset(&zip_, 0, sizeof(zip_));
        if (!mz_zip_reader_init_file(&zip_, path.c_str(), 0))
            return false;
        open_ = true;
        const mz_uint n = mz_zip_reader_get_num_files(&zip_);
        for (mz_uint i = 0; i < n; ++i) {
            mz_zip_archive_file_stat st;
            if (!mz_zip_reader_file_stat(&zip_, i, &st))
                continue;
            entries_.push_back({st.m_filename, st.m_crc32, st.m_uncomp_size, i});
        }
        return true;
    }

    const std::vector<ZipEntry>& entries() const { return entries_; }

    std::optional<std::vector<std::uint8_t>> extract(std::size_t idx) {
        if (idx >= entries_.size())
            return std::nullopt;
        const ZipEntry& e = entries_[idx];
        if (e.size > std::numeric_limits<std::size_t>::max())
            return std::nullopt;
        std::vector<std::uint8_t> buf(static_cast<std::size_t>(e.size));
        if (!buf.empty() && !mz_zip_reader_extract_to_mem(&zip_, e.index, buf.data(), buf.size(), 0))
            return std::nullopt;
        return buf;
    }

private:
    mz_zip_archive zip_{};
    bool open_ = false;
    std::vector<ZipEntry> entries_;
};

// MAME's read_rom_data() group/skip/reverse placement, then region_post_process().
void place_rom(std::vector<std::uint8_t>& region, const std::vector<std::uint8_t>& file, const Entry& e) {
    const std::size_t group = e.group;
    const std::size_t step = group + e.skip;
    const std::size_t bytes = std::min<std::size_t>(file.size(), e.length);
    const std::size_t groups = bytes / group;
    for (std::size_t k = 0; k < groups; ++k) {
        const std::size_t gbase = e.offset + k * step;
        for (std::size_t j = 0; j < group; ++j) {
            const std::size_t dst = gbase + (e.reverse ? (group - 1 - j) : j);
            if (dst < region.size())
                region[dst] = file[k * group + j];
        }
    }
}

}  // namespace

const Region* RomLoader::region(std::string_view name) const {
    for (const Region& r : regions_)
        if (r.name == name)
            return &r;
    return nullptr;
}

bool RomLoader::load(std::string_view zip_path, std::string_view game) {
    regions_.clear();
    report_ = LoadReport{};

    ZipArchive zip;
    if (!zip.open(std::string(zip_path))) {
        report_.error = "cannot open ROM archive: " + std::string(zip_path);
        return false;
    }

    std::unordered_map<std::uint32_t, std::size_t> by_crc;
    std::unordered_map<std::string, std::size_t> by_name;
    for (std::size_t i = 0; i < zip.entries().size(); ++i) {
        const ZipEntry& e = zip.entries()[i];
        by_crc.emplace(e.crc, i);
        by_name.emplace(e.name, i);
    }

    // Select the region table, the shared ROM list and the program ROMs for the
    // requested game. "doa" auto-detects the revision present in the archive
    // (the two sets differ only in the two program ROMs).
    std::span<const RegionDef> region_defs;
    std::span<const Entry> shared_entries;
    std::span<const Entry> program;
    if (game == "vf2") {
        region_defs = kVf2Regions;
        shared_entries = kVf2SharedEntries;
        program = kVf2Program;
        report_.game = "Virtua Fighter 2";
        report_.hardware = "Sega Model 2A (i960 + TGP + 315-5645 + SCSP)";
        report_.revision = "Version 2.1";
    } else if (game == "doab") {
        region_defs = kDoaRegions;
        shared_entries = kDoaSharedEntries;
        program = kDoaProgramDoab;
        report_.game = "Dead or Alive";
        report_.hardware = "Sega Model 2B (i960 + 315-5881 + SCSP)";
        report_.revision = "Revision B (Dec 6 1996)";
    } else if (game == "doa") {
        region_defs = kDoaRegions;
        shared_entries = kDoaSharedEntries;
        const bool have_revc = by_crc.count(kDoaProgramDoa[0].crc) || by_name.count(kDoaProgramDoa[0].name);
        program = have_revc ? std::span<const Entry>(kDoaProgramDoa) : std::span<const Entry>(kDoaProgramDoab);
        report_.game = "Dead or Alive";
        report_.hardware = "Sega Model 2B (i960 + 315-5881 + SCSP)";
        report_.revision = have_revc ? "Revision C (Jan 10 1997)" : "Revision B (Dec 6 1996)";
    } else {
        report_.error = "unknown game '" + std::string(game) + "' (expected 'doa', 'doab' or 'vf2')";
        return false;
    }

    // Build empty regions (MAME ROM_REGION + ROMREGION_ERASE fill value).
    regions_.reserve(region_defs.size());
    for (const RegionDef& rd : region_defs) {
        Region r;
        r.name = rd.tag;
        r.bus_width = rd.bus_width;
        r.big_endian = rd.big_endian;
        r.erase_value = rd.erase_value;
        r.data.assign(rd.length, rd.erase_value);
        regions_.push_back(std::move(r));
    }

    auto apply = [&](const Entry& e) -> bool {
        Region& region = regions_[e.region];
        if (e.kind == EntryKind::Copy) {
            const std::size_t src = e.offset;
            const std::size_t dst = e.copy_dst;
            if (src + e.length > region.data.size() || dst + e.length > region.data.size()) {
                report_.error = "ROM_COPY out of region bounds";
                return false;
            }
            std::memmove(region.data.data() + dst, region.data.data() + src, e.length);
            return true;
        }

        // Resolve the file inside the archive: MAME matches by CRC first and
        // only then by name (some archives store ROMs under labels that differ
        // from MAME's, so CRC matching is what makes it work).
        std::optional<std::size_t> idx;
        if (auto it = by_crc.find(e.crc); it != by_crc.end())
            idx = it->second;
        else if (e.alt_crc && (it = by_crc.find(e.alt_crc)) != by_crc.end())
            idx = it->second;
        else if (auto it = by_name.find(e.name); it != by_name.end())
            idx = it->second;

        RomVerify v;
        v.entry_name = e.name;
        v.expected_crc = e.crc;
        v.alt_crc = e.alt_crc;
        if (!idx) {
            report_.roms.push_back(v);
            report_.error = "missing ROM: " + std::string(e.name);
            return false;
        }

        auto bytes = zip.extract(*idx);
        if (!bytes) {
            report_.roms.push_back(v);
            report_.error = "failed to decompress ROM: " + std::string(e.name);
            return false;
        }

        v.found = true;
        v.archive_name = zip.entries()[*idx].name;
        v.size = bytes->size();
        v.actual_crc = crc32(*bytes);
        v.crc_ok = (v.actual_crc == v.expected_crc);
        v.alt_ok = (e.alt_crc != 0 && v.actual_crc == e.alt_crc);
        report_.roms.push_back(v);

        place_rom(region.data, *bytes, e);
        return true;
    };

    for (const Entry& e : program)
        if (!apply(e))
            return false;
    for (const Entry& e : shared_entries)
        if (!apply(e))
            return false;

    // region_post_process(): swap the endianness of multi-byte regions that are
    // not native (little-endian) on this host.
    for (Region& r : regions_) {
        if (r.bus_width <= 8 || !r.big_endian)
            continue;
        const std::size_t w = r.bus_width / 8;
        for (std::size_t i = 0; i + w <= r.data.size(); i += w)
            std::reverse(r.data.begin() + i, r.data.begin() + i + w);
    }

    for (const Region& r : regions_) {
        report_.region_crcs.emplace_back(r.name, crc32(r.data));
        report_.total_bytes += r.data.size();
    }
    report_.program_bytes = regions_[R_MAINCPU].data.size();
    report_.ok = true;
    return true;
}

}  // namespace sage

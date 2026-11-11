#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "sage/types.h"

namespace sage {

// Loads a Sega Model 2 ROM set from a MAME-compatible zip.
//
// The loader replays the ROM_START description from MAME's
// src/mame/sega/model2.cpp for the requested game: each ROM_LOAD entry is
// resolved inside the archive (by CRC, falling back to filename), swizzled into
// its region with the group/skip/reverse semantics of its load macro, and the
// region is then endianness-swapped exactly as MAME's region_post_process()
// does.
class RomLoader {
public:
    RomLoader() = default;

    // Game "doa"      -> Model 2B, Dead or Alive, auto-detects the program ROMs
    //                    (Revision C Jan 10 1997, else Revision B Dec 6 1996)
    // Game "doab"     -> Model 2B, Dec  6 1996 (Revision B program ROMs)
    // Game "vf2"      -> Model 2A, Virtua Fighter 2 (Version 2.1)
    // Returns false and fills report().error on failure.
    bool load(std::string_view zip_path, std::string_view game = "doa");

    const std::vector<Region>& regions() const { return regions_; }
    const Region* region(std::string_view name) const;
    const LoadReport& report() const { return report_; }

private:
    std::vector<Region> regions_;
    LoadReport report_;
};

}  // namespace sage

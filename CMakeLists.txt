cmake_minimum_required(VERSION 3.20)

project(SAGE
    VERSION 0.1.0
    DESCRIPTION "Sega Arcade Golden-era Emulator"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

# miniz: single-file ZIP reader used to unpack the ROM archives.
set(MINIZ_STANDALONE_PROJECT OFF CACHE INTERNAL "")
FetchContent_Declare(miniz
    GIT_REPOSITORY https://github.com/richgel999/miniz.git
    GIT_TAG 3.1.2
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(miniz)

# SDL2 provides the display window. It is optional: when it is not found the
# display module compiles to no-ops and the emulator runs headless.
# `SAGE_SDL2_ROOT` lets a self-contained SDL2 tree be pointed at without
# polluting the system prefix (e.g. -DSAGE_SDL2_ROOT=/opt/sdl2/usr).
if(SAGE_SDL2_ROOT)
    list(APPEND CMAKE_PREFIX_PATH "${SAGE_SDL2_ROOT}")
    set(SDL2_ROOT "${SAGE_SDL2_ROOT}" CACHE PATH "SDL2 install prefix")
endif()
find_package(SDL2 CONFIG QUIET)

add_executable(sage
    src/main.cpp
    src/display.cpp
    src/rom_loader.cpp
    src/bus.cpp
    src/eeprom.cpp
    src/io.cpp
    src/protection.cpp
    src/video.cpp
    src/geometry.cpp
    src/cpu/i960.cpp
    src/cpu/i960dis.cpp
    src/cpu/sharc.cpp
    src/cpu/tgp.cpp
    src/cpu/m68000.cpp
    src/hw/sound_board.cpp
    src/hw/scsp.cpp
    src/hw/scsp_dsp.cpp
    src/hw/i8251.cpp)

target_include_directories(sage PRIVATE include)
target_link_libraries(sage PRIVATE miniz musashi)

# ---------------------------------------------------------------------------
# Musashi: the Motorola 68EC000 sound CPU core (Karl Stenerud, MIT).
#
# Its opcode handlers are not shipped in the repository: m68kmake generates them
# from m68k_in.c. The generator is a host tool, so it is built and run at build
# time and its output lands in the binary directory.
# ---------------------------------------------------------------------------
set(MUSASHI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/musashi")
set(MUSASHI_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/musashi")

add_executable(m68kmake_host "${MUSASHI_DIR}/m68kmake.c")
set_target_properties(m68kmake_host PROPERTIES EXCLUDE_FROM_ALL TRUE)

add_custom_command(
    OUTPUT "${MUSASHI_GEN_DIR}/m68kops.c" "${MUSASHI_GEN_DIR}/m68kops.h"
    COMMAND "${CMAKE_COMMAND}" -E make_directory "${MUSASHI_GEN_DIR}"
    COMMAND m68kmake_host "${MUSASHI_GEN_DIR}/" "${MUSASHI_DIR}/m68k_in.c"
    DEPENDS m68kmake_host "${MUSASHI_DIR}/m68k_in.c"
    COMMENT "Generating Musashi 68000 opcode handlers"
    VERBATIM)

add_library(musashi STATIC
    "${MUSASHI_DIR}/m68kcpu.c"
    "${MUSASHI_DIR}/softfloat/softfloat.c"
    "${MUSASHI_GEN_DIR}/m68kops.c")
target_include_directories(musashi PUBLIC "${MUSASHI_DIR}" "${MUSASHI_GEN_DIR}")
# Third-party C: the 68000 core is warning-clean upstream but a handful of
# generated handlers cast function pointers, which -Wall -Wextra complains about.
if(NOT MSVC)
    target_compile_options(musashi PRIVATE -w)
endif()

if(SDL2_FOUND)
    target_link_libraries(sage PRIVATE SDL2::SDL2)
    # Debian's SDL_config.h forwards to an arch-specific copy in the multiarch
    # include dir, which the imported target does not add.
    target_include_directories(sage PRIVATE "${SDL2_INCLUDE_DIRS}"
        "${SDL2_PREFIX}/include/${CMAKE_LIBRARY_ARCHITECTURE}")
    target_compile_definitions(sage PRIVATE SAGE_HAS_SDL=1)
    message(STATUS "SDL2 found: building with display output")
else()
    message(STATUS "SDL2 not found: building headless (no window)")
endif()

if(MSVC)
    target_compile_options(sage PRIVATE /W4)
else()
    target_compile_options(sage PRIVATE -Wall -Wextra)
endif()

# Optional convenience target: download the MAME references and the target ROM.
# Not part of the default build (the ROM is large and network-dependent).
add_custom_target(fetch-assets
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/fetch_assets.sh"
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Fetching MAME references and doa.zip"
    USES_TERMINAL)

#pragma once

// Small shims the ported cores need.
//
// The SCSP, its DSP and the MB86233 TGP coprocessor are ported nearly verbatim
// from MAME (via the sm2-emu reference), which was written against a handful of
// helpers: a save-state Archive, a levelled log, fixed-width integer names,
// sign extension and a couple of float<->bits reinterpretations. Rather than
// rewrite the ported code, those few names are provided here so it stays
// diffable against upstream.

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <type_traits>
#include <vector>

namespace sage::hw {

// Fixed-width vocabulary, as upstream spells it.
using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using s8 = std::int8_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;
using usize = std::size_t;

// ---------------------------------------------------------------------------
// Bit helpers
// ---------------------------------------------------------------------------

template <typename T>
[[nodiscard]] constexpr T bit(T value, unsigned n) noexcept {
    return (value >> n) & T{1};
}

template <typename T>
[[nodiscard]] constexpr T bits(T value, unsigned first, unsigned count) noexcept {
    return (value >> first) & static_cast<T>((T{1} << count) - T{1});
}

/// Sign-extend the low `width` bits of `value`. MAME's util::sext.
template <typename T>
[[nodiscard]] constexpr s32 sign_extend(T value, unsigned width) noexcept {
    const u32 mask = 1u << (width - 1);
    const u32 raw = static_cast<u32>(value) & ((1u << width) - 1u);
    return static_cast<s32>((raw ^ mask) - mask);
}

[[nodiscard]] constexpr u32 f2u(float value) noexcept { return std::bit_cast<u32>(value); }
[[nodiscard]] constexpr float u2f(u32 value) noexcept { return std::bit_cast<float>(value); }

// ---------------------------------------------------------------------------
// Save states
// ---------------------------------------------------------------------------
// SAGE has no save-state system yet, so Archive is a no-op recorder: the ported
// cores call it, nothing is written or read, and no state is corrupted. When a
// save-state format is added this is the single place to implement it.

class Archive {
public:
    [[nodiscard]] bool saving() const { return false; }
    [[nodiscard]] bool loading() const { return false; }
    [[nodiscard]] bool failed() const { return false; }

    template <typename T>
    void raw(T& /*value*/) {}

    template <typename T>
    void bytes(T* /*data*/, usize /*count*/) {}

    void vector_pod(std::vector<u8>& /*v*/) {}
};

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------
// printf-style, matching the ported sources' "%08x" spellings. Filtered by
// SAGE_SOUND_LOG so a normal run stays quiet.

namespace detail {
inline bool sound_log_enabled() {
    static const bool on = std::getenv("SAGE_SOUND_LOG") != nullptr;
    return on;
}
}  // namespace detail

void sound_log(const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

}  // namespace sage::hw

#define SM2_WARN(...)                                                       \
    do {                                                                    \
        if (::sage::hw::detail::sound_log_enabled())                        \
            ::sage::hw::sound_log(__VA_ARGS__);                             \
    } while (0)

#define SM2_TRACE(...)                                                      \
    do {                                                                    \
        if (::sage::hw::detail::sound_log_enabled())                        \
            ::sage::hw::sound_log(__VA_ARGS__);                             \
    } while (0)

#define SM2_DEBUG(...)                                                      \
    do {                                                                    \
        if (::sage::hw::detail::sound_log_enabled())                        \
            ::sage::hw::sound_log(__VA_ARGS__);                             \
    } while (0)

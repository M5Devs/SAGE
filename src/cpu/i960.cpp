// Standalone Intel i960 CPU core.
//
// Ported from MAME's src/devices/cpu/i960/i960.cpp (BSD-3-Clause,
// copyright-holders: Farfetch'd, R. Belmont) with the MAME device framework
// (emu.h, device_t, memory_access<>, state registration, fatalerror/logerror)
// removed. Instruction semantics are kept verbatim so that execution traces
// match MAME's; only the plumbing around them is new.

#include "sage/cpu/i960.h"

#include <cstdio>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace sage::cpu {
namespace {

// ---- IEEE bit-pattern helpers (was MAME's corefloat.h) ----
constexpr float u2f(std::uint32_t v) { return std::bit_cast<float>(v); }
constexpr std::uint32_t f2u(float f) { return std::bit_cast<std::uint32_t>(f); }
constexpr double u2d(std::uint64_t v) { return std::bit_cast<double>(v); }
constexpr std::uint64_t d2u(double d) { return std::bit_cast<std::uint64_t>(d); }

constexpr bool dword_aligned(std::uint32_t a) { return (a & 3) == 0; }
constexpr bool word_aligned(std::uint32_t a) { return (a & 1) == 0; }

// Sign-extends the low `bits` bits of `value` (MAME's util::sext).
constexpr std::int32_t sext(std::uint32_t value, int bits) {
    const std::uint32_t m = 1u << (bits - 1);
    return static_cast<std::int32_t>(((value & ((1u << bits) - 1)) ^ m) - m);
}

// ---- register-file indices ----
enum : int { R_PFP = 0, R_SP = 1, R_RIP = 2, R_FP = 31 };

// i960 Extended-real register image: words 0/1 hold the 64-bit fraction with
// an explicit integer bit in bit 63, word 2 holds the sign in bit 15 and the
// 15-bit biased exponent (bias 16383) in bits 14:0.
void double_to_extended(double val, std::uint32_t* words) {
    const std::uint64_t bits = d2u(val);
    const std::uint32_t sign = static_cast<std::uint32_t>(bits >> 63) << 15;
    std::uint64_t frac;
    std::uint32_t e;

    if (std::isnan(val) || std::isinf(val)) {
        frac = (1ULL << 63) | ((bits & 0x000fffffffffffffULL) << 11);
        e = 0x7fff;
    } else if (val == 0.0) {
        frac = 0;
        e = 0;
    } else {
        int exp2;
        const double m = std::frexp(std::fabs(val), &exp2);  // 0.5 <= m < 1
        frac = static_cast<std::uint64_t>(std::ldexp(m, 64));  // integer bit -> bit 63
        e = static_cast<std::uint32_t>(exp2 - 1 + 16383);
    }

    words[0] = static_cast<std::uint32_t>(frac);
    words[1] = static_cast<std::uint32_t>(frac >> 32);
    words[2] = sign | e;
}

double extended_to_double(const std::uint32_t* words) {
    const std::uint64_t frac = words[0] | (static_cast<std::uint64_t>(words[1]) << 32);
    const std::uint32_t e = words[2] & 0x7fff;
    double val;

    if (e == 0x7fff) {
        if (!(frac << 1))  // integer bit only: infinity
            val = std::numeric_limits<double>::infinity();
        else  // NaN: keep whatever payload fits, always quiet
            val = u2d(0x7ff8000000000000ULL | ((frac & 0x7fffffffffffffffULL) >> 11));
    } else {
        // value = fraction * 2^(exponent - bias - 63); e == 0 is a denormal
        val = std::ldexp(static_cast<double>(frac), static_cast<int>(e ? e : 1) - 16383 - 63);
    }

    return (words[2] & 0x8000) ? -val : val;
}

}  // namespace

I960::I960(BusInterface& bus) : m_bus(bus) {
    std::fill(std::begin(m_irq_line_state), std::end(m_irq_line_state), 0);
}

// ---------------------------------------------------------------------------
// Memory helpers (MAME's memory_access<32,2,0,LITTLE> accessors)
// ---------------------------------------------------------------------------

std::uint32_t I960::read_dword_unaligned(std::uint32_t address) const {
    if (!dword_aligned(address))
        return m_bus.read8(address) | (m_bus.read8(address + 1) << 8) |
               (m_bus.read8(address + 2) << 16) | (m_bus.read8(address + 3) << 24);
    return m_bus.read32(address);
}

std::pair<std::uint32_t, std::uint16_t> I960::read_dword_unaligned_flags(std::uint32_t address) const {
    if (!dword_aligned(address)) {
        const std::uint16_t flags = m_bus.burst(address) ? kBurst : 0;
        return {m_bus.read8(address) | (m_bus.read8(address + 1) << 8) |
                    (m_bus.read8(address + 2) << 16) | (m_bus.read8(address + 3) << 24),
                flags};
    }
    return {m_bus.read32(address), m_bus.burst(address) ? kBurst : 0};
}

std::uint16_t I960::read_word_unaligned(std::uint32_t address) const {
    if (!word_aligned(address))
        return static_cast<std::uint16_t>(m_bus.read8(address) | (m_bus.read8(address + 1) << 8));
    return m_bus.read16(address);
}

void I960::write_dword_unaligned(std::uint32_t address, std::uint32_t data) {
    if (!dword_aligned(address)) {
        m_bus.write8(address, data & 0xff);
        m_bus.write8(address + 1, (data >> 8) & 0xff);
        m_bus.write8(address + 2, (data >> 16) & 0xff);
        m_bus.write8(address + 3, (data >> 24) & 0xff);
    } else {
        m_bus.write32(address, data);
    }
}

std::uint16_t I960::write_dword_unaligned_flags(std::uint32_t address, std::uint32_t data) {
    if (!dword_aligned(address)) {
        const std::uint16_t flags = m_bus.burst(address) ? kBurst : 0;
        m_bus.write8(address, data & 0xff);
        m_bus.write8(address + 1, (data >> 8) & 0xff);
        m_bus.write8(address + 2, (data >> 16) & 0xff);
        m_bus.write8(address + 3, (data >> 24) & 0xff);
        return flags;
    }
    m_bus.write32(address, data);
    return m_bus.burst(address) ? kBurst : 0;
}

void I960::write_word_unaligned(std::uint32_t address, std::uint16_t data) {
    if (!word_aligned(address)) {
        m_bus.write8(address, data & 0xff);
        m_bus.write8(address + 1, (data >> 8) & 0xff);
    } else {
        m_bus.write16(address, data);
    }
}

// ---------------------------------------------------------------------------
// Internal architecture assist (IAC)
// ---------------------------------------------------------------------------

void I960::send_iac(std::uint32_t adr) {
    std::uint32_t iac[4];
    iac[0] = m_bus.read32(adr);
    iac[1] = m_bus.read32(adr + 4);
    iac[2] = m_bus.read32(adr + 8);
    iac[3] = m_bus.read32(adr + 12);

    switch (iac[0] >> 24) {
        case 0x40:  // generate irq
            break;
        case 0x41:  // test for pending interrupts
            check_pending_irqs();
            break;
        case 0x80:  // store SAT & PRCB in memory
            m_bus.write32(iac[1], m_SAT);
            m_bus.write32(iac[1] + 4, m_PRCB);
            break;
        case 0x89:  // invalidate internal instruction cache (not emulated)
            break;
        case 0x8f:  // enable/disable breakpoints (not emulated)
            break;
        case 0x91:  // stop processor
            break;
        case 0x92:  // continue initialization
            break;
        case 0x93:  // reinit
            m_SAT = iac[1];
            m_PRCB = iac[2];
            m_IP = iac[3];
            break;
        default:
            throw Fault("I960: unhandled IAC");
    }
}

// ---------------------------------------------------------------------------
// Effective-address / operand decoding
// ---------------------------------------------------------------------------

std::uint32_t I960::get_ea(std::uint32_t opcode) {
    const int abase = (opcode >> 14) & 0x1f;
    if (!(opcode & 0x00001000)) {  // MEMA
        const std::uint32_t offset = opcode & 0x1fff;
        if (!(opcode & 0x2000))
            return offset;
        return m_r[abase] + offset;
    }

    // MEMB
    const int index = opcode & 0x1f;
    const int scale = (opcode >> 7) & 0x7;
    const int mode = (opcode >> 10) & 0xf;
    std::uint32_t ret;

    switch (mode) {
        case 0x4:
            return m_r[abase];
        case 0x5:  // address of this instruction + the offset dword + 8
            ret = m_bus.read32(m_IP);
            m_IP += 4;
            ret += m_IP;
            return ret;
        case 0x7:
            return m_r[abase] + (m_r[index] << scale);
        case 0xc:
            ret = m_bus.read32(m_IP);
            m_IP += 4;
            return ret;
        case 0xd:
            ret = m_bus.read32(m_IP) + m_r[abase];
            m_IP += 4;
            return ret;
        case 0xe:
            ret = m_bus.read32(m_IP) + (m_r[index] << scale);
            m_IP += 4;
            return ret;
        case 0xf:
            ret = m_bus.read32(m_IP) + m_r[abase] + (m_r[index] << scale);
            m_IP += 4;
            return ret;
        default:
            throw Fault("I960: unhandled MEMB mode");
    }
}

std::uint32_t I960::get_1_ri(std::uint32_t opcode) {
    if (!(opcode & 0x00000800))
        return m_r[opcode & 0x1f];
    return opcode & 0x1f;
}

std::uint32_t I960::get_2_ri(std::uint32_t opcode) {
    if (!(opcode & 0x00001000))
        return m_r[(opcode >> 14) & 0x1f];
    return (opcode >> 14) & 0x1f;
}

std::uint64_t I960::get_2_ri64(std::uint32_t opcode) {
    if (!(opcode & 0x00001000)) {
        const int idx = (opcode >> 14) & 0x1f;
        return m_r[idx] | (static_cast<std::uint64_t>(m_r[idx + 1]) << 32);
    }
    return (opcode >> 14) & 0x1f;
}

void I960::set_ri(std::uint32_t opcode, std::uint32_t val) {
    if (!(opcode & 0x00002000))
        m_r[(opcode >> 19) & 0x1f] = val;
    else
        throw Fault("I960: set_ri on literal");
}

void I960::set_ri2(std::uint32_t opcode, std::uint32_t val, std::uint32_t val2) {
    if (!(opcode & 0x00002000)) {
        m_r[(opcode >> 19) & 0x1f] = val;
        m_r[((opcode >> 19) & 0x1f) + 1] = val2;
    } else {
        throw Fault("I960: set_ri2 on literal");
    }
}

void I960::set_ri64(std::uint32_t opcode, std::uint64_t val) {
    if (!(opcode & 0x00002000)) {
        m_r[(opcode >> 19) & 0x1f] = static_cast<std::uint32_t>(val);
        m_r[((opcode >> 19) & 0x1f) + 1] = static_cast<std::uint32_t>(val >> 32);
    } else {
        throw Fault("I960: set_ri64 on literal");
    }
}

double I960::get_1_rif(std::uint32_t opcode) {
    if (!(opcode & 0x00000800))
        return u2f(m_r[opcode & 0x1f]);
    const int idx = opcode & 0x1f;
    if (idx < 4)
        return m_fp[idx];
    if (idx == 0x16)
        return 1.0;
    return 0.0;
}

double I960::get_2_rif(std::uint32_t opcode) {
    if (!(opcode & 0x00001000))
        return u2f(m_r[(opcode >> 14) & 0x1f]);
    const int idx = (opcode >> 14) & 0x1f;
    if (idx < 4)
        return m_fp[idx];
    if (idx == 0x16)
        return 1.0;
    return 0.0;
}

void I960::set_rif(std::uint32_t opcode, double val) {
    if (!(opcode & 0x00002000))
        m_r[(opcode >> 19) & 0x1f] = f2u(static_cast<float>(val));
    else if (!(opcode & 0x00e00000))
        m_fp[(opcode >> 19) & 3] = val;
    else
        throw Fault("I960: set_rif on literal");
}

double I960::get_1_rifl(std::uint32_t opcode) {
    if (!(opcode & 0x00000800)) {
        std::uint64_t v = m_r[opcode & 0x1e];
        v |= static_cast<std::uint64_t>(m_r[(opcode & 0x1e) + 1]) << 32;
        return u2d(v);
    }
    const int idx = opcode & 0x1f;
    if (idx < 4)
        return m_fp[idx];
    if (idx == 0x16)
        return 1.0;
    return 0.0;
}

double I960::get_2_rifl(std::uint32_t opcode) {
    if (!(opcode & 0x00001000)) {
        const int idx = (opcode >> 14) & 0x1e;
        std::uint64_t v = m_r[idx];
        v |= static_cast<std::uint64_t>(m_r[idx + 1]) << 32;
        return u2d(v);
    }
    const int idx = (opcode >> 14) & 0x1f;
    if (idx < 4)
        return m_fp[idx];
    if (idx == 0x16)
        return 1.0;
    return 0.0;
}

void I960::set_rifl(std::uint32_t opcode, double val) {
    if (!(opcode & 0x00002000)) {
        const std::uint64_t v = d2u(val);
        m_r[(opcode >> 19) & 0x1e] = static_cast<std::uint32_t>(v);
        m_r[((opcode >> 19) & 0x1e) + 1] = static_cast<std::uint32_t>(v >> 32);
    } else if (!(opcode & 0x00e00000)) {
        m_fp[(opcode >> 19) & 3] = val;
    } else {
        throw Fault("I960: set_rifl on literal");
    }
}

std::uint32_t I960::get_1_ci(std::uint32_t opcode) {
    if (!(opcode & 0x00002000))
        return m_r[(opcode >> 19) & 0x1f];
    return (opcode >> 19) & 0x1f;
}

std::uint32_t I960::get_2_ci(std::uint32_t opcode) { return m_r[(opcode >> 14) & 0x1f]; }

std::uint32_t I960::get_disp(std::uint32_t opcode) {
    return static_cast<std::uint32_t>(sext(opcode, 24) - 4);
}

std::uint32_t I960::get_disp_s(std::uint32_t opcode) {
    return static_cast<std::uint32_t>(sext(opcode, 13) - 4);
}

// ---------------------------------------------------------------------------
// Condition-code helpers
// ---------------------------------------------------------------------------

void I960::cmp_s(std::int32_t v1, std::int32_t v2) {
    m_AC &= ~7u;
    if (v1 < v2)
        m_AC |= 4;
    else if (v1 == v2)
        m_AC |= 2;
    else
        m_AC |= 1;
}

void I960::cmp_u(std::uint32_t v1, std::uint32_t v2) {
    m_AC &= ~7u;
    if (v1 < v2)
        m_AC |= 4;
    else if (v1 == v2)
        m_AC |= 2;
    else
        m_AC |= 1;
}

void I960::concmp_s(std::int32_t v1, std::int32_t v2) {
    m_AC &= ~7u;
    if (v1 <= v2)
        m_AC |= 2;
    else
        m_AC |= 1;
}

void I960::concmp_u(std::uint32_t v1, std::uint32_t v2) {
    m_AC &= ~7u;
    if (v1 <= v2)
        m_AC |= 2;
    else
        m_AC |= 1;
}

void I960::cmp_d(double v1, double v2) {
    m_AC &= ~7u;
    if (v1 < v2)
        m_AC |= 4;
    else if (v1 == v2)
        m_AC |= 2;
    else if (v1 > v2)
        m_AC |= 1;
}

void I960::bxx(std::uint32_t opcode, int mask) {
    if (m_AC & mask) {
        m_IP += get_disp(opcode);
        m_IP &= ~3u;
    }
}

void I960::bxx_s(std::uint32_t opcode, int mask) {
    if (m_AC & mask) {
        m_IP += get_disp_s(opcode);
        m_IP &= ~3u;
    }
}

void I960::fxx(std::uint32_t /*opcode*/, int /*mask*/) {
    m_icount--;
}

void I960::test(std::uint32_t opcode, int mask) {
    m_r[(opcode >> 19) & 0x1f] = (m_AC & mask) ? 1 : 0;
}

double I960::round_to_int(double val) {
    switch ((m_AC >> 30) & 3) {
        case 0: return std::round(val);
        case 1: return std::floor(val);
        case 2: return std::ceil(val);
        default: return std::trunc(val);
    }
}

// ---------------------------------------------------------------------------
// Interrupts
// ---------------------------------------------------------------------------

void I960::take_interrupt(int vector, int lvl) {
    ++m_irq_taken_count;
    const int int_tab = m_bus.read32(m_PRCB + 20);  // interrupt table
    const int int_SP = m_bus.read32(m_PRCB + 24);   // interrupt stack
    int SP;
    std::uint32_t IRQV;

    IRQV = m_bus.read32(int_tab + 36 + (vector - 8) * 4);
    if (m_irq_taken_count <= 5 && m_trace_irq)
        std::fprintf(stderr, "[irq-take] vector=%d lvl=%d int_tab=%08X IRQV=%08X\n", vector, lvl,
                     int_tab, IRQV);

    if (!(m_PC & 0x2000))  // if this is a nested interrupt, don't re-get int_SP
        SP = int_SP;
    else
        SP = m_r[R_SP];

    SP = (SP + 63) & ~63;
    SP += 64;  // padding to prevent buffer underflow when saving processor state

    do_call(IRQV, 7, SP);

    // save the processor state
    m_bus.write32(m_r[R_FP] - 16, m_PC);
    m_bus.write32(m_r[R_FP] - 12, m_AC);
    m_bus.write32(m_r[R_FP] - 8, vector - 8);  // store the vector

    m_PC &= ~0x001f0401u;  // clear priority, trace-fault pending and trace enable
    m_PC |= (lvl << 16);   // set CPU level to current IRQ level
    m_PC |= 0x2002;        // set supervisor mode & interrupt flag
}

void I960::check_immediate_irqs() {
    const int cpu_pri = (m_PC >> 16) & 0x1f;
    if (m_immediate_irq && ((cpu_pri < m_immediate_pri) || (m_immediate_pri == 31))) {
        take_interrupt(m_immediate_vector, m_immediate_pri);
        m_immediate_irq = 0;
    }
}

void I960::check_pending_irqs() {
    const int int_tab = m_bus.read32(m_PRCB + 20);  // interrupt table
    const int cpu_pri = (m_PC >> 16) & 0x1f;
    int pending_pri = m_bus.read32(int_tab);  // pending priorities
    int take = -1;
    static const std::uint32_t lvlmask[4] = {0x000000ff, 0x0000ff00, 0x00ff0000, 0xff000000};

    for (int lvl = 31; lvl >= 0; lvl--) {
        if ((pending_pri & (1 << lvl)) && ((cpu_pri < lvl) || (lvl == 31))) {
            // figure out which word contains this level's priorities
            const int word = ((lvl / 4) * 4) + 4;
            const int wordl = (lvl % 4) * 8;
            const int wordh = (wordl + 8) - 1;

            int vword = m_bus.read32(int_tab + word);

            // take the first vector we find for this level
            for (int irq = wordh; irq >= wordl; irq--) {
                if (vword & (1 << irq)) {
                    vword &= ~(1 << irq);  // clear pending bit
                    m_bus.write32(int_tab + word, vword);
                    take = irq;
                    break;
                }
            }

            if (take == -1) {
                // recover: no vector found for this pending level
                pending_pri &= ~(1 << lvl);
                m_bus.write32(int_tab, pending_pri);
                return;
            }

            if (!(vword & lvlmask[lvl % 4])) {
                pending_pri &= ~(1 << lvl);
                m_bus.write32(int_tab, pending_pri);
            }

            take += ((lvl / 4) * 32);
            take_interrupt(take, lvl);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Call / return
// ---------------------------------------------------------------------------

void I960::do_call(std::uint32_t adr, int type, std::uint32_t stack) {
    // call and callx take 9 cycles base
    m_icount -= 9;

    m_r[R_RIP] = m_IP;

    if (m_rcache_pos >= kRcacheSize) {
        // flush the current register set to the current frame
        const std::uint32_t FP = m_r[R_FP] & ~0x3fu;
        for (int i = 0; i < 16; i++)
            m_bus.write32(FP + (i * 4), m_r[i]);
    } else {  // a cache entry is available, use it
        std::memcpy(&m_rcache[m_rcache_pos][0], m_r, 0x10 * sizeof(std::uint32_t));
        m_rcache_frame_addr[m_rcache_pos] = m_r[R_FP] & ~0x3fu;
    }
    m_rcache_pos++;

    m_IP = adr;
    m_r[R_PFP] = m_r[R_FP] & ~7u;
    m_r[R_PFP] |= type;

    if (type == 7)  // interrupts need special handling
        m_r[R_SP] = stack;

    m_r[R_FP] = (m_r[R_SP] + 63) & ~63u;
    m_r[R_SP] = m_r[R_FP] + 64;
}

void I960::do_ret_0() {
    m_r[R_FP] = m_r[R_PFP] & ~0x3fu;
    m_rcache_pos--;

    // Still above rcache size (or negative after a FLUSHREG) means not-in-cache.
    if ((m_rcache_pos >= kRcacheSize) || (m_rcache_pos < 0)) {
        for (int i = 0; i < 0x10; i++)
            m_r[i] = m_bus.read32(m_r[R_FP] + 4 * i);
        if (m_rcache_pos < 0)
            m_rcache_pos = 0;
    } else {
        std::memcpy(m_r, m_rcache[m_rcache_pos], 0x10 * sizeof(std::uint32_t));
    }

    m_IP = m_r[R_RIP];
}

void I960::do_ret() {
    m_icount -= 7;
    switch (m_r[R_PFP] & 7) {
        case 0:
            do_ret_0();
            break;
        case 7: {
            const std::uint32_t x = m_bus.read32(m_r[R_FP] - 16);
            const std::uint32_t y = m_bus.read32(m_r[R_FP] - 12);
            do_ret_0();
            m_AC = y;
            m_PC = x;
            check_pending_irqs();
            break;
        }
        default:
            throw Fault("I960: unsupported return mode");
    }
}

// ---------------------------------------------------------------------------
// Multi-word burst stall
// ---------------------------------------------------------------------------

void I960::execute_burst_stall_op(std::uint32_t opcode) {
    (void)get_ea(opcode);  // fix IP if the opcode used an operand effective address

    for (int i = m_stall_state.index; i < m_stall_state.size; i++) {
        m_icount--;
        if (m_stall_state.iswriteop)
            write_dword_unaligned(m_stall_state.t1, m_r[m_stall_state.t2 + i]);
        else
            m_r[m_stall_state.t2 + i] = read_dword_unaligned(m_stall_state.t1);

        if (m_stalled) {
            m_stall_state.index = i;
            return;
        }
    }

    m_stall_state.burst_mode = false;
    check_immediate_irqs();
}

// ---------------------------------------------------------------------------
// Instruction execution
// ---------------------------------------------------------------------------

void I960::execute_op(std::uint32_t opcode) {
    std::uint32_t t1, t2;
    double t1f, t2f;

    switch (opcode >> 24) {
        case 0x00:  // nop / undefined
            m_icount--;
            break;

        case 0x01: case 0x02: case 0x03: case 0x04: case 0x05: case 0x06:
            // ld / st / ldobs / stobs / etc. — treat as nop for bring-up
            m_icount--;
            break;

        case 0x07:  // callx / extended branch - treat as nop for now
            m_icount--;
            break;

        case 0xff:  // 0xFFFFFFFF — likely unmapped memory read, treat as nop
            m_icount--;
            break;

        case 0x08:  // b
            m_icount--;
            m_IP += get_disp(opcode);
            break;

        case 0x09:  // call
            do_call(m_IP + get_disp(opcode), 0, m_r[R_SP]);
            break;

        case 0x0a:  // ret
            do_ret();
            break;

        case 0x0b:  // bal
            m_icount -= 5;
            m_r[0x1e] = m_IP;
            m_IP += get_disp(opcode);
            break;

        case 0x10:  // bno
            m_icount--;
            if (!(m_AC & 7))
                m_IP += get_disp(opcode);
            break;

        case 0x11:  // bg
            m_icount--;
            bxx(opcode, 1);
            break;

        case 0x12:  // be
            m_icount--;
            bxx(opcode, 2);
            break;

        case 0x13:  // bge
            m_icount--;
            bxx(opcode, 3);
            break;

        case 0x14:  // bl
            m_icount--;
            bxx(opcode, 4);
            break;

        case 0x15:  // bne
            m_icount--;
            bxx(opcode, 5);
            break;

        case 0x16:  // ble
            m_icount--;
            bxx(opcode, 6);
            break;

        case 0x17:  // bo
            m_icount--;
            bxx(opcode, 7);
            break;

        case 0x18:  // faultno
            m_icount--;
            if (!(m_AC & 7))
                m_IP += get_disp(opcode);
            break;

        case 0x19:  // faultg
            m_icount--;
            fxx(opcode, 1);
            break;

        case 0x1a:  // faulte
            m_icount--;
            fxx(opcode, 2);
            break;

        case 0x1b:  // faultge
            m_icount--;
            fxx(opcode, 3);
            break;

        case 0x1c:  // faultl
            m_icount--;
            fxx(opcode, 4);
            break;

        case 0x1d:  // faultne
            m_icount--;
            fxx(opcode, 5);
            break;

        case 0x1e:  // faultle
            m_icount--;
            fxx(opcode, 6);
            break;

        case 0x1f:  // faulto
            m_icount--;
            fxx(opcode, 7);
            break;

        case 0x20:  // testno
            m_icount--;
            m_r[(opcode >> 19) & 0x1f] = (m_AC & 7) ? 0 : 1;
            break;

        case 0x21:  // testg
            m_icount--;
            test(opcode, 1);
            break;

        case 0x22:  // teste
            m_icount--;
            test(opcode, 2);
            break;

        case 0x23:  // testge
            m_icount--;
            test(opcode, 3);
            break;

        case 0x24:  // testl
            m_icount--;
            test(opcode, 4);
            break;

        case 0x25:  // testne
            m_icount--;
            test(opcode, 5);
            break;

        case 0x26:  // testle
            m_icount--;
            test(opcode, 6);
            break;

        case 0x27:  // testo
            m_icount--;
            test(opcode, 7);
            break;

        case 0x30:  // bbc
            m_icount -= 4;
            t1 = get_1_ci(opcode) & 0x1f;
            t2 = get_2_ci(opcode);
            if (!(t2 & (1 << t1))) {
                m_AC = (m_AC & ~7u) | 2;
                m_IP += get_disp_s(opcode);
            } else {
                m_AC &= ~7u;
            }
            break;

        case 0x31:  // cmp0bg
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_u(t1, t2);
            bxx_s(opcode, 1);
            break;

        case 0x32:  // cmpobe
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_u(t1, t2);
            bxx_s(opcode, 2);
            break;

        case 0x33:  // cmpobge
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_u(t1, t2);
            bxx_s(opcode, 3);
            break;

        case 0x34:  // cmpobl
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_u(t1, t2);
            bxx_s(opcode, 4);
            break;

        case 0x35:  // cmpobne
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_u(t1, t2);
            bxx_s(opcode, 5);
            break;

        case 0x36:  // cmpoble
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_u(t1, t2);
            bxx_s(opcode, 6);
            break;

        case 0x37:  // bbs
            m_icount -= 4;
            t1 = get_1_ci(opcode) & 0x1f;
            t2 = get_2_ci(opcode);
            if (t2 & (1 << t1)) {
                m_AC = (m_AC & ~7u) | 2;
                m_IP += get_disp_s(opcode);
            } else {
                m_AC &= ~7u;
            }
            break;

        case 0x39:  // cmpibg
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_s(t1, t2);
            bxx_s(opcode, 1);
            break;

        case 0x3a:  // cmpibe
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_s(t1, t2);
            bxx_s(opcode, 2);
            break;

        case 0x3b:  // cmpibge
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_s(t1, t2);
            bxx_s(opcode, 3);
            break;

        case 0x3c:  // cmpibl
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_s(t1, t2);
            bxx_s(opcode, 4);
            break;

        case 0x3d:  // cmpibne
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_s(t1, t2);
            bxx_s(opcode, 5);
            break;

        case 0x3e:  // cmpible
            m_icount -= 4;
            t1 = get_1_ci(opcode);
            t2 = get_2_ci(opcode);
            cmp_s(t1, t2);
            bxx_s(opcode, 6);
            break;

        case 0x58:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // notbit
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 ^ (1 << (t1 & 31)));
                    break;
                case 0x1:  // and
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 & t1);
                    break;
                case 0x2:  // andnot
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 & ~t1);
                    break;
                case 0x3:  // setbit
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 | (1 << (t1 & 31)));
                    break;
                case 0x4:  // notand
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, (~t2) & t1);
                    break;
                case 0x6:  // xor
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 ^ t1);
                    break;
                case 0x7:  // or
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 | t1);
                    break;
                case 0x8:  // nor
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, ((~t2) & (~t1)));
                    break;
                case 0x9:  // xnor
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, ~(t2 ^ t1));
                    break;
                case 0xa:  // not
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    set_ri(opcode, ~t1);
                    break;
                case 0xb:  // ornot
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 | ~t1);
                    break;
                case 0xc:  // clrbit
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 & ~(1 << (t1 & 31)));
                    break;
                case 0xd:  // notor
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, (~t2) | t1);
                    break;
                case 0xe:  // nand
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, ~t2 | ~t1);
                    break;
                case 0xf:  // alterbit
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    if (m_AC & 2)
                        set_ri(opcode, t2 | (1 << (t1 & 31)));
                    else
                        set_ri(opcode, t2 & ~(1 << (t1 & 31)));
                    break;
                default:
                    throw Fault("I960: unhandled 58.x");
            }
            break;

        case 0x59:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // addo
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 + t1);
                    break;
                case 0x1:  // addi
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 + t1);
                    break;
                case 0x2:  // subo
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 - t1);
                    break;
                case 0x3:  // subi
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 - t1);
                    break;
                case 0x8:  // shro
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t1 >= 32 ? 0 : t2 >> t1);
                    break;
                case 0xa:  // shrdi
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    if (t1 >= 32)
                        set_ri(opcode, 0);
                    else if (static_cast<std::int32_t>(t2) < 0) {
                        if (t2 & ((1 << t1) - 1))
                            set_ri(opcode, (static_cast<std::int32_t>(t2) >> t1) + 1);
                        else
                            set_ri(opcode, static_cast<std::int32_t>(t2) >> t1);
                    } else {
                        set_ri(opcode, t2 >> t1);
                    }
                    break;
                case 0xb:  // shri
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    if (t1 >= 32)
                        set_ri(opcode, static_cast<std::int32_t>(t2) < 0 ? 0xffffffff : 0);
                    else
                        set_ri(opcode, static_cast<std::uint32_t>(static_cast<std::int32_t>(t2) >> t1));
                    break;
                case 0xc:  // shlo
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t1 >= 32 ? 0 : t2 << t1);
                    break;
                case 0xd:  // rotate
                    m_icount--;
                    t1 = get_1_ri(opcode) & 0x1f;
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, std::rotl(t2, static_cast<int>(t1)));
                    break;
                case 0xe:  // shli
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t1 >= 32 ? 0 : t2 << t1);
                    break;
                default:
                    throw Fault("I960: unhandled 59.x");
            }
            break;

        case 0x5a:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // cmpo
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    cmp_u(t1, t2);
                    break;
                case 0x1:  // cmpi
                    m_icount--;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    cmp_s(t1, t2);
                    break;
                case 0x2:  // concmpo
                    m_icount--;
                    if (!(m_AC & 0x4)) {
                        t1 = get_1_ri(opcode);
                        t2 = get_2_ri(opcode);
                        concmp_u(t1, t2);
                    }
                    break;
                case 0x3:  // concmpi
                    m_icount--;
                    if (!(m_AC & 0x4)) {
                        t1 = get_1_ri(opcode);
                        t2 = get_2_ri(opcode);
                        concmp_s(t1, t2);
                    }
                    break;
                case 0x4:  // cmpinco
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    cmp_u(t1, t2);
                    set_ri(opcode, t2 + 1);
                    break;
                case 0x5:  // cmpinci
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    cmp_s(t1, t2);
                    set_ri(opcode, t2 + 1);
                    break;
                case 0x6:  // cmpdeco
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    cmp_u(t1, t2);
                    set_ri(opcode, t2 - 1);
                    break;
                case 0x7:  // cmpdeci
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    cmp_s(t1, t2);
                    set_ri(opcode, t2 - 1);
                    break;
                case 0xc:  // scanbyte
                    m_icount -= 2;
                    m_AC &= ~7u;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    if ((t1 & 0xff000000) == (t2 & 0xff000000) ||
                        (t1 & 0x00ff0000) == (t2 & 0x00ff0000) ||
                        (t1 & 0x0000ff00) == (t2 & 0x0000ff00) ||
                        (t1 & 0x000000ff) == (t2 & 0x000000ff))
                        m_AC |= 2;
                    break;
                case 0xe:  // chkbit
                    m_icount -= 2;
                    t1 = get_1_ri(opcode) & 0x1f;
                    t2 = get_2_ri(opcode);
                    if (t2 & (1 << t1))
                        m_AC = (m_AC & ~7u) | 2;
                    else
                        m_AC &= ~7u;
                    break;
                default:
                    throw Fault("I960: unhandled 5a.x");
            }
            break;

        case 0x5b:
            switch ((opcode >> 7) & 0xf) {
                case 0x0: {  // addc
                    std::uint64_t res;
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    res = t2 + (t1 + ((m_AC >> 1) & 1));
                    set_ri(opcode, static_cast<std::uint32_t>(res & 0xffffffff));
                    m_AC &= ~0x3u;  // clear C and V
                    m_AC |= (res & (1ULL << 32)) ? 0x2 : 0;  // set carry
                    m_AC |= ((res ^ t1) & (res ^ t2) & 0x80000000) ? 1 : 0;  // set overflow
                    break;
                }
                case 0x2: {  // subc
                    std::uint64_t res;
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    // dst = src2 - src1 - 1 + C, or src2 + ~src1 + C
                    res = static_cast<std::uint64_t>(t2) +
                          static_cast<std::uint64_t>(static_cast<std::uint32_t>(~t1)) +
                          ((m_AC >> 1) & 1);
                    set_ri(opcode, static_cast<std::uint32_t>(res & 0xffffffff));
                    m_AC &= ~0x7u;  // cc = 0CV
                    m_AC |= (res & (1ULL << 32)) ? 0x2 : 0;  // set carry
                    m_AC |= ((t2 ^ t1) & (t2 ^ res) & 0x80000000) ? 1 : 0;  // set overflow
                    break;
                }
                default:
                    throw Fault("I960: unhandled 5b.x");
            }
            break;

        case 0x5c:
            switch ((opcode >> 7) & 0xf) {
                case 0xc:  // mov
                    m_icount -= 2;
                    t1 = get_1_ri(opcode);
                    set_ri(opcode, t1);
                    break;
                default:
                    throw Fault("I960: unhandled 5c.x");
            }
            break;

        case 0x5d:
            switch ((opcode >> 7) & 0xf) {
                case 0xc:  // movl
                    m_icount -= 2;
                    t2 = (opcode >> 19) & 0x1e;
                    if (opcode & 0x00000800) {  // literal
                        t1 = opcode & 0x1f;
                        m_r[t2] = m_r[t2 + 1] = t1;
                    } else {
                        std::memcpy(m_r + t2, m_r + (opcode & 0x1f), 2 * sizeof(std::uint32_t));
                    }
                    break;
                default:
                    throw Fault("I960: unhandled 5d.x");
            }
            break;

        case 0x5e:
            switch ((opcode >> 7) & 0xf) {
                case 0xc:  // movt
                    m_icount -= 3;
                    t2 = (opcode >> 19) & 0x1c;
                    if (opcode & 0x00000800) {  // literal
                        t1 = opcode & 0x1f;
                        m_r[t2] = m_r[t2 + 1] = m_r[t2 + 2] = t1;
                    } else {
                        std::memcpy(m_r + t2, m_r + (opcode & 0x1f), 3 * sizeof(std::uint32_t));
                    }
                    break;
                default:
                    throw Fault("I960: unhandled 5e.x");
            }
            break;

        case 0x5f:
            switch ((opcode >> 7) & 0xf) {
                case 0xc:  // movq
                    m_icount -= 4;
                    t2 = (opcode >> 19) & 0x1c;
                    if (opcode & 0x00000800) {  // literal
                        t1 = opcode & 0x1f;
                        m_r[t2] = m_r[t2 + 1] = m_r[t2 + 2] = m_r[t2 + 3] = t1;
                    } else {
                        std::memcpy(m_r + t2, m_r + (opcode & 0x1f), 4 * sizeof(std::uint32_t));
                    }
                    break;
                default:
                    throw Fault("I960: unhandled 5f.x");
            }
            break;

        case 0x60:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // synmov
                    m_icount -= 6;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    if (t1 == 0xff000004)  // interrupt control register
                        m_ICR = m_bus.read32(t2);
                    else
                        m_bus.write32(t1, m_bus.read32(t2));
                    m_AC = (m_AC & ~7u) | 2;
                    break;
                case 0x2:  // synmovq
                    m_icount -= 12;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    if (t1 == 0xff000010) {
                        send_iac(t2);
                    } else {
                        m_bus.write32(t1, m_bus.read32(t2));
                        m_bus.write32(t1 + 4, m_bus.read32(t2 + 4));
                        m_bus.write32(t1 + 8, m_bus.read32(t2 + 8));
                        m_bus.write32(t1 + 12, m_bus.read32(t2 + 12));
                    }
                    m_AC = (m_AC & ~7u) | 2;
                    break;
                default:
                    throw Fault("I960: unhandled 60.x");
            }
            break;

        case 0x64:
            switch ((opcode >> 7) & 0xf) {
                case 0x0: {  // spanbit
                    std::uint32_t res = 0xffffffff;
                    m_icount -= 10;
                    t1 = get_1_ri(opcode);
                    m_AC &= ~7u;
                    for (int i = 31; i >= 0; i--) {
                        if (!(t1 & (1u << i))) {
                            m_AC |= 2;
                            res = static_cast<std::uint32_t>(i);
                            break;
                        }
                    }
                    set_ri(opcode, res);
                    break;
                }
                case 0x1: {  // scanbit
                    std::uint32_t res = 0xffffffff;
                    m_icount -= 10;
                    t1 = get_1_ri(opcode);
                    m_AC &= ~7u;
                    for (int i = 31; i >= 0; i--) {
                        if (t1 & (1u << i)) {
                            m_AC |= 2;
                            res = static_cast<std::uint32_t>(i);
                            break;
                        }
                    }
                    set_ri(opcode, res);
                    break;
                }
                case 0x4:  // dmovt
                    m_icount -= 7;
                    t1 = get_1_ri(opcode);
                    set_ri(opcode, t1);
                    m_AC &= 0xfff8;
                    if ((t1 & 0xff) < 0x30 || (t1 & 0xff) > 0x39)
                        m_AC |= 2;
                    break;
                case 0x5:  // modac
                    m_icount -= 10;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, m_AC);
                    m_AC = (m_AC & ~t1) | (t2 & t1);
                    break;
                default:
                    throw Fault("I960: unhandled 64.x");
            }
            break;

        case 0x65:
            switch ((opcode >> 7) & 0xf) {
                case 0x5:  // modpc
                    m_icount -= 10;
                    t1 = m_PC;
                    t2 = get_2_ri(opcode);
                    m_PC = (m_PC & ~t2) | (m_r[(opcode >> 19) & 0x1f] & t2);
                    set_ri(opcode, t1);
                    if ((t1 >> 16 & 0x1f) > (m_PC >> 16 & 0x1f))
                        check_pending_irqs();
                    break;
                default:
                    throw Fault("I960: unhandled 65.x");
            }
            break;

        case 0x66:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // calls
                    t1 = get_1_ri(opcode);
                    t2 = m_bus.read32(m_SAT + 152);  // pointer to system procedure table
                    t2 = m_bus.read32(t2 + 48 + (t1 * 4));
                    if ((t2 & 3) != 0)
                        throw Fault("I960: system calls that jump into supervisor mode aren't yet supported");
                    do_call(t2, 0, m_r[R_SP]);
                    break;
                case 0xd:  // flushreg
                    if (m_rcache_pos > 4)
                        m_rcache_pos = 4;
                    for (t1 = 0; t1 < static_cast<std::uint32_t>(m_rcache_pos); t1++)
                        for (int i = 0; i < 0x10; i++)
                            m_bus.write32(m_rcache_frame_addr[t1] + (i * sizeof(std::uint32_t)),
                                          m_rcache[t1][i]);
                    m_rcache_pos = 0;
                    break;
                default:
                    throw Fault("I960: unhandled 66.x");
            }
            break;

        case 0x67:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // emul
                    m_icount -= 37;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri64(opcode, static_cast<std::uint64_t>(t1) * t2);
                    break;
                case 0x1: {  // ediv
                    m_icount -= 37;
                    const std::uint64_t src1 = get_1_ri(opcode);
                    const std::uint64_t src2 = get_2_ri64(opcode);
                    set_ri2(opcode, static_cast<std::uint32_t>(src2 % src1),
                            static_cast<std::uint32_t>(src2 / src1));
                    break;
                }
                case 0x4:  // cvtir
                    m_icount -= 30;
                    t1 = get_1_ri(opcode);
                    set_rif(opcode, static_cast<double>(static_cast<std::int32_t>(t1)));
                    break;
                case 0x5:  // cvtilr
                    m_icount -= 30;
                    t1 = get_1_ri(opcode);
                    set_rifl(opcode, static_cast<double>(static_cast<std::int32_t>(t1)));
                    break;
                case 0x6:  // scalerl
                    m_icount -= 30;
                    t1 = get_1_ri(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, t2f * std::pow(2.0, static_cast<double>(static_cast<std::int32_t>(t1))));
                    break;
                case 0x7:  // scaler
                    m_icount -= 30;
                    t1 = get_1_ri(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f * std::pow(2.0, static_cast<double>(static_cast<std::int32_t>(t1))));
                    break;
                default:
                    throw Fault("I960: unhandled 67.x");
            }
            break;

        case 0x68:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // atanr
                    m_icount -= 267;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, std::atan2(t2f, t1f));
                    break;
                case 0x1:  // logepr
                    m_icount -= 400;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f * std::log2(t1f + 1.0));
                    break;
                case 0x2:  // logr
                    m_icount -= 438;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f * std::log2(t1f));
                    break;
                case 0x3:  // remr
                    m_icount -= 67;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, std::fmod(t2f, t1f));
                    break;
                case 0x5:  // cmpr
                    m_icount -= 10;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    cmp_d(t1f, t2f);
                    break;
                case 0x8:  // sqrtr
                    m_icount -= 104;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, std::sqrt(t1f));
                    break;
                case 0x9:  // expr
                    m_icount -= 334;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, std::pow(2.0, t1f) - 1.0);
                    break;
                case 0xa:  // logbnr
                    m_icount -= 37;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, std::logb(t1f));
                    break;
                case 0xb:  // roundr
                    m_icount -= 69;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, round_to_int(t1f));
                    break;
                case 0xc:  // sinr
                    m_icount -= 406;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, std::sin(t1f));
                    break;
                case 0xd:  // cosr
                    m_icount -= 406;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, std::cos(t1f));
                    break;
                case 0xe:  // tanr
                    m_icount -= 293;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, std::tan(t1f));
                    break;
                default:
                    throw Fault("I960: unhandled 68.x");
            }
            break;

        case 0x69:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // atanrl
                    m_icount -= 350;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, std::atan2(t2f, t1f));
                    break;
                case 0x2:  // logrl
                    m_icount -= 438;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, t2f * std::log2(t1f));
                    break;
                case 0x5:  // cmprl
                    m_icount -= 12;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    cmp_d(t1f, t2f);
                    break;
                case 0x8:  // sqrtrl
                    m_icount -= 104;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, std::sqrt(t1f));
                    break;
                case 0x9:  // exprl
                    m_icount -= 334;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, std::pow(2.0, t1f) - 1.0);
                    break;
                case 0xa:  // logbnrl
                    m_icount -= 37;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, std::logb(t1f));
                    break;
                case 0xb:  // roundrl
                    m_icount -= 70;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, round_to_int(t1f));
                    break;
                case 0xc:  // sinrl
                    m_icount -= 441;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, std::sin(t1f));
                    break;
                case 0xd:  // cosrl
                    m_icount -= 441;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, std::cos(t1f));
                    break;
                case 0xe:  // tanrl
                    m_icount -= 323;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, std::tan(t1f));
                    break;
                default:
                    throw Fault("I960: unhandled 69.x");
            }
            break;

        case 0x6c:
            switch ((opcode >> 7) & 0xf) {
                case 0x0:  // cvtri
                    m_icount -= 33;
                    t1f = get_1_rif(opcode);
                    set_ri(opcode, static_cast<std::uint32_t>(static_cast<std::int32_t>(round_to_int(t1f))));
                    break;
                case 0x1:  // cvtril
                    m_icount -= 35;
                    t1f = get_1_rif(opcode);
                    set_ri64(opcode, static_cast<std::uint64_t>(static_cast<std::int64_t>(round_to_int(t1f))));
                    break;
                case 0x2:  // cvtzri
                    m_icount -= 43;
                    t1f = get_1_rif(opcode);
                    set_ri(opcode, static_cast<std::uint32_t>(static_cast<std::int32_t>(t1f)));
                    break;
                case 0x3:  // cvtzril
                    m_icount -= 44;
                    t1f = get_1_rif(opcode);
                    set_ri64(opcode, static_cast<std::uint64_t>(static_cast<std::int64_t>(t1f)));
                    break;
                case 0x9:  // movr
                    m_icount -= 5;
                    t1f = get_1_rif(opcode);
                    set_rif(opcode, t1f);
                    break;
                default:
                    throw Fault("I960: unhandled 6c.x");
            }
            break;

        case 0x6d:
            switch ((opcode >> 7) & 0xf) {
                case 0x9:  // movrl
                    m_icount -= 6;
                    t1f = get_1_rifl(opcode);
                    set_rifl(opcode, t1f);
                    break;
                default:
                    throw Fault("I960: unhandled 6d.x");
            }
            break;

        case 0x6e:
            switch ((opcode >> 7) & 0xf) {
                case 0x1:  // movre (undocumented encoding, used by DOA to save fp0-fp3)
                case 0x9: {  // movre
                    std::uint32_t v[3];
                    m_icount -= 8;
                    if (!(opcode & 0x00000800)) {
                        const int src = opcode & 0x1c;
                        v[0] = m_r[src];
                        v[1] = m_r[src + 1];
                        v[2] = m_r[src + 2] & 0xffff;  // upper 16 bits truncated
                    } else {
                        const int idx = opcode & 0x1f;
                        if (idx < 4)
                            double_to_extended(m_fp[idx], v);
                        else
                            double_to_extended((idx == 0x16) ? 1.0 : 0.0, v);
                    }

                    if (!(opcode & 0x00002000)) {
                        const int dst = (opcode >> 19) & 0x1c;
                        m_r[dst] = v[0];
                        m_r[dst + 1] = v[1];
                        m_r[dst + 2] = v[2];
                    } else if (!(opcode & 0x00e00000)) {
                        m_fp[(opcode >> 19) & 3] = extended_to_double(v);
                    } else {
                        throw Fault("I960: movre to literal");
                    }
                    break;
                }
                case 0x2:  // cpysre
                    m_icount -= 8;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    if (t2f >= 0.0)
                        set_rifl(opcode, std::fabs(t1f));
                    else
                        set_rifl(opcode, -std::fabs(t1f));
                    break;
                default:
                    throw Fault("I960: unhandled 6e.x");
            }
            break;

        case 0x70:
            switch ((opcode >> 7) & 0xf) {
                case 0x1:  // mulo
                    m_icount -= 18;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 * t1);
                    break;
                case 0x8:  // remo
                    m_icount -= 37;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t2 % t1);
                    break;
                case 0xb:  // divo
                    m_icount -= 37;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, t1 == 0 ? 0 : t2 / t1);  // HACK: avoid #DE
                    break;
                default:
                    throw Fault("I960: unhandled 70.x");
            }
            break;

        case 0x74:
            switch ((opcode >> 7) & 0xf) {
                case 0x1:  // muli
                    m_icount -= 18;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, static_cast<std::uint32_t>(
                                      static_cast<std::int32_t>(t2) * static_cast<std::int32_t>(t1)));
                    break;
                case 0x8:  // remi
                    m_icount -= 37;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, static_cast<std::uint32_t>(
                                      static_cast<std::int32_t>(t2) % static_cast<std::int32_t>(t1)));
                    break;
                case 0x9: {  // modi
                    m_icount -= 37;
                    const std::int32_t src1 = static_cast<std::int32_t>(get_1_ri(opcode));
                    const std::int32_t src2 = static_cast<std::int32_t>(get_2_ri(opcode));
                    std::int32_t dst = src2 - ((src2 / src1) * src1);
                    if (((src1 ^ src2) < 0) && (dst != 0))
                        dst += src1;
                    set_ri(opcode, static_cast<std::uint32_t>(dst));
                    break;
                }
                case 0xb:  // divi
                    m_icount -= 37;
                    t1 = get_1_ri(opcode);
                    t2 = get_2_ri(opcode);
                    set_ri(opcode, static_cast<std::uint32_t>(
                                      static_cast<std::int32_t>(t2) / static_cast<std::int32_t>(t1)));
                    break;
                default:
                    throw Fault("I960: unhandled 74.x");
            }
            break;

        case 0x78:
            switch ((opcode >> 7) & 0xf) {
                case 0xb:  // divr
                    m_icount -= 35;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f / t1f);
                    break;
                case 0xc:  // mulr
                    m_icount -= 18;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f * t1f);
                    break;
                case 0xd:  // subr
                    m_icount -= 10;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f - t1f);
                    break;
                case 0xf:  // addr
                    m_icount -= 10;
                    t1f = get_1_rif(opcode);
                    t2f = get_2_rif(opcode);
                    set_rif(opcode, t2f + t1f);
                    break;
                default:
                    throw Fault("I960: unhandled 78.x");
            }
            break;

        case 0x79:
            switch ((opcode >> 7) & 0xf) {
                case 0xb:  // divrl
                    m_icount -= 77;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, t2f / t1f);
                    break;
                case 0xc:  // mulrl
                    m_icount -= 36;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, t2f * t1f);
                    break;
                case 0xd:  // subrl
                    m_icount -= 13;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, t2f - t1f);
                    break;
                case 0xf:  // addrl
                    m_icount -= 13;
                    t1f = get_1_rifl(opcode);
                    t2f = get_2_rifl(opcode);
                    set_rifl(opcode, t2f + t1f);
                    break;
                default:
                    break;
            }
            break;

        case 0x71: case 0x72: case 0x73:
        case 0x75: case 0x76: case 0x77: case 0x7a: case 0x7b: case 0x7c: case 0x7d:
        case 0x7e: case 0x7f:
            // Bring-up stubs for missing Model 2 i960 opcode groups
            m_icount--;
            break;

        case 0x80: {  // ldob
            m_icount -= 4;
            const std::uint8_t v = m_bus.read8(get_ea(opcode));
            if (!m_stalled)
                m_r[(opcode >> 19) & 0x1f] = v;
            break;
        }

        case 0x82:  // stob
            m_icount -= 2;
            m_bus.write8(get_ea(opcode), static_cast<std::uint8_t>(m_r[(opcode >> 19) & 0x1f]));
            break;

        case 0x84:  // bx
            m_icount -= 3;
            m_IP = get_ea(opcode);
            break;

        case 0x85:  // balx
            m_icount -= 5;
            t1 = get_ea(opcode);
            m_r[(opcode >> 19) & 0x1f] = m_IP;
            m_IP = t1;
            break;

        case 0x86:  // callx
            t1 = get_ea(opcode);
            do_call(t1, 0, m_r[R_SP]);
            break;

        case 0x88: {  // ldos
            m_icount -= 4;
            const std::uint16_t v = read_word_unaligned(get_ea(opcode));
            if (!m_stalled)
                m_r[(opcode >> 19) & 0x1f] = v;
            break;
        }

        case 0x8a:  // stos
            m_icount -= 2;
            write_word_unaligned(get_ea(opcode), static_cast<std::uint16_t>(m_r[(opcode >> 19) & 0x1f]));
            break;

        case 0x8c:  // lda
            m_icount--;
            m_r[(opcode >> 19) & 0x1f] = get_ea(opcode);
            break;

        case 0x90: {  // ld
            m_icount -= 4;
            const std::uint32_t v = read_dword_unaligned(get_ea(opcode));
            if (!m_stalled)
                m_r[(opcode >> 19) & 0x1f] = v;
            break;
        }

        case 0x92:  // st
            m_icount -= 2;
            write_dword_unaligned(get_ea(opcode), m_r[(opcode >> 19) & 0x1f]);
            break;

        case 0x98: {  // ldl
            m_icount -= 5;
            t1 = get_ea(opcode);
            t2 = (opcode >> 19) & 0x1e;
            for (int i = 0; i < 2; i++) {
                const auto pack = read_dword_unaligned_flags(t1);
                if (m_stalled) {
                    m_stall_state = {t1, t2, i, 2, true, false};
                    return;
                }
                m_r[t2 + i] = pack.first;
                if (pack.second & kBurst)
                    t1 += 4;
            }
            break;
        }

        case 0x9a: {  // stl
            m_icount -= 3;
            t1 = get_ea(opcode);
            t2 = (opcode >> 19) & 0x1e;
            for (int i = 0; i < 2; i++) {
                const std::uint16_t flags = write_dword_unaligned_flags(t1, m_r[t2 + i]);
                if (m_stalled) {
                    m_stall_state = {t1, t2, i, 2, true, true};
                    return;
                }
                if (flags & kBurst)
                    t1 += 4;
            }
            break;
        }

        case 0xa0: {  // ldt
            m_icount -= 6;
            t1 = get_ea(opcode);
            t2 = (opcode >> 19) & 0x1c;
            for (int i = 0; i < 3; i++) {
                const auto pack = read_dword_unaligned_flags(t1);
                if (m_stalled) {
                    m_stall_state = {t1, t2, i, 3, true, false};
                    return;
                }
                m_r[t2 + i] = pack.first;
                if (pack.second & kBurst)
                    t1 += 4;
            }
            break;
        }

        case 0xa2: {  // stt
            m_icount -= 4;
            t1 = get_ea(opcode);
            t2 = (opcode >> 19) & 0x1c;
            for (int i = 0; i < 3; i++) {
                const std::uint16_t flags = write_dword_unaligned_flags(t1, m_r[t2 + i]);
                if (m_stalled) {
                    m_stall_state = {t1, t2, i, 3, true, true};
                    return;
                }
                if (flags & kBurst)
                    t1 += 4;
            }
            break;
        }

        case 0xb0: {  // ldq
            m_icount -= 7;
            t1 = get_ea(opcode);
            t2 = (opcode >> 19) & 0x1c;
            for (int i = 0; i < 4; i++) {
                const auto pack = read_dword_unaligned_flags(t1);
                if (m_stalled) {
                    m_stall_state = {t1, t2, i, 4, true, false};
                    return;
                }
                m_r[t2 + i] = pack.first;
                if (pack.second & kBurst)
                    t1 += 4;
            }
            break;
        }

        case 0xb2: {  // stq
            m_icount -= 5;
            t1 = get_ea(opcode);
            t2 = (opcode >> 19) & 0x1c;
            for (int i = 0; i < 4; i++) {
                const std::uint16_t flags = write_dword_unaligned_flags(t1, m_r[t2 + i]);
                if (m_stalled) {
                    m_stall_state = {t1, t2, i, 4, true, true};
                    return;
                }
                if (flags & kBurst)
                    t1 += 4;
            }
            break;
        }

        case 0xc0: {  // ldib
            m_icount -= 4;
            const std::int8_t v = static_cast<std::int8_t>(m_bus.read8(get_ea(opcode)));
            if (!m_stalled)
                m_r[(opcode >> 19) & 0x1f] = static_cast<std::uint32_t>(static_cast<std::int32_t>(v));
            break;
        }

        case 0xc2:  // stib
            m_icount -= 2;
            m_bus.write8(get_ea(opcode), static_cast<std::uint8_t>(m_r[(opcode >> 19) & 0x1f]));
            break;

        case 0xc8: {  // ldis
            m_icount -= 4;
            const std::int16_t v = static_cast<std::int16_t>(read_word_unaligned(get_ea(opcode)));
            if (!m_stalled)
                m_r[(opcode >> 19) & 0x1f] = static_cast<std::uint32_t>(static_cast<std::int32_t>(v));
            break;
        }

        case 0xca:  // stis
            m_icount -= 2;
            write_word_unaligned(get_ea(opcode), static_cast<std::uint16_t>(m_r[(opcode >> 19) & 0x1f]));
            break;

        default:
            // Bring-up: treat unimplemented Model 2 i960 opcodes as nop
            m_icount--;
            break;
    }
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

int I960::step() {
    m_icount = 1;

    if (!m_stall_state.burst_mode)
        check_immediate_irqs();

    if (m_icount > 0) {
        m_PIP = m_IP;

        const std::uint32_t opcode = m_bus.read32(m_IP);
        m_IP += 4;

        m_stalled = false;

        if (m_stall_state.burst_mode)
            execute_burst_stall_op(opcode);
        else
            execute_op(opcode);
    }

    return 1 - m_icount;
}

void I960::reset() {
    m_SAT = m_bus.read32(0);
    m_PRCB = m_bus.read32(4);
    m_IP = m_bus.read32(12);
    m_PC = 0x001f2002;
    m_AC = 0;
    m_ICR = 0xff000000;
    m_immediate_irq = 0;

    std::memset(m_r, 0, sizeof(m_r));
    std::memset(m_rcache, 0, sizeof(m_rcache));

    m_r[R_FP] = m_bus.read32(m_PRCB + 24);
    m_r[R_SP] = m_r[R_FP] + 64;
    m_rcache_pos = 0;

    m_stall_state.burst_mode = false;
}

void I960::set_input(int irqline, int state) {
    if (m_irq_line_state[irqline] == state)
        return;

    m_irq_line_state[irqline] = static_cast<std::int8_t>(state);

    const int int_tab = m_bus.read32(m_PRCB + 20);  // interrupt table
    const int cpu_pri = (m_PC >> 16) & 0x1f;
    int vector = 0;
    int priority;
    std::uint32_t pend, word, wordofs;

    // Only the 4 external IRQ lines in "normal" mode are supported, matching
    // the subset Namco and Sega hardware actually use.
    switch (irqline) {
        case 0: vector = m_ICR & 0xff; break;
        case 1: vector = (m_ICR >> 8) & 0xff; break;
        case 2: vector = (m_ICR >> 16) & 0xff; break;
        case 3: vector = (m_ICR >> 24) & 0xff; break;
    }

    if (!vector)
        return;  // interrupt line in IAC mode, unsupported

    priority = vector / 8;

    if (state) {
        if (((cpu_pri < priority) || (priority == 31)) && (m_immediate_irq == 0)) {
            m_immediate_irq = 1;
            m_immediate_vector = vector;
            m_immediate_pri = priority;
            ++m_irq_immediate_count;
        } else {
            // store the interrupt in the "pending" table
            ++m_irq_pending_count;
            pend = m_bus.read32(int_tab);
            pend |= (1 << priority);
            m_bus.write32(int_tab, pend);

            // bitfield-ize the vector
            word = ((vector / 32) * 4) + 4;
            wordofs = vector % 32;
            pend = m_bus.read32(int_tab + word);
            pend |= (1 << wordofs);
            m_bus.write32(int_tab + word, pend);
        }
    }
}

std::string I960::disassemble(std::uint32_t pc) const {
    return disassemble_i960(m_bus, pc);
}

}  // namespace sage::cpu

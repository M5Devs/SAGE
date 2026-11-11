// Fujitsu MB86233 / MB86234 "TGP" geometry coprocessor.
//
// Ported from MAME's src/devices/cpu/mb86233/mb86233.cpp and the Model 2 /
// Model 2A integration in src/mame/sega/model2.cpp (both BSD-3-Clause,
// copyright-holders Olivier Galibert, R. Belmont, ElSemi, Angelo Salese,
// Matthew Daniels), building on Elsemi's original reverse engineering. The
// wrapper structure follows the sm2-emu reference port. The bodies keep MAME's
// spelling so they stay diffable against upstream.
//
// MAME's own notes on the hardware, kept because they record findings that are
// not deducible from the code:
//
//   The 86232 has 512 32-bit words of triple-port memory (1 write, 2 read). The
//   86233 and 86234 have instead two normal (1 read, 1 write, non-simultaneous)
//   independent RAM banks, one of 256 words and one of 512.
//
//   The RAM banks are mapped at 0x000-0x0ff and 0x200-0x3ff, which is proven by
//   geometrizer code that clears the RAM at startup. Move and load instructions
//   sort of target a specific bank, but do it by adding 0x200 to the address on
//   one side or the other, which can then end up anywhere. In particular the
//   Model 1 coprocessor has its output FIFO at 0x400, which is sometimes hit by
//   having x1 at 0x200 and using the automatic 0x200 adder. External accesses to
//   0x100-0x1ff and 0x400 upwards appear to be routed externally.
//
//   The part can in theory work in either floating point (32-bit IEEE) or fixed
//   point (32/36/48-bit) modes. Every Sega program starts by selecting floating
//   point and stays there, so fixed point is not implemented.
//
//   An interrupt updates the rf0 register in the coprocessor programs. It is on
//   bit 1 of the mask, vector 0x004, and is probably periodic, maybe on vertical
//   blank. The coprocessor programs never initialise the stack pointer.
//   Interrupts are not implemented.
//
//   In the event of two writes to the same register in one instruction, priority
//   varies with the ALU operation: transfers take precedence over integer ops,
//   while floating point ops take precedence over transfers. This is probably
//   because a floating point op takes more than one cycle, so the register is
//   updated after the transfer.

#include "sage/cpu/tgp.h"




#include <algorithm>
#include <cmath>
#include <cstdio>

namespace sage::cpu::mb86233 {

namespace {

// The TGP has no NaN or infinity: exponent 255 is the largest magnitude,
// overflow saturates and 0/0 gives 0.
[[nodiscard]] inline float tgp_float(u32 v)
{
    if (((v >> 23) & 0xff) == 0xff) {
        v = (v & 0x80000000) | 0x7f7fffff;
    }
    return u2f(v);
}

[[nodiscard]] inline u32 tgp_bits(float f)
{
    u32 v = f2u(f);
    if (((v >> 23) & 0xff) == 0xff) {
        v = (v & 0x007fffff) ? 0 : ((v & 0x80000000) | 0x7f7fffff);
    }
    return v;
}

} // namespace

Tgp::Tgp(TgpBus& bus) : m_bus(&bus)
{
    reset();
}

void Tgp::reset()
{
    m_pc  = 0;
    m_ppc = 0;
    m_st  = kZeroC | kZeroD | kZeroX0 | kZeroX1 | kZeroX2 | kZeroC0 | kZeroC1;
    m_sp  = 0;
    m_a   = 0;
    m_b   = 0;
    m_d   = 0;
    m_p   = 0;
    // The repeat counters come up at one, not zero: a value of one means "do not
    // repeat", so zero would mean an instruction runs 256 times.
    m_r    = 1;
    m_rpc  = 1;
    m_c0   = 1;
    m_c1   = 1;
    m_b0   = 0;
    m_b1   = 0;
    m_x0   = 0;
    m_x1   = 0;
    m_i0   = 0;
    m_i1   = 0;
    m_sft  = 0;
    m_vsm  = 0;
    m_vsmr = 7;
    m_mask = 0;
    m_m    = 1;

    m_alu_stmask = 0;
    m_alu_stset  = 0;
    m_alu_r1     = 0;
    m_alu_r2     = 0;
    std::fill(std::begin(m_pcs), std::end(m_pcs), u16{0});

    m_stall = false;

    m_gpio0 = m_gpio1 = m_gpio2 = m_gpio3 = false;
}

void Tgp::set_gpio(u32 index, bool state)
{
    switch (index) {
        case 0: m_gpio0 = state; break;
        case 1: m_gpio1 = state; break;
        case 2: m_gpio2 = state; break;
        case 3: m_gpio3 = state; break;
        default: break;
    }
}

s32 Tgp::run(s32 cycles)
{
    if (m_halted || cycles <= 0) {
        // Consume the whole budget: a halted part is not going to make progress
        // and the caller should not spin trying.
        return cycles;
    }

    const s32 before = cycles;
    m_icount         = cycles;
    execute_run();
    const s32 used = before - m_icount;
    m_instructions += static_cast<u64>(used > 0 ? used : 0);
    return used;
}

std::string Tgp::state_string() const
{
    char buffer[256];
    std::snprintf(buffer, sizeof(buffer),
                  "PC=%04x PPC=%04x ST=%08x SP=%04x A=%08x B=%08x D=%08x P=%08x "
                  "R=%02x X0=%04x X1=%04x I0=%04x I1=%04x",
                  m_pc, m_ppc, m_st, m_sp, m_a, m_b, m_d, m_p, m_r, m_x0, m_x1, m_i0,
                  m_i1);
    return buffer;
}

void Tgp::serialize(Archive& ar)
{
    ar.raw(m_st);
    ar.raw(m_a);
    ar.raw(m_b);
    ar.raw(m_d);
    ar.raw(m_p);
    ar.raw(m_alu_stmask);
    ar.raw(m_alu_stset);
    ar.raw(m_alu_r1);
    ar.raw(m_alu_r2);
    ar.raw(m_ppc);
    ar.raw(m_pc);
    ar.raw(m_sp);
    ar.raw(m_b0);
    ar.raw(m_b1);
    ar.raw(m_x0);
    ar.raw(m_x1);
    ar.raw(m_i0);
    ar.raw(m_i1);
    ar.raw(m_vsmr);
    ar.bytes(m_pcs, 4);
    ar.raw(m_mask);
    ar.raw(m_m);
    ar.raw(m_r);
    ar.raw(m_rpc);
    ar.raw(m_c0);
    ar.raw(m_c1);
    ar.raw(m_sft);
    ar.raw(m_vsm);
    ar.raw(m_gpio0);
    ar.raw(m_gpio1);
    ar.raw(m_gpio2);
    ar.raw(m_gpio3);
    ar.raw(m_stall);
    ar.raw(m_halted);
    ar.raw(m_instructions);
}

u32 Tgp::set_exp(u32 val, u32 exp)
{
	return (val & 0x807fffff) | ((exp & 0xff) << 23);
}

u32 Tgp::set_mant(u32 val, u32 mant)
{
	return (val & 0x07f800000) | ((mant & 0x00800000) << 8) | (mant & 0x007fffff);
}

u32 Tgp::get_exp(u32 val)
{
	return (val >> 23) & 0xff;
}

u32 Tgp::get_mant(u32 val)
{
	return val & 0x80000000 ? val | 0x7f800000 : val & 0x807fffff;
}

void Tgp::pcs_push()
{
	for(unsigned int i=3; i; i--)
		m_pcs[i] = m_pcs[i-1];
	m_pcs[0] = m_pc;
}

void Tgp::pcs_pop()
{
	m_pc = m_pcs[0];
	for(unsigned int i=0; i != 3; i++)
		m_pcs[i] = m_pcs[i+1];
}

void Tgp::stset_set_sz_int(u32 val)
{
	m_alu_stset = val ? (val & 0x80000000 ? F_SGD : 0) : F_ZRD;
}

void Tgp::stset_set_sz_fp(u32 val)
{
	m_alu_stset = (val & 0x7fffffff) ? (val & 0x80000000 ? F_SGD : 0) : F_ZRD;
}

void Tgp::alu_pre(u32 alu)
{
	switch(alu) {
	case 0x00: break; // no alu

	case 0x01: {
		// andd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d & m_a;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x02: {
		// orad
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d | m_a;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x03: {
		// eord
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d ^ m_a;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x04: {
		// notd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = ~m_d;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x05: {
		// fcpd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		u32 r = tgp_bits(tgp_float(m_d) - tgp_float(m_a));
		stset_set_sz_fp(r);
		break;
	}

	case 0x06: {
		// fadd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_d) + tgp_float(m_a));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x07: {
		// fsbd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_d) - tgp_float(m_a));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x08: {
		// fml
		m_alu_stmask = 0;
		m_alu_r1 = tgp_bits(tgp_float(m_a) * tgp_float(m_b));
		m_alu_stset = 0;
		break;
	}

	case 0x09: {
		// fmsd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_d) + tgp_float(m_p));
		m_alu_r2 = tgp_bits(tgp_float(m_a) * tgp_float(m_b));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x0a: {
		// fmrd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_d) - tgp_float(m_p));
		m_alu_r2 = tgp_bits(tgp_float(m_a) * tgp_float(m_b));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x0b: {
		// fabd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d & 0x7fffffff;
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x0c: {
		// fsmd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_d) + tgp_float(m_p));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x0d: {
		// fspd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_p;
		m_alu_r2 = tgp_bits(tgp_float(m_a) * tgp_float(m_b));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x0e: {
		// cxfd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(s32(m_d));
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x0f: {
		// cfxd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		switch((m_m >> 1) & 3) {
		case 0: m_alu_r1 = s32(roundf(tgp_float(m_d))); break;
		case 1: m_alu_r1 = s32(ceilf(tgp_float(m_d))); break;
		case 2: m_alu_r1 = s32(floorf(tgp_float(m_d))); break;
		case 3: m_alu_r1 = s32(tgp_float(m_d)); break;
		}
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x10: {
		// fdvd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_d) / tgp_float(m_a));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x11: {
		// fned
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d ? m_d ^ 0x80000000 : 0;
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x13: {
		// d = b + a
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_b) + tgp_float(m_a));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x14: {
		// d = b - a
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = tgp_bits(tgp_float(m_b) - tgp_float(m_a));
		stset_set_sz_fp(m_alu_r1);
		break;
	}

	case 0x16: {
		// lsrd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d >> m_sft;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x17: {
		// lsld
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d << m_sft;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x18: {
		// asrd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = s32(m_d) >> m_sft;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x19: {
		// asld
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = s32(m_d) << m_sft;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x1a: {
		// addd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d + m_a;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	case 0x1b: {
		// subd
		m_alu_stmask = F_ZRD|F_SGD|F_CPD|F_OVD|F_DVZD;
		m_alu_r1 = m_d - m_a;
		stset_set_sz_int(m_alu_r1);
		break;
	}

	default:
		SM2_WARN("unhandled alu pre %02x\n", alu);
		break;
	}
}

void Tgp::alu_update_st()
{
	m_st = (m_st & ~m_alu_stmask) | m_alu_stset;
}

void Tgp::alu_post_1(u32 alu)
{
	// integer alu post ops
	switch(alu) {
	case 0x01: case 0x02: case 0x03: case 0x04:
	case 0x0e: case 0x0f: case 0x16: case 0x17:
	case 0x18: case 0x19: case 0x1a: case 0x1b:
		// d update
		m_d = m_alu_r1;
		alu_update_st();
		break;

	default:
		break;
	}
}

void Tgp::alu_post_2(u32 alu)
{
	// floating point alu post ops
	// assume each one takes 2 cycles
	switch (alu) {
	case 0x05:
		// flags only
		alu_update_st();
		m_icount--;
		break;

	case 0x06: case 0x07: case 0x0b: case 0x0c:
	case 0x10: case 0x11: case 0x13: case 0x14:
		// d update
		m_d = m_alu_r1;
		alu_update_st();
		m_icount--;
		break;

	case 0x08:
		// p update
		m_p = m_alu_r1;
		m_icount--;
		break;

	case 0x09: case 0x0a: case 0x0d:
		// d, p update
		m_d = m_alu_r1;
		m_p = m_alu_r2;
		alu_update_st();
		m_icount--;
		break;

	default:
		break;
	}
}

u16 Tgp::ea_pre_0(u32 r)
{
	switch(r & 0x180) {
	case 0x000: return r & 0x7f;
	case 0x080: case 0x100: return (r & 0x7f) + m_b0 + m_x0;
	case 0x180: {
		switch(r & 0x60) {
		case 0x00: return m_b0 + m_x0;
		case 0x20: return m_x0;
		case 0x40: return m_b0 + (m_x0 & m_vsmr);
		case 0x60: return m_x0 & m_vsmr;
		}
	}
	}
	return 0;
}

void Tgp::ea_post_0(u32 r)
{
	if(!(r & 0x100))
		return;
	if(!(r & 0x080))
		m_x0 += m_i0;
	else
		m_x0 += sign_extend(r, 5);
}

u16 Tgp::ea_pre_1(u32 r)
{
	switch(r & 0x180) {
	case 0x000: return r & 0x7f;
	case 0x080: case 0x100: return (r & 0x7f) + m_b1 + m_x1;
	case 0x180: {
		switch(r & 0x60) {
		case 0x00: return m_b1 + m_x1;
		case 0x20: return m_x1;
		case 0x40: return m_b1 + (m_x1 & m_vsmr);
		case 0x60: return m_x1 & m_vsmr;
		}
	}
	}
	return 0;
}

void Tgp::ea_post_1(u32 r)
{
	if(!(r & 0x100))
		return;
	if(!(r & 0x080))
		m_x1 += m_i1;
	else
		m_x1 += sign_extend(r, 5);
}

u32 Tgp::read_reg(u32 r)
{
	r &= 0x3f;
	if(r >= 0x20 && r < 0x30)
		return m_bus->read_rf(r & 0x1f);
	switch(r) {
	case 0x00: return m_b0;
	case 0x01: return m_b1;
	case 0x02: return m_x0;
	case 0x03: return m_x1;

	case 0x0c: return m_c0;
	case 0x0d: return m_c1;

	case 0x10: return m_a;
	case 0x11: return get_exp(m_a);
	case 0x12: return get_mant(m_a);
	case 0x13: return m_b;
	case 0x14: return get_exp(m_b);
	case 0x15: return get_mant(m_b);
	case 0x19: return m_d;
		/* c */
	case 0x1a: return get_exp(m_d);
	case 0x1b: return get_mant(m_d);
	case 0x1c: return m_p;
	case 0x1d: return get_exp(m_p);
	case 0x1e: return get_mant(m_p);
	case 0x1f: return m_sft;

	case 0x34: return m_rpc;

	default:
		SM2_WARN("unimplemented read_reg(%02x) (%x)\n", r, m_ppc);
		return 0;
	}
}

void Tgp::write_reg(u32 r, u32 v)
{
	r &= 0x3f;
	if(r >= 0x20 && r < 0x30) {
		m_bus->write_rf(r & 0x1f, v);
		return;
	}
	switch(r) {
	case 0x00: m_b0 = v; break;
	case 0x01: m_b1 = v; break;
	case 0x02: m_x0 = v; break;
	case 0x03: m_x1 = v; break;

	case 0x05: m_i0 = v; break;
	case 0x06: m_i1 = v; break;

	case 0x08: m_sp = v; break;

	case 0x0a: m_vsm = v & 7; m_vsmr = (8 << m_vsm) - 1; break;

	case 0x0c:
		m_c0 = v;
		if(m_c0 == 1)
			m_st |= F_ZC0;
		else
			m_st &= ~F_ZC0;
		break;

	case 0x0d:
		m_c1 = v;
		if(m_c1 == 1)
			m_st |= F_ZC1;
		else
			m_st &= ~F_ZC1;
		break;

	case 0x0f: break;

	case 0x10: m_a = v; break;
	case 0x11: m_a = set_exp(m_a, v); break;
	case 0x12: m_a = set_mant(m_a, v); break;
	case 0x13: m_b = v; break;
	case 0x14: m_b = set_exp(m_b, v); break;
	case 0x15: m_b = set_mant(m_b, v); break;
		/* c */
	case 0x19: m_d = v; break;
	case 0x1a: m_d = set_exp(m_d, v); break;
	case 0x1b: m_d = set_mant(m_d, v); break;
	case 0x1c: m_p = v; break;
	case 0x1d: m_p = set_exp(m_p, v); break;
	case 0x1e: m_p = set_mant(m_p, v); break;
	case 0x1f: m_sft = v; break;

	case 0x34: m_rpc = v; break;
	case 0x3c: m_mask = v; break;

	default:
		SM2_WARN("unimplemented write_reg(%02x, %08x) (%x)\n", r, v, m_ppc);
		break;
	}
}

void Tgp::write_mem_internal_1(u32 r, u32 v, bool bank)
{
	u16 ea = ea_pre_1(r);
	if(bank)
		ea += 0x200;
	m_bus->write_data(ea, v);
	ea_post_1(r);
}

void Tgp::write_mem_io_1(u32 r, u32 v)
{
	u16 ea = ea_pre_1(r);
	m_bus->write_io(ea, v);
	ea_post_1(r);
}

void Tgp::execute_run()
{
	while(m_icount > 0) {
		// Halt can be asserted part-way through a slice by a fifo callback, so it
		// is checked here rather than only on entry.
		if(m_halted)
			break;
		m_ppc = m_pc;
		if(m_trace_hook)
			m_trace_hook(m_trace_context, m_ppc);
		u32 opcode = m_bus->fetch(m_pc++);

		switch((opcode >> 26) & 0x3f) {
		case 0x00: {
			// lab
			u32 r1 = opcode & 0x1ff;
			u32 r2 = (opcode >> 9) & 0x1ff;
			u32 alu = (opcode >> 21) & 0x1f;
			u32 op = (opcode >> 18) & 0x7;

			alu_pre(alu);

			switch(op) {
			case 0: case 1: {
				// lab mem, mem (e)

				u32 ea1 = ea_pre_0(r1);
				u32 v1 = m_bus->read_data(ea1);
				if(m_stall) goto do_stall;

				u32 ea2 = ea_pre_1(r2);
				u32 v2 = m_bus->read_io(ea2);
				if(m_stall) goto do_stall;

				ea_post_0(r1);
				ea_post_1(r2);

				m_a = v1;
				m_b = v2;
				break;
			}

			case 3: {
				// lab mem, mem + 0x200

				u32 ea1 = ea_pre_0(r1);
				u32 v1 = m_bus->read_data(ea1);
				if(m_stall) goto do_stall;

				u32 ea2 = ea_pre_1(r2) + 0x200;
				u32 v2 = m_bus->read_data(ea2);
				if(m_stall) goto do_stall;

				ea_post_0(r1);
				ea_post_1(r2);

				m_a = v1;
				m_b = v2;
				break;
			}

			case 4: {
				// lab mem + 0x200, mem

				u32 ea1 = ea_pre_0(r1) + 0x200;
				u32 v1 = m_bus->read_data(ea1);
				if(m_stall) goto do_stall;

				u32 ea2 = ea_pre_1(r2);
				u32 v2 = m_bus->read_data(ea2);
				if(m_stall) goto do_stall;

				ea_post_0(r1);
				ea_post_1(r2);

				m_a = v1;
				m_b = v2;
				break;
			}

			default:
				SM2_WARN("unhandled lab subop %x\n", op);
				SM2_WARN("%x\n", m_ppc);
				break;

			}

			alu_post_1(alu);
			alu_post_2(alu);
			break;
		}


		case 0x07: {
			// ld / mov
			u32 r1 = opcode & 0x1ff;
			u32 r2 = (opcode >> 9) & 0x1ff;
			u32 alu = (opcode >> 21) & 0x1f;
			u32 op = (opcode >> 18) & 0x7;

			alu_pre(alu);

			switch(op) {
			case 0: {
				// mov mem, mem (e)
				u32 ea = ea_pre_0(r1);
				u32 v = m_bus->read_data(ea);
				if(m_stall) goto do_stall;
				ea_post_0(r1);
				alu_post_1(alu);
				write_mem_io_1(r2, v);
				break;
			}

			case 1: {
				// mov mem, mem (e)
				u32 ea = ea_pre_0(r1);
				u32 v = m_bus->read_data(ea);
				if(m_stall) goto do_stall;
				ea_post_0(r1);
				alu_post_1(alu);
				write_mem_io_1(r2, v);
				break;
			}

			case 2: {
				// mov mem (e), mem
				u32 ea = ea_pre_0(r1);
				u32 v = m_bus->read_io(ea);
				if(m_stall) goto do_stall;
				ea_post_0(r1);
				alu_post_1(alu);
				write_mem_internal_1(r2, v, false);
				break;
			}

			case 3: {
				// mov mem, mem + 0x200
				u32 ea = ea_pre_0(r1);
				u32 v = m_bus->read_data(ea);
				if(m_stall) goto do_stall;
				ea_post_0(r1);
				alu_post_1(alu);
				write_mem_internal_1(r2, v, true);
				break;
			}

			case 4: {
				// mov mem + 0x200, mem
				u32 ea = ea_pre_0(r1) + 0x200;
				u32 v = m_bus->read_data(ea);
				if(m_stall) goto do_stall;
				ea_post_0(r1);
				alu_post_1(alu);
				write_mem_internal_1(r2, v, false);
				break;
			}

			case 5: {
				// mov mem (o), mem
				u32 ea = ea_pre_0(r1);
				u32 v = m_bus->read_program(ea);
				if(m_stall) goto do_stall;
				ea_post_0(r1);
				alu_post_1(alu);
				write_mem_internal_1(r2, v, false);
				break;
			}

			case 7: {
				switch(r2 >> 6) {
				case 0: {
					// mov reg, mem
					u32 v = read_reg(r2);
					if(m_stall) goto do_stall;
					alu_post_1(alu);
					write_mem_internal_1(r1, v, false);
					break;
				}

				case 1: {
					// mov reg, mem (e)
					u32 v = read_reg(r2);
					if(m_stall) goto do_stall;
					alu_post_1(alu);
					write_mem_io_1(r1, v);
					break;
				}

				case 2: {
					// mov mem + 0x200, reg
					u32 ea = ea_pre_1(r1) + 0x200;
					u32 v = m_bus->read_data(ea);
					if(m_stall) goto do_stall;
					ea_post_1(r1);
					alu_post_1(alu);
					write_reg(r2, v);
					break;
				}

				case 3: {
					// mov mem, reg
					u32 ea = ea_pre_1(r1);
					u32 v = m_bus->read_data(ea);
					if(m_stall) goto do_stall;
					ea_post_1(r1);
					alu_post_1(alu);
					write_reg(r2, v);
					break;
				}

				case 4: {
					// mov mem (e), reg
					u32 ea = ea_pre_1(r1);
					u32 v = m_bus->read_io(ea);
					if(m_stall) goto do_stall;
					ea_post_1(r1);
					alu_post_1(alu);
					write_reg(r2, v);
					break;
				}

				case 5: {
					// mov mem (o), reg
					u32 ea = ea_pre_0(r1);
					u32 v = m_bus->read_program(ea);
					if(m_stall) goto do_stall;
					ea_post_0(r1);
					alu_post_1(alu);
					write_reg(r2, v);
					break;
				}

				case 6: {
					// mov reg, reg
					u32 v = read_reg(r1);
					if(m_stall) goto do_stall;
					alu_post_1(alu);
					write_reg(r2, v);
					break;
				}

				default:
					alu_post_1(alu);
					SM2_WARN("unhandled ld/mov subop 7/%x (%x)\n", r2 >> 6, m_ppc);
					break;
				}
				break;
			}

			default:
				alu_post_1(alu);
				SM2_WARN("unhandled ld/mov subop %x (%x)\n", op, m_ppc);
				break;
			}

			// For floating point ops, registers are updated after transfer
			alu_post_2(alu);
			break;
		}

		case 0x0d: {
			// stm/clm
			u32 sub2 = (opcode >> 17) & 7;

			// Theorically has restricted alu too

			switch(sub2) {
			case 5:
				// stmh
				// bit 0 = floating point
				// bit 1-2 = rounding mode
				m_m = opcode;
				break;

			default:
				SM2_WARN("unimplemented opcode 0d/%x (%x)\n", sub2, m_ppc);
				break;
			}
			break;
		}

		case 0x0e: {
			// lipl / lia / lib / lid
			switch((opcode >> 24) & 0x3) {
			case 0:
				m_p = (m_p & 0xffffff000000) | (opcode & 0xffffff);
				break;
			case 1:
				m_a = sign_extend(opcode, 24);
				break;
			case 2:
				m_b = sign_extend(opcode, 24);
				break;
			case 3:
				m_d = sign_extend(opcode, 24);
				break;
			}
			break;
		}

		case 0x0f: {
			// rep/clr0/clr1/set
			u32 alu = (opcode >> 20) & 0x1f;
			u32 sub2 = (opcode >> 17) & 7;

			alu_pre(alu);

			switch(sub2) {
			case 0:
				// clr0
				if(opcode & 0x0004) m_a = 0;
				if(opcode & 0x0008) m_b = 0;
				if(opcode & 0x0010) m_d = 0;
				break;

			case 1:
				// clr1 - flags mapping unknown
				break;

			case 2: {
				// rep
				u8 r = opcode & 0x8000 ? read_reg(opcode) : opcode;
				if(m_stall) goto do_stall;
				m_r = r;
				goto rep_start;
			}

			case 3:
				// set - flags mapping unknown
				// 0800 = enable interrupt flag
				break;

			default:
				SM2_WARN("unimplemented opcode 0f/%x (%x)\n", sub2, m_ppc);
				break;
			}

			alu_post_1(alu);
			break;
		}

		case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f: {
			// ldi
			write_reg(opcode >> 24, sign_extend(opcode, 24));
			break;
		}

		case 0x2f: case 0x3f: {
			// Conditional branch of every kind
			u32 cond = ( opcode >> 20 ) & 0x1f;
			u32 subtype = ( opcode >> 17 ) & 7;
			u32 data = opcode & 0xffff;
			bool invert = opcode & 0x40000000;

			bool cond_passed = false;

			switch(cond) {
			case 0x00: // zrd - d zero
				cond_passed = m_st & F_ZRD;
				break;

			case 0x01: // ged - d >= 0
				cond_passed = !(m_st & F_SGD);
				break;

			case 0x02: // led - d <= 0
				cond_passed = m_st & (F_ZRD | F_SGD);
				break;

			case 0x0a: // gpio0
				cond_passed = m_gpio0;
				break;

			case 0x0b: // gpio1
				cond_passed = m_gpio1;
				break;

			case 0x0c: // gpio2
				cond_passed = m_gpio2;
				break;

			case 0x10: // zc0 - c0 == 1
				cond_passed = !(m_st & F_ZC0);
				break;

			case 0x11: // zc1 - c1 == 1
				cond_passed = !(m_st & F_ZC1);
				break;

			case 0x12: // gpio3
				cond_passed = m_gpio3;
				break;

			case 0x16: // alw - always
				cond_passed = true;
				break;

			default:
				SM2_WARN("unimplemented condition %x (%x)\n", cond, m_ppc);
				break;
			}
			if(invert)
				cond_passed = !cond_passed;

			if(cond_passed) {
				switch(subtype) {
				case 0: // brif #adr
					m_pc = data;
					break;

				case 1: // brul
					if(opcode & 0x4000) {
						// brul reg
						u32 v = read_reg(opcode);
						if(m_stall) goto do_stall;
						m_pc = v;
					} else {
						// brul adr
						u32 ea = ea_pre_0(opcode);
						u32 v = m_bus->read_data(ea);
						if(m_stall) goto do_stall;
						ea_post_0(opcode);
						m_pc = v;
					}
					break;

				case 2: // bsif #adr
					pcs_push();
					m_pc = data;
					break;

				case 3: // bsul
					if(opcode & 0x4000) {
						// bsul reg
						u32 v = read_reg(opcode);
						if(m_stall) goto do_stall;
						pcs_push();
						m_pc = v;
					} else {
						// bsul adr
						u32 ea = ea_pre_0(opcode);
						u32 v = m_bus->read_data(ea);
						if(m_stall) goto do_stall;
						ea_post_0(opcode);
						pcs_push();
						m_pc = v;
					}
					break;

				case 5: // rtif #adr
					pcs_pop();
					break;

				case 6: { // ldif adr, rn
					u32 ea = ea_pre_0(opcode);
					u32 v = m_bus->read_data(ea);
					if(m_stall) goto do_stall;
					ea_post_0(opcode);
					write_reg(opcode >> 9, v);
					break;
				}

				default:
					SM2_WARN("unimplemented branch subtype %x (%x)\n", subtype, m_ppc);
					break;
				}
			}

			if(subtype < 2)
				switch(cond) {
				case 0x10:
					if(m_c0 != 1) {
						m_c0 --;
						if(m_c0 == 1)
							m_st |= F_ZC0;
					}
					break;

				case 0x11:
					if(m_c1 != 1) {
						m_c1 --;
						if(m_c1 == 1)
							m_st |= F_ZC1;
					}
				break;
				}

			break;
		}

		default:
			SM2_WARN("unimplemented opcode type %02x (%x)\n", (opcode >> 26) & 0x3f, m_ppc);
			break;
		}

		if(m_r != 1) {
			m_pc = m_ppc;
			m_r --;
		}

	rep_start:
		if(0) {
		do_stall:
			m_pc = m_ppc;
			m_stall = false;
		}
		m_icount--;
	}
}

}  // namespace sage::cpu::mb86233

// ---------------------------------------------------------------------------
// The host FIFOs
// ---------------------------------------------------------------------------

namespace sage::cpu {

void TgpFifo::configure(std::size_t depth) {
    clear();
    m_depth = depth;
}

void TgpFifo::clear() {
    m_values.clear();
    m_overflow.clear();

    // A cleared FIFO is empty, so a destination waiting on it stays waiting; but
    // it is no longer full, so a halted source has to be released or it would
    // never run again.
    if (m_full_halted) {
        m_full_halted = false;
        if (m_on_unfull) {
            m_on_unfull();
        }
    }
    m_empty_halted = false;
}

std::uint32_t TgpFifo::pop() {
    if (empty()) {
        if (m_on_empty_retry) {
            m_on_empty_retry();
        }
        if (!m_empty_halted) {
            m_empty_halted = true;
            if (m_on_empty_halt) {
                m_on_empty_halt();
            }
        }
        return 0;
    }

    const std::uint32_t value = m_values.front();
    m_values.pop_front();

    // Take one back from the overflow queue, and release the source once the
    // queue has drained.
    if (!m_overflow.empty()) {
        m_values.push_back(m_overflow.front());
        m_overflow.pop_front();

        if (m_overflow.empty() && m_full_halted) {
            m_full_halted = false;
            if (m_on_unfull) {
                m_on_unfull();
            }
        }
    }
    return value;
}

void TgpFifo::push(std::uint32_t value) {
    if (!m_overflow.empty() || full()) {
        m_overflow.push_back(value);
        m_peak_overflow = std::max(m_peak_overflow, m_overflow.size());

        if (!m_full_halted) {
            m_full_halted = true;
            if (m_on_full) {
                m_on_full();
            }
        }
        return;
    }

    m_values.push_back(value);

    if (m_empty_halted) {
        m_empty_halted = false;
        if (m_on_unempty) {
            m_on_unempty();
        }
    }
}

void TgpFifo::serialize(sage::hw::Archive& ar) {
    // SAGE's Archive is a no-op recorder, so the queued values and the halt
    // latches are simply offered to it; the flow-control callbacks are excluded
    // (re-bound by the owner's wiring).
    ar.raw(m_depth);
    ar.raw(m_empty_halted);
    ar.raw(m_full_halted);
}

}  // namespace sage::cpu

// ---------------------------------------------------------------------------
// The coprocessor and its surroundings
// ---------------------------------------------------------------------------

namespace sage::cpu {

namespace {

using namespace sage::cpu::mb86233;

/// io space addresses of the mathematical units, in words.
constexpr u16 kSinCosFirst = 0x0020;
constexpr u16 kSinCosLast  = 0x0023;
constexpr u16 kAtanFirst   = 0x0024;
constexpr u16 kAtanLast    = 0x0027;
constexpr u16 kInverseFirst = 0x0028;
constexpr u16 kInverseLast  = 0x0029;
constexpr u16 kInverseSqrtFirst = 0x002a;
constexpr u16 kInverseSqrtLast  = 0x002b;

/// Base index of each table within the CPU board's table ROM, in words.
constexpr u32 kSinCosTable      = 0x0000;
constexpr u32 kAtanTable        = 0x4000;
constexpr u32 kInverseTable     = 0x8000;
constexpr u32 kInverseSqrtTable = 0xc000;

/// Words of display list buffer the coprocessor can reach.
constexpr u32 kBufferWords = 0x8000;

}  // namespace

TgpCoprocessor::TgpCoprocessor()
    : m_cpu(*this)
    , m_program(kProgramWords, 0)
    , m_data_low(0x100, 0)
    , m_data_high(0x200, 0)
{
    m_fifo_in.configure(kFifoDepth);
    m_fifo_out.configure(kFifoDepth);

    // Flow control. The coprocessor consumes the input FIFO and produces the
    // output one, so it is the destination of one and the source of the other.
    // The host is the opposite in both cases, and the host's side is wired up by
    // the machine because only it can halt the host CPU.
    m_fifo_in.set_on_empty_retry([this] { m_cpu.stall(); });
    m_fifo_in.set_on_empty_halt([this] { m_cpu.set_halted(true); });
    m_fifo_in.set_on_unempty([this] { m_cpu.set_halted(false); });

    m_fifo_out.set_on_full([this] { m_cpu.set_halted(true); });
    m_fifo_out.set_on_unfull([this] { m_cpu.set_halted(false); });
}

void TgpCoprocessor::attach(std::span<const u32> tables, std::span<const u32> data_rom,
                            std::span<const u32> other_data, std::span<u32> buffer_ram) {
    m_tables     = tables;
    m_data_rom   = data_rom;
    m_other_data = other_data;
    m_buffer_ram = buffer_ram;

    if (m_tables.empty()) {
        SM2_WARN("tgp: no mathematical table ROM; every trigonometric result "
                 "will be zero");
    } else if (m_tables.size() < 0x10000) {
        SM2_WARN("tgp: the table ROM holds %zu of the expected 65536 words; "
                 "results above that will be zero",
                 m_tables.size());
    }
}

void TgpCoprocessor::reset() {
    m_cpu.reset();
    // Held until the host uploads microcode and releases it. Without this the
    // coprocessor would run whatever the program RAM happens to contain.
    m_cpu.set_halted(true);

    m_fifo_in.clear();
    m_fifo_out.clear();

    std::fill(m_data_low.begin(), m_data_low.end(), 0u);
    std::fill(m_data_high.begin(), m_data_high.end(), 0u);

    m_control      = 0;
    m_upload_count = 0;
    m_bank         = 0;

    m_sincos_base       = 0;
    m_inverse_base      = 0;
    m_inverse_sqrt_base = 0;
    std::fill(std::begin(m_atan_base), std::end(m_atan_base), 0u);

    m_activity = Activity{};
}

void TgpCoprocessor::serialize(sage::hw::Archive& ar) {
    m_cpu.serialize(ar);
    m_fifo_in.serialize(ar);
    m_fifo_out.serialize(ar);
    ar.bytes(m_program.data(), m_program.size());
    ar.bytes(m_data_low.data(), m_data_low.size());
    ar.bytes(m_data_high.data(), m_data_high.size());
    ar.raw(m_control);
    ar.raw(m_upload_count);
    ar.raw(m_bank);
    ar.raw(m_sincos_base);
    ar.raw(m_inverse_base);
    ar.raw(m_inverse_sqrt_base);
    ar.bytes(m_atan_base, 4);
}

s32 TgpCoprocessor::run(s32 cycles) {
    return m_cpu.run(cycles);
}

// ---------------------------------------------------------------------------
// Host side
// ---------------------------------------------------------------------------

void TgpCoprocessor::control_write(u32 value) {
    // Only a change of the top bit means anything; the rest of the register is
    // storage.
    if (((value ^ m_control) & 0x80000000u) != 0) {
        if ((value & 0x80000000u) != 0) {
            SM2_DEBUG("tgp: microcode upload started");
            m_upload_count = 0;
            m_cpu.set_halted(true);
        } else {
            SM2_DEBUG("tgp: booting, %u word(s) uploaded", m_upload_count);
            m_cpu.reset();
            m_cpu.set_halted(false);
        }
    }
    m_control = value;
}

void TgpCoprocessor::host_fifo_write(u32 value) {
    if ((m_control & 0x80000000u) != 0) {
        if (m_upload_count < kProgramWords) {
            m_program[m_upload_count] = value;
        }
        ++m_upload_count;
        return;
    }
    m_fifo_in.push(value);
}

u32 TgpCoprocessor::host_fifo_read() {
    return m_fifo_out.pop();
}

void TgpCoprocessor::function_port_write(u32 byte_offset, u32 value) {
    // The port's own address carries the function number, which the host folds
    // into the command word. Sixteen bytes per function, eight bits of function
    // number (MAME's copro_function_port_w: `a = (offset >> 2) & 0xff`, offset
    // being a dword count, so `a == (byte_offset >> 4) & 0xff`). Bits 22:20 of
    // the value are discarded because that is where the function number lands.
    const u32 function = (byte_offset >> 4) & 0xff;
    const u32 command  = (value & 0x800fffffu) | (function << 23);
    m_fifo_in.push(command);
}

// ---------------------------------------------------------------------------
// Program and data spaces
// ---------------------------------------------------------------------------

u32 TgpCoprocessor::fetch(u16 address) {
    return m_program[address & (kProgramWords - 1)];
}

u32 TgpCoprocessor::read_program(u16 address) {
    return m_program[address & (kProgramWords - 1)];
}

u32 TgpCoprocessor::read_data(u16 address) {
    // Only the two banks are decoded. Everything else, including the gap at
    // 0x100-0x1ff, reads as zero.
    if (address < 0x100) {
        return m_data_low[address];
    }
    if (address >= 0x200 && address < 0x400) {
        return m_data_high[address - 0x200];
    }
    return 0;
}

void TgpCoprocessor::write_data(u16 address, u32 value) {
    if (address < 0x100) {
        m_data_low[address] = value;
        return;
    }
    if (address >= 0x200 && address < 0x400) {
        m_data_high[address - 0x200] = value;
    }
}

// ---------------------------------------------------------------------------
// io space: the mathematical units, or a window onto external memory
// ---------------------------------------------------------------------------

u32 TgpCoprocessor::read_io(u16 address) {
    // The bank register switches the whole io space between external memory and
    // the mathematical units. That looks odd, because it means the tables are
    // unreachable while the window is open, but it is what the hardware does: the
    // window covers the entire space, so the program has to close it before doing
    // any trigonometry.
    if (external_window_enabled()) {
        return external_read(address);
    }

    if (address >= kSinCosFirst && address <= kSinCosLast) {
        return sincos_read(address - kSinCosFirst);
    }
    if (address >= kAtanFirst && address <= kAtanLast) {
        return atan_read();
    }
    if (address >= kInverseFirst && address <= kInverseLast) {
        return inverse_read(address - kInverseFirst);
    }
    if (address >= kInverseSqrtFirst && address <= kInverseSqrtLast) {
        return inverse_sqrt_read(address - kInverseSqrtFirst);
    }
    return 0;
}

void TgpCoprocessor::write_io(u16 address, u32 value) {
    if (external_window_enabled()) {
        external_write(address, value);
        return;
    }

    if (address >= kSinCosFirst && address <= kSinCosLast) {
        m_sincos_base = value;
        return;
    }
    if (address >= kAtanFirst && address <= kAtanLast) {
        m_atan_base[address - kAtanFirst] = value;
        // The unit drives a general-purpose input with the comparison of its two
        // arguments' magnitudes, which is how the program learns which octant it
        // is in without a branch on the result.
        m_cpu.set_gpio(0, (m_atan_base[0] & 0x7fffffffu) <= (m_atan_base[1] & 0x7fffffffu));
        return;
    }
    if (address >= kInverseFirst && address <= kInverseLast) {
        m_inverse_base = value;
        return;
    }
    if (address >= kInverseSqrtFirst && address <= kInverseSqrtLast) {
        m_inverse_sqrt_base = value;
    }
}

u32 TgpCoprocessor::external_read(u16 offset) const {
    const u32 address = (m_bank & 0xff0000u) | offset;

    if ((address & 0x800000u) != 0) {
        ++m_activity.data_rom_reads;
        // The game's coprocessor data ROM lives here. vf2 has none, and the
        // Model 2A video board's 1/(1+x) table occupies the same role on the
        // games that do, so it is used as the fallback when no data ROM exists.
        const std::span<const u32> rom = m_data_rom.empty() ? m_other_data : m_data_rom;
        if (rom.empty()) {
            return 0;
        }
        const u32 index  = address & (static_cast<u32>(rom.size()) - 1);
        return rom[index];
    }
    if ((address & 0x400000u) != 0) {
        ++m_activity.buffer_reads;
        const u32 index = address & (kBufferWords - 1);
        return index < m_buffer_ram.size() ? m_buffer_ram[index] : 0u;
    }
    return 0;
}

void TgpCoprocessor::external_write(u16 offset, u32 value) {
    const u32 address = (m_bank & 0xff0000u) | offset;

    // Only the display list buffer is writable; the data ROM half is not.
    if ((address & 0x400000u) != 0) {
        ++m_activity.buffer_writes;
        const u32 index = address & (kBufferWords - 1);
        if (index < m_buffer_ram.size()) {
            m_buffer_ram[index] = value;
        }
    }
}

// ---------------------------------------------------------------------------
// The mathematical units
// ---------------------------------------------------------------------------

u32 TgpCoprocessor::sincos_read(u16 offset) const {
    // One table of a quarter turn, reflected and negated to cover the circle. The
    // port offset adds a quarter turn, so reading offset 0 and offset 1 gives sine
    // and cosine of the same angle.
    const u32 angle = m_sincos_base + offset * 0x4000u;
    u32       index = angle & 0x3fff;
    if ((angle & 0x4000u) != 0) {
        index = std::min<u32>(0x4000u - index, 0x3fff);
    }
    u32 result = table(kSinCosTable | index);
    if ((angle & 0x8000u) != 0) {
        result ^= 0x80000000u;
    }
    return result;
}

u32 TgpCoprocessor::atan_read() const {
    // The program writes the two operands and a normalised ratio, and this folds
    // the table's first-octant answer out to the full circle using the signs and
    // the magnitude comparison.
    const u8   ie = static_cast<u8>(0x88 - (m_atan_base[3] >> 23));
    const bool s0 = (m_atan_base[0] & 0x80000000u) != 0;
    const bool s1 = (m_atan_base[1] & 0x80000000u) != 0;
    const bool s2 = (m_atan_base[0] & 0x7fffffffu) <= (m_atan_base[1] & 0x7fffffffu);

    const u32 im    = m_atan_base[3] & 0x7fffff;
    u32       index = ie <= 0x17 ? (im | 0x800000u) >> ie : 0u;
    if (index == 0x4000) {
        index = 0x3fff;
    }

    u32 result = table(kAtanTable | index);
    if (s0 ^ s1 ^ s2) {
        result >>= 16;
    }
    if (s2) {
        result += 0x4000;
    }
    if ((s0 && !s2) || (s1 && s2)) {
        result += 0x8000;
    }
    return result & 0xffff;
}

u32 TgpCoprocessor::inverse_read(u16 offset) const {
    // The table holds reciprocals of mantissas; the exponent is corrected here.
    const u32 index  = ((m_inverse_base >> 9) & 0x3ffe) | (offset & 1);
    u32       result = table(kInverseTable | index);

    const u8 base_exponent = static_cast<u8>((m_inverse_base >> 23) & 0xff);
    const u8 exponent = static_cast<u8>((result >> 23) + (0x7f - base_exponent));
    result = (result & 0x007fffffu) | (static_cast<u32>(exponent) << 23);

    if ((m_inverse_base & 0x80000000u) != 0 && offset != 0) {
        result |= 0x80000000u;
    }
    return result;
}

u32 TgpCoprocessor::inverse_sqrt_read(u16 offset) const {
    // The exclusive-or with 0x2000 is a table layout detail, not a sign trick: the
    // two halves of the mantissa range are stored the other way round.
    const u32 index  = 0x2000u ^ (((m_inverse_sqrt_base >> 10) & 0x3ffe) | (offset & 1));
    u32       result = table(kInverseSqrtTable | index);

    const u8 base_exponent = static_cast<u8>((m_inverse_sqrt_base >> 24) & 0x7f);
    const u8 exponent = static_cast<u8>((result >> 23) + (0x3f - base_exponent));
    result = (result & 0x807fffffu) | (static_cast<u32>(exponent) << 23);

    if ((offset & 1) == 0) {
        result &= 0x7fffffffu;
    }
    return result;
}

// ---------------------------------------------------------------------------
// rf space: the host FIFOs and the bank register
// ---------------------------------------------------------------------------

u32 TgpCoprocessor::read_rf(u8 address) {
    // Four bits of address on this part, so anything wider aliases down. The core
    // passes through what the instruction encoded, exactly as MAME's address space
    // would before truncating.
    switch (address & 0xf) {
        case 1:
            ++m_activity.commands_received;
            return m_fifo_in.pop();
        default:
            return 0;
    }
}

void TgpCoprocessor::write_rf(u8 address, u32 value) {
    switch (address & 0xf) {
        case 0:
            // Status or indicator lamps. Written by the coprocessor programs and
            // read by nothing.
            break;
        case 2:
            ++m_activity.results_sent;
            m_fifo_out.push(value);
            break;
        case 3:
            m_bank = value;
            break;
        default:
            break;
    }
}

}  // namespace sage::cpu

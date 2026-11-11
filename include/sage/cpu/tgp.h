#pragma once

// Fujitsu MB86233 / MB86234 "TGP" geometry coprocessor.
//
// The Model 2A CPU board replaces the Model 2B's ADSP-21062 "SHARC" with this
// part: a 32-bit floating-point DSP that the i960 feeds a microcode program and
// a stream of geometry commands through a pair of host FIFOs. It is not a
// general CPU -- the host uploads the program, boots the part, then talks to it
// only through the FIFOs.
//
// Two pieces live here, kept in one header because neither is useful alone:
//
//   mb86233::Tgp     the processor itself (registers, ALU, program loop)
//   TgpCoprocessor   the part plus everything wired around it: the microcode and
//                    data RAM, the two host FIFOs, the bank register and the
//                    mathematical lookup units
//
// Ported from MAME's src/devices/cpu/mb86233/mb86233.{cpp,h} and the Model 2 /
// Model 2A integration in src/mame/sega/model2.cpp (both BSD-3-Clause,
// copyright-holders Olivier Galibert, R. Belmont, ElSemi, Angelo Salese,
// Matthew Daniels), building on Elsemi's original reverse engineering. The
// structuring into a bus interface and a self-contained wrapper follows the
// sm2-emu reference port. The bodies keep MAME's spelling so they stay diffable
// against upstream.

#include "sage/hw/compat.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <span>
#include <string>
#include <vector>

namespace sage::cpu::mb86233 {

using sage::hw::Archive;
using sage::hw::f2u;
using sage::hw::s32;
using sage::hw::sign_extend;
using sage::hw::u16;
using sage::hw::u2f;
using sage::hw::u32;
using sage::hw::u64;
using sage::hw::u8;
using sage::hw::usize;

/// The four address spaces of an MB86233/MB86234.
///
/// Every access is a 32-bit word and every address counts words, not bytes, so
/// there is no width or alignment to describe. That is a genuine property of
/// the part: it has no byte addressing at all.
///
/// The spaces are separate buses, not regions of one address space:
///
///   program   4096 words of writable microcode, which the host CPU uploads
///   data      two independent RAM banks, at 0x000 and 0x200
///   io        external memory and the mathematical lookup tables
///   rf        sixteen "register file" ports, which is where the host FIFOs
///             and the external memory bank register live
///
/// Implementations mask each address to the width of the space they serve,
/// because the core passes addresses through with only the mask the instruction
/// encoding implies. MAME relies on its address spaces to do the same
/// truncation, so a wider mask here would diverge from it.
class TgpBus {
public:
    virtual ~TgpBus() = default;

    /// Instruction fetch. Separate from read_program so an implementation can
    /// treat it differently, and because it is by far the hottest access.
    [[nodiscard]] virtual u32 fetch(u16 address) = 0;

    /// Program memory read as data. Used by the table-lookup instructions.
    [[nodiscard]] virtual u32 read_program(u16 address) = 0;

    [[nodiscard]] virtual u32 read_data(u16 address) = 0;
    virtual void write_data(u16 address, u32 value) = 0;

    [[nodiscard]] virtual u32 read_io(u16 address) = 0;
    virtual void write_io(u16 address, u32 value) = 0;

    /// Register file port. Four bits of address on this part.
    [[nodiscard]] virtual u32 read_rf(u8 address) = 0;
    virtual void write_rf(u8 address, u32 value) = 0;
};

/// The geometry coprocessor of Model 1, Model 2 and Model 2A.
///
/// A 32-bit floating-point DSP. Three clocks per instruction, so the 50 MHz part
/// on a Model 2A retires about 16.7 million instructions a second.
///
/// Two behaviours matter more than the instruction set:
///
/// The part can be stalled. A read from a host FIFO that turns out to be empty
/// re-executes the whole instruction later rather than returning a value, so the
/// core has to be able to abandon an instruction after it has already had side
/// effects on the ALU. That is what stall() and the do_stall path are for.
///
/// The part can be halted from outside, which is how the host holds it while
/// uploading microcode and how the FIFO flow control blocks it. Halt is checked
/// every instruction, not only on entry, because a FIFO callback can assert it
/// part-way through a slice.
class Tgp {
public:
    /// Clock divider: three clocks per instruction.
    static constexpr u32 kClocksPerCycle = 3;

    /// Status register flags, from MAME. Only a few are exercised by the Sega
    /// programs, but the ALU sets them all.
    enum StatusFlags : u32 {
        kZeroC       = 0x00000001,
        kZeroD       = 0x00000002,
        kSignC       = 0x00000004,
        kSignD       = 0x00000008,
        kCompareC    = 0x00000010,
        kCompareD    = 0x00000020,
        kOverflowC   = 0x00000040,
        kOverflowD   = 0x00000080,
        kUnderflowC  = 0x00000100,
        kUnderflowD  = 0x00000200,
        kDivZeroC    = 0x00000400,
        kDivZeroD    = 0x00000800,
        kCarry       = 0x00001000,
        kCompareP    = 0x00002000,
        kOverflowM   = 0x00004000,
        kUnderflowM  = 0x00008000,
        kSif0        = 0x00010000,
        kSif1        = 0x00020000,
        kSof0        = 0x00040000,
        kPif         = 0x00100000,
        kPof         = 0x00200000,
        kPaif        = 0x00400000,
        kPaof        = 0x00800000,
        kF0s         = 0x01000000,
        kF1s         = 0x02000000,
        kIt          = 0x04000000,
        kZeroX0      = 0x08000000,
        kZeroX1      = 0x10000000,
        kZeroX2      = 0x20000000,
        kZeroC0      = 0x40000000,
        kZeroC1      = 0x80000000,
    };

    explicit Tgp(TgpBus& bus);

    Tgp(const Tgp&)            = delete;
    Tgp& operator=(const Tgp&) = delete;

    void reset();

    /// Execute up to `cycles` instructions. Returns the number consumed, which is
    /// the whole budget when the core is halted.
    [[nodiscard]] s32 run(s32 cycles);

    /// Abandon the instruction in progress and execute it again next time.
    ///
    /// Called from a bus read that could not be satisfied. The instruction's
    /// effects on the ALU registers are discarded by virtue of the core reloading
    /// the program counter and starting over.
    void stall() { m_stall = true; }

    /// External halt, used while microcode is uploaded and by FIFO flow control.
    void set_halted(bool halted) { m_halted = halted; }
    [[nodiscard]] bool halted() const { return m_halted; }

    /// The four general-purpose input pins. Only gpio0 is used on Model 2, where
    /// the arctangent unit drives it with the result of a magnitude comparison.
    void set_gpio(u32 index, bool state);

    // -- state, for diagnostics and tests ----------------------------------

    [[nodiscard]] u16 pc() const { return m_pc; }
    [[nodiscard]] u16 previous_pc() const { return m_ppc; }
    void set_pc(u16 value) { m_pc = value; m_ppc = value; }

    [[nodiscard]] u32 status() const { return m_st; }
    [[nodiscard]] u32 reg_a() const { return m_a; }
    [[nodiscard]] u32 reg_b() const { return m_b; }
    [[nodiscard]] u32 reg_d() const { return m_d; }
    [[nodiscard]] u32 reg_p() const { return m_p; }
    [[nodiscard]] u16 sp() const { return m_sp; }
    [[nodiscard]] u64 instructions() const { return m_instructions; }

    [[nodiscard]] std::string state_string() const;

    /// Save/restore the whole register/execution state (all POD; no dispatch
    /// tables). Bus and trace hook excluded.
    void serialize(Archive& ar);

    using TraceHook = void (*)(void* context, u16 address);
    void set_trace_hook(TraceHook hook, void* context) {
        m_trace_hook    = hook;
        m_trace_context = context;
    }

private:
    // Everything below this line keeps MAME's names so the ported body reads the
    // same as its original.

    // MAME's spellings for the flags above. They exist only so the ported ALU
    // stays textually identical to upstream and remains diffable against it.
    static constexpr u32 F_ZRC  = kZeroC;
    static constexpr u32 F_ZRD  = kZeroD;
    static constexpr u32 F_SGC  = kSignC;
    static constexpr u32 F_SGD  = kSignD;
    static constexpr u32 F_CPC  = kCompareC;
    static constexpr u32 F_CPD  = kCompareD;
    static constexpr u32 F_OVC  = kOverflowC;
    static constexpr u32 F_OVD  = kOverflowD;
    static constexpr u32 F_UNC  = kUnderflowC;
    static constexpr u32 F_UND  = kUnderflowD;
    static constexpr u32 F_DVZC = kDivZeroC;
    static constexpr u32 F_DVZD = kDivZeroD;
    static constexpr u32 F_CA   = kCarry;
    static constexpr u32 F_CPP  = kCompareP;
    static constexpr u32 F_OVM  = kOverflowM;
    static constexpr u32 F_UNM  = kUnderflowM;
    static constexpr u32 F_SIF0 = kSif0;
    static constexpr u32 F_SIF1 = kSif1;
    static constexpr u32 F_SOF0 = kSof0;
    static constexpr u32 F_PIF  = kPif;
    static constexpr u32 F_POF  = kPof;
    static constexpr u32 F_PAIF = kPaif;
    static constexpr u32 F_PAOF = kPaof;
    static constexpr u32 F_F0S  = kF0s;
    static constexpr u32 F_F1S  = kF1s;
    static constexpr u32 F_IT   = kIt;
    static constexpr u32 F_ZX0  = kZeroX0;
    static constexpr u32 F_ZX1  = kZeroX1;
    static constexpr u32 F_ZX2  = kZeroX2;
    static constexpr u32 F_ZC0  = kZeroC0;
    static constexpr u32 F_ZC1  = kZeroC1;

    static u32 set_exp(u32 val, u32 exp);
    static u32 set_mant(u32 val, u32 mant);
    static u32 get_exp(u32 val);
    static u32 get_mant(u32 val);

    void alu_update_st();
    void alu_pre(u32 alu);
    void alu_post_1(u32 alu);
    void alu_post_2(u32 alu);
    u16  ea_pre_0(u32 r);
    void ea_post_0(u32 r);
    u16  ea_pre_1(u32 r);
    void ea_post_1(u32 r);
    void pcs_push();
    void pcs_pop();
    inline void stset_set_sz_int(u32 val);
    inline void stset_set_sz_fp(u32 val);

    u32  read_reg(u32 r);
    void write_reg(u32 r, u32 v);
    void write_mem_internal_1(u32 r, u32 v, bool bank);
    void write_mem_external_1(u32 r, u32 v);
    void write_mem_io_1(u32 r, u32 v);

    void execute_run();

    TgpBus* m_bus = nullptr;

    int m_icount = 0;

    u32 m_st = 0, m_a = 0, m_b = 0, m_d = 0, m_p = 0;
    u32 m_alu_stmask = 0, m_alu_stset = 0, m_alu_r1 = 0, m_alu_r2 = 0;
    u16 m_ppc = 0, m_pc = 0, m_sp = 0, m_b0 = 0, m_b1 = 0, m_x0 = 0, m_x1 = 0;
    u16 m_i0 = 0, m_i1 = 0, m_vsmr = 0, m_pcs[4]{}, m_mask = 0, m_m = 0;
    u8  m_r = 0, m_rpc = 0, m_c0 = 0, m_c1 = 0, m_sft = 0, m_vsm = 0;
    bool m_gpio0 = false, m_gpio1 = false, m_gpio2 = false, m_gpio3 = false;

    bool m_stall  = false;
    bool m_halted = false;

    u64 m_instructions = 0;

    TraceHook m_trace_hook    = nullptr;
    void*     m_trace_context = nullptr;
};

}  // namespace sage::cpu::mb86233

namespace sage::cpu {

using sage::hw::s32;
using sage::hw::u16;
using sage::hw::u32;
using sage::hw::u64;
using sage::hw::u8;

/// A hardware FIFO with flow control, between the i960 and the TGP.
///
/// Derived from MAME's src/devices/machine/gen_fifo.{h,cpp} (BSD-3-Clause,
/// Olivier Galibert). The data behaviour is the same; the scheduling is not.
///
/// MAME resolves the two processors' relative progress with timers. Here the
/// caller drives both processors itself and can run the other side on demand, so
/// a pop from an empty FIFO applies its halt straight away. A processor may
/// therefore halt slightly earlier than on hardware, but no value is ever lost
/// and the order is unchanged, because an overflowing push is queued rather than
/// dropped, exactly as in MAME.
class TgpFifo {
public:
    /// Set the depth and clear. Handlers survive.
    void configure(std::size_t depth);

    void clear();

    /// Remove the oldest value. On an empty FIFO this returns zero and asks the
    /// destination to retry the instruction, then halts it.
    [[nodiscard]] std::uint32_t pop();

    void push(std::uint32_t value);

    [[nodiscard]] bool empty() const { return m_values.empty(); }
    [[nodiscard]] bool full() const { return m_values.size() >= m_depth; }
    [[nodiscard]] std::size_t size() const { return m_values.size() + m_overflow.size(); }
    [[nodiscard]] std::size_t peak_overflow() const { return m_peak_overflow; }

    void set_on_empty_retry(std::function<void()> handler) {
        m_on_empty_retry = std::move(handler);
    }
    void set_on_empty_halt(std::function<void()> handler) {
        m_on_empty_halt = std::move(handler);
    }
    void set_on_unempty(std::function<void()> handler) { m_on_unempty = std::move(handler); }
    void set_on_full(std::function<void()> handler) { m_on_full = std::move(handler); }
    void set_on_unfull(std::function<void()> handler) { m_on_unfull = std::move(handler); }

    void serialize(sage::hw::Archive& ar);

private:
    std::deque<std::uint32_t> m_values;    ///< Within the configured depth.
    std::deque<std::uint32_t> m_overflow;  ///< Pushed while full; never dropped.
    std::size_t m_depth = 0;
    bool m_empty_halted = false;
    bool m_full_halted  = false;
    std::size_t m_peak_overflow = 0;
    std::function<void()> m_on_empty_retry;
    std::function<void()> m_on_empty_halt;
    std::function<void()> m_on_unempty;
    std::function<void()> m_on_full;
    std::function<void()> m_on_unfull;
};

/// The Model 2 / Model 2A geometry coprocessor subsystem: the MB86234 together
/// with everything wired around it.
///
/// The MB86234 has no memory of its own beyond its register file. Everything it
/// touches is provided here:
///
///   program   4096 words of RAM that the host CPU fills with microcode
///   data      two RAM banks, 256 words at 0x000 and 512 words at 0x200
///   io        the mathematical lookup tables, or a window onto the display list
///             buffer and the coprocessor's data ROM, depending on a bank register
///   rf        the two host FIFOs and that bank register
///
/// The lookup tables are the reason a Model 2 can do trigonometry at all: there
/// is no such instruction. The program writes an argument to a port and reads the
/// result back, and the tables come from ROMs on the CPU board.
class TgpCoprocessor final : public mb86233::TgpBus {
public:
    /// Words of microcode RAM.
    static constexpr u32 kProgramWords = 0x1000;

    /// Depth of each host FIFO.
    static constexpr std::size_t kFifoDepth = 8;

    TgpCoprocessor();

    TgpCoprocessor(const TgpCoprocessor&)            = delete;
    TgpCoprocessor& operator=(const TgpCoprocessor&) = delete;

    /// Wire up the ROM tables and the display list buffer.
    ///
    /// `tables` is the CPU board's 64K-word table ROM (`copro_tgp_tables`): sine
    /// and cosine at 0x0000, arctangent at 0x4000, reciprocal at 0x8000 and
    /// inverse square root at 0xc000 -- which is exactly the "1/x, 1/sqrt(x)"
    /// data the task refers to. `data_rom` is the game's coprocessor data ROM
    /// (`copro_data`, collision and height-map data, empty on vf2). `other_data`
    /// is the Model 2A video board's 1/(1+x) table ROM; MAME leaves it off the
    /// TGP (it feeds the 315-5645 ULA), so it is only consulted as a fallback
    /// when no data ROM is present. `buffer_ram` is the host's display list
    /// buffer, which the coprocessor writes its results into; it must outlive
    /// this object.
    void attach(std::span<const u32> tables, std::span<const u32> data_rom,
                std::span<const u32> other_data, std::span<u32> buffer_ram);

    void reset();

    [[nodiscard]] mb86233::Tgp& cpu() { return m_cpu; }
    [[nodiscard]] const mb86233::Tgp& cpu() const { return m_cpu; }

    [[nodiscard]] TgpFifo& fifo_in() { return m_fifo_in; }
    [[nodiscard]] TgpFifo& fifo_out() { return m_fifo_out; }
    [[nodiscard]] const TgpFifo& fifo_in() const { return m_fifo_in; }
    [[nodiscard]] const TgpFifo& fifo_out() const { return m_fifo_out; }

    /// Advance the coprocessor by up to this many of its own cycles.
    [[nodiscard]] s32 run(s32 cycles);

    // -- host side ---------------------------------------------------------

    /// Write to the control register at 0x00980000.
    ///
    /// The top bit selects microcode upload. On the rising edge the coprocessor
    /// is halted and the word counter reset; on the falling edge it is released
    /// and reset, which starts the uploaded program.
    void control_write(u32 value);
    [[nodiscard]] u32 control_read() const { return m_control; }

    /// Write to the FIFO port at 0x00884000. During an upload this is the
    /// microcode itself rather than a command.
    void host_fifo_write(u32 value);

    /// Read a result from the coprocessor.
    [[nodiscard]] u32 host_fifo_read();

    /// Write to a function port at 0x00880000. The port number becomes part of
    /// the command word, which is how one write both selects a function and
    /// passes an argument.
    void function_port_write(u32 byte_offset, u32 value);

    /// True when there is no result waiting. This is what the host polls.
    [[nodiscard]] bool output_empty() const { return m_fifo_out.empty(); }

    /// Words of microcode uploaded since the last upload began.
    [[nodiscard]] u32 uploaded_words() const { return m_upload_count; }

    /// Counts of what the coprocessor did, for diagnostics.
    ///
    /// These separate "the coprocessor is running" from "the coprocessor is
    /// doing the work it exists for". A program can retire millions of
    /// instructions in a polling loop without ever touching the tables or
    /// writing a result.
    struct Activity {
        u64 commands_received = 0;  ///< Words popped from the host FIFO.
        u64 results_sent      = 0;  ///< Words pushed to the host FIFO.
        u64 buffer_writes     = 0;  ///< Words written into the display list.
        u64 buffer_reads      = 0;
        u64 data_rom_reads    = 0;
        u64 table_reads       = 0;  ///< Trigonometry and reciprocal lookups.
    };
    [[nodiscard]] const Activity& activity() const { return m_activity; }

    [[nodiscard]] std::span<const u32> program() const { return m_program; }

    /// Save/restore the TGP, both FIFOs, the uploaded microcode + data RAM,
    /// control/upload/bank scalars and the math-unit argument latches. Spans and
    /// the diagnostic Activity are excluded.
    void serialize(sage::hw::Archive& ar);

    // -- mb86233::TgpBus ---------------------------------------------------

    u32  fetch(u16 address) override;
    u32  read_program(u16 address) override;
    u32  read_data(u16 address) override;
    void write_data(u16 address, u32 value) override;
    u32  read_io(u16 address) override;
    void write_io(u16 address, u32 value) override;
    u32  read_rf(u8 address) override;
    void write_rf(u8 address, u32 value) override;

private:
    /// The external memory window, reachable through the io space when the bank
    /// register selects it.
    [[nodiscard]] u32 external_read(u16 offset) const;
    void external_write(u16 offset, u32 value);

    /// True when the bank register points the io space at external memory
    /// instead of the mathematical tables.
    [[nodiscard]] bool external_window_enabled() const {
        return (m_bank & 0xc00000) != 0;
    }

    // The mathematical units. Each takes an argument through a write and returns
    // a result through a read, with the port offset selecting which part of the
    // answer is wanted.
    [[nodiscard]] u32 sincos_read(u16 offset) const;
    [[nodiscard]] u32 atan_read() const;
    [[nodiscard]] u32 inverse_read(u16 offset) const;
    [[nodiscard]] u32 inverse_sqrt_read(u16 offset) const;

    [[nodiscard]] u32 table(u32 index) const {
        ++m_activity.table_reads;
        return index < m_tables.size() ? m_tables[index] : 0;
    }

    mb86233::Tgp m_cpu;

    TgpFifo m_fifo_in;
    TgpFifo m_fifo_out;

    std::vector<u32> m_program;
    std::vector<u32> m_data_low;   ///< 256 words at 0x000
    std::vector<u32> m_data_high;  ///< 512 words at 0x200

    std::span<const u32> m_tables;
    std::span<const u32> m_data_rom;
    std::span<const u32> m_other_data;
    std::span<u32>       m_buffer_ram;

    u32 m_control      = 0;
    u32 m_upload_count = 0;

    /// External memory bank register, written through rf port 3.
    u32 m_bank = 0;

    /// Arguments latched by the mathematical units.
    u32 m_sincos_base = 0;
    u32 m_inverse_base = 0;
    u32 m_inverse_sqrt_base = 0;
    u32 m_atan_base[4]{};

    /// Mutable because the table lookups are const reads that still count.
    mutable Activity m_activity;
};

}  // namespace sage::cpu

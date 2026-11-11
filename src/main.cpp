#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <set>
#include <vector>

#include "sage/bus.h"
#include "sage/display.h"
#include "sage/cpu/i960.h"
#include "sage/rom_loader.h"

namespace {

constexpr const char* kBanner =
    "  ____    _    ____ _____ \n"
    " / ___|  / \\  / ___| ____|\n"
    " \\___ \\ / _ \\| |  _|  _|  \n"
    "  ___) / ___ \\ |_| | |___ \n"
    " |____/_/   \\_\\____|_____|\n";

// i960KB on the Model 2 CPU board runs from a 50 MHz crystal divided by two.
constexpr std::uint32_t kCpuClock = 25'000'000;
constexpr int kFrameRate = 60;

void print_banner(const std::string& target) {
    std::printf("%s", kBanner);
    std::printf(" SAGE - Sega Arcade Golden-era Emulator\n");
    std::printf(" Target: %s\n", target.c_str());
    std::printf("============================================================\n\n");
}

std::string human_size(std::size_t bytes) {
    char buf[64];
    if (bytes >= 1024 * 1024)
        std::snprintf(buf, sizeof(buf), "%.2f MiB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    else if (bytes >= 1024)
        std::snprintf(buf, sizeof(buf), "%.1f KiB", static_cast<double>(bytes) / 1024.0);
    else
        std::snprintf(buf, sizeof(buf), "%zu B", bytes);
    return buf;
}

void print_rom_table(const sage::LoadReport& rep) {
    std::printf("ROM IC verification\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("  %-18s %-18s %10s  %-8s  %s\n", "IC (MAME)", "IC (archive)", "Size", "CRC32",
                "Status");
    std::printf("------------------------------------------------------------\n");
    std::size_t ok = 0, alt = 0;
    for (const auto& r : rep.roms) {
        const char* status = r.crc_ok ? "OK"
                                      : r.alt_ok ? "OK (alt dump)"
                                                 : (r.found ? "CRC MISMATCH" : "MISSING");
        if (r.crc_ok)
            ++ok;
        else if (r.alt_ok)
            ++alt;
        std::printf("  %-18s %-18s %10s  %08" PRIX32 "  %s\n", r.entry_name.c_str(),
                    r.archive_name.empty() ? "-" : r.archive_name.c_str(),
                    human_size(r.size).c_str(), r.actual_crc, status);
    }
    std::printf("------------------------------------------------------------\n");
    std::printf("  %zu / %zu ROM ICs verified", ok, rep.roms.size());
    if (alt)
        std::printf(" (%zu known alternate dump%s)", alt, alt == 1 ? "" : "s");
    std::printf("\n\n");
}

void print_region_table(const sage::RomLoader& loader, const sage::LoadReport& rep) {
    std::printf("Mapped memory regions\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("  %-12s %12s  %6s  %-8s  %s\n", "Region", "Size", "Width", "Endian", "CRC32");
    std::printf("------------------------------------------------------------\n");
    for (const auto& [name, crc] : rep.region_crcs) {
        const sage::Region* r = loader.region(name);
        std::printf("  %-12s %12s  %4u-bit  %-8s  %08" PRIX32 "\n", name.c_str(),
                    human_size(r ? r->data.size() : 0).c_str(), r ? r->bus_width : 0,
                    (r && r->big_endian) ? "big" : "little", crc);
    }
    std::printf("------------------------------------------------------------\n\n");
}

void print_boot_state(const sage::cpu::I960& cpu) {
    std::printf("i960 boot vector (read at reset from 0x00000000)\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("  SAT  (0x00)     : 0x%08" PRIX32 "\n", cpu.get_sat());
    std::printf("  PRCB (0x04)     : 0x%08" PRIX32 "\n", cpu.get_prcb());
    std::printf("  Entry IP (0x0C) : 0x%08" PRIX32 "\n", cpu.get_ip());
    std::printf("  PC (control)    : 0x%08" PRIX32 "\n", cpu.get_pc());
    std::printf("  AC (condition)  : 0x%08" PRIX32 "\n", cpu.get_ac());
    std::printf("  FP (g15)        : 0x%08" PRIX32 "\n", cpu.reg(31));
    std::printf("  SP (r1)         : 0x%08" PRIX32 "\n", cpu.reg(1));
    std::printf("------------------------------------------------------------\n\n");
}

void print_bus_state(const sage::Bus& bus, const sage::cpu::I960& cpu) {
    std::printf("Peripheral state\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("  Interrupt enable : 0x%08" PRIX32 "\n", bus.irq_enable());
    std::printf("  Interrupt request: 0x%08" PRIX32 "\n", bus.irq_request());
    std::printf("  Video control    : 0x%08" PRIX32 "\n", bus.video_control());
    std::printf("  Render mode      : %s\n", bus.render_mode() ? "60 Hz" : "30 Hz");
    std::printf("  VBLANKs raised   : %" PRIu64 "\n", bus.vblank_count());
    std::printf("  Timer IRQs       : %" PRIu64 "\n", bus.timer_irq_count());
    std::printf("  ICR (vectors)    : 0x%08" PRIX32 "\n", cpu.get_icr());
    std::printf("  IRQ immediate    : %" PRIu64 "\n", cpu.irq_immediate_count());
    std::printf("  IRQ pending      : %" PRIu64 "\n", cpu.irq_pending_count());
    std::printf("  IRQ taken        : %" PRIu64 "\n", cpu.irq_taken_count());
    std::printf("  GEO parses       : %" PRIu64 "  polygons rendered: %" PRIu64 "\n",
                bus.geometry().parse_count(), bus.geometry().polygons_rendered());
    std::printf("------------------------------------------------------------\n\n");
}

// Writes the current rendered frame as a binary PPM (P6) for inspection.
bool write_ppm(const char* path, const sage::Video& video) {
    std::FILE* f = std::fopen(path, "wb");
    if (!f)
        return false;
    std::fprintf(f, "P6\n%d %d\n255\n", sage::Video::kWidth, sage::Video::kHeight);
    const std::uint32_t* px = video.rgba();
    std::vector<std::uint8_t> row(static_cast<std::size_t>(sage::Video::kWidth) * 3);
    for (int y = 0; y < sage::Video::kHeight; ++y) {
        for (int x = 0; x < sage::Video::kWidth; ++x) {
            const std::uint32_t c = px[y * sage::Video::kWidth + x];
            row[x * 3 + 0] = static_cast<std::uint8_t>(c >> 16);
            row[x * 3 + 1] = static_cast<std::uint8_t>(c >> 8);
            row[x * 3 + 2] = static_cast<std::uint8_t>(c);
        }
        std::fwrite(row.data(), 1, row.size(), f);
    }
    std::fclose(f);
    return true;
}

// Appends one rendered frame as packed RGB24 to an open stream (SAGE_RAWVIDEO),
// so frames can be piped straight into ffmpeg.
void write_rgb24(std::FILE* f, const sage::Video& video) {
    const std::uint32_t* px = video.rgba();
    std::vector<std::uint8_t> row(static_cast<std::size_t>(sage::Video::kWidth) * 3);
    for (int y = 0; y < sage::Video::kHeight; ++y) {
        for (int x = 0; x < sage::Video::kWidth; ++x) {
            const std::uint32_t c = px[y * sage::Video::kWidth + x];
            row[x * 3 + 0] = static_cast<std::uint8_t>(c >> 16);
            row[x * 3 + 1] = static_cast<std::uint8_t>(c >> 8);
            row[x * 3 + 2] = static_cast<std::uint8_t>(c);
        }
        std::fwrite(row.data(), 1, row.size(), f);
    }
}

// Writes a 16-bit stereo WAV header. The data is streamed after it, so the
// sizes are patched up when the file is closed.
void write_wav_header(std::FILE* f, std::uint32_t sample_rate) {
    const std::uint32_t byte_rate = sample_rate * 4;
    const std::uint16_t block_align = 4;
    std::uint8_t h[44] = {};
    std::memcpy(h + 0, "RIFF", 4);
    std::memcpy(h + 8, "WAVEfmt ", 8);
    const std::uint32_t fmt_size = 16;
    const std::uint16_t pcm = 1, channels = 2, bits = 16;
    std::memcpy(h + 16, &fmt_size, 4);
    std::memcpy(h + 20, &pcm, 2);
    std::memcpy(h + 22, &channels, 2);
    std::memcpy(h + 24, &sample_rate, 4);
    std::memcpy(h + 28, &byte_rate, 4);
    std::memcpy(h + 32, &block_align, 2);
    std::memcpy(h + 34, &bits, 2);
    std::memcpy(h + 36, "data", 4);
    std::fwrite(h, 1, sizeof h, f);
}

void patch_wav_sizes(std::FILE* f, std::uint64_t data_bytes) {
    const std::uint32_t riff = static_cast<std::uint32_t>(36 + data_bytes);
    const std::uint32_t data = static_cast<std::uint32_t>(data_bytes);
    std::fseek(f, 4, SEEK_SET);
    std::fwrite(&riff, 4, 1, f);
    std::fseek(f, 40, SEEK_SET);
    std::fwrite(&data, 4, 1, f);
}

}  // namespace

int main(int argc, char** argv) {
    const std::string zip_path = (argc > 1) ? argv[1] : "roms/doa.zip";
    const std::string game = (argc > 2) ? argv[2] : "doa";
    const std::string target = (game == "vf2") ? "Virtua Fighter 2 - Sega Model 2A"
                                               : "Dead or Alive - Sega Model 2B";
    print_banner(target);

    const long frames = (argc > 3) ? std::atol(argv[3]) : 0;
    const long trace_instructions = std::getenv("SAGE_TRACE") ? 100'000 : 0;

    std::printf("Loading ROM set: %s\n\n", zip_path.c_str());

    sage::RomLoader loader;
    if (!loader.load(zip_path, game)) {
        std::printf("ERROR: %s\n", loader.report().error.c_str());
        return 1;
    }

    const sage::LoadReport& rep = loader.report();
    print_rom_table(rep);
    print_region_table(loader, rep);

    const sage::Region* cpu_region = loader.region("maincpu");
    const std::size_t cpu_end = cpu_region ? cpu_region->data.size() - 1 : 0;

    std::printf("Load summary\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("  Game            : %s\n", rep.game.c_str());
    std::printf("  Revision        : %s\n", rep.revision.c_str());
    std::printf("  Hardware        : %s\n", rep.hardware.c_str());
    std::printf("  ROM archive     : %s\n", zip_path.c_str());
    std::printf("  Regions mapped  : %zu\n", rep.region_crcs.size());
    std::printf("  Total ROM bytes : %s\n", human_size(rep.total_bytes).c_str());
    std::printf("  Program memory  : 0x00000000 - 0x%08zX (%s)\n", cpu_end,
                human_size(rep.program_bytes).c_str());
    std::printf("------------------------------------------------------------\n\n");

    // ---- Machine bring-up -------------------------------------------------
    sage::Bus bus;
    if (std::getenv("SAGE_TRACE"))
        bus.set_trace(true);
    if (std::getenv("SAGE_TRACE_IRQ"))
        bus.set_trace_irq(true);
    bus.attach(loader, game);

    if (const char* w = std::getenv("SAGE_WATCH")) {
        unsigned lo = 0, hi = 0;
        std::sscanf(w, "%x:%x", &lo, &hi);
        bus.set_watch(lo, hi, 200000, std::getenv("SAGE_WATCH_R") != nullptr);
    }

    sage::cpu::I960 cpu(bus);
    if (std::getenv("SAGE_TRACE_IRQ"))
        cpu.set_trace_irq(true);
    bus.set_cpu(&cpu);
    cpu.reset();

    std::map<std::uint32_t, std::uint64_t> sharc_pc_hist;
    if (std::getenv("SAGE_SHARC_PC"))
        bus.sharc_mutable().set_pc_hist(&sharc_pc_hist);
    if (const char* st = std::getenv("SAGE_SHARC_TRACE"))
        bus.sharc_mutable().m_trace_count = std::atol(st);
    if (const char* wd = std::getenv("SAGE_WAITDBG"))
        bus.sharc_mutable().m_wait_dbg = std::atol(wd);
    static std::uint32_t sharc_ring[1024];
    static std::uint64_t sharc_ring_op[1024];
    static std::uint32_t sharc_ring_d[1024], sharc_ring_f[1024], sharc_ring_nf[1024];
    if (std::getenv("SAGE_SHARC_RING"))
        bus.sharc_mutable().set_ring(sharc_ring, sharc_ring_op, sharc_ring_d, sharc_ring_f,
                                     sharc_ring_nf);
    if (const char* w = std::getenv("SAGE_SHARC_WIN")) {
        unsigned lo = 0, hi = 0;
        std::sscanf(w, "%x:%x", &lo, &hi);
        bus.sharc_mutable().set_window(lo, hi);
    }
    if (std::getenv("SAGE_SHARC_BADPC"))
        bus.sharc_mutable().m_badpc_trace = true;
    if (std::getenv("SAGE_SHARC_LOOPDBG"))
        bus.sharc_mutable().m_loop_dbg = true;
    if (std::getenv("SAGE_SHARC_CMD"))
        bus.sharc_mutable().m_cmd_trig = true;
    if (std::getenv("SAGE_SHARC_DM"))
        bus.sharc_mutable().m_dm_dbg = true;
    if (std::getenv("SAGE_SHARC_UNMAP"))
        bus.sharc_mutable().m_unmapped_dbg = true;

    if (const char* dp = std::getenv("SAGE_DISASM")) {
        char buf[4096];
        std::snprintf(buf, sizeof(buf), "%s", dp);
        for (char* tok = std::strtok(buf, ","); tok; tok = std::strtok(nullptr, ",")) {
            std::uint32_t a = static_cast<std::uint32_t>(std::strtoul(tok, nullptr, 16));
            std::printf("  [0x%08" PRIX32 "] %08" PRIX32 "  %s\n", a, bus.read32(a),
                        cpu.disassemble(a).c_str());
        }
    }

    if (const char* inj = std::getenv("SAGE_PRESS")) {
        unsigned frame = 0, port = 0, val = 0;
        std::sscanf(inj, "%u:%u:%x", &frame, &port, &val);
        (void)frame;
        if (port == 1)
            bus.inputs().p1 = static_cast<std::uint8_t>(val);
        else if (port == 2)
            bus.inputs().p2 = static_cast<std::uint8_t>(val);
        std::printf("Injected input port %u = %02X\n", port, val);
    }

    print_boot_state(cpu);

    if (const char* range = std::getenv("SAGE_DISASM")) {
        unsigned start = 0, count = 64;
        std::sscanf(range, "%x:%u", &start, &count);
        std::printf("Disassembly at 0x%08X\n", start);
        std::printf("------------------------------------------------------------\n");
        for (unsigned i = 0; i < count; ++i) {
            const std::uint32_t pc = start + i * 4;
            std::printf("  [0x%08" PRIX32 "] %08" PRIX32 "  %s\n", pc, bus.read32(pc),
                        cpu.disassemble(pc).c_str());
        }
        std::printf("------------------------------------------------------------\n\n");
    }

    std::uint64_t cycles = 0;
    std::uint64_t instructions = 0;
    long frames_run = 0;
    std::map<std::uint32_t, std::uint64_t> pc_pages;
    std::vector<std::uint32_t> pc_order;
    std::set<std::uint32_t> new_pc_seen;
    bool faulted = false;
    std::string fault_msg;

    sage::Display display;
    const bool want_window = std::getenv("SAGE_HEADLESS") == nullptr;
    if (frames > 0 && want_window) {
        if (display.open(("SAGE - " + target).c_str(), sage::Video::kWidth,
                         sage::Video::kHeight))
            std::printf("SDL window opened (%dx%d). ESC or close to quit.\n\n",
                        sage::Video::kWidth, sage::Video::kHeight);
        else
            std::printf("SDL window unavailable; continuing headless.\n\n");
    }

    if (frames > 0) {
        const std::uint64_t cycles_per_frame = kCpuClock / kFrameRate;
        std::printf("Execution: %ld frame%s at %d Hz (%" PRIu64 " cycles/frame)\n", frames,
                    frames == 1 ? "" : "s", kFrameRate, cycles_per_frame);
        std::printf("------------------------------------------------------------\n");

        std::FILE* rawvideo = nullptr;
        if (const char* rv = std::getenv("SAGE_RAWVIDEO")) {
            rawvideo = std::fopen(rv, "wb");
            if (rawvideo)
                std::printf("Raw video: streaming %dx%d RGB24 frames to %s\n",
                            sage::Video::kWidth, sage::Video::kHeight, rv);
            else
                std::printf("Raw video: cannot open %s\n", rv);
        }

        // Optional audio dump: SAGE_WAV writes the SCSP output as 16-bit
        // stereo so the emulated sound can be listened to or analysed.
        std::FILE* wav = nullptr;
        std::uint64_t wav_bytes = 0;
        if (const char* wp = std::getenv("SAGE_WAV")) {
            wav = std::fopen(wp, "wb");
            if (wav) {
                write_wav_header(wav, bus.sound().sample_rate());
                std::printf("Audio: streaming SCSP output at %u Hz to %s\n",
                            bus.sound().sample_rate(), wp);
            } else {
                std::printf("Audio: cannot open %s\n", wp);
            }
        }

        for (long frame = 0; frame < frames && !faulted; ++frame) {
            if (display.is_open() && !display.pump()) {
                std::printf("  window closed at frame %ld\n", frame);
                break;
            }
            std::uint64_t frame_cycles = 0;
            while (frame_cycles < cycles_per_frame) {
                const std::uint32_t pc = cpu.get_ip();
                const std::uint32_t opcode = bus.read32(pc);
                int used = 0;
                try {
                    used = cpu.step();
                } catch (const sage::cpu::Fault& f) {
                    std::printf("  [frame %ld] [0x%08" PRIX32 "] %08" PRIX32 "  FAULT: %s\n", frame,
                                pc, opcode, f.what());
                    fault_msg = f.what();
                    faulted = true;
                    break;
                }
                frame_cycles += static_cast<std::uint64_t>(used);
                bus.tick(used);
                bus.tick_copro(used);
                bus.tick_sound(used);
                cycles += static_cast<std::uint64_t>(used);
                ++instructions;
                if (bus.trace()) {
                    ++pc_pages[pc & 0x0000ffff];
                    if (new_pc_seen.insert(pc).second)
                        pc_order.push_back(pc);
                }
            }

            bus.trigger_vblank();
            bus.video().render();
            if (rawvideo)
                write_rgb24(rawvideo, bus.video());
            if (wav) {
                const auto samples = bus.sound().pending_samples();
                if (!samples.empty()) {
                    std::fwrite(samples.data(), sizeof(std::int16_t), samples.size(), wav);
                    wav_bytes += samples.size() * sizeof(std::int16_t);
                }
                bus.sound().clear_pending_samples();
            }
            if (display.is_open()) {
                display.present(bus.video().rgba());
                if (std::getenv("SAGE_REALTIME"))
                    display.pace_frame();
            }
            ++frames_run;

            if (frame % 250 == 0 || frame + 1 == frames) {
                std::printf("  frame %5ld  cycles %10" PRIu64 "  IP 0x%08" PRIX32
                            "  VBLANKs %" PRIu64 "\n",
                            frame, cycles, cpu.get_ip(), bus.vblank_count());
            }
        }
        if (rawvideo)
            std::fclose(rawvideo);
        if (wav) {
            patch_wav_sizes(wav, wav_bytes);
            std::fclose(wav);
            std::printf("Audio: wrote %" PRIu64 " bytes of PCM\n", wav_bytes);
        }
        std::printf("------------------------------------------------------------\n\n");
    } else {
        std::printf("Execution trace (%ld instructions)\n", trace_instructions);
        std::printf("------------------------------------------------------------\n");
        for (long i = 0; i < trace_instructions && !faulted; ++i) {
            const std::uint32_t pc = cpu.get_ip();
            const std::uint32_t opcode = bus.read32(pc);
            if (i < 20)
                std::printf("  [0x%08" PRIX32 "] %08" PRIX32 "  %s\n", pc, opcode,
                            cpu.disassemble(pc).c_str());

            int used = 0;
            try {
                used = cpu.step();
            } catch (const sage::cpu::Fault& f) {
                std::printf("  [0x%08" PRIX32 "] %08" PRIX32 "  FAULT: %s\n", pc, opcode, f.what());
                fault_msg = f.what();
                faulted = true;
                break;
            }
            bus.tick_copro(used);
            cycles += static_cast<std::uint64_t>(used);
            ++instructions;
        }
        std::printf("------------------------------------------------------------\n\n");
    }

    if (std::getenv("SAGE_TRACE")) {
        std::printf("Read histogram (page -> count)\n");
        for (const auto& [page, n] : bus.read_histogram())
            std::printf("  %08" PRIX32 " : %" PRIu64 "\n", page, n);
        std::printf("Write histogram (page -> count)\n");
        for (const auto& [page, n] : bus.write_histogram())
            std::printf("  %08" PRIX32 " : %" PRIu64 "\n", page, n);
        std::printf("ROM-write histogram (page -> count)\n");
        for (const auto& [page, n] : bus.rom_write_histogram())
            std::printf("  %08" PRIX32 " : %" PRIu64 "\n", page, n);
        std::printf("Unmapped reads (addr <- pc)\n");
        for (const auto& [a, pc] : bus.unmapped_read_log())
            std::printf("  %08" PRIX32 " <- %08" PRIX32 "\n", a, pc);
        std::printf("Unmapped-read histogram (page -> count)\n");
        for (const auto& [page, n] : bus.unmapped_read_histogram())
            std::printf("  %08" PRIX32 " : %" PRIu64 "\n", page, n);
        std::printf("Hottest PCs (top 20 of %zu)\n", pc_pages.size());
        {
            std::vector<std::pair<std::uint64_t, std::uint32_t>> v;
            v.reserve(pc_pages.size());
            for (const auto& [pc, n] : pc_pages)
                v.push_back({n, pc});
            std::sort(v.begin(), v.end(), std::greater<>());
            for (std::size_t i = 0; i < v.size() && i < 20; ++i)
                std::printf("  %08" PRIX32 " : %" PRIu64 "\n", v[i].second, v[i].first);
        }
        if (const char* dp = std::getenv("SAGE_DUMPRAW")) {
            auto dump = [&](const char* path, const std::uint8_t* d, std::size_t n) {
                if (std::FILE* f = std::fopen(path, "wb")) { std::fwrite(d, 1, n, f); std::fclose(f); }
            };
            dump((std::string(dp) + ".char").c_str(), bus.video().charram().data(), bus.video().charram().size());
            dump((std::string(dp) + ".tile").c_str(), bus.video().tileram().data(), bus.video().tileram().size());
            std::printf("dumped char+tile RAM to %s.{char,tile}\n", dp);
        }
        if (std::getenv("SAGE_DUMPVID")) {
            const auto& cx = bus.video().colorxlat();
            std::size_t nz = 0;
            for (std::uint8_t b : cx)
                if (b) ++nz;
            std::printf("colorxlat nonzero bytes: %zu / %zu\n", nz, cx.size());
            std::printf("  [0x0080]=%u [0x0280]=%u [0x4080]=%u [0x8080]=%u\n", cx[0x80], cx[0x280],
                        cx[0x4080], cx[0x8080]);
            const auto& pr = bus.video().palram();
            std::size_t pnz = 0;
            for (std::uint8_t b : pr)
                if (b) ++pnz;
            std::printf("palram nonzero bytes: %zu / %zu\n", pnz, pr.size());
            const auto& pal = bus.video().palette();
            std::set<std::uint32_t> uniq;
            for (std::uint32_t c : pal) uniq.insert(c);
            std::printf("palette distinct entries: %zu\n", uniq.size());
            std::printf("  distinct:");
            for (std::uint32_t c : uniq) std::printf(" %06X", c & 0xffffff);
            std::printf("\n");
        }
        if (std::getenv("SAGE_PCORDER")) {
            std::printf("Last discovered PCs (%zu total)\n", pc_order.size());
            const std::size_t from = pc_order.size() > 60 ? pc_order.size() - 60 : 0;
            for (std::size_t i = from; i < pc_order.size(); ++i)
                std::printf("  %08" PRIX32 "\n", pc_order[i]);
        }
        std::printf("Unmapped-write histogram (page -> count)\n");
        for (const auto& [page, n] : bus.unmapped_write_histogram())
            std::printf("  %08" PRIX32 " : %" PRIu64 "\n", page, n);
        std::printf("\n");
    }

    print_bus_state(bus, cpu);

    if (std::getenv("SAGE_INTTAB")) {
        const std::uint32_t prcb = cpu.get_prcb();
        const std::uint32_t int_tab = bus.read32(prcb + 20);
        std::printf("PRCB=0x%08X int_tab=0x%08X cpu_pri=%u PC=0x%08X\n", prcb, int_tab,
                    (cpu.get_pc() >> 16) & 0x1f, cpu.get_pc());
        std::printf("  pending priorities: %08X\n", bus.read32(int_tab));
        for (int v = 8; v < 16; ++v)
            std::printf("  vector %02X -> %08X\n", v, bus.read32(int_tab + 36 + (v - 8) * 4));
        std::printf("\n");
    }

    if (const char* ppm = std::getenv("SAGE_PPM"))
        if (write_ppm(ppm, bus.video()))
            std::printf("Frame written to %s\n\n", ppm);

    if (const char* ppm3d = std::getenv("SAGE_PPM3D")) {
        const std::uint32_t* px = bus.geometry().render();
        if (std::FILE* f = std::fopen(ppm3d, "wb")) {
            std::fprintf(f, "P6\n%d %d\n255\n", sage::Geometry::kWidth, sage::Geometry::kHeight);
            for (int y = 0; y < sage::Geometry::kHeight; ++y)
                for (int x = 0; x < sage::Geometry::kWidth; ++x) {
                    const std::uint32_t c = px[y * sage::Geometry::kDestStride + x];
                    const std::uint8_t rgb[3] = {static_cast<std::uint8_t>(c >> 16),
                                                 static_cast<std::uint8_t>(c >> 8),
                                                 static_cast<std::uint8_t>(c)};
                    std::fwrite(rgb, 1, 3, f);
                }
            std::fclose(f);
            std::printf("Geometry frame written to %s\n\n", ppm3d);
        }
    }

    if (std::getenv("SAGE_SHARC_PC")) {
        std::vector<std::pair<std::uint64_t, std::uint32_t>> v;
        v.reserve(sharc_pc_hist.size());
        for (const auto& [pc, n] : sharc_pc_hist)
            v.push_back({n, pc});
        std::sort(v.begin(), v.end(), std::greater<>());
        std::printf("SHARC hottest PCs (top 20 of %zu)\n", v.size());
        for (std::size_t i = 0; i < v.size() && i < 20; ++i)
            std::printf("  %06" PRIX32 " : %" PRIu64 "\n", v[i].second, v[i].first);
        std::printf("\n");
    }

    if (const char* sd = std::getenv("SAGE_SHARC_DUMP")) {
        unsigned base = 0, count = 32;
        std::sscanf(sd, "%x:%u", &base, &count);
        const std::uint32_t pc = bus.copro_pc();
        const unsigned from = (base ? base : pc) - 8;
        std::printf("SHARC PM dump around PC=0x%06X (mode1=%08X mode2=%08X dma=%08X irq=%08X idle=%d)\n",
                    pc, bus.sharc().mode1(), bus.sharc().mode2(), bus.sharc().dma_status(),
                    bus.sharc().irq_pending(), bus.sharc().idle() ? 1 : 0);
        for (unsigned i = 0; i < count; ++i) {
            const std::uint32_t w = from + i;
            const std::uint64_t op = bus.sharc().debug_pm48(w);
            std::printf("  %s PM[%06X] = %012llX\n", w == pc ? "->" : "  ", w,
                        (unsigned long long)op);
        }
        std::printf("\n");
    }

    std::printf("Execution summary\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("  Instructions    : %" PRIu64 "\n", instructions);
    std::printf("  Cycles          : %" PRIu64 "\n", cycles);
    std::printf("  Emulated time   : %.3f s\n",
                static_cast<double>(cycles) / static_cast<double>(kCpuClock));
    if (frames > 0)
        std::printf("  Frames          : %ld\n", frames_run);
    std::printf("  Final IP        : 0x%08" PRIX32 "\n", cpu.get_ip());
    std::printf("  Unmapped reads  : %" PRIu64 "\n", bus.unmapped_reads);
    std::printf("  Unmapped writes : %" PRIu64 "\n", bus.unmapped_writes);
    std::printf("  ROM writes      : %" PRIu64 "\n", bus.rom_writes);
    if (bus.is_model2a()) {
        const auto& act = bus.tgp().activity();
        std::printf("  Copro (TGP)     : %" PRIu64 " insns, PC=0x%04X, %u word(s) uploaded%s\n",
                    bus.tgp_instructions(), bus.tgp_pc(), bus.tgp_uploaded_words(),
                    bus.tgp_halted() ? " [halted]" : "");
        std::printf("  TGP activity    : %" PRIu64 " cmds in, %" PRIu64 " results out, "
                    "%" PRIu64 " table reads, %" PRIu64 " data ROM reads, "
                    "%" PRIu64 " buffer writes\n",
                    act.commands_received, act.results_sent, act.table_reads,
                    act.data_rom_reads, act.buffer_writes);
    } else {
        std::printf("  Copro (SHARC)   : %" PRIu64 " insns, PC=0x%06X%s\n",
                    bus.copro_instructions(), bus.copro_pc(),
                    bus.copro_fault().empty() ? "" : " [FAULT]");
        if (!bus.copro_fault().empty())
            std::printf("  Copro fault     : %s\n", bus.copro_fault().c_str());
    }

    // Sound board: the 68EC000 and the SCSP. The counters show whether the
    // sound program is running and the host has handed it any commands.
    {
        const auto& snd = bus.sound();
        const auto& scsp = snd.scsp();
        const auto& sc = scsp.stats();
        std::printf("  Sound 68EC000   : %s, PC=0x%06X, %" PRIu64 " cycles\n",
                    snd.present() ? "running" : "absent", snd.cpu().pc(), snd.cpu().cycles());
        std::printf("  SCSP (YMF292)   : %" PRIu64 " samples, %" PRIu64 " slot starts, peak %d\n",
                    sc.samples, sc.slot_starts, sc.peak_output);
        std::printf("  SCSP slots      : %u active of 32, %" PRIu64 " timer IRQs\n",
                    scsp.active_slots(), sc.timer_interrupts);
        std::printf("  Sound link      : %" PRIu64 " bytes to SCSP, %" PRIu64
                    " bytes from SCSP\n",
                    sc.midi_in_bytes, sc.midi_out_bytes);
        std::printf("  Sound bus       : %" PRIu64 " SCSP accesses, %" PRIu64
                    " sample reads, %" PRIu64 "/%" PRIu64 " unmapped R/W\n",
                    snd.counters().scsp_reads + snd.counters().scsp_writes,
                    snd.counters().sample_reads, snd.counters().unmapped_reads,
                    snd.counters().unmapped_writes);
        std::printf("  Host UART       : %" PRIu64 " bytes sent, %" PRIu64
                    " received, %" PRIu64 " overruns\n",
                    bus.uart().counters().bytes_sent, bus.uart().counters().bytes_received,
                    bus.uart().counters().overruns);
    }
    std::printf("------------------------------------------------------------\n");
    if (faulted)
        std::printf("  %s mapped and executed; CPU halted on an unimplemented opcode:\n    %s\n",
                    zip_path.c_str(), fault_msg.c_str());
    else
        std::printf("  %s mapped into simulated address space.\n  Ready for CPU execution.\n",
                    zip_path.c_str());
    return faulted ? 2 : 0;
}

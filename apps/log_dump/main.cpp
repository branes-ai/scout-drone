// log_dump: write a log's contents as CSV and YAML files, for plotting.
//
//   log_dump --log <run.brlog> --out-dir <dir>
//
// Writes into <dir> (created if needed, files overwritten):
//   header.yaml           provenance: baseline, hashes, reference flag
//   field.yaml            the field the log records, verbatim (SITL: the field flown)
//   vehicle.yaml          the vehicle file embedded in the log, verbatim
//   sim_config.yaml       the run's drawn randomization (SITL logs)
//   <channel>.csv         one per channel present: imu, camera, tick, truth,
//                         oracle_obstacles, state, reference, command
//
// The records are dumped by anatomy's log::dump_records() (log/dump.hpp),
// which documents the CSV formats; this file adds scout's own channel,
// oracle_obstacles (one row per obstacle seen), and the header files.

#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/log/dump.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

namespace scout = branes::scout;
namespace an = branes::anatomy;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "log_dump: %s\n", message.c_str());
    return 1;
}

// ── scout's channel ─────────────────────────────────────────────────

/// oracle_obstacles.csv: one row per obstacle seen; a frame with none contributes no row.
std::optional<std::string> obstacle_rows(const an::log::LogRecord& r, std::uint64_t& count) {
    using namespace an::log::csv;
    const auto o = scout::codec::decode_obstacles(r.payload);
    if (!o) {
        return std::nullopt;
    }
    const std::string time = start(r.t);
    std::string rows;
    for (std::uint32_t i = 0; i < o->count; ++i) {
        const scout::ObstacleObservation<double>& b = o->obstacles[i];
        rows += time;
        put(rows, std::uint64_t{o->count});
        put(rows, std::uint64_t{i});
        put(rows, std::int64_t{b.id});
        put(rows, b.position);
        put(rows, b.radius);
        put(rows, b.height);
        rows += '\n';
    }
    count = o->count;
    return rows;
}

std::string obstacle_columns() {
    return "t_ns,t_s,count,index,id" + an::log::csv::vec3("position") + ",radius,height";
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "out-dir"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    const auto out_dir_arg = args->value("out-dir");
    if (!in_path || !out_dir_arg) {
        return fail("usage: --log <run.brlog> --out-dir <dir>");
    }
    const std::filesystem::path out_dir(*out_dir_arg);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        return fail(out_dir.string() + ": " + ec.message());
    }

    auto reader = an::log::LogReader::open(*in_path, scout::log::kScoutChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    const an::log::LogHeader& h = reader->header();
    const std::string provenance = "baseline: " + h.baseline_name + "\ncomposition_hash: " + h.composition_hash +
                                   "\nfield_hash: " + h.world_hash + "\nvehicle_hash: " + h.vehicle_hash +
                                   "\nscenario_hash: " + h.scenario_hash +
                                   "\nreference: " + (h.reference ? "true" : "false") + "\n";
    for (const auto& [name, text] : {std::pair<const char*, const std::string&>{"header.yaml", provenance},
                                     {"field.yaml", h.world_yaml},
                                     {"vehicle.yaml", h.vehicle_yaml}}) {
        if (!an::log::write_text(out_dir / name, text)) {
            return fail((out_dir / name).string() + ": cannot write the file");
        }
    }

    const std::array<an::log::ChannelFormat, 1> formats = {an::log::ChannelFormat{
        scout::log::oracle_obstacles, "oracle_obstacles.csv", obstacle_columns(), &obstacle_rows}};
    const auto dumped = an::log::dump_records(*reader, out_dir, formats);
    if (!dumped) {
        return fail(dumped.error());
    }
    for (const an::log::DumpedFile& f : dumped->files) {
        std::printf("log_dump: %-20s %llu records, %llu rows -> %s\n",
                    f.file_name.c_str(),
                    static_cast<unsigned long long>(f.records),
                    static_cast<unsigned long long>(f.rows),
                    f.path.string().c_str());
    }
    std::printf(
        "log_dump: %llu records -> %s\n", static_cast<unsigned long long>(dumped->records), out_dir.string().c_str());
    return 0;
}

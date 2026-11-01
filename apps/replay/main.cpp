// replay_<baseline>: run a baseline over a recorded log.
//
//   replay_<baseline> --log <in.brlog> --out <out.brlog>
//                     [--field <field.yaml>] [--vehicle <vehicle.yaml>] [--verify]
//
// The field and the vehicle come from the log header (the field flown, its
// obstacles listed, and the vehicle file), so a log replays on its own.
// --field / --vehicle override them for a what-if replay, and the output is
// then marked non-reference; a --field must list its obstacles, since a field
// that generates them needs the run's seed.
//
// The output log holds the input events and this baseline's outputs. --verify
// also checks that the outputs equal the ones recorded in the input log,
// record for record; it needs a log recorded by this same baseline.

#include <branes/anatomy/harness/files.hpp>
#include <branes/anatomy/harness/replay.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/log/channels.hpp>

#include "scout_baseline.hpp"

#include <cstdio>
#include <span>
#include <string>

namespace scout = branes::scout;
namespace an = branes::anatomy;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "replay_%s: %s\n", std::string(scout::baseline::name).c_str(), message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {"verify"});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "out", "field", "vehicle", "verify"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    const auto out_path = args->value("out");
    if (!in_path || !out_path) {
        return fail("usage: --log <in.brlog> --out <out.brlog> [--field <yaml>] [--vehicle <yaml>] [--verify]");
    }
    const bool verify = args->has("verify");

    auto baseline =
        scout::harness::check_baseline(SCOUT_BASELINES_DIR, scout::baseline::name, scout::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }

    auto reader = an::log::LogReader::open(*in_path, scout::log::kScoutChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    const an::log::LogHeader& in_header = reader->header();
    if (verify && (in_header.baseline_name != scout::baseline::name ||
                   in_header.composition_hash != scout::baseline::composition_hash)) {
        return fail("--verify needs a log recorded by this baseline (log: " + in_header.baseline_name + ")");
    }

    an::log::LogHeader header = in_header;
    header.baseline_name = std::string(scout::baseline::name);
    header.composition_hash = std::string(scout::baseline::composition_hash);
    if (const auto f = args->value("field")) {
        const auto file = an::harness::read_with_hash(*f);
        if (!file) {
            return fail("cannot read " + *f);
        }
        header.world_yaml = file->text;
        header.world_hash = file->hash;
        header.reference = false;
    }
    if (const auto v = args->value("vehicle")) {
        const auto file = an::harness::read_with_hash(*v);
        if (!file) {
            return fail("cannot read " + *v);
        }
        header.vehicle_yaml = file->text;
        header.vehicle_hash = file->hash;
        header.reference = false;
    }
    if (verify && !header.reference) {
        return fail("--verify cannot be combined with --field / --vehicle overrides");
    }

    auto field = scout::harness::field_from_log(header, *in_path);
    if (!field) {
        return fail(field.error());
    }
    auto vehicle = an::config::parse_vehicle(header.vehicle_yaml, "vehicle (from " + *in_path + ")");
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }
    auto built = scout::harness::make_stack<scout::baseline::Composition<double>>(*field, *vehicle, *baseline);
    if (!built) {
        return fail(built.error());
    }
    auto& stack = *built;

    auto writer = an::log::LogWriter::open(*out_path, header);
    if (!writer) {
        return fail(writer.error().path + ": " + writer.error().message);
    }
    // oracle_obstacles is the stack's input (ScoutProfile), so there are no pass-through channels.
    const auto kept = an::harness::replay_log(*reader, stack, std::span<const an::log::Channel>{}, *writer, verify);
    if (!kept) {
        return fail(kept.error());
    }
    const auto records = writer->record_count();
    if (auto closed = writer->close(); !closed) {
        return fail(closed.error().path + ": " + closed.error().message);
    }

    if (verify) {
        const auto verified = an::harness::verify_outputs(*kept);
        if (!verified) {
            return fail(verified.error());
        }
        std::printf("replay_%s: verified %zu outputs\n", std::string(scout::baseline::name).c_str(), *verified);
    }
    std::printf("replay_%s: %llu records -> %s%s\n",
                std::string(scout::baseline::name).c_str(),
                static_cast<unsigned long long>(records),
                out_path->c_str(),
                header.reference ? "" : " (non-reference)");
    return 0;
}

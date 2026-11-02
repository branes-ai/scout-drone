// trace_export: write a SITL log as a reflex-trace/1 replay, for the docs site.
//
//   trace_export --log <run.brlog> --scenario <scenario.yaml> --out <replay.json>
//                [--rate <Hz>] [--commit <hash>]
//
// The replay plays in reflex's trace player (@branes-ai/trace-player), as on
// docs/guides/forest-replay.mdx, with the field the log records as its scene:
// the trees, the goal and the geofence (sim/replay_trace.hpp). --scenario is
// the file the log was recorded from, as a repo-relative path: it must hash
// to the log's scenario_hash, and it gives the trace its scenario, seed and
// physics step, and the scorer its collision radius and speed limit, which
// the log doesn't carry. The trace samples the truth stream at --rate
// (default 20 Hz), with takeoff and the run's outcome as events. --commit
// names the tree that recorded the run (tools/gen_replay.sh passes
// `git describe`).

#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/util/sha256.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/sim/replay_trace.hpp>

#include <cmath>
#include <cstdio>
#include <exception>
#include <fstream>
#include <string>

namespace scout = branes::scout;
namespace an = branes::anatomy;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "trace_export: %s\n", message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "scenario", "out", "rate", "commit"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    const auto scenario_path = args->value("scenario");
    const auto out_path = args->value("out");
    if (!in_path || !scenario_path || !out_path) {
        return fail("usage: --log <run.brlog> --scenario <scenario.yaml> --out <replay.json> [--rate <Hz>] "
                    "[--commit <hash>]");
    }
    scout::sim::ReplayOptions options;
    if (const auto r = args->value("rate")) {
        const auto v = an::harness::parse_number<double>(*r);
        if (!v || !std::isfinite(*v) || !(*v > 0)) {
            return fail("--rate must be a positive number");
        }
        options.rate_hz = *v;
    }

    auto scenario = scout::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    const auto scenario_hash = an::sha256_file_hex(*scenario_path);
    if (!scenario_hash) {
        return fail("cannot read " + *scenario_path);
    }
    auto reader = an::log::LogReader::open(*in_path, scout::log::kScoutChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    const an::log::LogHeader& h = reader->header();
    if (!h.reference) {
        return fail(*in_path + " is a what-if replay (its inputs were overridden): export reference runs only");
    }
    if (h.scenario_hash != *scenario_hash) {
        return fail(*in_path + " was not recorded from " + *scenario_path + " (scenario_hash differs)");
    }
    options.collision_radius_m = scout::config::collision_radius_m(*scenario);
    options.speed_limit_m_s = scenario->speed_limit_m_s;

    scout::sim::ReplayProvenance prov;
    prov.commit = args->value("commit").value_or("unknown");
    prov.baseline = h.baseline_name;
    prov.scenario = *scenario_path;
    prov.seed = scenario->seed;
    prov.dt_sim = 1 / scenario->sim.physics_rate_hz;
    try {
        auto trace = scout::sim::replay_trace(*reader, prov, options);
        if (!trace) {
            return fail(*in_path + ": " + trace.error());
        }
        const std::string json = trace->to_json();
        std::ofstream out(*out_path, std::ios::binary | std::ios::trunc);
        out << json;
        if (!out.flush()) {
            return fail("cannot write " + *out_path);
        }
        std::printf("trace_export: %zu samples -> %s\n", trace->samples(), out_path->c_str());
    } catch (const std::exception& e) {
        // The writer refuses a trace that breaks the format (reflex-trace/1, W1–W9).
        return fail(e.what());
    }
    return 0;
}

// sitl_batch_<baseline>: run a baseline over many seeds of a scenario and score them.
//
//   sitl_batch_<baseline> --scenario <scenario.yaml> --seeds <n>
//                         [--first-seed <s>] [--min-success <fraction>]
//                         [--set <key=value[,key=value...]>]
//                         [--jobs <j>] [--jsonl <scores.jsonl>]
//
// Run i uses seed first-seed + i (default: the scenario's seed), which grows
// its own field, so any run reproduces on its own with sitl_<baseline> and
// that seed. Each run is the closed loop of sitl_<baseline> without a log
// (sim/batch.hpp, run_one), scored as it goes (sim::Scorer, with the
// scenario's collision radius and speed limit), and stopped once its outcome
// is settled. The runs are independent, so --jobs changes the wall time,
// never the scores.
//
// --set overrides numeric parameters of the baseline for this batch (the
// cruise speed of a speed sweep: docs/plan/bring-up/); each key must be one
// the baseline file has. sitl and replay don't take it, so a reference log
// always has the file's parameters.
//
// It prints a summary (the success rate, each outcome's count, the time to
// the goal, the minimum clearance, the path length and the peak speed),
// optionally one JSON line per run, and exits nonzero if the success fraction
// is below --min-success (default 1).

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/harness/parallel.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/sim/batch.hpp>
#include <branes/scout/sim/score.hpp>

#include "scout_baseline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace scout = branes::scout;
namespace an = branes::anatomy;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "sitl_batch_%s: %s\n", std::string(scout::baseline::name).c_str(), message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"scenario", "seeds", "first-seed", "min-success", "set", "jobs", "jsonl"})) {
        return fail(*bad);
    }
    const auto scenario_path = args->value("scenario");
    const auto seeds_arg = args->value("seeds");
    if (!scenario_path || !seeds_arg) {
        return fail("usage: --scenario <yaml> --seeds <n> [--first-seed <s>] [--min-success <f>] "
                    "[--set <key=value,...>] [--jobs <j>] [--jsonl <out>]");
    }
    const auto seeds = an::harness::parse_number<std::uint64_t>(*seeds_arg);
    if (!seeds || *seeds == 0 || *seeds > 1'000'000) {
        return fail("--seeds must be an integer in [1, 1000000]");
    }
    double min_success = 1.0;
    if (const auto m = args->value("min-success")) {
        const auto v = an::harness::parse_number<double>(*m);
        if (!v || !(*v >= 0 && *v <= 1)) {
            return fail("--min-success must be a fraction in [0, 1]");
        }
        min_success = *v;
    }
    unsigned jobs = std::max(1u, std::thread::hardware_concurrency());
    if (const auto j = args->value("jobs")) {
        const auto v = an::harness::parse_number<unsigned>(*j);
        if (!v || *v == 0 || *v > 1024) {
            return fail("--jobs must be an integer in [1, 1024]");
        }
        jobs = *v;
    }

    auto baseline =
        scout::harness::check_baseline(SCOUT_BASELINES_DIR, scout::baseline::name, scout::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }
    const auto overrides = args->value("set");
    if (overrides) {
        auto set = scout::harness::override_parameters(*baseline, *overrides);
        if (!set) {
            return fail(set.error());
        }
        baseline = std::move(set);
    }
    const double control_hz =
        an::config::numeric_parameter(*baseline, "control_rate_hz").value_or(an::config::kDefaultControlRateHz);
    auto scenario = scout::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    std::uint64_t first = scenario->seed;
    if (const auto f = args->value("first-seed")) {
        const auto v = an::harness::parse_number<std::uint64_t>(*f);
        if (!v) {
            return fail("--first-seed must be a non-negative integer");
        }
        first = *v;
    }
    if (first > UINT64_MAX - (*seeds - 1)) {
        return fail("--first-seed + --seeds overflows the seed range");
    }
    auto field_config = scout::config::load_field(scenario->world_path);
    if (!field_config) {
        return fail(field_config.error().to_string());
    }
    auto vehicle = an::config::load_vehicle(scenario->vehicle_path);
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }

    std::vector<scout::sim::Run> runs(static_cast<std::size_t>(*seeds));
    an::harness::for_each_index(*seeds, jobs, [&](std::uint64_t i) {
        runs[i] = scout::sim::run_one<scout::baseline::Composition<double>>(
            *baseline, *vehicle, *field_config, *scenario, control_hz, first + i);
    });

    std::ofstream jsonl;
    if (const auto path = args->value("jsonl")) {
        jsonl.open(*path, std::ios::binary);
        if (!jsonl) {
            return fail("cannot write " + *path);
        }
    }
    constexpr std::array outcomes = {scout::sim::Outcome::reached,
                                     scout::sim::Outcome::collided,
                                     scout::sim::Outcome::geofence,
                                     scout::sim::Outcome::speed_limit,
                                     scout::sim::Outcome::timed_out};
    std::map<scout::sim::Outcome, std::uint64_t> counts;
    std::map<std::string, std::uint64_t> failures;
    std::vector<double> times;
    std::vector<double> clearances;
    std::vector<double> paths;
    std::vector<double> speeds;
    for (const scout::sim::Run& r : runs) {
        if (!r.error.empty()) {
            return fail("seed " + std::to_string(r.seed) + ": " + r.error);
        }
        const scout::sim::Score& s = r.score;
        if (jsonl.is_open()) {
            jsonl << an::sim::to_json(r) << "\n";
        }
        ++counts[s.outcome];
        if (s.success) {
            times.push_back(s.time_to_goal_s);
        } else {
            ++failures[an::sim::failure_reason(s.failure)];
            std::printf("  seed %llu: %s\n", static_cast<unsigned long long>(r.seed), s.failure.c_str());
        }
        if (std::isfinite(s.min_clearance_m)) {
            clearances.push_back(s.min_clearance_m);
        }
        paths.push_back(s.path_length_m);
        speeds.push_back(s.max_speed_m_s);
    }
    const std::uint64_t successes = counts[scout::sim::Outcome::reached];
    const double rate = static_cast<double>(successes) / static_cast<double>(*seeds);
    std::printf("sitl_batch_%s: %s, seeds %llu..%llu\n",
                std::string(scout::baseline::name).c_str(),
                scenario->name.c_str(),
                static_cast<unsigned long long>(first),
                static_cast<unsigned long long>(first + *seeds - 1));
    if (overrides) {
        std::printf("  overrides     %s\n", overrides->c_str());
    }
    std::printf("  success       %llu / %llu = %.2f %% (need >= %.2f %%)\n",
                static_cast<unsigned long long>(successes),
                static_cast<unsigned long long>(*seeds),
                100.0 * rate,
                100.0 * min_success);
    std::printf("  outcomes     ");
    for (const scout::sim::Outcome o : outcomes) {
        std::printf(
            " %s %llu", std::string(scout::sim::to_string(o)).c_str(), static_cast<unsigned long long>(counts[o]));
    }
    std::printf("\n");
    if (times.empty()) {
        std::printf("  time to goal  none reached\n");
    } else {
        std::printf("  time to goal  p50 %.3f s, p95 %.3f s (%zu reached)\n",
                    an::sim::percentile(times, 0.5),
                    an::sim::percentile(times, 0.95),
                    times.size());
    }
    if (clearances.empty()) {
        std::printf("  clearance     none: no run flew among obstacles\n");
    } else {
        std::printf("  clearance     p5 %.3f m, p50 %.3f m (%zu runs among obstacles)\n",
                    an::sim::percentile(clearances, 0.05),
                    an::sim::percentile(clearances, 0.5),
                    clearances.size());
    }
    std::printf("  path length   p50 %.3f m\n", an::sim::percentile(paths, 0.5));
    const double speed_p50 = an::sim::percentile(speeds, 0.5);  // sorts speeds
    std::printf("  max speed     p50 %.3f m/s, max %.3f m/s", speed_p50, speeds.back());
    if (scenario->speed_limit_m_s) {
        std::printf(" (limit %.3f m/s)", *scenario->speed_limit_m_s);
    }
    std::printf("\n");
    for (const auto& [why, n] : failures) {
        std::printf("  failures      %llu x %s\n", static_cast<unsigned long long>(n), why.c_str());
    }
    return rate >= min_success ? 0 : 1;
}

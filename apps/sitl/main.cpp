// sitl_<baseline>: run a baseline against a simulated vehicle and write the log.
//
//   sitl_<baseline> --scenario <scenario.yaml> --out <run.brlog>
//
// The scenario names the field and the vehicle; --field and --vehicle are
// rejected, so there is one source of truth. The field is the scenario's
// field file grown with its seed (config::load_scenario_field), and the
// vehicle flies on anatomy's closed loop with scout's obstacle oracle
// (sim::Simulator). The log holds every input event (the obstacles seen
// among them), the sim channels (truth, the drawn randomization) and every
// output, so replay_<baseline> reproduces the run, and score_log scores it.
// Its header records the field flown, its obstacles listed
// (harness::flown_field_yaml), since a generated field needs the seed.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/harness/files.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/sim/sitl_loop.hpp>
#include <branes/anatomy/util/sha256.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/sim/simulator.hpp>

#include "scout_baseline.hpp"

#include <cstdio>
#include <span>
#include <string>

namespace scout = branes::scout;
namespace an = branes::anatomy;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "sitl_%s: %s\n", std::string(scout::baseline::name).c_str(), message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (args->has("field") || args->has("vehicle")) {
        return fail("--field / --vehicle are not accepted: the scenario names them");
    }
    if (auto bad = args->only({"scenario", "out"})) {
        return fail(*bad);
    }
    const auto scenario_path = args->value("scenario");
    const auto out_path = args->value("out");
    if (!scenario_path || !out_path) {
        return fail("usage: --scenario <scenario.yaml> --out <run.brlog>");
    }

    auto baseline =
        scout::harness::check_baseline(SCOUT_BASELINES_DIR, scout::baseline::name, scout::baseline::composition_hash);
    if (!baseline) {
        return fail(baseline.error());
    }
    const double control_hz =
        an::config::numeric_parameter(*baseline, "control_rate_hz").value_or(an::config::kDefaultControlRateHz);

    auto scenario = scout::config::load_scenario(*scenario_path);
    if (!scenario) {
        return fail(scenario.error().to_string());
    }
    auto field_config = scout::config::load_field(scenario->world_path);
    if (!field_config) {
        return fail(field_config.error().to_string());
    }
    auto field = scout::config::field_for_seed(*field_config, scenario->seed);
    if (!field) {
        return fail(field.error());
    }
    auto vehicle = an::config::load_vehicle(scenario->vehicle_path);
    if (!vehicle) {
        return fail(vehicle.error().to_string());
    }
    const auto vehicle_file = an::harness::read_with_hash(scenario->vehicle_path);
    const auto scenario_hash = an::sha256_file_hex(*scenario_path);
    if (!vehicle_file || !scenario_hash) {
        return fail("cannot read the scenario's input files");
    }

    // Validate everything before LogWriter::open truncates --out.
    auto sim = scout::sim::Simulator::create(*vehicle, *field, *scenario, control_hz);
    if (!sim) {
        return fail(sim.error());
    }
    auto built = scout::harness::make_stack<scout::baseline::Composition<double>>(*field, *vehicle, *baseline);
    if (!built) {
        return fail(built.error());
    }
    auto& stack = *built;

    an::log::LogHeader header;
    header.baseline_name = std::string(scout::baseline::name);
    header.composition_hash = std::string(scout::baseline::composition_hash);
    header.world_yaml = scout::harness::flown_field_yaml(*field_config, *field);
    header.world_hash = an::sha256_hex(header.world_yaml);
    header.vehicle_hash = vehicle_file->hash;
    header.scenario_hash = *scenario_hash;
    header.vehicle_yaml = vehicle_file->text;
    auto writer = an::log::LogWriter::open(*out_path, header);
    if (!writer) {
        return fail(writer.error().path + ": " + writer.error().message);
    }

    // oracle_obstacles is the stack's input (ScoutProfile), so there are no pass-through channels.
    const auto ticks = an::sim::run_sitl(*sim, stack, std::span<const an::log::Channel>{}, *writer);
    if (!ticks) {
        return fail(ticks.error());
    }
    const auto records = writer->record_count();
    if (auto closed = writer->close(); !closed) {
        return fail(closed.error().path + ": " + closed.error().message);
    }
    std::printf("sitl_%s: %s, %llu ticks, %llu records -> %s\n",
                std::string(scout::baseline::name).c_str(),
                scenario->name.c_str(),
                static_cast<unsigned long long>(*ticks),
                static_cast<unsigned long long>(records),
                out_path->c_str());
    return 0;
}

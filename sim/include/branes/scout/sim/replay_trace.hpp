#pragma once
/// @file replay_trace.hpp
/// @brief A SITL log as a reflex-trace/1 replay with the field as its scene, for the docs site.
///
/// The docs site plays a replay in the player reflex publishes,
/// `@branes-ai/trace-player` (reflex docs/standards/animation, "Boundary with
/// mission SITL"), and the trace is written with reflex's writer,
/// branes::reflex_trace. replay_trace() reads a log once:
///
///  - **Channels**, sampled from the truth on a uniform grid: the format's
///    reserved position and velocity [m, m/s, world], attitude [w, x, y, z,
///    body → world], body_rates [rad/s, body], rotor_speeds [rad/s], and
///    rotor_thrusts [N], k_f·ω² with the vehicle's thrust coefficient. With
///    references in the log, also position_setpoint [m]: the latest reference
///    sample logged before the trace sample, and the first truth position
///    before the first reference.
///  - **The scene**, the field the log records (harness::field_from_log):
///    the geofence as the boundary, each obstacle a cylinder drawn as a tree,
///    and the goal a sphere of its radius. The cylinders are the obstacles
///    the scorer tests, so the player's collision shapes are the scorer's.
///  - **Events**: takeoff, and the run's outcome as the Scorer judges it
///    (sim/score.hpp), fed every truth sample: the goal reached, or the first
///    failure, with the obstacle hit.
///  - **Span**: from the first truth sample to the end of the log, or, once
///    the outcome is settled, to `tail_s` after it.
///
/// The scorer takes the scenario's collision radius and speed limit, which
/// the log doesn't carry (assumption 9 of docs/plan/bring-up/), from the
/// options.

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/reflex/trace/writer.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/sim/score.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace branes::scout::sim {

/// What a replay is (reflex-trace/1 provenance).
struct ReplayProvenance {
    std::string commit;    ///< short git hash of the tree that recorded and exported the run
    std::string baseline;  ///< the log's baseline, e.g. "s0-gt-pid" (the trace's block)
    std::string scenario;  ///< repo-relative path of the scenario file
    std::uint64_t seed{0};
    double dt_sim{0};  ///< physics step [s]
};

/// How to sample and score a run.
struct ReplayOptions {
    double rate_hz{20};                     ///< sample rate; 1e9 / rate_hz must be a multiple of the physics step [ns]
    double tail_s{2};                       ///< kept after the outcome is settled [s]
    double collision_radius_m{0};           ///< the scenario's sim.collision_radius_m
    std::optional<double> speed_limit_m_s;  ///< the scenario's speed limit
};

/// The failure text without its time: "obstacle collision at t=3.658 s (obstacle 5)"
/// becomes "obstacle collision (obstacle 5)".
[[nodiscard]] inline std::string without_time(std::string s) {
    if (const auto at = s.find(" at t="); at != std::string::npos) {
        const auto end = s.find(" s", at + 6);
        s.erase(at, end == std::string::npos ? std::string::npos : end + 2 - at);
    }
    return s;
}

/// Reads every record of `reader` and writes it as a replay. Fails on a log
/// with no truth, with truth samples off the sample grid, with an
/// undecodable record, or whose header doesn't record a field and a vehicle.
/// Pre: options.rate_hz > 0; options.tail_s ≥ 0; collision_radius_m ≥ 0;
/// prov.dt_sim > 0.
[[nodiscard]] inline anatomy::result<reflex::trace::Trace, std::string>
replay_trace(anatomy::log::LogReader& reader, const ReplayProvenance& prov, const ReplayOptions& options) {
    assert(options.rate_hz > 0 && options.tail_s >= 0 && options.collision_radius_m >= 0 && prov.dt_sim > 0);  // Pre
    const anatomy::log::LogHeader& h = reader.header();
    auto field = harness::field_from_log(h, "the log");
    if (!field) {
        return field.error();
    }
    auto vehicle = anatomy::config::parse_vehicle(h.vehicle_yaml, "vehicle (from the log header)");
    if (!vehicle) {
        return vehicle.error().to_string();
    }
    const double k_f = vehicle->rotor.thrust_coefficient;
    const auto step_ns = static_cast<std::int64_t>(std::llround(1e9 / options.rate_hz));
    if (step_ns <= 0) {
        return std::string("the sample rate is too high");
    }
    const auto tail_ns = static_cast<std::int64_t>(std::llround(options.tail_s * 1e9));

    // One pass over the log: the samples, the setpoint, and the events. The
    // setpoint channel is declared only if the log has a reference, so the
    // trace is built after the pass.
    struct Sample {
        anatomy::TruthState<double> x;
        anatomy::Vec3<double> setpoint;
    };
    std::vector<Sample> samples;
    Scorer scorer(*field, options.collision_radius_m, options.speed_limit_m_s);
    std::optional<anatomy::Timestamp> t0;
    std::optional<anatomy::Timestamp> takeoff;
    std::optional<anatomy::Timestamp> settled;
    std::optional<anatomy::Vec3<double>> setpoint;
    bool any_reference = false;
    for (;;) {
        auto rec = reader.next();
        if (!rec) {
            return rec.error().path + ": " + rec.error().message;
        }
        if (!rec->has_value()) {
            break;
        }
        const anatomy::log::LogRecord& r = **rec;
        if (r.channel == anatomy::log::Channel::reference) {
            const auto ref = anatomy::codec::decode_reference(r.payload);
            if (!ref) {
                return "undecodable reference record at t=" + std::to_string(r.t.ns);
            }
            if (ref->count > 0) {
                setpoint = ref->samples[0].position;
                any_reference = true;
            }
        } else if (r.channel == anatomy::log::Channel::state) {
            const auto x = anatomy::codec::decode_state(r.payload);
            if (!x) {
                return "undecodable state record at t=" + std::to_string(r.t.ns);
            }
            scorer.on_estimate(*x);
        } else if (r.channel == anatomy::log::Channel::truth) {
            const auto x = anatomy::codec::decode_truth(r.payload);
            if (!x) {
                return "undecodable truth record at t=" + std::to_string(r.t.ns);
            }
            if (!samples.empty() && x->t < samples.back().x.t) {
                return "truth record at t=" + std::to_string(x->t.ns) + " is earlier than the one before it";
            }
            if (settled && x->t.ns > settled->ns + tail_ns) {
                break;  // the tail after the outcome is complete
            }
            scorer.on_truth(*x);
            t0 = t0.value_or(x->t);
            if (!takeoff && !x->on_ground) {
                takeoff = x->t;
            }
            if (!settled && (scorer.complete() || scorer.failed())) {
                settled = x->t;
            }
            const std::int64_t since = x->t.ns - t0->ns;
            if (since % step_ns != 0) {
                continue;
            }
            if (since / step_ns != static_cast<std::int64_t>(samples.size())) {
                return "the truth stream has no sample at t=" +
                       std::to_string(t0->ns + static_cast<std::int64_t>(samples.size()) * step_ns) +
                       ": the sample rate must divide the physics rate";
            }
            samples.push_back({*x, setpoint.value_or(samples.empty() ? x->position : samples.front().x.position)});
        }
    }
    if (samples.empty()) {
        return std::string("the log has no truth channel: export SITL logs only");
    }

    reflex::trace::Trace tr({.commit = prov.commit,
                             .block = prov.baseline,
                             .scenario = prov.scenario,
                             .seed = static_cast<long>(prov.seed),
                             .compiler = {},
                             .dt_sim = prov.dt_sim,
                             .dt_trace = static_cast<double>(step_ns) / 1e9});
    tr.vehicle_arm_length(vehicle->arm_length);
    for (const char* name : {"position", "velocity", "attitude", "body_rates", "rotor_speeds", "rotor_thrusts"}) {
        tr.channel(name, reflex::trace::reserved_width(name));
    }
    if (any_reference) {
        tr.channel("position_setpoint", 3);
    }
    for (std::size_t k = 0; k < samples.size(); ++k) {
        const anatomy::TruthState<double>& x = samples[k].x;
        tr.sample(static_cast<double>(static_cast<std::int64_t>(k) * step_ns) / 1e9);
        tr.set("position", {x.position[0], x.position[1], x.position[2]});
        tr.set("velocity", {x.velocity[0], x.velocity[1], x.velocity[2]});
        tr.set("attitude", {x.attitude.w, x.attitude.x, x.attitude.y, x.attitude.z});
        tr.set("body_rates", {x.body_rates[0], x.body_rates[1], x.body_rates[2]});
        const std::array<double, 4>& w = x.rotor_speeds;
        tr.set("rotor_speeds", {w[0], w[1], w[2], w[3]});
        tr.set("rotor_thrusts", {k_f * w[0] * w[0], k_f * w[1] * w[1], k_f * w[2] * w[2], k_f * w[3] * w[3]});
        if (any_reference) {
            const anatomy::Vec3<double>& s = samples[k].setpoint;
            tr.set("position_setpoint", {s[0], s[1], s[2]});
        }
    }

    // The scene: the field, as the scorer sees it.
    const anatomy::Geofence<double>& g = field->geofence;
    tr.entity({.id = "geofence",
               .role = "boundary",
               .shape = reflex::trace::Shape::box(g.max[0] - g.min[0], g.max[1] - g.min[1], g.max[2] - g.min[2]),
               .pose = {.position = std::array<double, 3>{(g.min[0] + g.max[0]) / 2,
                                                          (g.min[1] + g.max[1]) / 2,
                                                          (g.min[2] + g.max[2]) / 2},
                        .attitude = std::nullopt},
               .visual = std::nullopt,
               .label = "the geofence"});
    for (std::uint32_t i = 0; i < field->obstacle_count; ++i) {
        const Obstacle<double>& o = field->obstacles[i];
        tr.entity({.id = "obstacle-" + std::to_string(i),
                   .role = "obstacle",
                   .shape = reflex::trace::Shape::cylinder(o.radius, o.height),
                   .pose = {.position = std::array<double, 3>{o.base[0], o.base[1], o.base[2] + o.height / 2},
                            .attitude = std::nullopt},
                   .visual = reflex::trace::Visual::tree(),
                   .label = "obstacle " + std::to_string(i)});
    }
    tr.entity(
        {.id = "goal",
         .role = "goal",
         .shape = reflex::trace::Shape::sphere(field->goal.radius),
         .pose = {.position =
                      std::array<double, 3>{field->goal.position[0], field->goal.position[1], field->goal.position[2]},
                  .attitude = std::nullopt},
         .visual = std::nullopt,
         .label = "the goal"});

    // The events, within the samples kept.
    const double end_s = static_cast<double>(static_cast<std::int64_t>(samples.size() - 1) * step_ns) / 1e9;
    // Times in s as ns / 1e9, so a whole number of ms prints as one (0.027, not 0.027000000000000003).
    const auto at = [&](anatomy::Timestamp t) { return static_cast<double>(t.ns - t0->ns) / 1e9; };
    if (takeoff && at(*takeoff) <= end_s) {
        tr.event(at(*takeoff), "takeoff");
    }
    if (settled && at(*settled) <= end_s) {
        const Score s = scorer.score();
        tr.event(at(*settled), s.success ? "reached the goal" : without_time(s.failure));
    }
    return tr;
}

}  // namespace branes::scout::sim

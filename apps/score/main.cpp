// score_log: score a SITL log against the field it flew.
//
//   score_log --log <run.brlog> [--collision-radius <m>] [--speed-limit <m/s>]
//
// Reads the field from the log header (the field flown, its obstacles listed:
// harness::field_from_log) and the true state from the `truth` channel, and
// prints one JSON line: the outcome, the time to the goal, the minimum
// clearance, the path length, the peak speed, the distance from the goal and
// the first failure (sim/score.hpp). The header doesn't carry the scenario,
// so its collision radius (`sim.collision_radius_m`, default 0.25 m) and
// speed limit are given here. Any baseline's log scores the same way, so
// the scores compare like with like.

#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/sim/score.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

namespace scout = branes::scout;
namespace an = branes::anatomy;

namespace {

int fail(const std::string& message) {
    std::fprintf(stderr, "score_log: %s\n", message.c_str());
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    auto args = an::harness::Args::parse(argc, argv, {});
    if (!args) {
        return fail(args.error());
    }
    if (auto bad = args->only({"log", "collision-radius", "speed-limit"})) {
        return fail(*bad);
    }
    const auto in_path = args->value("log");
    if (!in_path) {
        return fail("usage: --log <run.brlog> [--collision-radius <m>] [--speed-limit <m/s>]");
    }
    double radius = scout::config::kScoutWorldSim[2].fallback;
    if (const auto r = args->value("collision-radius")) {
        const auto v = an::harness::parse_number<double>(*r);
        if (!v || !std::isfinite(*v) || !(*v >= 0)) {
            return fail("--collision-radius must be a number >= 0");
        }
        radius = *v;
    }
    std::optional<double> speed_limit;
    if (const auto l = args->value("speed-limit")) {
        const auto v = an::harness::parse_number<double>(*l);
        if (!v || !std::isfinite(*v) || !(*v > 0)) {
            return fail("--speed-limit must be a positive number");
        }
        speed_limit = *v;
    }
    auto reader = an::log::LogReader::open(*in_path, scout::log::kScoutChannels);
    if (!reader) {
        return fail(reader.error().path + ": " + reader.error().message);
    }
    auto field = scout::harness::field_from_log(reader->header(), *in_path);
    if (!field) {
        return fail(field.error());
    }
    scout::sim::Scorer scorer(*field, radius, speed_limit);
    std::uint64_t truths = 0;
    std::optional<an::Timestamp> last_truth;
    for (;;) {
        auto rec = reader->next();
        if (!rec) {
            return fail(rec.error().path + ": " + rec.error().message);
        }
        if (!rec->has_value()) {
            break;
        }
        const an::log::LogRecord& r = **rec;
        if (r.channel == an::log::Channel::truth) {
            const auto x = an::codec::decode_truth(r.payload);
            if (!x) {
                return fail("undecodable truth record at t=" + std::to_string(r.t.ns));
            }
            // The log is untrusted input: check the scorer's I1 precondition here.
            if (last_truth && x->t < *last_truth) {
                return fail("truth record at t=" + std::to_string(x->t.ns) +
                            " is earlier than the previous one (t=" + std::to_string(last_truth->ns) + ")");
            }
            last_truth = x->t;
            scorer.on_truth(*x);
            ++truths;
        } else if (r.channel == an::log::Channel::state) {
            const auto x = an::codec::decode_state(r.payload);
            if (!x) {
                return fail("undecodable state record at t=" + std::to_string(r.t.ns));
            }
            scorer.on_estimate(*x);
        }
    }
    if (truths == 0) {
        return fail(*in_path + " has no truth channel: score SITL logs only");
    }
    std::printf("%s\n", scout::sim::to_json(scorer.score()).c_str());
    return 0;
}

#pragma once
/// @file score.hpp
/// @brief Scorer: a scout run's outcome and measures, from its truth.
///
/// Fed the truth in time order (and the stack's estimates, which S0 doesn't
/// score), it judges the run against the field (docs/plan/bring-up/):
///
///  - **Takeoff** is the first truth sample off the ground. The measures run
///    from it until the run is complete.
///  - **Reached:** a sample within the goal's radius of its position. The run
///    is then complete, and the samples after it are not scored, so a vehicle
///    that holds at the goal can't fail the run (assumption 5).
///  - **Collided:** a sample whose clearance to an obstacle is ≤ 0, or ground
///    contact after takeoff. The clearance is the vehicle centre's distance to
///    the obstacle (msg/field.hpp, distance()) less the collision radius
///    (`sim.collision_radius_m`). The truth comes at the physics rate, so a
///    collision is judged at those samples, not between them.
///  - **Left the geofence:** a sample outside it.
///  - **Over the speed limit:** a true speed above the scenario's
///    speed_limit_m_s after takeoff.
///  - **Timed out:** none of these by the end of the run.
///
/// The first failure is the run's outcome. Once failed, the run can't recover:
/// the samples after it are not scored, and a later sample at the goal doesn't
/// count. At equal times a failure comes before the goal: a sample that both
/// reaches the goal and fails, fails.
///
/// The measures: the time from takeoff to the goal; the minimum clearance to
/// any obstacle (none in a field without obstacles); the path length, the sum
/// of the distances between consecutive samples; the peak true speed; and the
/// distance from the goal at the last sample scored.
///
/// Model
/// -----
///   field_  : Field<double> [owned, immutable]
///   radius_ : double [immutable] — the collision radius [m]
///   limit_  : optional speed limit [m/s] [immutable]
///   prev_   : the last sample scored
///   took_off_, start_ : takeoff, and its time
///   reached_ : the goal has been reached
///   score_  : the measures so far, and the first failure
///
/// Invariants
/// ----------
///   I1: truth samples arrive in non-decreasing time
///   I2: complete() ⇒ !failed(), and failed() ⇒ !complete(); neither changes once set
///   I3: score_.min_clearance_m is the minimum over the samples scored after takeoff

#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/scout/msg/field.hpp>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <string_view>

namespace branes::scout::sim {

/// How a run ended.
enum class Outcome : std::uint8_t {
    timed_out,    ///< none of the others by the end of the run
    reached,      ///< within the goal radius
    collided,     ///< an obstacle, or the ground after takeoff
    geofence,     ///< left the geofence
    speed_limit,  ///< over the scenario's speed limit
};

[[nodiscard]] constexpr std::string_view to_string(Outcome o) noexcept {
    switch (o) {
    case Outcome::timed_out:
        return "timed_out";
    case Outcome::reached:
        return "reached";
    case Outcome::collided:
        return "collided";
    case Outcome::geofence:
        return "geofence";
    case Outcome::speed_limit:
        return "speed_limit";
    }
    return "unknown";
}

struct Score {
    Outcome outcome{Outcome::timed_out};
    bool success{false};                                              ///< outcome == reached
    double time_to_goal_s{-1};                                        ///< from takeoff; −1 = not reached
    double min_clearance_m{std::numeric_limits<double>::infinity()};  ///< +∞ = no obstacle (or no sample after takeoff)
    double path_length_m{0};                                          ///< from takeoff
    double max_speed_m_s{0};                                          ///< from takeoff
    double goal_distance_m{-1};                                       ///< at the last sample scored; −1 = no sample
    std::string failure;                                              ///< the first failure, or why the run timed out
};

class Scorer {
public:
    /// Pre: valid(field); collision_radius_m finite and ≥ 0; speed_limit, if given, > 0.
    Scorer(const Field<double>& field, double collision_radius_m, std::optional<double> speed_limit = std::nullopt)
        : field_(field), radius_(collision_radius_m), limit_(speed_limit) {
        assert(valid(field_));                           // Pre
        assert(std::isfinite(radius_) && radius_ >= 0);  // Pre
        assert(!limit_ || *limit_ > 0);                  // Pre
    }

    /// The estimates aren't scored in S0.
    void on_estimate(const anatomy::StateEstimate<double>&) noexcept {}

    /// Pre: x.t ≥ the previous sample's t (I1).
    void on_truth(const anatomy::TruthState<double>& x) {
        assert(!prev_ || prev_->t <= x.t);  // Pre / I1
        if (reached_ || failed()) {
            return;  // I2: the outcome is settled
        }
        const anatomy::Vec3<double>& p = x.position;
        if (!contains(field_.geofence, p)) {
            fail(Outcome::geofence, "geofence breach at t=" + seconds(x.t));
        }
        if (!x.on_ground && !took_off_) {
            took_off_ = true;
            start_ = x.t;
        }
        if (took_off_) {
            const double v = norm(x.velocity);
            if (v > score_.max_speed_m_s) {
                score_.max_speed_m_s = v;
            }
            if (limit_ && v > *limit_) {
                fail(Outcome::speed_limit, "speed limit exceeded at t=" + seconds(x.t));
            }
            if (x.on_ground) {
                fail(Outcome::collided, "ground contact at t=" + seconds(x.t));
            }
            for (std::uint32_t i = 0; i < field_.obstacle_count; ++i) {
                const double c = distance(field_.obstacles[i], p) - radius_;
                if (c < score_.min_clearance_m) {
                    score_.min_clearance_m = c;  // I3
                }
                if (c <= 0) {
                    fail(Outcome::collided,
                         "obstacle collision at t=" + seconds(x.t) + " (obstacle " + std::to_string(i) + ")");
                }
            }
            if (prev_ && !prev_->on_ground) {
                score_.path_length_m += norm(sub(p, prev_->position));
            }
        }
        score_.goal_distance_m = norm(sub(p, field_.goal.position));
        if (!failed() && score_.goal_distance_m <= field_.goal.radius) {
            reached_ = true;
            score_.time_to_goal_s = static_cast<double>(x.t.ns - (took_off_ ? start_ : x.t).ns) * 1e-9;
        }
        prev_ = x;
    }

    /// True once the goal is reached without a failure first.
    [[nodiscard]] bool complete() const noexcept {
        return reached_;
    }
    /// True once the run has failed; it can't recover.
    [[nodiscard]] bool failed() const noexcept {
        return !score_.failure.empty();
    }

    [[nodiscard]] Score score() const {
        Score s = score_;
        if (reached_) {
            s.outcome = Outcome::reached;
            s.success = true;
        } else if (s.failure.empty()) {
            s.outcome = Outcome::timed_out;
            char buf[64];
            std::snprintf(buf, sizeof buf, "timed out: %.3f m from the goal", s.goal_distance_m);
            s.failure = prev_ ? buf : "timed out: no truth sample";
        }
        return s;
    }

private:
    [[nodiscard]] static double norm(const anatomy::Vec3<double>& v) noexcept {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
    [[nodiscard]] static anatomy::Vec3<double> sub(const anatomy::Vec3<double>& a,
                                                   const anatomy::Vec3<double>& b) noexcept {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
    [[nodiscard]] static std::string seconds(anatomy::Timestamp t) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.3f s", static_cast<double>(t.ns) * 1e-9);
        return buf;
    }
    void fail(Outcome o, const std::string& why) {
        if (score_.failure.empty()) {
            score_.outcome = o;
            score_.failure = why;
        }
    }

    Field<double> field_;
    double radius_;
    std::optional<double> limit_;
    std::optional<anatomy::TruthState<double>> prev_;
    bool took_off_{false};
    anatomy::Timestamp start_{};
    bool reached_{false};
    Score score_;
};
static_assert(anatomy::sim::RunScorer<Scorer>);

/// One JSON line for a score. A measure with no value (no obstacle, not
/// reached) is null.
[[nodiscard]] inline std::string to_json(const Score& s) {
    const auto number = [](double v, bool known) {
        if (!known) {
            return std::string("null");
        }
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.6f", v);
        return std::string(buf);
    };
    std::string out = "{\"outcome\":\"" + std::string(to_string(s.outcome)) + "\"";
    out += ",\"success\":";
    out += s.success ? "true" : "false";
    out += ",\"time_to_goal_s\":" + number(s.time_to_goal_s, s.success);
    out += ",\"min_clearance_m\":" + number(s.min_clearance_m, std::isfinite(s.min_clearance_m));
    out += ",\"path_length_m\":" + number(s.path_length_m, true);
    out += ",\"max_speed_m_s\":" + number(s.max_speed_m_s, true);
    out += ",\"goal_distance_m\":" + number(s.goal_distance_m, s.goal_distance_m >= 0);
    out += ",\"failure\":\"" + anatomy::sim::json_escape(s.failure) + "\"}";
    return out;
}

}  // namespace branes::scout::sim

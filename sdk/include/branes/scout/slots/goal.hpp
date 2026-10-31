#pragma once
/// @file goal.hpp
/// @brief GoalPlanner: carrot-following along a straight line to the goal.
///
/// Baseline s0-gt-pid's planner (docs/plan/bring-up/). It doesn't avoid
/// obstacles: it ignores the obstacles seen, so the baseline is the lower
/// bound every avoiding planner must beat. On the first plan() it lays out
/// the path from the vehicle's position:
///
///   start → climb (straight up, to the goal's height) → goal
///
/// and holds at the goal. The climb lifts the vehicle off the pad before it
/// heads for the goal, as racing-drone's WaypointPlanner climbs to its first
/// gate.
///
/// Each plan() projects the position onto the current segment and advances to
/// the line to the goal once the projection passes the climb's end or the
/// vehicle is within the switch radius of it. The reference is one sample:
///
///  - position: the point `lookahead` further along the path (the carrot),
///    which stops at the goal;
///  - velocity: the segment direction × speed, where speed is the cruise
///    speed, scaled down linearly within the slow-down distance of the
///    segment's end, so it is 0 at the top of the climb and at the goal;
///  - yaw: the heading from the start to the goal (the start yaw if the goal
///    is straight above the start);
///  - progress: the arc length of the projection.
///
/// Model
/// -----
///   p_        : Params [immutable] — speeds and distances
///   built_    : bool — the path has been laid out (first plan())
///   points_   : array<Vec3, 3> — start, climb, goal
///   segment_  : uint32 — current segment (points_[segment_] → points_[segment_ + 1])
///   yaw_      : T — the heading of the line to the goal
///
/// Invariants
/// ----------
///   I1: segment_ ≤ 1                          — constrains segment_
///   I2: segment_ never decreases              — constrains segment_
///   I3: the parameters satisfy params_valid() — constrains p_

#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/slots/slot_context.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacles.hpp>
#include <branes/scout/slots/context.hpp>
#include <branes/scout/slots/slots.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <string>

namespace branes::scout {

template <typename T>
class GoalPlanner {
    using Vec3 = anatomy::Vec3<T>;

public:
    struct Params {
        T cruise_speed{3};    ///< [m/s]
        T lookahead{T(0.5)};  ///< carrot distance along the path [m]
        T switch_radius{1};   ///< leave the climb within this distance of its top [m], > lookahead
        T slowdown{3};        ///< the distance before a segment's end over which the speed falls to 0 [m]
    };

    /// I3: every value finite; cruise_speed > 0; 0 ≤ lookahead < switch_radius
    /// (so a vehicle converging on a carrot held at the top of the climb comes
    /// within the switch radius and advances); slowdown > 0.
    [[nodiscard]] static bool params_valid(const Params& p) noexcept {
        using std::isfinite;
        return isfinite(p.cruise_speed) && isfinite(p.lookahead) && isfinite(p.switch_radius) && isfinite(p.slowdown) &&
               p.cruise_speed > T(0) && p.lookahead >= T(0) && p.lookahead < p.switch_radius && p.slowdown > T(0);
    }

    /// Default parameters. Post: I1–I3, no path yet.
    GoalPlanner() = default;

    /// Pre: params_valid(p). Post: I1–I3, no path yet.
    explicit GoalPlanner(const Params& p) : p_(p) {
        assert(params_valid(p_));  // Pre / I3
    }

    /// From the baseline parameters cruise_speed_m_s, lookahead_m,
    /// switch_radius_m and slowdown_m (each optional, defaulting to Params{}).
    [[nodiscard]] static anatomy::result<GoalPlanner, std::string> from_context(const SlotContext& ctx) {
        const Params d{};
        const auto& b = ctx.baseline;
        const auto cruise = anatomy::parameter_in(b, "cruise_speed_m_s", double(d.cruise_speed), 0.1, 50.0);
        const auto look = anatomy::parameter_in(b, "lookahead_m", double(d.lookahead), 0.0, 10.0);
        const auto sw = anatomy::parameter_in(b, "switch_radius_m", double(d.switch_radius), 0.01, 10.0);
        const auto slow = anatomy::parameter_in(b, "slowdown_m", double(d.slowdown), 0.01, 50.0);
        for (const auto* r : {&cruise, &look, &sw, &slow}) {
            if (!*r) {
                return r->error();
            }
        }
        Params p;
        p.cruise_speed = T(*cruise);
        p.lookahead = T(*look);
        p.switch_radius = T(*sw);
        p.slowdown = T(*slow);
        if (!params_valid(p)) {
            return std::string("GoalPlanner: need lookahead_m < switch_radius_m");
        }
        return GoalPlanner(p);
    }

    /// One reference sample for state x. It ignores the obstacles.
    /// Pre: valid(field); x.position finite; on later calls, the same field.
    /// Post: I1–I3; valid(result), result.count == 1.
    [[nodiscard]] anatomy::Reference<T>
    plan(const Field<T>& field, const ObstacleObservations<T>&, const anatomy::StateEstimate<T>& x) {
        if (!built_) {
            build(field, x.position);
        }
        [[maybe_unused]] const std::uint32_t before = segment_;
        const Vec3& p = x.position;

        // Leave the climb once past its top or within the switch radius of it.
        T s = projection(p);
        if (segment_ == 0 && (s >= T(1) || distance(p, points_[1]) < p_.switch_radius)) {
            segment_ = 1;
            s = projection(p);
        }
        s = s < T(0) ? T(0) : (s > T(1) ? T(1) : s);

        const Vec3 a = points_[segment_];
        const Vec3 b = points_[segment_ + 1];
        const T len = distance(a, b);
        const Vec3 dir = len > T(0) ? scaled(sub(b, a), T(1) / len) : Vec3{};
        const T remaining = (T(1) - s) * len;
        const T speed = remaining < p_.slowdown ? p_.cruise_speed * remaining / p_.slowdown : p_.cruise_speed;

        anatomy::Reference<T> r{};
        r.count = 1;
        anatomy::ReferenceSample<T>& out = r.samples[0];
        out.t = x.t_valid;
        out.position = carrot(add(a, scaled(sub(b, a), s)), remaining);
        out.velocity = scaled(dir, speed);
        out.yaw = yaw_;
        out.progress = (segment_ == 1 ? distance(points_[0], points_[1]) : T(0)) + s * len;

        assert(segment_ >= before);  // I2
        assert(check_invariant());
        return r;
    }

    /// Path inspection, for tests and tools. Pre for point(): built() and k ≤ 2.
    [[nodiscard]] bool built() const noexcept {
        return built_;
    }
    [[nodiscard]] std::uint32_t segment() const noexcept {
        return segment_;
    }
    [[nodiscard]] Vec3 point(std::uint32_t k) const noexcept {
        assert(built_ && k < points_.size());  // Pre
        return points_[k];
    }
    [[nodiscard]] const Params& params() const noexcept {
        return p_;
    }

private:
    [[nodiscard]] static Vec3 add(const Vec3& a, const Vec3& b) noexcept {
        return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
    }
    [[nodiscard]] static Vec3 sub(const Vec3& a, const Vec3& b) noexcept {
        return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }
    [[nodiscard]] static Vec3 scaled(const Vec3& a, T k) noexcept {
        return {k * a[0], k * a[1], k * a[2]};
    }
    [[nodiscard]] static T dot(const Vec3& a, const Vec3& b) noexcept {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    [[nodiscard]] static T distance(const Vec3& a, const Vec3& b) noexcept {
        using std::sqrt;
        const Vec3 d = sub(b, a);
        return sqrt(dot(d, d));
    }

    void build(const Field<T>& field, const Vec3& from) {
        assert(valid(field));  // Pre
        const Vec3& goal = field.goal.position;
        points_ = {from, Vec3{from[0], from[1], goal[2]}, goal};
        const T dx = goal[0] - from[0];
        const T dy = goal[1] - from[1];
        using std::atan2;
        using std::sqrt;
        yaw_ = sqrt(dx * dx + dy * dy) > T(0) ? atan2(dy, dx) : field.start.yaw;
        segment_ = 0;
        built_ = true;
    }

    /// The projection parameter of p on the current segment (1 for a zero-length one).
    [[nodiscard]] T projection(const Vec3& p) const noexcept {
        const Vec3 a = points_[segment_];
        const Vec3 ab = sub(points_[segment_ + 1], a);
        const T l2 = dot(ab, ab);
        return l2 > T(0) ? dot(sub(p, a), ab) / l2 : T(1);
    }

    /// The point `lookahead` along the path from `from`, which lies on the
    /// current segment `remaining` before its end; it stops at the goal.
    [[nodiscard]] Vec3 carrot(const Vec3& from, T remaining) const noexcept {
        T left = p_.lookahead;
        for (std::uint32_t k = segment_; k < 2; ++k) {
            const Vec3 a = k == segment_ ? from : points_[k];
            const Vec3 b = points_[k + 1];
            const T len = k == segment_ ? remaining : distance(a, b);
            if (left <= len) {
                return len > T(0) ? add(a, scaled(sub(b, a), left / len)) : a;
            }
            left -= len;
        }
        return points_[2];
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        return params_valid(p_) && segment_ <= 1;  // I3, I1
    }

    Params p_{};
    bool built_{false};
    std::array<Vec3, 3> points_{};
    std::uint32_t segment_{0};
    T yaw_{0};
};

static_assert(Planner<GoalPlanner<double>, double>);

}  // namespace branes::scout

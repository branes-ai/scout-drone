// The scorer (sim/score.hpp), on made-up truth: each outcome and when it is
// settled, the measures, and the JSON line.

#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/sim/closed_loop.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/sim/score.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

using namespace branes::scout;
using namespace branes::scout::sim;
using branes::anatomy::Timestamp;
using branes::anatomy::TruthState;
using branes::anatomy::Vec3;
using Catch::Matchers::WithinAbs;

namespace {

/// 60 × 30 × 10 m; the start at (5, 15) on the pad, the goal at (55, 15, 2)
/// within 1 m, and one obstacle of radius 0.5 m at (30, 20).
Field<double> field() {
    Field<double> f;
    f.geofence = {{0, 0, 0}, {60, 30, 10}};
    f.start = {{5, 15, 0.1}, 0};
    f.goal = {{55, 15, 2}, 1};
    f.obstacles[0] = Obstacle<double>{{30, 20, 0}, 0.5, 6};
    f.obstacle_count = 1;
    return f;
}

/// A truth sample at t seconds; `v` the velocity.
TruthState<double> at(double t, Vec3<double> p, bool on_ground = false, Vec3<double> v = {}) {
    TruthState<double> x{};
    x.t = Timestamp{static_cast<std::int64_t>(std::llround(t * 1e9))};
    x.position = p;
    x.velocity = v;
    x.attitude = {1, 0, 0, 0};
    x.on_ground = on_ground;
    return x;
}

}  // namespace

TEST_CASE("score: reaching the goal completes the run, and holding there can't fail it", "[score]") {
    Scorer s(field(), 0.25);
    s.on_truth(at(0, {5, 15, 0.1}, true));
    s.on_truth(at(1, {5, 15, 2}, false, {0, 0, 2}));  // takeoff
    s.on_truth(at(11, {35, 15, 2}, false, {3, 0, 0}));
    CHECK_FALSE(s.complete());
    s.on_truth(at(17, {54.5, 15, 2}, false, {1, 0, 0}));  // within 1 m of the goal
    CHECK(s.complete());
    CHECK_FALSE(s.failed());
    // After the goal, nothing is scored: not the ground, not the geofence, not an obstacle.
    s.on_truth(at(18, {54.5, 15, 0}, true));
    s.on_truth(at(19, {70, 15, 2}));
    s.on_truth(at(20, {30, 20, 2}));
    CHECK(s.complete());
    CHECK_FALSE(s.failed());

    const Score r = s.score();
    CHECK(r.outcome == Outcome::reached);
    CHECK(r.success);
    CHECK(r.failure.empty());
    CHECK_THAT(r.time_to_goal_s, WithinAbs(16, 1e-9));        // from takeoff at 1 s
    CHECK_THAT(r.path_length_m, WithinAbs(30 + 19.5, 1e-9));  // the samples after takeoff
    CHECK_THAT(r.max_speed_m_s, WithinAbs(3, 1e-12));
    CHECK_THAT(r.goal_distance_m, WithinAbs(0.5, 1e-12));
    // Nearest the obstacle at (35, 15, 2): √50 m from its axis, less its radius and the collision radius.
    const double clearance = std::sqrt(25.0 + 25.0) - 0.5 - 0.25;
    CHECK_THAT(r.min_clearance_m, WithinAbs(clearance, 1e-12));
}

TEST_CASE("score: an obstacle within the collision radius is a collision, and the run can't recover", "[score]") {
    Scorer s(field(), 0.25);
    s.on_truth(at(0, {5, 15, 0.1}, true));
    s.on_truth(at(1, {29, 20, 2}));  // 1 m from the axis: 0.5 m from the side, beyond 0.25 m
    CHECK_FALSE(s.failed());
    s.on_truth(at(2, {29.3, 20, 2}));  // 0.2 m from the side: within the collision radius
    CHECK(s.failed());
    s.on_truth(at(3, {55, 15, 2}));  // at the goal, too late
    CHECK_FALSE(s.complete());
    const Score r = s.score();
    CHECK(r.outcome == Outcome::collided);
    CHECK_FALSE(r.success);
    CHECK(r.failure == "obstacle collision at t=2.000 s (obstacle 0)");
    CHECK(branes::anatomy::sim::failure_reason(r.failure) == "obstacle collision");
    CHECK_THAT(r.min_clearance_m, WithinAbs(0.2 - 0.25, 1e-9));
    CHECK(r.time_to_goal_s == -1);
}

TEST_CASE("score: the collision radius is the scenario's", "[score]") {
    Scorer wide(field(), 0.6);
    wide.on_truth(at(1, {29, 20, 2}));  // 0.5 m from the side
    CHECK(wide.failed());
    Scorer point(field(), 0.0);
    point.on_truth(at(1, {29.4, 20, 2}));  // 0.1 m from the side
    CHECK_FALSE(point.failed());
}

TEST_CASE("score: ground contact after takeoff is a collision; on the pad before it isn't", "[score]") {
    Scorer s(field(), 0.25);
    s.on_truth(at(0, {5, 15, 0.1}, true));
    s.on_truth(at(1, {5, 15, 0.1}, true));
    CHECK_FALSE(s.failed());
    s.on_truth(at(2, {5, 15, 1}));
    s.on_truth(at(3, {8, 15, 0}, true));
    const Score r = s.score();
    CHECK(r.outcome == Outcome::collided);
    CHECK(r.failure == "ground contact at t=3.000 s");
}

TEST_CASE("score: leaving the geofence fails the run", "[score]") {
    Scorer s(field(), 0.25);
    s.on_truth(at(1, {5, 15, 2}));
    s.on_truth(at(2, {5, 31, 2}));
    const Score r = s.score();
    CHECK(r.outcome == Outcome::geofence);
    CHECK(branes::anatomy::sim::failure_reason(r.failure) == "geofence breach");
}

TEST_CASE("score: a speed over the limit fails the run; without a limit it is only measured", "[score]") {
    Scorer limited(field(), 0.25, 4.0);
    limited.on_truth(at(1, {5, 15, 2}, false, {3, 0, 0}));
    CHECK_FALSE(limited.failed());
    limited.on_truth(at(2, {10, 15, 2}, false, {4, 3, 0}));  // 5 m/s
    CHECK(limited.score().outcome == Outcome::speed_limit);
    CHECK_THAT(limited.score().max_speed_m_s, WithinAbs(5, 1e-12));

    Scorer free(field(), 0.25);
    free.on_truth(at(1, {5, 15, 2}, false, {4, 3, 0}));
    CHECK_FALSE(free.failed());
}

TEST_CASE("score: a failure on the sample that reaches the goal comes first", "[score]") {
    Field<double> f = field();
    f.obstacles[1] = Obstacle<double>{{55, 16.4, 0}, 0.3, 6};  // beside the goal
    f.obstacle_count = 2;
    REQUIRE(valid(f));
    Scorer s(f, 0.25);
    s.on_truth(at(1, {5, 15, 2}));
    s.on_truth(at(2, {55, 15.9, 2}));  // within the goal radius, and 0.2 m from the obstacle's side
    CHECK(s.failed());
    CHECK_FALSE(s.complete());
    CHECK(s.score().outcome == Outcome::collided);
}

TEST_CASE("score: a run that ends short of the goal timed out", "[score]") {
    Scorer s(field(), 0.25);
    s.on_truth(at(0, {5, 15, 0.1}, true));
    s.on_truth(at(1, {5, 15, 2}));
    s.on_truth(at(2, {25, 15, 2}));
    const Score r = s.score();
    CHECK(r.outcome == Outcome::timed_out);
    CHECK_FALSE(r.success);
    CHECK(r.failure == "timed out: 30.000 m from the goal");
    CHECK(branes::anatomy::sim::failure_reason(r.failure) == "timed out");
    CHECK_THAT(r.path_length_m, WithinAbs(20, 1e-12));

    // A vehicle that never left the pad has no measures, and no clearance.
    Scorer pad(field(), 0.25);
    pad.on_truth(at(0, {5, 15, 0.1}, true));
    pad.on_truth(at(9, {5, 15, 0.1}, true));
    const Score p = pad.score();
    CHECK(p.outcome == Outcome::timed_out);
    CHECK(p.path_length_m == 0);
    CHECK(p.max_speed_m_s == 0);
    CHECK(p.min_clearance_m == std::numeric_limits<double>::infinity());
    CHECK(Scorer(field(), 0.25).score().failure == "timed out: no truth sample");
}

TEST_CASE("score: the JSON line, with null for a measure that has no value", "[score]") {
    Score s;
    s.outcome = Outcome::timed_out;
    s.path_length_m = 1.5;
    s.max_speed_m_s = 2;
    s.goal_distance_m = 30;
    s.failure = "timed out: 30.000 m from the \"goal\"";
    CHECK(to_json(s) == "{\"outcome\":\"timed_out\",\"success\":false,\"time_to_goal_s\":null,"
                        "\"min_clearance_m\":null,\"path_length_m\":1.500000,\"max_speed_m_s\":2.000000,"
                        "\"goal_distance_m\":30.000000,\"failure\":\"timed out: 30.000 m from the \\\"goal\\\"\"}");
    s.outcome = Outcome::reached;
    s.success = true;
    s.time_to_goal_s = 16;
    s.min_clearance_m = 0.75;
    s.failure.clear();
    CHECK(to_json(s) == "{\"outcome\":\"reached\",\"success\":true,\"time_to_goal_s\":16.000000,"
                        "\"min_clearance_m\":0.750000,\"path_length_m\":1.500000,\"max_speed_m_s\":2.000000,"
                        "\"goal_distance_m\":30.000000,\"failure\":\"\"}");
}

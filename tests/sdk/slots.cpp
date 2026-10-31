// Scout's slots (slots/): the nulls and the ground-truth estimator model their
// concepts and pass on what they should; GoalPlanner climbs off the pad, flies
// the straight line to the goal at the cruise speed, slows into the goal and
// holds there, only moves forward, ignores the obstacles seen, and reads its
// baseline parameters.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/slots/slot_context.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/slots/context.hpp>
#include <branes/scout/slots/goal.hpp>
#include <branes/scout/slots/ground_truth.hpp>
#include <branes/scout/slots/null.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <string>
#include <utility>

using namespace branes::scout;
using branes::anatomy::ContextBuilt;
using branes::anatomy::Quat;
using branes::anatomy::Reference;
using branes::anatomy::ReferenceSample;
using branes::anatomy::StateEstimate;
using branes::anatomy::Timestamp;
using branes::anatomy::TruthConsumer;
using branes::anatomy::TruthState;
using branes::anatomy::Vec3;
using Catch::Matchers::WithinAbs;

static_assert(Estimator<NullEstimator<double>, double>);
static_assert(Planner<NullPlanner<double>, double>);
static_assert(Estimator<GroundTruthEstimator<double>, double>);
static_assert(TruthConsumer<GroundTruthEstimator<double>, double>);
static_assert(!TruthConsumer<NullEstimator<double>, double>);
static_assert(Planner<GoalPlanner<double>, double>);
static_assert(Planner<GoalPlanner<float>, float>);  // scalar-generic
static_assert(Estimator<GroundTruthEstimator<float>, float>);
static_assert(ContextBuilt<GoalPlanner<double>, SlotContext>);
static_assert(!ContextBuilt<GroundTruthEstimator<double>, SlotContext>);

namespace {

const std::filesystem::path src = SCOUT_SOURCE_DIR;

Field<double> open_field() {
    const auto c = config::load_field(src / "fields/open-60.yaml");
    REQUIRE(c);
    return c->field;
}

StateEstimate<double> at(Vec3<double> p, std::int64_t t_ns = 0) {
    StateEstimate<double> x{};
    x.t_valid = Timestamp{t_ns};
    x.position = p;
    return x;
}

ObstacleObservations<double> one_seen() {
    ObstacleObservations<double> o{};
    o.count = 1;
    o.obstacles[0] = ObstacleObservation<double>{0, {2.0, 0.0, -2.0}, 0.4, 8.0};
    return o;
}

double dist(const Vec3<double>& a, const Vec3<double>& b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

double norm(const Vec3<double>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

config::BaselineConfig baseline_with(std::map<std::string, std::string> params) {
    config::BaselineConfig b;
    b.name = "t";
    b.parameters = std::move(params);
    return b;
}

}  // namespace

// ── The nulls and GroundTruthEstimator ──────────────────────────────

TEST_CASE("null slots: the estimator reports rest, the planner holds the current position", "[slots][null]") {
    NullEstimator<double> e;
    e.process_obstacles(one_seen());
    const StateEstimate<double> x = e.state_at(Timestamp{7});
    CHECK(valid(x));
    CHECK(x.t_valid == Timestamp{7});
    CHECK(e.health().ok);

    const NullPlanner<double> p;
    const Reference<double> r = p.plan(open_field(), one_seen(), at({3, 4, 5}, 9));
    REQUIRE(r.count == 1);
    CHECK(r.samples[0].position == Vec3<double>{3, 4, 5});
    CHECK(r.samples[0].velocity == Vec3<double>{});
    CHECK(r.samples[0].t == Timestamp{9});
}

TEST_CASE("ground truth: the latest truth is the estimate; frames and obstacles change nothing", "[slots][estimator]") {
    GroundTruthEstimator<double> e;
    CHECK_FALSE(e.health().ok);  // no truth yet
    TruthState<double> x{};
    x.t = Timestamp{2'000'000};
    x.position = {5, 15, 2};
    x.velocity = {1, 0, 0.5};
    x.attitude = Quat<double>{0.9238795325112867, 0, 0, 0.3826834323650898};  // 45° yaw, unit
    e.process_truth(x);
    e.process_obstacles(one_seen());
    const StateEstimate<double> s = e.state_at(Timestamp{3'000'000});
    CHECK(s.t_valid == x.t);
    CHECK(s.position == x.position);
    CHECK(s.velocity == x.velocity);
    CHECK(s.attitude == x.attitude);
    CHECK(e.health().ok);
}

// ── GoalPlanner ─────────────────────────────────────────────────────

TEST_CASE("goal: the path climbs from the vehicle to the goal's height, then runs to the goal", "[slots][planner]") {
    const Field<double> f = open_field();
    GoalPlanner<double> p;
    CHECK_FALSE(p.built());
    const Vec3<double> pad = f.start.position;
    (void)p.plan(f, {}, at(pad));
    REQUIRE(p.built());
    CHECK(p.point(0) == pad);
    CHECK(p.point(1) == Vec3<double>{pad[0], pad[1], f.goal.position[2]});
    CHECK(p.point(2) == f.goal.position);
}

TEST_CASE("goal: up the climb, the cruise line, the slow-down and the hold", "[slots][planner]") {
    const Field<double> f = open_field();  // pad (5, 15, 0.1), goal (55, 15, 2)
    GoalPlanner<double> p;
    const auto& q = p.params();
    const auto one = [&](Vec3<double> pos) { return p.plan(f, {}, at(pos)).samples[0]; };

    // On the pad: straight up, 1.9 m below the top, so within the slow-down distance.
    const ReferenceSample<double> pad = one({5, 15, 0.1});
    CHECK(p.segment() == 0u);
    CHECK_THAT(pad.velocity[2], WithinAbs(q.cruise_speed * 1.9 / q.slowdown, 1e-12));
    CHECK_THAT(pad.velocity[0], WithinAbs(0.0, 1e-12));
    CHECK_THAT(dist(pad.position, {5, 15, 0.1 + q.lookahead}), WithinAbs(0.0, 1e-12));
    CHECK_THAT(pad.yaw, WithinAbs(0.0, 1e-12));  // facing the goal, along +x

    // Within the switch radius of the top: onto the line, at the cruise speed.
    const ReferenceSample<double> top = one({5, 15, 2.0 - 0.8 * q.switch_radius});
    CHECK(p.segment() == 1u);
    CHECK_THAT(top.velocity[0], WithinAbs(q.cruise_speed, 1e-12));
    CHECK_THAT(top.velocity[2], WithinAbs(0.0, 1e-12));
    CHECK_THAT(top.progress, WithinAbs(1.9, 1e-12));

    const ReferenceSample<double> mid = one({30, 15, 2});
    CHECK_THAT(norm(mid.velocity), WithinAbs(q.cruise_speed, 1e-12));
    CHECK_THAT(dist(mid.position, {30 + q.lookahead, 15, 2}), WithinAbs(0.0, 1e-12));
    CHECK_THAT(mid.progress, WithinAbs(1.9 + 25.0, 1e-12));

    const double before = 0.5 * q.slowdown;  // halfway through the slow-down
    const ReferenceSample<double> slow = one({55 - before, 15, 2});
    CHECK_THAT(norm(slow.velocity), WithinAbs(0.5 * q.cruise_speed, 1e-12));

    for (const Vec3<double>& pos : {Vec3<double>{55, 15, 2}, Vec3<double>{57, 15, 2.5}}) {  // at, and past
        const ReferenceSample<double> hold = one(pos);
        CHECK(hold.position == f.goal.position);
        CHECK_THAT(norm(hold.velocity), WithinAbs(0.0, 1e-12));
    }
    CHECK(p.segment() == 1u);  // never back onto the climb
}

TEST_CASE("goal: a vehicle on the carrot moves forward only and holds at the goal", "[slots][planner]") {
    const Field<double> f = open_field();
    GoalPlanner<double> p;
    Vec3<double> pos = f.start.position;
    std::uint32_t last_segment = 0;
    double last_progress = 0;
    std::int64_t t = 0;
    Reference<double> r{};
    // The path is 1.9 + 50 m and the carrot 0.5 m ahead: the goal is reached
    // within 104 steps.
    for (int step = 0; step < 300; ++step) {
        r = p.plan(f, {}, at(pos, t));
        REQUIRE(valid(r));
        REQUIRE(r.count == 1u);
        REQUIRE(p.segment() >= last_segment);  // I2
        REQUIRE(p.segment() <= 1u);            // I1
        REQUIRE(r.samples[0].progress >= last_progress);
        REQUIRE(r.samples[0].t == Timestamp{t});
        last_segment = p.segment();
        last_progress = r.samples[0].progress;
        pos = r.samples[0].position;
        t += 4'000'000;
    }
    CHECK(dist(pos, f.goal.position) < 1e-9);
    CHECK(r.samples[0].velocity == Vec3<double>{});  // hold
}

TEST_CASE("goal: the obstacles seen don't change the plan", "[slots][planner]") {
    const Field<double> f = open_field();
    GoalPlanner<double> blind;
    GoalPlanner<double> seeing;
    for (const Vec3<double>& pos : {Vec3<double>{5, 15, 0.1}, Vec3<double>{5, 15, 1.5}, Vec3<double>{20, 15, 2}}) {
        CHECK(blind.plan(f, {}, at(pos)) == seeing.plan(f, one_seen(), at(pos)));
    }
}

TEST_CASE("goal: from_context reads and checks the baseline parameters", "[slots][planner]") {
    const auto vehicle = branes::anatomy::config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(vehicle);
    const Field<double> f = open_field();
    const auto make = [&](std::map<std::string, std::string> params) {
        const auto b = baseline_with(std::move(params));
        return GoalPlanner<double>::from_context(SlotContext{*vehicle, b, f});
    };
    const auto p = make({{"cruise_speed_m_s", "4.5"}, {"slowdown_m", "2"}});
    REQUIRE(p);
    CHECK(p->params().cruise_speed == 4.5);
    CHECK(p->params().slowdown == 2.0);
    CHECK(p->params().lookahead == GoalPlanner<double>::Params{}.lookahead);  // default
    CHECK_FALSE(make({{"cruise_speed_m_s", "fast"}}));
    CHECK_FALSE(make({{"cruise_speed_m_s", "-1"}}));
    CHECK_FALSE(make({{"slowdown_m", "0"}}));
    const auto look = make({{"lookahead_m", "1.0"}, {"switch_radius_m", "1.0"}});  // lookahead must be < switch
    REQUIRE_FALSE(look);
    CHECK(look.error().find("lookahead_m") != std::string::npos);
}

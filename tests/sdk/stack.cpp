// Scout's stack (compose/stack.hpp) and its profile (compose/profile.hpp): a
// null stack produces valid outputs; the Runner feeds the stack the truth and
// the obstacles seen, which reach the planner; a malformed oracle_obstacles
// record is reported; create() builds the context slots and reports their
// errors; a tick returns the reference its planner computed.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/harness/runner.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/anatomy/mobility/slots/neutral_nulls.hpp>
#include <branes/scout/compose/profile.hpp>
#include <branes/scout/compose/stack.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/harness/harness.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/slots/goal.hpp>
#include <branes/scout/slots/ground_truth.hpp>
#include <branes/scout/slots/null.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <map>
#include <span>
#include <string>
#include <utility>

namespace scout = branes::scout;
namespace an = branes::anatomy;
using an::Timestamp;
using an::Vec3;

namespace {

template <typename T>
using NullStack = scout::Stack<T,
                               scout::NullEstimator<T>,
                               scout::NullPlanner<T>,
                               an::NullController<T>,
                               an::NullSafetyFilter<T>,
                               an::NullDaaTracker<T>,
                               an::NullFcBridge<T>>;

template <typename T>
using GtStack = scout::Stack<T,
                             scout::GroundTruthEstimator<T>,
                             scout::GoalPlanner<T>,
                             an::NullController<T>,
                             an::NullSafetyFilter<T>,
                             an::NullDaaTracker<T>,
                             an::NullFcBridge<T>>;

static_assert(an::RunnableStack<NullStack<double>>);
static_assert(an::RunnableStack<GtStack<double>>);
static_assert(an::RunnableStack<GtStack<float>>);  // scalar-generic
static_assert(an::is_profile_input<scout::ScoutProfile>(scout::log::oracle_obstacles));
// A MobileStack: its profile adds oracle_obstacles to MobilityProfile's inputs,
// and anatomy's loops take it (docs/plan/bring-up/, assumption 8).
static_assert(an::MobileStack<GtStack<double>>);
static_assert(an::MobileStack<GtStack<float>>);

const std::filesystem::path src = SCOUT_SOURCE_DIR;

scout::Field<double> open_field() {
    const auto c = scout::config::load_field(src / "fields/open-60.yaml");
    REQUIRE(c);
    return c->field;
}

an::config::VehicleConfig rp1() {
    const auto v = an::config::load_vehicle(src / "vehicles/rp1.yaml");
    REQUIRE(v);
    return *v;
}

an::config::BaselineConfig baseline_with(std::map<std::string, std::string> params) {
    an::config::BaselineConfig b;
    b.name = "t";
    b.parameters = std::move(params);
    return b;
}

an::TruthState<double> truth(std::int64_t t_ns, Vec3<double> p, Vec3<double> v = {}) {
    an::TruthState<double> x{};
    x.t = Timestamp{t_ns};
    x.position = p;
    x.velocity = v;
    return x;
}

scout::ObstacleObservations<double> seen_at(std::int64_t t_ns) {
    scout::ObstacleObservations<double> o{};
    o.t = Timestamp{t_ns};
    o.count = 1;
    o.obstacles[0] = scout::ObstacleObservation<double>{4, {6.0, 1.0, -2.0}, 0.25, 7.0};
    return o;
}

}  // namespace

TEST_CASE("stack: a null composition produces valid, time-stamped outputs", "[stack]") {
    const scout::Field<double> f = open_field();
    NullStack<double> stack(f);
    stack.on_imu(an::ImuSample<double>{});
    const an::TickOutput<double> y = stack.on_tick(Timestamp{1'000'000});
    CHECK(valid(y.state));
    CHECK(valid(y.command));
    CHECK(valid(y.reference));
    CHECK(y.state.t_valid == Timestamp{1'000'000});
    CHECK(y.command.t == Timestamp{1'000'000});
    CHECK(y.reference == stack.planner().plan(stack.field(), stack.seen(), y.state));
    CHECK(stack.seen().count == 0u);  // none seen yet

    // The same composition in float.
    NullStack<float> fstack(scout::cast<float>(f));
    CHECK(valid(fstack.on_tick(Timestamp{0}).command));
}

TEST_CASE("stack: the Runner feeds the truth and the obstacles seen", "[stack]") {
    const scout::Field<double> f = open_field();
    auto s = scout::harness::make_stack<GtStack<double>>(f, rp1(), baseline_with({}));
    REQUIRE(s);
    an::harness::Runner runner(*s, std::span<const an::log::Channel>{});

    const an::TruthState<double> x = truth(2'000'000, {5, 15, 0.1}, {0, 0, 0.2});
    REQUIRE(runner.feed(an::log::LogRecord{an::log::Channel::truth, x.t, an::codec::encode(x)}));
    const auto seen = seen_at(2'000'000);
    const auto fed = runner.feed(an::log::LogRecord{scout::log::oracle_obstacles, seen.t, scout::codec::encode(seen)});
    REQUIRE(fed);
    CHECK(fed->empty());  // an input: consumed, nothing produced
    CHECK(s->seen() == seen);

    const auto out = runner.feed(an::log::LogRecord{an::log::Channel::tick, Timestamp{2'000'000}, {}});
    REQUIRE(out);
    REQUIRE(out->size() == 3u);  // state, reference, command
    const auto state = an::codec::decode_state((*out)[0].payload);
    REQUIRE(state);
    CHECK(state->position == x.position);
    CHECK(state->t_valid == x.t);

    const auto broken = runner.feed(an::log::LogRecord{scout::log::oracle_obstacles, Timestamp{3'000'000}, {}});
    REQUIRE_FALSE(broken);
    CHECK(broken.error() == "undecodable oracle_obstacles record at t=3000000");
    CHECK(s->seen() == seen);  // unchanged
}

TEST_CASE("stack: the obstacles seen reach a stack in float, cast", "[stack]") {
    const scout::Field<double> f = open_field();
    auto s = scout::harness::make_stack<GtStack<float>>(f, rp1(), baseline_with({}));
    REQUIRE(s);
    an::harness::Runner runner(*s, std::span<const an::log::Channel>{});
    const auto seen = seen_at(1'000'000);
    REQUIRE(runner.feed(an::log::LogRecord{scout::log::oracle_obstacles, seen.t, scout::codec::encode(seen)}));
    CHECK(s->seen() == scout::cast<float>(seen));

    // Finite in double, infinite in float: refused in the stack's type, not handed on.
    auto wide = seen_at(2'000'000);
    wide.obstacles[0].radius = 1e39;
    const auto refused =
        runner.feed(an::log::LogRecord{scout::log::oracle_obstacles, wide.t, scout::codec::encode(wide)});
    REQUIRE_FALSE(refused);
    CHECK(refused.error() == "oracle_obstacles record at t=2000000 is not valid in the stack's scalar type");
    CHECK(s->seen() == scout::cast<float>(seen));  // unchanged
}

TEST_CASE("stack: create builds context slots and reports their errors", "[stack]") {
    const scout::Field<double> f = open_field();
    const auto vehicle = rp1();
    auto s = scout::harness::make_stack<GtStack<double>>(f, vehicle, baseline_with({{"cruise_speed_m_s", "2.0"}}));
    REQUIRE(s);
    CHECK(s->planner().params().cruise_speed == 2.0);
    const auto bad =
        scout::harness::make_stack<GtStack<double>>(f, vehicle, baseline_with({{"cruise_speed_m_s", "x"}}));
    REQUIRE_FALSE(bad);
    CHECK(bad.error().find("cruise_speed_m_s") != std::string::npos);
}

TEST_CASE("stack: a tick returns the reference its planner computed, with the obstacles seen", "[stack]") {
    const scout::Field<double> f = open_field();
    auto s = scout::harness::make_stack<GtStack<double>>(f, rp1(), baseline_with({}));
    REQUIRE(s);
    s->on_truth(truth(1'000'000, {5, 15, 0.1}));
    (void)s->on_tick(Timestamp{1'000'000});  // the first tick lays out the path
    s->on_truth(truth(2'000'000, {5, 15, 0.3}, {0, 0, 0.5}));
    s->on_obstacles(seen_at(2'000'000));
    auto twin = s->planner();  // the planner as the second tick finds it
    const an::TickOutput<double> y = s->on_tick(Timestamp{2'000'000});
    REQUIRE(y.reference.count == 1u);
    CHECK(y.reference == twin.plan(s->field(), seen_at(2'000'000), y.state));
}

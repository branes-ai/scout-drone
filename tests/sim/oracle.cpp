// The obstacle oracle (sim/oracle.hpp): it sees the obstacles within its range
// of the true position, nearest first and at most kMaxVisibleObstacles, each
// at its foot's centre relative to the vehicle in the body frame; its noise is
// drawn from its own stream, the same for the same seed, and bounded by σ·√3.

#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/sim/oracle.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <numbers>

using namespace branes::scout;
using branes::anatomy::Quat;
using branes::anatomy::Timestamp;
using branes::anatomy::TruthState;
using branes::anatomy::Vec3;
using Catch::Matchers::WithinAbs;

namespace {

/// 60 × 30 × 10 m, the start at one end and the goal at the other, no obstacles.
Field<double> open_field() {
    Field<double> f;
    f.geofence = {{0, 0, 0}, {60, 30, 10}};
    f.start = {{5, 15, 0.1}, 0};
    f.goal = {{55, 15, 2}, 1};
    return f;
}

void add(Field<double>& f, Vec3<double> base, double radius = 0.25, double height = 6) {
    f.obstacles[f.obstacle_count++] = Obstacle<double>{base, radius, height};
}

TruthState<double> at(Vec3<double> p, double yaw = 0) {
    TruthState<double> x{};
    x.t = Timestamp{1'000'000};
    x.position = p;
    x.attitude = Quat<double>{std::cos(yaw / 2), 0, 0, std::sin(yaw / 2)};
    return x;
}

}  // namespace

TEST_CASE("oracle: the distance to an obstacle is to its nearest point", "[oracle]") {
    const Obstacle<double> o{{10, 10, 0}, 1, 4};
    CHECK(distance(o, Vec3<double>{10.5, 10, 2}) == 0);                     // inside
    CHECK_THAT(distance(o, Vec3<double>{13, 10, 2}), WithinAbs(2, 1e-12));  // beside: to its side
    CHECK_THAT(distance(o, Vec3<double>{10, 10, 7}), WithinAbs(3, 1e-12));  // above: to its top
    CHECK_THAT(distance(o, Vec3<double>{14, 10, 7}),
               WithinAbs(std::sqrt(18.0), 1e-12));  // to its top edge: 3 out, 3 up
}

TEST_CASE("oracle: it sees the obstacles within its range, nearest first", "[oracle]") {
    Field<double> f = open_field();
    add(f, {20, 15, 0});  // 0: 4.75 m from the vehicle's position to its side
    add(f, {18, 15, 0});  // 1: 2.75 m
    add(f, {40, 15, 0});  // 2: out of range
    add(f, {15, 20, 0});  // 3: 4.75 m, the same as 0: after it, by index
    REQUIRE(valid(f));
    const auto o = sim::see_obstacles(f, at({15, 15, 2}), Timestamp{7}, 6.0);
    REQUIRE(valid(o));
    CHECK(o.t == Timestamp{7});
    REQUIRE(o.count == 3u);
    CHECK(o.obstacles[0].id == 1);
    CHECK(o.obstacles[1].id == 0);
    CHECK(o.obstacles[2].id == 3);
    // The foot's centre relative to the vehicle, here with the body frame the world's.
    CHECK(o.obstacles[0].position == Vec3<double>{3, 0, -2});
    CHECK(o.obstacles[0].radius == 0.25);
    CHECK(o.obstacles[0].height == 6);
    CHECK(sim::see_obstacles(f, at({15, 15, 2}), Timestamp{7}, 0.0).count == 0u);
}

TEST_CASE("oracle: positions are in the body frame", "[oracle]") {
    Field<double> f = open_field();
    add(f, {18, 15, 0});  // 3 m ahead along world x
    // Yawed 90° left, the obstacle along world +x is on the body's right (−y).
    const auto o = sim::see_obstacles(f, at({15, 15, 2}, std::numbers::pi / 2), Timestamp{0}, 10.0);
    REQUIRE(o.count == 1u);
    CHECK_THAT(o.obstacles[0].position[0], WithinAbs(0, 1e-12));
    CHECK_THAT(o.obstacles[0].position[1], WithinAbs(-3, 1e-12));
    CHECK_THAT(o.obstacles[0].position[2], WithinAbs(-2, 1e-12));
}

TEST_CASE("oracle: it keeps the nearest kMaxVisibleObstacles", "[oracle]") {
    Field<double> f = open_field();
    for (std::uint32_t i = 0; i < 40; ++i) {
        add(f, {20.0 + 0.5 * i, 25, 0}, 0.1);  // a row, each farther than the one before
    }
    REQUIRE(valid(f));
    const auto o = sim::see_obstacles(f, at({20, 15, 2}), Timestamp{0}, 50.0);
    REQUIRE(o.count == kMaxVisibleObstacles);
    for (std::uint32_t k = 0; k < o.count; ++k) {
        CHECK(o.obstacles[k].id == static_cast<std::int32_t>(k));
    }
}

TEST_CASE("oracle: with no noise it reports what it sees, on oracle_obstacles", "[oracle]") {
    Field<double> f = open_field();
    add(f, {18, 15, 0});
    add(f, {15, 19, 0});
    sim::ObstacleOracle oracle(f, 10.0, 0.0, 7);
    const TruthState<double> x = at({15, 15, 2}, 0.3);
    const auto ev = oracle.observe(x, Timestamp{40'000'000});
    REQUIRE(ev);
    CHECK(ev->channel == log::oracle_obstacles);
    CHECK(ev->t == Timestamp{40'000'000});
    const auto o = codec::decode_obstacles(ev->payload);
    REQUIRE(o);
    CHECK(*o == sim::see_obstacles(f, x, Timestamp{40'000'000}, 10.0));
}

TEST_CASE("oracle: its noise follows the seed, and is bounded by sigma times root 3", "[oracle]") {
    Field<double> f = open_field();
    add(f, {18, 15, 0});
    add(f, {15, 19, 0});
    const TruthState<double> x = at({15, 15, 2});
    const auto exact = sim::see_obstacles(f, x, Timestamp{0}, 10.0);
    const double sigma = 0.05;
    const auto seen = [&](std::uint64_t seed) {
        sim::ObstacleOracle oracle(f, 10.0, sigma, seed);
        (void)oracle.observe(x, Timestamp{0});  // a second frame draws on from the first
        return *codec::decode_obstacles(oracle.observe(x, Timestamp{0})->payload);
    };
    const auto a = seen(7);
    CHECK(a == seen(7));
    CHECK_FALSE(a == seen(8));
    REQUIRE(a.count == exact.count);
    for (std::uint32_t k = 0; k < a.count; ++k) {
        CHECK(a.obstacles[k].id == exact.obstacles[k].id);
        CHECK(a.obstacles[k].radius == exact.obstacles[k].radius);  // no noise on the size
        CHECK(a.obstacles[k].height == exact.obstacles[k].height);
        for (std::size_t i = 0; i < 3; ++i) {
            const double n = a.obstacles[k].position[i] - exact.obstacles[k].position[i];
            CHECK(n != 0);
            CHECK(std::abs(n) <= sigma * std::sqrt(3.0));
        }
    }
}

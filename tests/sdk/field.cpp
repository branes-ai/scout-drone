// The field contract (msg/field.hpp): its invariants, inside(), and cast().

#include <branes/scout/msg/field.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace branes::scout;

namespace {

/// A valid field: 60 × 30 × 10 m, the start and the goal 50 m apart, one obstacle
/// between them. Every value is exact in float, so cast() round-trips.
Field<double> one_tree() {
    Field<double> f;
    f.geofence = {{0.0, 0.0, 0.0}, {60.0, 30.0, 10.0}};
    f.start = {{5.0, 15.0, 0.125}, 0.0};
    f.goal = {{55.0, 15.0, 2.0}, 1.0};
    f.obstacle_count = 1;
    f.obstacles[0] = {{30.0, 15.0, 0.0}, 0.25, 8.0};
    return f;
}

}  // namespace

TEST_CASE("field: a valid field", "[field]") {
    REQUIRE(valid(one_tree()));
    Field<double> empty = one_tree();
    empty.obstacle_count = 0;
    REQUIRE(valid(empty));
}

TEST_CASE("field: each invariant, broken", "[field]") {
    Field<double> f = one_tree();
    f.geofence.max[2] = f.geofence.min[2];
    CHECK_FALSE(valid(f));  // I1

    f = one_tree();
    f.start.position[0] = -1.0;
    CHECK_FALSE(valid(f));  // I2: the start outside
    f = one_tree();
    f.goal.position[2] = 11.0;
    CHECK_FALSE(valid(f));  // I2: the goal outside

    f = one_tree();
    f.goal.radius = 0.0;
    CHECK_FALSE(valid(f));  // I3

    f = one_tree();
    f.obstacle_count = kMaxObstacles + 1;
    CHECK_FALSE(valid(f));  // I4: too many
    f = one_tree();
    f.obstacles[0].radius = 0.0;
    CHECK_FALSE(valid(f));  // I4: no radius
    f = one_tree();
    f.obstacles[0].height = -1.0;
    CHECK_FALSE(valid(f));  // I4: no height
    f = one_tree();
    f.obstacles[0].base[1] = 31.0;
    CHECK_FALSE(valid(f));  // I4: the base outside

    f = one_tree();
    f.obstacles[0].base = {5.1, 15.0, 0.0};
    CHECK_FALSE(valid(f));  // I5: on the start
    f = one_tree();
    f.obstacles[0].base = {55.0, 15.2, 0.0};
    CHECK_FALSE(valid(f));  // I5: on the goal
}

TEST_CASE("field: inside an obstacle is within its radius and between its foot and its top", "[field]") {
    const Obstacle<double> o{{10.0, 10.0, 0.0}, 0.5, 4.0};
    CHECK(inside(o, {10.0, 10.4, 2.0}));
    CHECK_FALSE(inside(o, {10.0, 10.5, 2.0}));   // on the surface: outside
    CHECK(inside(o, {10.0, 10.0, 4.0}));         // the top
    CHECK_FALSE(inside(o, {10.0, 10.0, 4.1}));   // above
    CHECK_FALSE(inside(o, {10.0, 10.0, -0.1}));  // below the foot
}

TEST_CASE("field: cast to float and back keeps a field of exact values", "[field]") {
    const Field<double> f = one_tree();
    const Field<float> g = cast<float>(f);
    REQUIRE(valid(g));
    REQUIRE(g.obstacle_count == f.obstacle_count);
    REQUIRE(cast<double>(g) == f);
}

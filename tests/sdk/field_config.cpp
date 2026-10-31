// The field file (config/field.hpp): loading the shipped fields, the errors,
// the round trip, and the field a seed generates.

#include <branes/anatomy/util/splitmix64.hpp>
#include <branes/scout/config/field.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <filesystem>
#include <string>

using namespace branes::scout;
using namespace branes::scout::config;

namespace {

const std::filesystem::path src = SCOUT_SOURCE_DIR;

FieldConfig load(const char* name) {
    auto c = load_field(src / "fields" / name);
    INFO((c ? std::string() : c.error().to_string()));
    REQUIRE(c);
    return *c;
}

/// The error parse_field reports for `yaml`, as "key: message".
std::string error_of(const std::string& yaml) {
    auto c = parse_field(yaml, "test");
    REQUIRE_FALSE(c);
    return c.error().key + ": " + c.error().message;
}

const std::string kHead = "name: t\n"
                          "geofence: {min_m: [0, 0, 0], max_m: [60, 30, 10]}\n"
                          "start: {position_m: [5, 15, 0.1], yaw_rad: 0}\n"
                          "goal: {position_m: [55, 15, 2], radius_m: 1}\n";

double ground_distance(const branes::anatomy::Vec3<double>& p, const branes::anatomy::Vec3<double>& q) {
    return std::hypot(p[0] - q[0], p[1] - q[1]);
}

}  // namespace

TEST_CASE("field file: the shipped fields load", "[field][config]") {
    const FieldConfig open = load("open-60.yaml");
    REQUIRE(open.name == "open-60");
    REQUIRE(valid(open.field));
    REQUIRE(open.field.obstacle_count == 0);
    REQUIRE_FALSE(open.generate);

    const FieldConfig forest = load("forest-60.yaml");
    REQUIRE(forest.name == "forest-60");
    REQUIRE(forest.generate);
    REQUIRE(forest.field.obstacle_count == 0);
    // The density times the ground area, as the file's comment says.
    const auto& g = forest.field.geofence;
    const double area = (g.max[0] - g.min[0]) * (g.max[1] - g.min[1]);
    REQUIRE(generated_count(*forest.generate, g) == std::llround(forest.generate->density_per_m2 * area));
}

TEST_CASE("field file: emitting and loading again gives the same config", "[field][config]") {
    for (const char* name : {"open-60.yaml", "forest-60.yaml"}) {
        const FieldConfig c = load(name);
        auto again = parse_field(emit_field(c), "emitted");
        REQUIRE(again);
        REQUIRE(*again == c);
    }
    auto listed = parse_field(kHead + "obstacles:\n  - {base_m: [30, 15, 0], radius_m: 0.3, height_m: 8}\n", "t");
    REQUIRE(listed);
    REQUIRE(listed->field.obstacle_count == 1);
    auto again = parse_field(emit_field(*listed), "emitted");
    REQUIRE(again);
    REQUIRE(*again == *listed);
}

TEST_CASE("field file: each error names its key", "[field][config]") {
    CHECK(error_of(kHead + "trees: 3\n") == "trees: unknown key");
    CHECK(error_of("name: t\n") == "geofence: missing");
    CHECK(error_of("name: t\ngeofence: {min_m: [0, 0, 0], max_m: [60, 0, 10]}\n") ==
          "geofence: min_m must be < max_m on every axis");
    CHECK(error_of(kHead.substr(0, kHead.find("goal")) + "goal: {position_m: [55, 15, 2], radius_m: 0}\n") ==
          "goal.radius_m: must be > 0");
    CHECK(error_of(kHead + "obstacles:\n  - {base_m: [30, 40, 0], radius_m: 0.3, height_m: 8}\n") ==
          "obstacles[0].base_m: must be inside the geofence");
    CHECK(error_of(kHead + "obstacles:\n  - {base_m: [5, 15, 0], radius_m: 0.3, height_m: 8}\n") ==
          "obstacles[0]: contains the start position");
    CHECK(error_of(kHead + "generate: {density_per_m2: 0.01, radius_m: [0.5, 0.2], height_m: [4, 8], "
                           "clear_radius_m: 4}\n") == "generate.radius_m: expected [min, max] with 0 < min ≤ max");
    CHECK(error_of(kHead + "generate: {density_per_m2: 1, radius_m: [0.2, 0.4], height_m: [4, 8], "
                           "clear_radius_m: 4}\n") ==
          "generate.density_per_m2: the listed and generated obstacles must number at most " +
              std::to_string(kMaxObstacles));
    // Far beyond any count: rejected, not wrapped round to a small one.
    CHECK(error_of(kHead + "generate: {density_per_m2: 1e30, radius_m: [0.2, 0.4], height_m: [4, 8], "
                           "clear_radius_m: 4}\n") ==
          "generate.density_per_m2: the listed and generated obstacles must number at most " +
              std::to_string(kMaxObstacles));
}

TEST_CASE("field of a seed: a listed field is the same for every seed", "[field][config]") {
    const FieldConfig open = load("open-60.yaml");
    auto a = field_for_seed(open, 1);
    auto b = field_for_seed(open, 2);
    REQUIRE(a);
    REQUIRE(b);
    REQUIRE(*a == open.field);
    REQUIRE(*b == open.field);
}

TEST_CASE("field of a seed: a generated field is valid, in range, and clear of the start and the goal",
          "[field][config]") {
    const FieldConfig forest = load("forest-60.yaml");
    const FieldGeneration& g = *forest.generate;
    for (const std::uint64_t seed : {1ull, 1700ull, 20261031ull}) {
        auto f = field_for_seed(forest, seed);
        REQUIRE(f);
        REQUIRE(valid(*f));
        REQUIRE(f->obstacle_count == generated_count(g, f->geofence));
        for (std::uint32_t i = 0; i < f->obstacle_count; ++i) {
            const Obstacle<double>& o = f->obstacles[i];
            CHECK(o.radius >= g.radius_min_m);
            CHECK(o.radius <= g.radius_max_m);
            CHECK(o.height >= g.height_min_m);
            CHECK(o.height <= g.height_max_m);
            CHECK(o.base[2] == f->geofence.min[2]);
            CHECK(ground_distance(o.base, f->start.position) >= g.clear_radius_m + o.radius);
            CHECK(ground_distance(o.base, f->goal.position) >= g.clear_radius_m + o.radius);
        }
    }
}

TEST_CASE("field of a seed: the same seed grows the same field, another seed another", "[field][config]") {
    const FieldConfig forest = load("forest-60.yaml");
    auto a = field_for_seed(forest, 1700);
    auto b = field_for_seed(forest, 1700);
    auto c = field_for_seed(forest, 1701);
    REQUIRE(a);
    REQUIRE(b);
    REQUIRE(c);
    REQUIRE(*a == *b);
    REQUIRE_FALSE(*a == *c);
}

TEST_CASE("field of a seed: the first obstacle comes from the field's own stream", "[field][config]") {
    // Each obstacle draws its radius and its height, then its position, so
    // the first two draws of the stream are the first obstacle's.
    const FieldConfig forest = load("forest-60.yaml");
    const FieldGeneration& g = *forest.generate;
    auto f = field_for_seed(forest, 1700);
    REQUIRE(f);
    branes::anatomy::SplitMix64 rng(branes::anatomy::stream_seed(1700, kFieldStream));
    const double radius = g.radius_min_m + (g.radius_max_m - g.radius_min_m) * rng.uniform01();
    const double height = g.height_min_m + (g.height_max_m - g.height_min_m) * rng.uniform01();
    REQUIRE(f->obstacles[0].radius == radius);
    REQUIRE(f->obstacles[0].height == height);
}

TEST_CASE("field of a seed: an obstacle that can't be placed fails the field", "[field][config]") {
    // A 10 × 10 m field whose clear zones cover all of it.
    auto c = parse_field("name: t\n"
                         "geofence: {min_m: [0, 0, 0], max_m: [10, 10, 10]}\n"
                         "start: {position_m: [2, 5, 0.1], yaw_rad: 0}\n"
                         "goal: {position_m: [8, 5, 2], radius_m: 1}\n"
                         "generate: {density_per_m2: 0.05, radius_m: [0.2, 0.3], height_m: [4, 8], "
                         "clear_radius_m: 20}\n",
                         "t");
    REQUIRE(c);
    auto f = field_for_seed(*c, 1);
    REQUIRE_FALSE(f);
    REQUIRE(f.error().find("could not be placed") != std::string::npos);
}

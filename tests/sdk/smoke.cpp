// The skeleton's smoke test: the mission library builds and links against
// anatomy's mobility profile and simulator. The mission's own tests arrive
// with its world, slots and simulator (docs/plan/bring-up/).

#include <branes/anatomy/anatomy.hpp>
#include <branes/anatomy/mobility/slots/context.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>
#include <branes/scout/scout.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {

/// The least a world needs to build slots from: the vehicle's start pose.
struct StartOnly {
    branes::anatomy::StartPose<double> start{{0.0, 0.0, 0.1}, 0.0};
};

}  // namespace

TEST_CASE("scout: the library builds and names itself", "[smoke]") {
    STATIC_REQUIRE(branes::scout::kLibraryName == "scout");
    STATIC_REQUIRE(branes::anatomy::kLibraryName == "anatomy");
}

TEST_CASE("scout: anatomy's mobility profile and simulator are reachable", "[smoke]") {
    STATIC_REQUIRE(branes::anatomy::MissionWorld<StartOnly>);
    STATIC_REQUIRE(branes::anatomy::sim::OracleSensor<branes::anatomy::sim::NoOracle>);
}

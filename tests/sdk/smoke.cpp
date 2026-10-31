// The smoke test: the mission library builds and links against anatomy, and
// scout's world is one anatomy can build slots from.

#include <branes/anatomy/anatomy.hpp>
#include <branes/anatomy/mobility/slots/context.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/scout.hpp>

#include <catch2/catch_test_macros.hpp>

TEST_CASE("scout: the library builds and names itself", "[smoke]") {
    STATIC_REQUIRE(branes::scout::kLibraryName == "scout");
    STATIC_REQUIRE(branes::anatomy::kLibraryName == "anatomy");
}

TEST_CASE("scout: the field is a world anatomy builds slots from", "[smoke]") {
    STATIC_REQUIRE(branes::anatomy::MissionWorld<branes::scout::Field<double>>);
}

// The obstacles seen (msg/obstacles.hpp) and their log codec
// (msg/obstacle_codec.hpp): the invariants, the cast, the round trip, and the
// records the decoder refuses. And scout's channel (log/channels.hpp): it
// reads only with kScoutChannels.

#include <branes/anatomy/log/log.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <filesystem>
#include <limits>
#include <string>
#include <system_error>
#include <vector>

using namespace branes::scout;
using branes::anatomy::Timestamp;

namespace {

ObstacleObservations<double> two_seen() {
    ObstacleObservations<double> o{};
    o.t = Timestamp{40'000'000};
    o.count = 2;
    o.obstacles[0] = ObstacleObservation<double>{3, {4.0, -1.5, -2.0}, 0.375, 8.0};
    o.obstacles[1] = ObstacleObservation<double>{-1, {9.25, 2.0, -1.75}, 0.125, 4.5};
    return o;
}

/// Remove a test file without throwing (on Windows a still-open file cannot be removed).
void cleanup(const std::filesystem::path& p) {
    std::error_code ec;
    std::filesystem::remove(p, ec);
}

}  // namespace

TEST_CASE("obstacles: valid() holds the count and each obstacle in use to its invariants", "[obstacles][msg]") {
    CHECK(valid(ObstacleObservations<double>{}));  // none seen
    CHECK(valid(two_seen()));

    auto over = two_seen();
    over.count = kMaxVisibleObstacles + 1;
    CHECK_FALSE(valid(over));  // I1

    auto flat = two_seen();
    flat.obstacles[1].radius = 0;
    CHECK_FALSE(valid(flat));  // I2
    auto low = two_seen();
    low.obstacles[0].height = -1;
    CHECK_FALSE(valid(low));  // I2
    auto nan = two_seen();
    nan.obstacles[0].position[2] = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(valid(nan));  // I2
    auto wide = two_seen();
    wide.obstacles[1].radius = std::numeric_limits<double>::infinity();
    CHECK_FALSE(valid(wide));  // I2: finite
    auto tall = two_seen();
    tall.obstacles[0].height = std::numeric_limits<double>::infinity();
    CHECK_FALSE(valid(tall));  // I2: finite

    auto unused = two_seen();
    unused.obstacles[5].radius = -3;  // beyond count: not in use
    CHECK(valid(unused));
}

TEST_CASE("obstacles: cast converts every entry, and back", "[obstacles][msg]") {
    const auto o = two_seen();
    const ObstacleObservations<float> f = cast<float>(o);
    CHECK(f.t == o.t);
    CHECK(f.count == 2u);
    CHECK(f.obstacles[0].id == 3);
    CHECK(f.obstacles[1].position[0] == 9.25f);
    CHECK(valid(f));
    CHECK(cast<double>(f) == o);  // every value here is a dyadic fraction, exact in float
}

TEST_CASE("obstacle codec: the round trip, with none seen and with some", "[obstacles][codec]") {
    for (const auto& o : {ObstacleObservations<double>{}, two_seen()}) {
        const auto d = codec::decode_obstacles(codec::encode(o));
        REQUIRE(d);
        CHECK(*d == o);
    }
    // Only the entries in use are encoded: 8 + 4 bytes, then 4 + 5·8 per obstacle.
    CHECK(codec::encode(ObstacleObservations<double>{}).size() == 12u);
    CHECK(codec::encode(two_seen()).size() == 12u + 2u * 44u);
}

TEST_CASE("obstacle codec: a malformed or invalid record does not decode", "[obstacles][codec]") {
    const std::vector<std::byte> good = codec::encode(two_seen());
    CHECK_FALSE(codec::decode_obstacles({}));
    CHECK_FALSE(codec::decode_obstacles(std::vector<std::byte>(good.begin(), good.end() - 1)));  // truncated
    auto trailing = good;
    trailing.push_back(std::byte{0});
    CHECK_FALSE(codec::decode_obstacles(trailing));

    auto over = two_seen();
    over.count = kMaxVisibleObstacles + 1;
    CHECK_FALSE(codec::decode_obstacles(codec::encode(over)));  // count beyond capacity

    auto nan = two_seen();
    nan.obstacles[1].height = std::numeric_limits<double>::infinity();
    CHECK_FALSE(codec::decode_obstacles(codec::encode(nan)));  // not finite
    auto flat = two_seen();
    flat.obstacles[0].radius = 0;
    CHECK_FALSE(codec::decode_obstacles(codec::encode(flat)));  // not valid()
}

TEST_CASE("log: scout's oracle_obstacles channel reads only with kScoutChannels", "[obstacles][log]") {
    namespace alog = branes::anatomy::log;
    const auto p = std::filesystem::temp_directory_path() / "scout_log_test_mission_channel.brlog";
    alog::LogHeader h;
    h.baseline_name = "null";
    h.composition_hash = "abc";
    h.world_hash = "f";
    h.vehicle_hash = "v";
    h.world_yaml = "name: field\n";
    h.vehicle_yaml = "name: vehicle\n";
    {
        auto w = alog::LogWriter::open(p, h);
        REQUIRE(w);
        w->write(log::oracle_obstacles, Timestamp{10}, codec::encode(two_seen()));
        REQUIRE(w->close());
    }
    {
        auto r = alog::LogReader::open(p, log::kScoutChannels);  // scoped: closed before cleanup()
        REQUIRE(r);
        const auto rec = r->next();
        REQUIRE(rec);
        REQUIRE(rec->has_value());
        CHECK((*rec)->channel == log::oracle_obstacles);
        const auto o = codec::decode_obstacles((*rec)->payload);
        REQUIRE(o);
        CHECK(*o == two_seen());
    }
    {
        auto r = alog::LogReader::open(p, {});  // no mission channels: 7 is unknown
        REQUIRE(r);
        const auto rec = r->next();
        REQUIRE_FALSE(rec);
        CHECK(rec.error().message == "unknown channel 7");
    }
    cleanup(p);
}

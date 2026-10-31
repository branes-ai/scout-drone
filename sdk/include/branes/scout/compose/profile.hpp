#pragma once
/// @file profile.hpp
/// @brief ScoutProfile: the stack profile of scout's stack.
///
/// A roaming vehicle's profile (anatomy MobilityProfile: the IMU and the
/// camera in; the estimate, the reference and the command out), with one
/// more input: the obstacles seen, on scout's oracle_obstacles channel, which
/// the harness hands to the stack's on_obstacles(). The truth, the outputs
/// and their codecs are MobilityProfile's.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <array>
#include <string>

namespace branes::scout {

struct ScoutProfile : anatomy::MobilityProfile {
    static constexpr std::array<anatomy::log::Channel, 3> inputs = {
        anatomy::log::Channel::imu, anatomy::log::Channel::camera, log::oracle_obstacles};

    /// MobilityProfile's hooks, and on_obstacles for the obstacles seen.
    template <class S>
    static constexpr bool drives =
        anatomy::MobilityProfile::drives<S> &&
        requires(S s, const ObstacleObservations<typename S::scalar_type>& o) { s.on_obstacles(o); };

    /// Hand one IMU, camera or oracle_obstacles record to `s`. The obstacles
    /// are cast to the stack's scalar type and must be valid() in it; the
    /// IMU and the camera are MobilityProfile's. Pre: rec.channel is one of `inputs`.
    template <class S>
    [[nodiscard]] static anatomy::result<anatomy::ok_t, std::string> feed(S& s, const anatomy::log::LogRecord& rec) {
        using T = typename S::scalar_type;
        if (rec.channel != log::oracle_obstacles) {
            return anatomy::MobilityProfile::feed(s, rec);
        }
        const auto o = codec::decode_obstacles(rec.payload);
        if (!o) {
            return std::string("undecodable oracle_obstacles record at t=" + std::to_string(rec.t.ns));
        }
        const ObstacleObservations<T> ot = cast<T>(*o);
        if (!valid(ot)) {
            return std::string("oracle_obstacles record at t=" + std::to_string(rec.t.ns) +
                               " is not valid in the stack's scalar type");
        }
        s.on_obstacles(ot);
        return anatomy::ok;
    }
};
static_assert(anatomy::StackProfile<ScoutProfile>);

}  // namespace branes::scout

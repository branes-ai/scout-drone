#pragma once
/// @file channels.hpp
/// @brief Scout's own log channels, beside the framework's (anatomy log/log.hpp).
///
/// A mission picks its own channel ids (racing-drone ADR 0001, amendment 5).
/// Scout's are kept apart from racing-drone's (oracle_gates is 5), so a log of
/// the other mission fails to open with scout's channels rather than being
/// read as scout's.

#include <branes/anatomy/log/log.hpp>

#include <array>
#include <cstdint>

namespace branes::scout::log {

using anatomy::log::Channel;

/// ObstacleObservations computed from the truth (SITL), a sim channel the
/// stack consumes (compose/profile.hpp).
inline constexpr Channel oracle_obstacles = static_cast<Channel>(7);

/// The channels scout adds to the framework's, for LogReader::open.
inline constexpr std::array<Channel, 1> kScoutChannels = {oracle_obstacles};

static_assert(!anatomy::log::is_framework(static_cast<std::uint16_t>(oracle_obstacles)),
              "scout channels are not framework channels");
static_assert(!anatomy::log::is_output(oracle_obstacles), "oracle_obstacles is an input (sim) channel");

}  // namespace branes::scout::log

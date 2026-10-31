#pragma once
/// @file scout.hpp
/// @brief branes::scout: the scout-drone mission.
///
/// Scout-drone flies to a goal through terrain it has never seen
/// (racing-drone ADR 0007), on the mission framework anatomy (racing-drone
/// ADR 0008). It supplies its task: its world, the slots that see it, its
/// stack and its scorer (docs/plan/bring-up/). This header names the library;
/// include the headers of the parts you use.

#include <string_view>

namespace branes::scout {

/// The library's name, as logs and tools refer to it.
inline constexpr std::string_view kLibraryName = "scout";

}  // namespace branes::scout

#pragma once
/// @file config.hpp
/// @brief Scout's configs: the field, scenario and baseline files.
///
/// The vehicle, scenario and baseline loaders are anatomy's (mobility/config/,
/// config/baseline.hpp); this header adds what scout supplies to them, its
/// scenario schema and its slot list, and the loaders built on them, so scout
/// code loads its files with one-argument calls. The field file is scout's
/// own (field.hpp).
///
/// Configs are load-time data and are stored in double. Components that run
/// in another scalar type convert what they need (e.g. cast<T>(Field)).

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/mobility/config/scenario.hpp>
#include <branes/scout/config/field.hpp>

#include <array>
#include <cassert>
#include <filesystem>
#include <string>
#include <string_view>

namespace branes::scout::config {

using anatomy::config::BaselineConfig;
using anatomy::config::ScenarioConfig;

// ── Scenario ────────────────────────────────────────────────────────

/// Scout's own `sim:` parameters, in the order they are stored and emitted.
/// The obstacle oracle (docs/plan/bring-up/, S0d) reads them.
inline constexpr std::array<anatomy::config::WorldSimParam, 2> kScoutWorldSim = {
    anatomy::config::WorldSimParam{"oracle_range_m", 10.0, 0.0},  ///< obstacles seen within this range [m]
    anatomy::config::WorldSimParam{"oracle_sigma_m", 0.0, 0.0},   ///< noise on each seen obstacle's position, 1σ [m]
};

/// Scout's scenario schema: the world file is the field (`field:`).
inline constexpr anatomy::config::ScenarioSchema kScoutScenario{"field", kScoutWorldSim};

[[nodiscard]] inline anatomy::result<ScenarioConfig, ConfigError> load_scenario(const std::filesystem::path& path) {
    return anatomy::config::load_scenario(path, kScoutScenario);
}

/// `base_dir` resolves the field and vehicle references.
[[nodiscard]] inline anatomy::result<ScenarioConfig, ConfigError>
parse_scenario(std::string_view yaml, std::string_view origin, const std::filesystem::path& base_dir) {
    return anatomy::config::parse_scenario(yaml, origin, base_dir, kScoutScenario);
}

/// Pre: follows_scout_schema(s).
[[nodiscard]] inline std::string emit_scenario(const ScenarioConfig& s) {
    assert(s.world_sim.size() == kScoutWorldSim.size());  // Pre
    return anatomy::config::emit_scenario(s, kScoutScenario);
}

/// True if `s` carries scout's `sim:` parameters (one value per parameter of
/// kScoutWorldSim). Every scenario loaded with kScoutScenario does; a scenario
/// built in code must set `world_sim = default_world_sim(kScoutScenario)`.
[[nodiscard]] inline bool follows_scout_schema(const ScenarioConfig& s) noexcept {
    return s.world_sim.size() == kScoutWorldSim.size();
}

/// The oracle's sensing range [m] (`sim.oracle_range_m`). Pre: follows_scout_schema(s).
[[nodiscard]] inline double oracle_range_m(const ScenarioConfig& s) noexcept {
    assert(follows_scout_schema(s));  // Pre
    return s.world_sim[0];
}

/// The oracle's position noise, 1σ [m] (`sim.oracle_sigma_m`). Pre: follows_scout_schema(s).
[[nodiscard]] inline double oracle_sigma_m(const ScenarioConfig& s) noexcept {
    assert(follows_scout_schema(s));  // Pre
    return s.world_sim[1];
}

/// The field a scenario flies: its field file, with the scenario's seed.
[[nodiscard]] inline anatomy::result<Field<double>, std::string> load_scenario_field(const ScenarioConfig& s) {
    auto c = load_field(s.world_path);
    if (!c) {
        return c.error().to_string();
    }
    return field_for_seed(*c, s.seed);
}

// ── Baseline ────────────────────────────────────────────────────────

/// Scout's six slots (slots/slots.hpp), sorted: the names a baseline's
/// `composition:` block gives an implementation each (cmake/slot_registry.cmake).
inline constexpr std::array<std::string_view, 6> kSlotNames = {
    "controller",
    "daa_tracker",
    "estimator",
    "fc_bridge",
    "planner",
    "safety_filter",
};

[[nodiscard]] inline anatomy::result<BaselineConfig, ConfigError> load_baseline(const std::filesystem::path& path) {
    return anatomy::config::load_baseline(path, kSlotNames);
}

[[nodiscard]] inline anatomy::result<BaselineConfig, ConfigError> parse_baseline(std::string_view yaml,
                                                                                 std::string_view origin) {
    return anatomy::config::parse_baseline(yaml, origin, kSlotNames);
}

/// SHA-256 (hex) of scout's canonical composition — identical to the hash
/// cmake/baselines.cmake compiles into each scout binary.
[[nodiscard]] inline std::string composition_hash(const BaselineConfig& b) {
    return anatomy::config::composition_hash(b, kSlotNames);
}

}  // namespace branes::scout::config

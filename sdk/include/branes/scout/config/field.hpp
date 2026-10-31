#pragma once
/// @file field.hpp
/// @brief The field file: type, loader, emitter, and the field a seed generates.
///
/// Every loader validates the whole file before it returns a value, so a config
/// that loads satisfies all the invariants listed on its type. Loading never
/// throws; it returns result<X, ConfigError>. Every emitter writes a file that
/// loads back to an equal value (round trip), with doubles written to 17
/// significant digits.
///
/// A field file lists obstacles (`obstacles:`), generates them from the run's
/// seed (`generate:`), or both. field_for_seed() builds the field a run flies:
/// the listed obstacles, then the generated ones, drawn from the seed's
/// kFieldStream (anatomy::stream_seed), so the field does not shift the
/// simulator's own random streams.

#include <branes/anatomy/config/error.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/scout/msg/field.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace branes::scout::config {

using anatomy::config::ConfigError;

/// The random stream of a field's generated obstacles ("FIELD").
inline constexpr std::uint64_t kFieldStream = 0x4649454C44;

/// How a field file generates obstacles (`generate:`).
///
/// Invariants: density_per_m2 ≥ 0; 0 < radius_min_m ≤ radius_max_m;
/// 0 < height_min_m ≤ height_max_m; clear_radius_m ≥ 0.
struct FieldGeneration {
    double density_per_m2{0};  ///< obstacles per m² of the geofence's ground area
    double radius_min_m{0};
    double radius_max_m{0};
    double height_min_m{0};
    double height_max_m{0};
    double clear_radius_m{0};  ///< no generated obstacle comes nearer the start or the goal, horizontally [m]

    friend bool operator==(const FieldGeneration&, const FieldGeneration&) = default;
};

/// fields/<name>.yaml.
///
/// Invariants: valid(field), with field.obstacles the listed ones; if
/// `generate` is set, the listed obstacles and generated_count() of them fit
/// in kMaxObstacles, and the geofence is wider than 2·radius_max_m on x and y.
struct FieldConfig {
    std::string name;
    Field<double> field;                      ///< the listed obstacles only
    std::optional<FieldGeneration> generate;  ///< absent: the listed obstacles are the field

    friend bool operator==(const FieldConfig&, const FieldConfig&) = default;
};

[[nodiscard]] anatomy::result<FieldConfig, ConfigError> load_field(const std::filesystem::path& path);
[[nodiscard]] anatomy::result<FieldConfig, ConfigError> parse_field(std::string_view yaml, std::string_view origin);
[[nodiscard]] std::string emit_field(const FieldConfig& c);

/// The number of obstacles `g` generates on `f`'s geofence: the density times
/// the ground area, rounded to the nearest integer.
[[nodiscard]] std::uint32_t generated_count(const FieldGeneration& g, const anatomy::Geofence<double>& f);

/// The field a run seeded `seed` flies: `c`'s listed obstacles, then, if `c`
/// generates, generated_count() more. Each generated obstacle draws its radius,
/// its height, then its position, uniform in their ranges; a position nearer
/// the start or the goal than clear_radius_m (from the obstacle's edge, on the
/// ground) is drawn again, at most kMaxPlacementDraws times.
///
/// Post: success ⇒ valid(result), and the same (c, seed) gives the same field
/// on every platform. Fails if an obstacle can't be placed.
[[nodiscard]] anatomy::result<Field<double>, std::string> field_for_seed(const FieldConfig& c, std::uint64_t seed);

/// The draws allowed for one generated obstacle's position.
inline constexpr int kMaxPlacementDraws = 1000;

}  // namespace branes::scout::config

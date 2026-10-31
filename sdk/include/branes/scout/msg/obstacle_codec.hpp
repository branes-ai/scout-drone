#pragma once
/// @file obstacle_codec.hpp
/// @brief Log payload encoding of ObstacleObservations, scout's oracle_obstacles channel.
///
/// The framework's codecs and their helpers are anatomy's (mobility/msg/codec.hpp).

#include <branes/anatomy/mobility/msg/codec.hpp>
#include <branes/anatomy/util/bytes.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <vector>

namespace branes::scout::codec {

// ── ObstacleObservations (sim channel) ──────────────────────────────

/// Encodes the time and the `count` used entries only.
[[nodiscard]] inline std::vector<std::byte> encode(const ObstacleObservations<double>& o) {
    anatomy::ByteWriter w;
    w.i64(o.t.ns);
    w.u32(o.count);
    for (std::uint32_t i = 0; i < o.count && i < kMaxVisibleObstacles; ++i) {
        const ObstacleObservation<double>& b = o.obstacles[i];
        w.i32(b.id);
        anatomy::codec::detail::put(w, b.position);
        w.f64(b.radius);
        w.f64(b.height);
    }
    return w.take();
}

/// The decoded observations, or nullopt if `b` is malformed, holds a value
/// that is not finite, or decodes to observations that are not valid().
[[nodiscard]] inline std::optional<ObstacleObservations<double>> decode_obstacles(std::span<const std::byte> b) {
    anatomy::ByteReader r(b);
    ObstacleObservations<double> o{};
    o.t.ns = r.i64();
    o.count = r.u32();
    if (r.failed() || o.count > kMaxVisibleObstacles) {
        return std::nullopt;
    }
    for (std::uint32_t i = 0; i < o.count; ++i) {
        ObstacleObservation<double>& s = o.obstacles[i];
        s.id = r.i32();
        s.position = anatomy::codec::detail::get_vec3(r);
        s.radius = r.f64();
        s.height = r.f64();
        if (!anatomy::codec::detail::finite(s.position) || !std::isfinite(s.radius) || !std::isfinite(s.height)) {
            return std::nullopt;
        }
    }
    if (!r.done() || !valid(o)) {
        return std::nullopt;
    }
    return o;
}

}  // namespace branes::scout::codec

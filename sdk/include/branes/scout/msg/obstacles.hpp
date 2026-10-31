#pragma once
/// @file obstacles.hpp
/// @brief ObstacleObservations: the obstacles the vehicle sees at one camera frame.
///
/// Until scout has a depth pipeline, the obstacle oracle (S0d) computes them
/// from the truth at each camera frame, as the obstacles within its sensing
/// range, and logs them on scout's oracle_obstacles channel
/// (log/channels.hpp). Positions are relative to the vehicle and in its body
/// frame, as a sensor on the vehicle reports them: the world-frame offset of
/// an obstacle's foot from the vehicle, rotated by the inverse of the attitude
/// (TruthState::attitude is body → world). Fixed capacity, so a cycle never
/// allocates (SW-01): an oracle that sees more keeps the nearest.
///
/// Model
/// -----
///   ObstacleObservations<T>
///     t         : Timestamp — the time the obstacles were seen at
///     count     : number of obstacles in use
///     obstacles : std::array<ObstacleObservation<T>, kMaxVisibleObstacles>, the first count in use
///
/// Invariants (valid())
/// ----------
///   I1: count ≤ kMaxVisibleObstacles
///   I2: every obstacle in use has a finite position, and a finite radius > 0 and height > 0

#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>

#include <array>
#include <cmath>
#include <cstdint>

namespace branes::scout {

inline constexpr std::uint32_t kMaxVisibleObstacles = 32;

/// One obstacle seen: a vertical cylinder, located by the centre of its foot.
template <typename T>
struct ObstacleObservation {
    std::int32_t id{-1};          ///< the obstacle's index in the field, −1 = unknown
    anatomy::Vec3<T> position{};  ///< its foot's centre relative to the vehicle, body frame [m]
    T radius{0};                  ///< [m]
    T height{0};                  ///< [m]

    friend constexpr bool operator==(const ObstacleObservation&, const ObstacleObservation&) = default;
};

template <typename T>
struct ObstacleObservations {
    anatomy::Timestamp t;
    std::uint32_t count{0};
    std::array<ObstacleObservation<T>, kMaxVisibleObstacles> obstacles{};

    friend constexpr bool operator==(const ObstacleObservations&, const ObstacleObservations&) = default;
};

template <typename T>
[[nodiscard]] constexpr bool valid(const ObstacleObservations<T>& o) noexcept {
    using std::isfinite;
    if (o.count > kMaxVisibleObstacles) {
        return false;  // I1
    }
    for (std::uint32_t i = 0; i < o.count; ++i) {
        const ObstacleObservation<T>& b = o.obstacles[i];
        if (!(isfinite(b.position[0]) && isfinite(b.position[1]) && isfinite(b.position[2]) && isfinite(b.radius) &&
              isfinite(b.height) && b.radius > T(0) && b.height > T(0))) {
            return false;  // I2
        }
    }
    return true;
}

/// Obstacle observations in another scalar type, element by element (all entries, used or not).
template <typename To, typename From>
[[nodiscard]] constexpr ObstacleObservations<To> cast(const ObstacleObservations<From>& o) noexcept {
    using anatomy::cast;
    ObstacleObservations<To> out{};
    out.t = o.t;
    out.count = o.count;
    for (std::uint32_t i = 0; i < kMaxVisibleObstacles; ++i) {
        const ObstacleObservation<From>& b = o.obstacles[i];
        out.obstacles[i] = ObstacleObservation<To>{b.id, cast<To>(b.position), To(b.radius), To(b.height)};
    }
    return out;
}

}  // namespace branes::scout

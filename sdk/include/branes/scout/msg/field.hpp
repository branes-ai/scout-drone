#pragma once
/// @file field.hpp
/// @brief Field: scout's world, as the slots see it.
///
/// A start pose, a geofence, a goal, and the static obstacles between them,
/// each a vertical cylinder standing on the ground (docs/plan/bring-up/). The
/// field a run flies is built from a field file and the run's seed
/// (config::field_for_seed): the file lists obstacles, generates them, or
/// both. Loaded in double, and converted to the stack's scalar type with
/// cast<T>().
///
/// Model
/// -----
///   Field<T>
///     geofence       : Geofence<T> — where the vehicle may fly
///     start          : StartPose<T> — the pad (MissionWorld)
///     goal           : Goal<T> — where it must reach
///     obstacle_count : number of obstacles in use
///     obstacles      : std::array<Obstacle<T>, kMaxObstacles>, the first obstacle_count in use
///
/// Invariants (valid())
/// ----------
///   I1: geofence.min < geofence.max on every axis
///   I2: the start position and the goal position are inside the geofence
///   I3: goal.radius > 0
///   I4: obstacle_count ≤ kMaxObstacles; every obstacle in use has radius > 0,
///       height > 0, and its base inside the geofence
///   I5: neither the start position nor the goal position is inside an obstacle

#include <branes/anatomy/mobility/msg/region.hpp>
#include <branes/anatomy/msg/geometry.hpp>

#include <array>
#include <cstdint>

namespace branes::scout {

inline constexpr std::uint32_t kMaxObstacles = 256;

/// A vertical cylinder standing at `base` (the centre of its foot).
template <typename T>
struct Obstacle {
    anatomy::Vec3<T> base{};
    T radius{0};  ///< [m]
    T height{0};  ///< [m]

    friend constexpr bool operator==(const Obstacle&, const Obstacle&) = default;
};

/// The goal: reached within `radius` of `position`.
template <typename T>
struct Goal {
    anatomy::Vec3<T> position{};
    T radius{0};  ///< [m]

    friend constexpr bool operator==(const Goal&, const Goal&) = default;
};

template <typename T>
struct Field {
    anatomy::Geofence<T> geofence{};
    anatomy::StartPose<T> start{};
    Goal<T> goal{};
    std::uint32_t obstacle_count{0};
    std::array<Obstacle<T>, kMaxObstacles> obstacles{};

    friend constexpr bool operator==(const Field&, const Field&) = default;
};

/// True if `p` is inside obstacle `o`: within its radius horizontally, and
/// between its foot and its top.
template <typename T>
[[nodiscard]] constexpr bool inside(const Obstacle<T>& o, const anatomy::Vec3<T>& p) noexcept {
    const T dx = p[0] - o.base[0];
    const T dy = p[1] - o.base[1];
    return dx * dx + dy * dy < o.radius * o.radius && p[2] >= o.base[2] && p[2] <= o.base[2] + o.height;
}

template <typename T>
[[nodiscard]] constexpr bool valid(const Field<T>& f) noexcept {
    for (std::size_t a = 0; a < 3; ++a) {
        if (!(f.geofence.min[a] < f.geofence.max[a])) {
            return false;  // I1
        }
    }
    if (!contains(f.geofence, f.start.position) || !contains(f.geofence, f.goal.position)) {
        return false;  // I2
    }
    if (!(f.goal.radius > T(0))) {
        return false;  // I3
    }
    if (f.obstacle_count > kMaxObstacles) {
        return false;  // I4
    }
    for (std::uint32_t i = 0; i < f.obstacle_count; ++i) {
        const Obstacle<T>& o = f.obstacles[i];
        if (!(o.radius > T(0) && o.height > T(0) && contains(f.geofence, o.base))) {
            return false;  // I4
        }
        if (inside(o, f.start.position) || inside(o, f.goal.position)) {
            return false;  // I5
        }
    }
    return true;
}

/// Convert a field to another scalar type, element by element.
template <typename To, typename From>
[[nodiscard]] constexpr Field<To> cast(const Field<From>& f) noexcept {
    using anatomy::cast;
    Field<To> out{};
    out.geofence = anatomy::Geofence<To>{cast<To>(f.geofence.min), cast<To>(f.geofence.max)};
    out.start = anatomy::StartPose<To>{cast<To>(f.start.position), To(f.start.yaw)};
    out.goal = Goal<To>{cast<To>(f.goal.position), To(f.goal.radius)};
    out.obstacle_count = f.obstacle_count;
    for (std::uint32_t i = 0; i < kMaxObstacles; ++i) {
        const Obstacle<From>& o = f.obstacles[i];
        out.obstacles[i] = Obstacle<To>{cast<To>(o.base), To(o.radius), To(o.height)};
    }
    return out;
}

}  // namespace branes::scout

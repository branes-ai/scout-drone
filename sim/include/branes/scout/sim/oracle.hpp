#pragma once
/// @file oracle.hpp
/// @brief ObstacleOracle: the obstacles a perfect sensor sees, computed from the truth.
///
/// Scout's oracle sensor (anatomy sim::OracleSensor). It stands in for
/// perception until cortex's depth pipeline exists (docs/plan/bring-up/). At
/// each camera frame it sees the field's obstacles whose distance from the true
/// position (msg/field.hpp, distance()) is within its range, keeps the nearest
/// kMaxVisibleObstacles of them, nearest first (ties by field index), and
/// reports each as the field reports it, with its foot's centre relative to the
/// vehicle in the body frame (msg/obstacles.hpp).
///
/// With a noise σ > 0, each reported position gets noise on every axis,
/// uniform with that σ as the simulated IMU's is (anatomy sim/imu.hpp), so a
/// seed draws the same noise on every platform. The draws come from the
/// oracle's own stream (kOracleStream), three per obstacle seen, in the order
/// reported, so the oracle never shifts the simulator's streams. With σ = 0
/// it draws nothing.
///
/// Model
/// -----
///   ObstacleOracle
///     field_ : Field<double> [owned, immutable] — the obstacles
///     range_ : double [immutable] — the sensing range [m]
///     amp_   : double [immutable] — the noise's half-width, σ·√3 [m]
///     rng_   : SplitMix64 — the noise stream
///
/// Invariants
/// ----------
///   I1: valid(field_)
///   I2: range_ ≥ 0 and amp_ ≥ 0, both finite

#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/msg/geometry.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/anatomy/sim/basic_simulator.hpp>
#include <branes/anatomy/sim/camera_model.hpp>
#include <branes/anatomy/util/splitmix64.hpp>
#include <branes/scout/log/channels.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacle_codec.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <optional>
#include <utility>

namespace branes::scout::sim {

/// The random stream of the oracle's noise ("ORACLE").
inline constexpr std::uint64_t kOracleStream = 0x4F5241434C45;

/// The obstacles of `field` within `range` of x's position, as a perfect sensor
/// at x sees them at time t: nearest first, at most kMaxVisibleObstacles, with
/// no noise. Pre: valid(field), valid(x), range ≥ 0. Post: valid(result).
[[nodiscard]] inline ObstacleObservations<double>
see_obstacles(const Field<double>& field, const anatomy::TruthState<double>& x, anatomy::Timestamp t, double range) {
    assert(valid(field) && range >= 0);  // Pre
    std::array<std::pair<double, std::uint32_t>, kMaxObstacles> near{};
    std::uint32_t n = 0;
    for (std::uint32_t i = 0; i < field.obstacle_count; ++i) {
        const double d = distance(field.obstacles[i], x.position);
        if (d <= range) {
            near[n++] = {d, i};
        }
    }
    std::sort(near.begin(), near.begin() + n);  // by distance, then by index
    ObstacleObservations<double> o{};
    o.t = t;
    o.count = std::min(n, kMaxVisibleObstacles);
    const anatomy::Quat<double> world_to_body = anatomy::sim::detail::conjugate(x.attitude);
    for (std::uint32_t k = 0; k < o.count; ++k) {
        const Obstacle<double>& b = field.obstacles[near[k].second];
        const anatomy::Vec3<double> d{b.base[0] - x.position[0], b.base[1] - x.position[1], b.base[2] - x.position[2]};
        o.obstacles[k] = ObstacleObservation<double>{static_cast<std::int32_t>(near[k].second),
                                                     anatomy::sim::detail::rotate(world_to_body, d),
                                                     b.radius,
                                                     b.height};
    }
    assert(valid(o));  // Post
    return o;
}

class ObstacleOracle {
public:
    /// Pre: valid(field); range_m and sigma_m finite and ≥ 0. Post: I1, I2.
    ObstacleOracle(const Field<double>& field, double range_m, double sigma_m, std::uint64_t seed)
        : field_(field), range_(range_m), amp_(sigma_m * std::sqrt(3.0)),
          rng_(anatomy::stream_seed(seed, kOracleStream)) {
        assert(valid(field_));                                                             // Pre / I1
        assert(std::isfinite(range_) && range_ >= 0 && std::isfinite(amp_) && amp_ >= 0);  // Pre / I2
    }

    /// The obstacles seen from x at the frame's time t, on oracle_obstacles. Pre: valid(x).
    [[nodiscard]] std::optional<anatomy::sim::SimEvent> observe(const anatomy::TruthState<double>& x,
                                                                anatomy::Timestamp t) {
        ObstacleObservations<double> o = see_obstacles(field_, x, t, range_);
        if (amp_ > 0) {
            for (std::uint32_t k = 0; k < o.count; ++k) {
                for (double& c : o.obstacles[k].position) {
                    c += rng_.symmetric(amp_);
                }
            }
        }
        return anatomy::sim::SimEvent{log::oracle_obstacles, t, codec::encode(o)};
    }

    [[nodiscard]] const Field<double>& field() const noexcept {
        return field_;
    }

private:
    Field<double> field_;
    double range_;
    double amp_;
    anatomy::SplitMix64 rng_;
};
static_assert(anatomy::sim::OracleSensor<ObstacleOracle>);

}  // namespace branes::scout::sim

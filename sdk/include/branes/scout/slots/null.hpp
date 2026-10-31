#pragma once
/// @file null.hpp
/// @brief Null implementations of scout's task slots, for the `null` baseline.
///
/// The nulls of the mission-neutral slots are anatomy's
/// (mobility/slots/neutral_nulls.hpp), included here.
///
/// Stateless and deterministic: each returns a valid, fixed output derived only
/// from its inputs. They exercise the framework end to end (composition,
/// harnesses, log, replay) before any real component exists.

#include <branes/anatomy/mobility/slots/neutral_nulls.hpp>
#include <branes/scout/slots/slots.hpp>

namespace branes::scout {

/// Reports the vehicle at rest at the world origin, level, at the requested time.
template <typename T>
class NullEstimator {
public:
    void process_imu(const anatomy::ImuSample<T>&) noexcept {}
    void process_frame(const anatomy::CameraFrame&) noexcept {}
    void process_obstacles(const ObstacleObservations<T>&) noexcept {}

    [[nodiscard]] anatomy::StateEstimate<T> state_at(anatomy::Timestamp now) const noexcept {
        anatomy::StateEstimate<T> x{};
        x.t_valid = now;  // attitude defaults to identity, covariance to zero: valid()
        return x;
    }
    [[nodiscard]] anatomy::EstimatorHealth<T> health() const noexcept {
        return anatomy::EstimatorHealth<T>{T(0), 0, true};
    }
};

/// Plans "stay where you are": one sample at the current state.
template <typename T>
class NullPlanner {
public:
    [[nodiscard]] anatomy::Reference<T>
    plan(const Field<T>&, const ObstacleObservations<T>&, const anatomy::StateEstimate<T>& x) const noexcept {
        anatomy::Reference<T> r{};
        r.count = 1;
        r.samples[0].t = x.t_valid;
        r.samples[0].position = x.position;
        return r;
    }
};

static_assert(Estimator<NullEstimator<double>, double>);
static_assert(Planner<NullPlanner<double>, double>);

}  // namespace branes::scout

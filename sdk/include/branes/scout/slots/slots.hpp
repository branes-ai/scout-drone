#pragma once
/// @file slots.hpp
/// @brief Scout's two task slots: Estimator and Planner.
///
/// Scout's stack has six slots (docs/plan/bring-up/). The two that see the
/// task are defined here; the four mission-neutral ones (Controller,
/// SafetyFilter, DaaTracker, FcBridge) are anatomy's (mobility/slots/neutral.hpp).
/// There is no detector slot: until a depth pipeline exists, the obstacles
/// come from the oracle (msg/obstacles.hpp).
///
/// A baseline fills every slot with one implementation, chosen at compile time
/// (cmake/baselines.cmake). Every slot is templated on the scalar type T.

#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/imu.hpp>
#include <branes/anatomy/mobility/slots/neutral.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <concepts>

namespace branes::scout {

using anatomy::Controller;
using anatomy::DaaTracker;
using anatomy::FcBridge;
using anatomy::SafetyFilter;

/// Fuses the IMU, the camera and the obstacles seen into the world-frame state.
template <class E, class T>
concept Estimator = requires(E e,
                             const E& ce,
                             const anatomy::ImuSample<T>& imu,
                             const anatomy::CameraFrame& frame,
                             const ObstacleObservations<T>& obstacles,
                             anatomy::Timestamp now) {
    e.process_imu(imu);
    e.process_frame(frame);
    e.process_obstacles(obstacles);                                   // may be empty (none in range)
    { ce.state_at(now) } -> std::same_as<anatomy::StateEstimate<T>>;  // delay-compensated (EST-06)
    { ce.health() } -> std::same_as<anatomy::EstimatorHealth<T>>;     // NIS, staleness (SAF-03)
};

/// Plans the reference to the field's goal, around the obstacles seen.
template <class P, class T>
concept Planner =
    requires(P p, const Field<T>& field, const ObstacleObservations<T>& obstacles, const anatomy::StateEstimate<T>& x) {
        { p.plan(field, obstacles, x) } -> std::same_as<anatomy::Reference<T>>;  // obstacles: the latest seen
    };

}  // namespace branes::scout

#pragma once
/// @file ground_truth.hpp
/// @brief GroundTruthEstimator: scout's ground-truth estimator (SITL only).
///
/// Baseline s0-gt-pid's estimator (docs/plan/bring-up/). The estimate itself
/// is anatomy's GroundTruthCore (mobility/slots/ground_truth_core.hpp); this
/// adds the frame and obstacle inputs scout's Estimator slot takes, which it
/// ignores.

#include <branes/anatomy/mobility/slots/ground_truth_core.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/scout/msg/obstacles.hpp>
#include <branes/scout/slots/slots.hpp>

namespace branes::scout {

template <typename T>
class GroundTruthEstimator : public anatomy::GroundTruthCore<T> {
public:
    void process_frame(const anatomy::CameraFrame&) noexcept {}
    void process_obstacles(const ObstacleObservations<T>&) noexcept {}
};

static_assert(Estimator<GroundTruthEstimator<double>, double>);

}  // namespace branes::scout

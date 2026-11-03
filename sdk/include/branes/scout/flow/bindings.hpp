#pragma once
/// @file bindings.hpp
/// @brief The bindings of scout's two task composites, the estimator and the
/// planner: the CPU calling convention of their operators (anatomy
/// docs/adr/0004, step 2; anatomy docs/plan/flow-graph, 2c).
///
/// One binding per task slot concept of slots/slots.hpp, as anatomy's are for
/// the mission-neutral ones (mobility/flow/bindings.hpp). Each handler calls
/// the method the concept names, so every implementation of the slot is an
/// operator of the composite, unchanged. Every handler of a composite must
/// fire (G6), so a method no graph reads has none: the estimator's health().
///
/// The estimator's truth handler hands the true state to an estimator that
/// takes it (TruthConsumer), as Stack::on_truth() does, and is a no-op for one
/// that doesn't. So the graph reads the truth, and is SITL-only, whichever
/// estimator fills it.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/mobility/flow/domains.hpp>
#include <branes/anatomy/mobility/msg/control.hpp>
#include <branes/anatomy/mobility/msg/estimate.hpp>
#include <branes/anatomy/mobility/msg/imu.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/mobility/slots/context.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/msg/time.hpp>
#include <branes/scout/flow/domains.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <type_traits>

namespace branes::scout::graph {

namespace fl = anatomy::flow;

/// Estimator: imu(imu), frame(camera), obstacles(obstacles) and truth(truth)
/// update its state; estimate at a time reads it.
template <class T>
struct EstimatorBinding {
    using signature = fl::signature<fl::ports<fl::port<"imu", anatomy::ImuSample<T>, fl::direction::in>,
                                              fl::port<"camera", anatomy::CameraFrame, fl::direction::in>,
                                              fl::port<"obstacles", ObstacleObservations<T>, fl::direction::in>,
                                              fl::port<"truth", anatomy::TruthState<T>, fl::direction::in>,
                                              fl::port<"estimate", anatomy::StateEstimate<T>, fl::direction::out>>>;

    struct imu : fl::handler<"imu", fl::ins<"imu">, fl::outs<>, fl::state_role::update> {
        static void call(auto& est, const anatomy::ImuSample<T>& s) {
            est.process_imu(s);
        }
    };
    struct frame : fl::handler<"frame", fl::ins<"camera">, fl::outs<>, fl::state_role::update> {
        static void call(auto& est, const anatomy::CameraFrame& f) {
            est.process_frame(f);
        }
    };
    struct obstacles : fl::handler<"obstacles", fl::ins<"obstacles">, fl::outs<>, fl::state_role::update> {
        static void call(auto& est, const ObstacleObservations<T>& o) {
            est.process_obstacles(o);
        }
    };
    struct truth : fl::handler<"truth", fl::ins<"truth">, fl::outs<>, fl::state_role::update> {
        template <class E>
        static void call(E& est, const anatomy::TruthState<T>& x) {
            if constexpr (anatomy::TruthConsumer<E, T>) {
                est.process_truth(x);
            } else {
                (void)est;
                (void)x;
            }
        }
    };
    struct estimate
        : fl::handler<"estimate", fl::ins<>, fl::outs<"estimate">, fl::state_role::read, fl::takes_time::yes> {
        static anatomy::StateEstimate<T> call(const auto& est, anatomy::Timestamp t) {
            return est.state_at(t);
        }
    };

    using handlers = fl::handlers<imu, frame, obstacles, truth, estimate>;
};

/// Planner: plan(field, obstacles, estimate) → reference. A planner may lay
/// out its path on the first plan() (GoalPlanner), so plan updates its state.
template <class T>
struct PlannerBinding {
    using signature = fl::signature<fl::ports<fl::port<"field", Field<T>, fl::direction::in>,
                                              fl::port<"obstacles", ObstacleObservations<T>, fl::direction::in>,
                                              fl::port<"estimate", anatomy::StateEstimate<T>, fl::direction::in>,
                                              fl::port<"reference", anatomy::Reference<T>, fl::direction::out>>>;

    struct plan : fl::handler<"plan",
                              fl::ins<"field", "obstacles", "estimate">,
                              fl::outs<"reference">,
                              fl::state_role::update> {
        static anatomy::Reference<T> call(auto& p,
                                          const Field<T>& field,
                                          const ObstacleObservations<T>& obstacles,
                                          const anatomy::StateEstimate<T>& x) {
            return p.plan(field, obstacles, x);
        }
    };

    using handlers = fl::handlers<plan>;
};

}  // namespace branes::scout::graph

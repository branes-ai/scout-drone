#pragma once
/// @file graph.hpp
/// @brief Scout's stack as a graph: its ports, its composites and its regions
/// (anatomy docs/adr/0004, step 2; anatomy docs/plan/flow-graph, 2c), and
/// graph::Stack, the CPU stack that runs it.
///
/// The graph is the slot stack (compose/stack.hpp) written down: one region per
/// input, its firings in Stack's order, so a baseline's graph build and its
/// slot build write the same records.
///  - imu: the estimator takes the sample;
///  - truth: the estimator takes the true state, if it reads it (SITL only);
///  - frame: the estimate at the frame's time is read before the estimator
///    takes the frame, and the DAA tracker is updated with that prior and the
///    detections, of which there are none yet;
///  - obstacles: the estimator takes the obstacles seen, and the planner reads
///    the latest at the tick (hold);
///  - tick: estimate, plan, conflicts, step, filter and send, at the tick's
///    time, with the field's geofence for the safety filter.
///
/// A baseline's file names the body of each composite (`graph:`), and
/// cmake/baselines.cmake configures the header that names their types.

#include <branes/anatomy/flow/builder.hpp>
#include <branes/anatomy/flow/domains.hpp>
#include <branes/anatomy/flow/graph.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/flow/bindings.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/scout/compose/profile.hpp>
#include <branes/scout/flow/bindings.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacles.hpp>
#include <branes/scout/slots/context.hpp>

#include <cassert>
#include <optional>
#include <string>
#include <utility>

namespace branes::scout::graph {

/// The graph's external ports.
template <class T>
struct Ports {
    fl::input_port<anatomy::ImuSample<T>> imu;
    fl::input_port<anatomy::CameraFrame> camera;
    fl::input_port<ObstacleObservations<T>> obstacles;
    fl::input_port<anatomy::TruthState<T>> truth;
    fl::input_port<anatomy::Timestamp> tick;
    fl::constant_port<Field<T>> field;
    fl::constant_port<anatomy::IntruderDetections<T>> detections;  ///< no producer yet: none
    fl::output_port<anatomy::StateEstimate<T>> estimate;
    fl::output_port<anatomy::Reference<T>> reference;
    fl::output_port<anatomy::ThrustRates<T>> command;
};

template <class T>
[[nodiscard]] Ports<T> ports(fl::builder& g, const Field<T>& field) {
    Ports<T> p;
    p.imu = fl::input<anatomy::ImuSample<T>>(g, "imu");
    p.camera = fl::input<anatomy::CameraFrame>(
        g,
        "camera",
        {.trigger = "frame", .time = [](const anatomy::CameraFrame& f) { return f.header.t_mid; }, .sitl_only = false});
    p.obstacles = fl::input<ObstacleObservations<T>>(g, "obstacles");
    p.truth = fl::input<anatomy::TruthState<T>>(g, "truth", {.trigger = "truth", .time = nullptr, .sitl_only = true});
    p.tick = g.tick("tick");
    p.field = fl::constant<Field<T>>(g, "field", field);
    p.detections = fl::constant<anatomy::IntruderDetections<T>>(g, "detections");
    p.estimate = fl::output<anatomy::StateEstimate<T>>(g, "estimate", p.tick);
    p.reference = fl::output<anatomy::Reference<T>>(g, "reference", p.tick);
    p.command = fl::output<anatomy::ThrustRates<T>>(g, "command", p.tick);
    return p;
}

/// The scoreboard graph: scout's six composites, filled by Bodies.
template <class T, class Bodies, class G>
Ports<T> scoreboard(G& g, const Field<T>& field) {
    const Ports<T> p = ports<T>(g, field);
    const auto est = fl::composite<EstimatorBinding<T>, typename Bodies::estimator>(g, "estimator");
    const auto planner = fl::composite<PlannerBinding<T>, typename Bodies::planner>(g, "planner");
    const auto ctl = fl::composite<fl::ControllerBinding<T>, typename Bodies::controller>(g, "controller");
    const auto safe = fl::composite<fl::SafetyFilterBinding<T>, typename Bodies::safety_filter>(g, "safety_filter");
    const auto daa = fl::composite<fl::DaaTrackerBinding<T>, typename Bodies::daa_tracker>(g, "daa_tracker");
    const auto fc = fl::composite<fl::FcBridgeBinding<T>, typename Bodies::fc_bridge>(g, "fc_bridge");

    fl::scope truth = g.region(p.truth);
    fl::fire<"truth">(truth, est, p.truth);

    fl::scope imu = g.region(p.imu);
    fl::fire<"imu">(imu, est, p.imu);

    fl::scope frame = g.region(p.camera);
    auto [prior] = fl::fire<"estimate">(frame, est, fl::at(p.camera));
    fl::fire<"frame">(frame, est, p.camera);
    fl::fire<"update">(frame, daa, p.detections, prior);

    fl::scope obstacles = g.region(p.obstacles);
    fl::fire<"obstacles">(obstacles, est, p.obstacles);

    fl::scope tick = g.region(p.tick);
    auto [x] = fl::fire<"estimate">(tick, est, fl::at(p.tick));
    auto [reference] = fl::fire<"plan">(tick, planner, p.field, fl::hold(p.obstacles), x);
    auto [conflicts] = fl::fire<"conflicts">(tick, daa, x);
    auto [command] = fl::fire<"step">(tick, ctl, x, reference, conflicts);
    auto [safe_command] =
        fl::fire<"filter">(tick, safe, x, command, conflicts, fl::project<"geofence", &Field<T>::geofence>(p.field));
    fl::fire<"send">(tick, fc, fl::at(p.tick), safe_command);
    g.produce(p.estimate, x);
    g.produce(p.reference, reference);
    g.produce(p.command, safe_command);
    return p;
}

/// A baseline's stack: the CPU stack of its graph, which the harnesses drive
/// as they drive the slot stack (RunnableStack, with ScoutProfile). Each hook
/// hands the stack its input, which runs that input's region; the tick
/// returns the tick region's outputs. `Baseline` names what fills the graph
/// (cmake/scout_graph.hpp.in).
///
/// Model
/// -----
///   Stack<T, Baseline>
///     stack_     : anatomy::flow::stack [owned] — the operators, the values, the firings
///     ports_     : Ports<T> — the handles of the graph's external ports
///     last_tick_ : std::optional<Timestamp> — time of the previous on_tick
///
/// Invariants
/// ----------
///   I1: stack_ runs the checked graph of Baseline's file   — constrains stack_
///   I2: ticks arrive in non-decreasing time order          — constrains last_tick_
template <class T, class Baseline>
class Stack {
public:
    using scalar_type = T;
    using profile = ScoutProfile;  ///< the slot stack's (compose/profile.hpp)

    static_assert(Baseline::kind == fl::baseline_class::scoreboard, "scout's graph baselines are scoreboard baselines");

    /// Build the graph and its operators from `ctx` (make_slot), and check the
    /// baseline file against the graph (G8); the first failure is returned.
    /// Pre: valid(field). Post: I1, I2 (no tick yet).
    [[nodiscard]] static anatomy::result<Stack, std::string> create(const Field<T>& field, const SlotContext& ctx) {
        assert(valid(field));  // Pre
        fl::stack_builder<SlotContext> g{"scout", Baseline::kind, ctx};
        const Ports<T> p = scoreboard<T, typename Baseline::bodies>(g, field);
        auto built = std::move(g).build_stack();
        if (!built) {
            return built.error().to_string();
        }
        if (const auto c = fl::check_baseline(ctx.baseline, Baseline::name, built->graph()); !c) {
            return c.error().to_string();
        }
        return Stack(std::move(built).value(), p);
    }

    /// Pre: valid(x).
    void on_truth(const anatomy::TruthState<T>& x) {
        assert(valid(x));  // Pre
        stack_.input(ports_.truth, x);
    }

    /// Pre: valid(s).
    void on_imu(const anatomy::ImuSample<T>& s) {
        assert(valid(s));  // Pre
        stack_.input(ports_.imu, s);
    }

    /// Pre: valid(f).
    void on_frame(const anatomy::CameraFrame& f) {
        assert(valid(f));  // Pre
        stack_.input(ports_.camera, f);
    }

    /// Pre: valid(o).
    void on_obstacles(const ObstacleObservations<T>& o) {
        assert(valid(o));  // Pre
        stack_.input(ports_.obstacles, o);
    }

    /// Run the tick region at `now`. Pre: now ≥ the previous tick (I2).
    /// Post: I2 with last_tick_ == now.
    anatomy::TickOutput<T> on_tick(anatomy::Timestamp now) {
        assert(!last_tick_ || *last_tick_ <= now);  // Pre / I2
        stack_.input(ports_.tick, now);
        last_tick_ = now;
        return anatomy::TickOutput<T>{
            stack_.output(ports_.estimate), stack_.output(ports_.reference), stack_.output(ports_.command)};
    }

    /// SHA-256 of the checked graph's canonical form, which the log records as
    /// its composition_hash.
    [[nodiscard]] const std::string& hash() const noexcept {
        return stack_.hash();
    }

    [[nodiscard]] const fl::stack& stack() const noexcept {
        return stack_;
    }

private:
    Stack(fl::stack s, Ports<T> p) : stack_(std::move(s)), ports_(std::move(p)) {}

    fl::stack stack_;
    Ports<T> ports_;
    std::optional<anatomy::Timestamp> last_tick_;
};

}  // namespace branes::scout::graph

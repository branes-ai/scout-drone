#pragma once
/// @file stack.hpp
/// @brief Stack<T, ...>: one scout baseline's slot composition, driven event by event.
///
/// It wires scout's six slots. What a harness needs from any stack
/// (RunnableStack) is anatomy's (compose/profile.hpp); the events this stack
/// takes are ScoutProfile's (compose/profile.hpp).
///
/// The harnesses feed the stack the same events — an IMU sample, a camera
/// frame, the obstacles seen, a control tick — so a log of the events replays
/// to the same outputs (SYS-08, SW-03). SITL and its replays add the true
/// state (on_truth), which only TruthConsumer slots read. The stack itself
/// reads no clock and draws no random numbers: time arrives with the events.
///
/// create() builds each slot from a SlotContext (slots/context.hpp) where the
/// slot supports it; the constructor takes the slots ready-made.
///
/// Model
/// -----
///   Stack<T, Est, Plan, Ctl, Safe, Daa, Fc>
///     field_     : Field<T> [owned, immutable] — the world: start, geofence, goal, obstacles
///     est_ … fc_ : one instance per slot [owned]
///     seen_      : ObstacleObservations<T> — the latest obstacles seen (none before the first)
///     last_tick_ : std::optional<Timestamp> — time of the previous on_tick
///
/// Invariants
/// ----------
///   I1: valid(field_)                                 — constrains field_
///   I2: valid(seen_)                                  — constrains seen_
///   I3: ticks arrive in non-decreasing time order     — constrains last_tick_
///
/// The planner gets the latest obstacles seen and the field, whose obstacles
/// a planner that avoids only what it sees must leave unread. The safety
/// filter gets the field's geofence. Intruder detections have no producing
/// slot yet; on_frame() hands the DAA tracker an empty set.

#include <branes/anatomy/compose/profile.hpp>
#include <branes/anatomy/mobility/compose/tick.hpp>
#include <branes/anatomy/mobility/msg/intruders.hpp>
#include <branes/anatomy/mobility/msg/truth.hpp>
#include <branes/anatomy/msg/camera.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/anatomy/slots/slot_context.hpp>
#include <branes/scout/compose/profile.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacles.hpp>
#include <branes/scout/slots/context.hpp>
#include <branes/scout/slots/slots.hpp>

#include <cassert>
#include <optional>
#include <string>
#include <utility>

namespace branes::scout {

template <typename T, class Est, class Plan, class Ctl, class Safe, class Daa, class Fc>
    requires Estimator<Est, T> && Planner<Plan, T> && Controller<Ctl, T> && SafetyFilter<Safe, T> &&
             DaaTracker<Daa, T> && FcBridge<Fc, T>
class Stack {
public:
    using scalar_type = T;
    using profile = ScoutProfile;  ///< a roaming vehicle's stack, with the obstacles seen (compose/profile.hpp)

    /// Pre: valid(field). Post: I1–I3 (no obstacles seen, no tick yet).
    explicit Stack(
        const Field<T>& field, Est est = {}, Plan plan = {}, Ctl ctl = {}, Safe safe = {}, Daa daa = {}, Fc fc = {})
        : field_(field), est_(std::move(est)), plan_(std::move(plan)), ctl_(std::move(ctl)), safe_(std::move(safe)),
          daa_(std::move(daa)), fc_(std::move(fc)) {
        assert(valid(field_));  // Pre / I1
        assert(check_invariant());
    }

    /// Build every slot from `ctx` (make_slot: from_context where the slot has
    /// it, else default), in slot order; the first failure is returned.
    /// Pre: valid(field). Post: I1–I3 (no obstacles seen, no tick yet).
    [[nodiscard]] static anatomy::result<Stack, std::string> create(const Field<T>& field, const SlotContext& ctx) {
        auto est = anatomy::make_slot<Est>(ctx);
        auto plan = anatomy::make_slot<Plan>(ctx);
        auto ctl = anatomy::make_slot<Ctl>(ctx);
        auto safe = anatomy::make_slot<Safe>(ctx);
        auto daa = anatomy::make_slot<Daa>(ctx);
        auto fc = anatomy::make_slot<Fc>(ctx);
        for (const std::string* e :
             {error_of(est), error_of(plan), error_of(ctl), error_of(safe), error_of(daa), error_of(fc)}) {
            if (e != nullptr) {
                return *e;
            }
        }
        return Stack(field,
                     std::move(est).value(),
                     std::move(plan).value(),
                     std::move(ctl).value(),
                     std::move(safe).value(),
                     std::move(daa).value(),
                     std::move(fc).value());
    }

    /// Hand the true state to every TruthConsumer slot (SITL only). Pre: valid(x).
    void on_truth(const anatomy::TruthState<T>& x) {
        assert(valid(x));  // Pre
        give_truth(est_, x);
        give_truth(plan_, x);
        give_truth(ctl_, x);
        give_truth(safe_, x);
        give_truth(daa_, x);
        give_truth(fc_, x);
    }

    /// Feed one IMU sample to the estimator. Pre: valid(s).
    void on_imu(const anatomy::ImuSample<T>& s) {
        assert(valid(s));  // Pre
        est_.process_imu(s);
    }

    /// Feed a camera frame to the estimator. Pre: valid(f).
    void on_frame(const anatomy::CameraFrame& f) {
        assert(valid(f));  // Pre
        const anatomy::StateEstimate<T> prior = est_.state_at(f.header.t_mid);
        est_.process_frame(f);
        daa_.update(anatomy::IntruderDetections<T>{}, prior);
    }

    /// Keep the obstacles seen for the planner, and feed them to the estimator.
    /// Pre: valid(o). Post: I2, with seen_ == o.
    void on_obstacles(const ObstacleObservations<T>& o) {
        assert(valid(o));  // Pre / I2
        seen_ = o;
        est_.process_obstacles(seen_);
    }

    /// Run one control cycle at `now` and send the command to the FC.
    /// Pre: now ≥ the previous tick (I3). Post: I3 with last_tick_ == now.
    anatomy::TickOutput<T> on_tick(anatomy::Timestamp now) {
        assert(!last_tick_ || *last_tick_ <= now);  // Pre / I3
        const anatomy::StateEstimate<T> x = est_.state_at(now);
        const anatomy::Reference<T> ref = plan_.plan(field_, seen_, x);
        const anatomy::ConflictSet<T> conflicts = daa_.conflicts(x);
        const anatomy::ThrustRates<T> u = safe_.filter(x, ctl_.step(x, ref, conflicts), conflicts, field_.geofence);
        fc_.send(u, now);
        last_tick_ = now;
        assert(check_invariant());
        return anatomy::TickOutput<T>{x, ref, u};
    }

    [[nodiscard]] const Field<T>& field() const noexcept {
        return field_;
    }
    [[nodiscard]] const ObstacleObservations<T>& seen() const noexcept {
        return seen_;
    }
    [[nodiscard]] Fc& fc_bridge() noexcept {
        return fc_;
    }
    [[nodiscard]] const Est& estimator() const noexcept {
        return est_;
    }
    [[nodiscard]] const Plan& planner() const noexcept {
        return plan_;
    }
    [[nodiscard]] const Ctl& controller() const noexcept {
        return ctl_;
    }

private:
    template <class S>
    [[nodiscard]] static const std::string* error_of(const anatomy::result<S, std::string>& r) noexcept {
        return r ? nullptr : &r.error();
    }

    template <class S>
    static void give_truth(S& slot, const anatomy::TruthState<T>& x) {
        if constexpr (anatomy::TruthConsumer<S, T>) {
            slot.process_truth(x);
        } else {
            (void)slot;
            (void)x;
        }
    }

    [[nodiscard]] bool check_invariant() const noexcept {
        return valid(field_) && valid(seen_);  // I1, I2 (I3 is checked at the tick entry)
    }

    Field<T> field_;
    Est est_;
    Plan plan_;
    Ctl ctl_;
    Safe safe_;
    Daa daa_;
    Fc fc_;
    ObstacleObservations<T> seen_{};
    std::optional<anatomy::Timestamp> last_tick_;
};

}  // namespace branes::scout

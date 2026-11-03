#pragma once
/// @file domains.hpp
/// @brief The domain of each of scout's messages: the field and the obstacles
/// seen (anatomy docs/adr/0004; anatomy docs/plan/flow-graph, 2c).
///
/// One domain_of per message struct in msg/, field by field, in declaration
/// order, with the struct's member names, as anatomy's are
/// (mobility/flow/domains.hpp). A count and the fixed array it counts are one
/// bounded vector of the element's record, named after the array: the field's
/// `obstacles` covers `obstacle_count`, and the observations' `obstacles`
/// covers `count`.

#include <branes/anatomy/flow/domain.hpp>
#include <branes/anatomy/mobility/flow/domains.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/msg/obstacles.hpp>

#include <cstdint>

namespace branes::anatomy::flow {

// ── field.hpp ───────────────────────────────────────────────────────

template <class T>
struct domain_of<scout::Obstacle<T>> {
    using type = record<field<"base", vec3<T>>, field<"radius", scalar<T>>, field<"height", scalar<T>>>;
};

template <class T>
struct domain_of<scout::Goal<T>> {
    using type = record<field<"position", vec3<T>>, field<"radius", scalar<T>>>;
};

template <class T>
struct domain_of<scout::Field<T>> {
    using type = record<field<"geofence", domain_of_t<Geofence<T>>>,
                        field<"start", domain_of_t<StartPose<T>>>,
                        field<"goal", domain_of_t<scout::Goal<T>>>,
                        field<"obstacles", vector<domain_of_t<scout::Obstacle<T>>, bounded<scout::kMaxObstacles>>>>;
};

// ── obstacles.hpp ───────────────────────────────────────────────────

template <class T>
struct domain_of<scout::ObstacleObservation<T>> {
    using type = record<field<"id", scalar<std::int32_t>>,
                        field<"position", vec3<T>>,
                        field<"radius", scalar<T>>,
                        field<"height", scalar<T>>>;
};

template <class T>
struct domain_of<scout::ObstacleObservations<T>> {
    using type = record<
        field<"t", timestamp>,
        field<"obstacles", vector<domain_of_t<scout::ObstacleObservation<T>>, bounded<scout::kMaxVisibleObstacles>>>>;
};

}  // namespace branes::anatomy::flow

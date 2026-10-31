#pragma once
/// @file context.hpp
/// @brief SlotContext: scout's slot context, over the field.
///
/// The machinery (BasicSlotContext, make_slot, ContextBuilt, TruthConsumer,
/// parameter_in) is anatomy's (slots/slot_context.hpp, mobility/slots/context.hpp);
/// scout's world is the field.

#include <branes/anatomy/mobility/slots/context.hpp>
#include <branes/scout/msg/field.hpp>

namespace branes::scout {

static_assert(anatomy::MissionWorld<Field<double>>, "the field is scout's world: it carries the start pose");

/// Scout's slot context: the vehicle, the baseline, and the field (`world`).
using SlotContext = anatomy::BasicSlotContext<Field<double>>;

}  // namespace branes::scout

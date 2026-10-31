#pragma once
/// @file harness.hpp
/// @brief Scout's harness pieces, over anatomy's (harness/).
///
/// The mission-neutral pieces are anatomy's (harness/args.hpp, files.hpp,
/// runner.hpp); this header adds:
///  - check_baseline() over scout's slot list (config::kSlotNames);
///  - make_stack(): a scout baseline's Stack, its slots built from the configs.
///
/// The log is double. A stack in another scalar type T gets its field and
/// inputs cast to T, and its outputs cast back; for T = double every cast is
/// a copy.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/harness/files.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/slots/context.hpp>

#include <filesystem>
#include <string>
#include <string_view>

namespace branes::scout::harness {

/// check_baseline() over scout's six slots.
[[nodiscard]] inline anatomy::result<config::BaselineConfig, std::string>
check_baseline(const std::filesystem::path& dir, std::string_view name, std::string_view compiled_hash) {
    return anatomy::harness::check_baseline(dir, name, compiled_hash, config::kSlotNames);
}

// ── stack construction ──────────────────────────────────────────────

/// Stack::create() for a harness: the field, vehicle, and baseline make the
/// slot context. The field is cast to the stack's scalar type, and a field
/// that isn't valid() there is an error. Pre: valid(field).
template <class StackT>
[[nodiscard]] anatomy::result<StackT, std::string> make_stack(const Field<double>& field,
                                                              const anatomy::config::VehicleConfig& vehicle,
                                                              const config::BaselineConfig& baseline) {
    using T = typename StackT::scalar_type;
    const Field<T> f = cast<T>(field);
    if (!valid(f)) {
        return std::string("the field is not valid in the stack's scalar type");
    }
    return StackT::create(f, SlotContext{vehicle, baseline, field});
}

}  // namespace branes::scout::harness

#pragma once
/// @file harness.hpp
/// @brief Scout's harness pieces, over anatomy's (harness/).
///
/// The mission-neutral pieces are anatomy's (harness/args.hpp, files.hpp,
/// runner.hpp); this header adds:
///  - check_baseline() over scout's slot list (config::kSlotNames);
///  - make_stack(): a scout baseline's Stack, its slots built from the configs;
///  - flown_field_yaml() and field_from_log(): the field a log records;
///  - override_parameters(): numeric baseline parameters set for one batch.
///
/// A log records the field a run flew, its obstacles listed, rather than the
/// field file: a generated field depends on the run's seed, which the log
/// header doesn't carry. So a log replays and scores on its own.
///
/// The log is double. A stack in another scalar type T gets its field and
/// inputs cast to T, and its outputs cast back; for T = double every cast is
/// a copy.

#include <branes/anatomy/config/baseline.hpp>
#include <branes/anatomy/harness/args.hpp>
#include <branes/anatomy/harness/files.hpp>
#include <branes/anatomy/log/log.hpp>
#include <branes/anatomy/mobility/config/vehicle.hpp>
#include <branes/anatomy/result.hpp>
#include <branes/scout/config/config.hpp>
#include <branes/scout/msg/field.hpp>
#include <branes/scout/slots/context.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

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

// ── the field a log records ─────────────────────────────────────────

/// The field file of `flown`, the field a run flew (grown from `c` with the
/// run's seed): `c`'s name, with every obstacle listed and none generated.
/// Pre: valid(flown).
[[nodiscard]] inline std::string flown_field_yaml(const config::FieldConfig& c, const Field<double>& flown) {
    return config::emit_field(config::FieldConfig{c.name, flown, std::nullopt});
}

/// The field a log's header records (flown_field_yaml()). `origin` names it in
/// errors. A field file that generates its obstacles needs the run's seed, so
/// it is refused here.
[[nodiscard]] inline anatomy::result<Field<double>, std::string> field_from_yaml(std::string_view yaml,
                                                                                 std::string_view origin) {
    auto c = config::parse_field(yaml, origin);
    if (!c) {
        return c.error().to_string();
    }
    if (c->generate) {
        return std::string(origin) + ": the field generates its obstacles, so it needs the run's seed; "
                                     "a log records the field flown, its obstacles listed";
    }
    return c->field;
}

[[nodiscard]] inline anatomy::result<Field<double>, std::string> field_from_log(const anatomy::log::LogHeader& h,
                                                                                std::string_view log_path) {
    return field_from_yaml(h.world_yaml, "field (from " + std::string(log_path) + ")");
}

// ── parameter overrides ─────────────────────────────────────────────

/// `b` with the parameters `spec` sets: "key=value[,key=value…]", each key a
/// parameter `b` already has, each value a finite number. A key that `b` lacks
/// is an error, so a misspelt key can't be silently ignored, and so is nan or
/// inf, which numeric_parameter() would drop in favour of a default.
[[nodiscard]] inline anatomy::result<config::BaselineConfig, std::string> override_parameters(config::BaselineConfig b,
                                                                                              std::string_view spec) {
    std::size_t at = 0;
    while (at <= spec.size()) {
        const std::size_t end = std::min(spec.find(',', at), spec.size());
        const std::string_view item = spec.substr(at, end - at);
        const std::size_t eq = item.find('=');
        if (eq == std::string_view::npos || eq == 0 || eq + 1 == item.size()) {
            return std::string("--set takes key=value[,key=value...], not '" + std::string(item) + "'");
        }
        const std::string key(item.substr(0, eq));
        const std::string value(item.substr(eq + 1));
        if (b.parameters.count(key) == 0) {
            return std::string("--set " + key + ": the baseline has no parameter " + key);
        }
        const auto number = anatomy::harness::parse_number<double>(value);
        if (!number || !std::isfinite(*number)) {
            return std::string("--set " + key + ": '" + value + "' is not a finite number");
        }
        b.parameters[key] = value;
        at = end + 1;
    }
    return b;
}

}  // namespace branes::scout::harness

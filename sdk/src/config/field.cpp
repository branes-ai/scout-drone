#include <branes/anatomy/config/yaml.hpp>
#include <branes/anatomy/util/splitmix64.hpp>
#include <branes/scout/config/field.hpp>

#include <array>
#include <cmath>
#include <string>

namespace branes::scout::config {

using namespace anatomy::config::yaml;

namespace {

/// A `[min, max]` range of positive numbers.
std::array<double, 2> get_range(const YAML::Node& n, std::string_view prefix, const std::string& key) {
    const auto r = get_array<2>(n, prefix, key);
    require(r[0] > 0 && r[0] <= r[1], join(prefix, key), "expected [min, max] with 0 < min ≤ max");
    return r;
}

/// Horizontal distance from `p` to `q`, squared.
double ground_distance_squared(const anatomy::Vec3<double>& p, const anatomy::Vec3<double>& q) {
    const double dx = p[0] - q[0];
    const double dy = p[1] - q[1];
    return dx * dx + dy * dy;
}

/// True if an obstacle of `radius` at `base` keeps clear_radius_m from `p`, on the ground.
bool clear_of(const anatomy::Vec3<double>& base, double radius, double clear, const anatomy::Vec3<double>& p) {
    const double d = clear + radius;
    return ground_distance_squared(base, p) >= d * d;
}

}  // namespace

// ── Field ───────────────────────────────────────────────────────────

anatomy::result<FieldConfig, ConfigError> parse_field(std::string_view yaml, std::string_view origin) {
    return guarded<FieldConfig>(origin, [&]() -> FieldConfig {
        const YAML::Node root = parse_root(yaml);
        only_keys(root, "", {"name", "geofence", "start", "goal", "obstacles", "generate"});
        FieldConfig c;
        Field<double>& f = c.field;
        c.name = get_string(root, "", "name");

        const YAML::Node fence = child(root, "", "geofence");
        only_keys(fence, "geofence", {"min_m", "max_m"});
        f.geofence.min = get_array<3>(fence, "geofence", "min_m");
        f.geofence.max = get_array<3>(fence, "geofence", "max_m");
        for (std::size_t a = 0; a < 3; ++a) {
            require(f.geofence.min[a] < f.geofence.max[a], "geofence", "min_m must be < max_m on every axis");
        }

        const YAML::Node start = child(root, "", "start");
        only_keys(start, "start", {"position_m", "yaw_rad"});
        f.start.position = get_array<3>(start, "start", "position_m");
        f.start.yaw = get_double(start, "start", "yaw_rad");
        require(contains(f.geofence, f.start.position), "start.position_m", "must be inside the geofence");

        const YAML::Node goal = child(root, "", "goal");
        only_keys(goal, "goal", {"position_m", "radius_m"});
        f.goal.position = get_array<3>(goal, "goal", "position_m");
        f.goal.radius = get_double(goal, "goal", "radius_m");
        require(contains(f.geofence, f.goal.position), "goal.position_m", "must be inside the geofence");
        require(f.goal.radius > 0, "goal.radius_m", "must be > 0");

        if (const YAML::Node obstacles = root["obstacles"]) {
            require(obstacles.IsSequence() && obstacles.size() <= kMaxObstacles,
                    "obstacles",
                    "expected a list of at most " + std::to_string(kMaxObstacles) + " obstacles");
            f.obstacle_count = static_cast<std::uint32_t>(obstacles.size());
            for (std::uint32_t i = 0; i < f.obstacle_count; ++i) {
                const std::string p = "obstacles[" + std::to_string(i) + "]";
                const YAML::Node o = obstacles[i];
                only_keys(o, p, {"base_m", "radius_m", "height_m"});
                Obstacle<double>& ob = f.obstacles[i];
                ob.base = get_array<3>(o, p, "base_m");
                ob.radius = get_double(o, p, "radius_m");
                ob.height = get_double(o, p, "height_m");
                require(contains(f.geofence, ob.base), p + ".base_m", "must be inside the geofence");
                require(ob.radius > 0 && ob.height > 0, p, "radius_m and height_m must be > 0");
                require(!inside(ob, f.start.position), p, "contains the start position");
                require(!inside(ob, f.goal.position), p, "contains the goal position");
            }
        }

        if (const YAML::Node gen = root["generate"]) {
            only_keys(gen, "generate", {"density_per_m2", "radius_m", "height_m", "clear_radius_m"});
            FieldGeneration g;
            g.density_per_m2 = get_double(gen, "generate", "density_per_m2");
            require(g.density_per_m2 >= 0, "generate.density_per_m2", "must be ≥ 0");
            const auto radius = get_range(gen, "generate", "radius_m");
            g.radius_min_m = radius[0];
            g.radius_max_m = radius[1];
            const auto height = get_range(gen, "generate", "height_m");
            g.height_min_m = height[0];
            g.height_max_m = height[1];
            g.clear_radius_m = get_double(gen, "generate", "clear_radius_m");
            require(g.clear_radius_m >= 0, "generate.clear_radius_m", "must be ≥ 0");
            for (std::size_t a = 0; a < 2; ++a) {
                require(f.geofence.max[a] - f.geofence.min[a] > 2 * g.radius_max_m,
                        "generate.radius_m",
                        "the geofence must be wider than twice the largest radius on x and y");
            }
            require(f.obstacle_count + generated_count(g, f.geofence) <= kMaxObstacles,
                    "generate.density_per_m2",
                    "the listed and generated obstacles must number at most " + std::to_string(kMaxObstacles));
            c.generate = g;
        }
        return c;
    });
}

anatomy::result<FieldConfig, ConfigError> load_field(const std::filesystem::path& path) {
    return load_file<FieldConfig>(path, parse_field);
}

std::string emit_field(const FieldConfig& c) {
    const Field<double>& f = c.field;
    YAML::Emitter out;
    out.SetDoublePrecision(17);
    out << YAML::BeginMap;
    out << YAML::Key << "name" << YAML::Value << c.name;
    out << YAML::Key << "geofence" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min_m" << YAML::Value;
    emit_seq(out, f.geofence.min);
    out << YAML::Key << "max_m" << YAML::Value;
    emit_seq(out, f.geofence.max);
    out << YAML::EndMap;
    out << YAML::Key << "start" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position_m" << YAML::Value;
    emit_seq(out, f.start.position);
    out << YAML::Key << "yaw_rad" << YAML::Value << f.start.yaw;
    out << YAML::EndMap;
    out << YAML::Key << "goal" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "position_m" << YAML::Value;
    emit_seq(out, f.goal.position);
    out << YAML::Key << "radius_m" << YAML::Value << f.goal.radius;
    out << YAML::EndMap;
    out << YAML::Key << "obstacles" << YAML::Value << YAML::BeginSeq;
    for (std::uint32_t i = 0; i < f.obstacle_count; ++i) {
        const Obstacle<double>& o = f.obstacles[i];
        out << YAML::BeginMap;
        out << YAML::Key << "base_m" << YAML::Value;
        emit_seq(out, o.base);
        out << YAML::Key << "radius_m" << YAML::Value << o.radius;
        out << YAML::Key << "height_m" << YAML::Value << o.height;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;
    if (c.generate) {
        const FieldGeneration& g = *c.generate;
        const std::array<double, 2> radius = {g.radius_min_m, g.radius_max_m};
        const std::array<double, 2> height = {g.height_min_m, g.height_max_m};
        out << YAML::Key << "generate" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "density_per_m2" << YAML::Value << g.density_per_m2;
        out << YAML::Key << "radius_m" << YAML::Value;
        emit_seq(out, radius);
        out << YAML::Key << "height_m" << YAML::Value;
        emit_seq(out, height);
        out << YAML::Key << "clear_radius_m" << YAML::Value << g.clear_radius_m;
        out << YAML::EndMap;
    }
    out << YAML::EndMap;
    return finish(out);
}

// ── The field of a seed ─────────────────────────────────────────────

std::uint32_t generated_count(const FieldGeneration& g, const anatomy::Geofence<double>& f) {
    const double area = (f.max[0] - f.min[0]) * (f.max[1] - f.min[1]);
    return static_cast<std::uint32_t>(std::llround(g.density_per_m2 * area));
}

anatomy::result<Field<double>, std::string> field_for_seed(const FieldConfig& c, std::uint64_t seed) {
    Field<double> f = c.field;
    if (!c.generate) {
        return f;
    }
    const FieldGeneration& g = *c.generate;
    const std::uint32_t n = generated_count(g, f.geofence);
    if (f.obstacle_count + n > kMaxObstacles) {
        return std::string("the field has more than " + std::to_string(kMaxObstacles) + " obstacles");
    }
    anatomy::SplitMix64 rng(anatomy::stream_seed(seed, kFieldStream));
    const auto in = [&rng](double lo, double hi) { return lo + (hi - lo) * rng.uniform01(); };
    for (std::uint32_t k = 0; k < n; ++k) {
        Obstacle<double> o;
        o.radius = in(g.radius_min_m, g.radius_max_m);
        o.height = in(g.height_min_m, g.height_max_m);
        o.base[2] = f.geofence.min[2];
        bool placed = false;
        for (int draw = 0; draw < kMaxPlacementDraws && !placed; ++draw) {
            o.base[0] = in(f.geofence.min[0] + o.radius, f.geofence.max[0] - o.radius);
            o.base[1] = in(f.geofence.min[1] + o.radius, f.geofence.max[1] - o.radius);
            placed = clear_of(o.base, o.radius, g.clear_radius_m, f.start.position) &&
                     clear_of(o.base, o.radius, g.clear_radius_m, f.goal.position);
        }
        if (!placed) {
            return std::string("generated obstacle " + std::to_string(k) +
                               " could not be placed clear of the start and the goal in " +
                               std::to_string(kMaxPlacementDraws) + " draws");
        }
        f.obstacles[f.obstacle_count++] = o;
    }
    return f;
}

}  // namespace branes::scout::config

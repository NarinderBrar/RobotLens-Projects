#pragma once

                                                                            
                                                                                 
                                                            

#include "robotlens/extension.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace robotlens::procgen {

inline constexpr int kVersion = 2;
inline constexpr int kOutputVersion = 2;
inline constexpr std::size_t kMaxObjects = 4096U;
inline constexpr double kPi = 3.14159265358979323846;

                                                                        

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline Vec2 operator+(Vec2 a, Vec2 b) {
    return {a.x + b.x, a.y + b.y};
}
inline Vec2 operator-(Vec2 a, Vec2 b) {
    return {a.x - b.x, a.y - b.y};
}
inline Vec2 operator*(Vec2 a, double s) {
    return {a.x * s, a.y * s};
}
inline Vec3 operator+(Vec3 a, Vec3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3 operator-(Vec3 a, Vec3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3 operator*(Vec3 a, double s) {
    return {a.x * s, a.y * s, a.z * s};
}

struct Quat {
    double w = 1.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static Quat yaw(double radians) {
        return {std::cos(radians * 0.5), 0.0, 0.0, std::sin(radians * 0.5)};
    }
    Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z, w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x, w * o.z + x * o.y - y * o.x + z * o.w};
    }
    [[nodiscard]] Vec3 rotate(Vec3 v) const {
        const Vec3 u{x, y, z};
        const Vec3 t{2.0 * (u.y * v.z - u.z * v.y), 2.0 * (u.z * v.x - u.x * v.z),
                     2.0 * (u.x * v.y - u.y * v.x)};
        return {v.x + w * t.x + (u.y * t.z - u.z * t.y), v.y + w * t.y + (u.z * t.x - u.x * t.z),
                v.z + w * t.z + (u.x * t.y - u.y * t.x)};
    }
};

struct Pose {
    Vec3 position;
    Quat rotation;

    static Pose at(double px, double py, double pz, double yawRadians = 0.0) {
        return {{px, py, pz}, Quat::yaw(yawRadians)};
    }
    [[nodiscard]] Vec3 apply(Vec3 local) const { return position + rotation.rotate(local); }
    Pose operator*(const Pose& local) const {
        return {apply(local.position), rotation * local.rotation};
    }
};

inline double polygonArea(const std::vector<Vec2>& polygon) {
    double area = 0.0;
    for (std::size_t i = 0U; i < polygon.size(); ++i) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[(i + 1U) % polygon.size()];
        area += a.x * b.y - b.x * a.y;
    }
    return area * 0.5;
}

inline bool polygonContains(const std::vector<Vec2>& polygon, Vec2 point) {
    bool inside = false;
    for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
        const Vec2& a = polygon[i];
        const Vec2& b = polygon[j];
        if ((a.y > point.y) != (b.y > point.y) &&
            point.x < (b.x - a.x) * (point.y - a.y) / (b.y - a.y) + a.x) {
            inside = !inside;
        }
    }
    return inside;
}

inline std::vector<Vec2> rectangle(double width, double depth, double erosion = 0.0) {
    const double hx = width * 0.5 - erosion;
    const double hy = depth * 0.5 - erosion;
    if (hx <= 0.0 || hy <= 0.0) {
        return {};
    }
    return {{-hx, -hy}, {hx, -hy}, {hx, hy}, {-hx, hy}};
}

                                                                          

class Rng {
  public:
    explicit Rng(std::uint64_t seed, std::uint64_t sequence = 54U)
        : increment_((sequence << 1U) | 1U) {
        next();
        state_ += seed;
        next();
    }

    std::uint32_t next() {
        const std::uint64_t old = state_;
        state_ = old * 6364136223846793005ULL + increment_;
        const auto shifted = static_cast<std::uint32_t>(((old >> 18U) ^ old) >> 27U);
        const auto rotation = static_cast<std::uint32_t>(old >> 59U);
        return (shifted >> rotation) | (shifted << ((~rotation + 1U) & 31U));
    }
    double unit() { return static_cast<double>(next()) * (1.0 / 4294967296.0); }
    double uniform(double low, double high) { return low + (high - low) * unit(); }
    int integer(int low, int high) {
        if (high <= low) {
            return low;
        }
        const auto span = static_cast<std::uint64_t>(high - low) + 1U;
        return low + static_cast<int>(static_cast<std::uint64_t>(next()) % span);
    }
    bool chance(double probability) { return unit() < probability; }
    template <class T> const T& pick(const std::vector<T>& items) {
        return items[static_cast<std::size_t>(integer(0, static_cast<int>(items.size()) - 1))];
    }

  private:
    std::uint64_t state_ = 0U;
    std::uint64_t increment_;
};

inline std::uint64_t mixKey(std::uint32_t seed, std::string_view key) {
    std::uint64_t hash = 14695981039346656037ULL ^ seed;
    for (const char character : key) {
        hash ^= static_cast<unsigned char>(character);
        hash *= 1099511628211ULL;
    }
    return hash;
}

                                                                                
                                                              
inline Rng stream(std::uint32_t seed, std::string_view key) {
    return Rng(mixKey(seed, key), mixKey(seed ^ 0x9E3779B9U, key) & 0xFFFFFFFFU);
}

inline std::string key(std::string_view base, int index) {
    return std::string(base) + "/" + std::to_string(index);
}

                                                                             

struct Material {
    Vec3 color{0.62, 0.66, 0.74};
    double roughness = 0.85;
    double metallic = 0.0;
};

inline Material palette(std::string_view partClass) {
    struct Entry {
        std::string_view name;
        Material material;
    };
    static const std::array<Entry, 24> kPalette{{
        {"floor", {{0.46, 0.45, 0.43}, 0.75, 0.0}},
        {"wall", {{0.82, 0.80, 0.75}, 0.9, 0.0}},
        {"doorway", {{0.42, 0.29, 0.18}, 0.6, 0.0}},
        {"upright", {{0.09, 0.22, 0.52}, 0.4, 0.2}},
        {"beam", {{0.92, 0.42, 0.06}, 0.45, 0.2}},
        {"shelf_board", {{0.62, 0.63, 0.64}, 0.35, 0.85}},
        {"pallet", {{0.66, 0.50, 0.30}, 0.85, 0.0}},
        {"box", {{0.63, 0.47, 0.30}, 0.9, 0.0}},
        {"bin", {{0.12, 0.33, 0.65}, 0.5, 0.0}},
        {"table", {{0.52, 0.36, 0.22}, 0.55, 0.0}},
        {"leg", {{0.12, 0.12, 0.13}, 0.4, 0.8}},
        {"obstacle", {{0.70, 0.16, 0.12}, 0.55, 0.0}},
        {"cabinet", {{0.88, 0.86, 0.82}, 0.5, 0.0}},
        {"cabinet_front", {{0.30, 0.40, 0.46}, 0.45, 0.0}},
        {"counter", {{0.16, 0.16, 0.17}, 0.25, 0.05}},
        {"appliance", {{0.78, 0.78, 0.80}, 0.28, 0.9}},
        {"metal", {{0.63, 0.64, 0.66}, 0.3, 1.0}},
        {"handle", {{0.80, 0.80, 0.82}, 0.15, 1.0}},
        {"door", {{0.40, 0.27, 0.17}, 0.55, 0.0}},
        {"glass", {{0.70, 0.82, 0.90}, 0.05, 0.0}},
        {"window_frame", {{0.92, 0.92, 0.92}, 0.4, 0.3}},
        {"conveyor", {{0.08, 0.08, 0.09}, 0.9, 0.0}},
        {"barrier", {{0.95, 0.75, 0.05}, 0.45, 0.1}},
        {"sink", {{0.75, 0.76, 0.78}, 0.2, 1.0}},
    }};
    for (const Entry& entry : kPalette) {
        if (entry.name == partClass) {
            return entry.material;
        }
    }
    return {};
}

inline constexpr int kColorVariants = 4;
inline constexpr double kSizeStep = 0.05;

                                                                              
                                                                
inline Material vary(Material material, Rng& rng, double amount) {
    const int variant =
        std::min(static_cast<int>(rng.uniform(0.0, 1.0) * kColorVariants), kColorVariants - 1);
    const double scale =
        1.0 + amount * (2.0 * variant / static_cast<double>(kColorVariants - 1) - 1.0);
    material.color = {std::clamp(material.color.x * scale, 0.0, 1.0),
                      std::clamp(material.color.y * scale, 0.0, 1.0),
                      std::clamp(material.color.z * scale, 0.0, 1.0)};
    return material;
}

                                                                              
                                       
inline Vec3 snapSize(Vec3 size, Vec3 low, Vec3 high, double step = kSizeStep) {
    const auto snap = [step](double value, double minimum, double maximum) {
        return std::clamp(std::round(value / step) * step, minimum, maximum);
    };
    return {snap(size.x, low.x, high.x), snap(size.y, low.y, high.y), snap(size.z, low.z, high.z)};
}

                                                                                 

struct Mesh {
    std::vector<double> vertices;
    std::vector<int> indices;
};

class Scene;

class Settings {
  public:
    Settings(const Json& configuration, Scene& scene)
        : configuration_(configuration), scene_(scene) {}

    double number(std::string_view name, double fallback, double minimum, double maximum);
    int integer(std::string_view name, int fallback, int minimum, int maximum);
    bool flag(std::string_view name, bool fallback) const {
        return configuration_[name].asBool(fallback);
    }
    std::string text(std::string_view name, std::string_view fallback) const {
        return std::string(configuration_[name].asString(fallback));
    }
    Vec3 color(std::string_view name, Vec3 fallback) const {
        const Json& value = configuration_[name];
        if (!value.isArray() || value.items().size() < 3U) {
            return fallback;
        }
        return {value[0U].asNumber(fallback.x), value[1U].asNumber(fallback.y),
                value[2U].asNumber(fallback.z)};
    }

  private:
    const Json& configuration_;
    Scene& scene_;
};

class Object {
  public:
    Object(Scene* scene, std::size_t index) : scene_(scene), index_(index) {}

    Object& name(std::string_view value) { return set("name", Json(value)); }
    Object& label(std::string_view value) { return set("label", Json(value)); }
    Object& group(std::string_view value) { return set("group", Json(value)); }
    Object& mass(double kilograms) { return set("mass", Json(kilograms)); }
    Object& collision(std::string_view shape, Vec3 size) {
        return set(
            "collision",
            Json::object({{"shape", Json(shape)},
                          {"size", Json::array({Json(size.x), Json(size.y), Json(size.z)})}}));
    }
    Object& noCollision();
    Object& support(std::string_view surfaceName, double z, const std::vector<Vec2>& polygon,
                    double clearHeight);
    Object& region(std::string_view regionName, std::string_view kind, Vec3 center, Vec3 size,
                   double yawRadians = 0.0);
    [[nodiscard]] bool isValid() const { return scene_ != nullptr; }

  private:
    Object& set(std::string field, Json value);
    Json* json();

    Scene* scene_;
    std::size_t index_;
};

class Scene {
  public:
    explicit Scene(const Json& request)
        : request_(request), settings_(request_["configuration"], *this),
          seed_(static_cast<std::uint32_t>(request_["seed"].asNumber(0.0))) {}

    Settings& settings() { return settings_; }
    [[nodiscard]] std::uint32_t seed() const { return seed_; }
    [[nodiscard]] Rng rng(std::string_view keyName) const { return stream(seed_, keyName); }
    [[nodiscard]] std::size_t objectCount() const { return objects_.size(); }

    Object box(std::string_view keyName, const Pose& pose, Vec3 size, const Material& material) {
        Object object = add(keyName, pose, "box", size, material);
        if (object.isValid()) {
            object.collision("box", size);
        }
        return object;
    }
    Object cylinder(std::string_view keyName, const Pose& pose, double diameter, double height,
                    const Material& material) {
        Object object = add(keyName, pose, "cylinder", {diameter, height, diameter}, material);
        if (object.isValid()) {
            object.collision("cylinder", {diameter, height, diameter});
        }
        return object;
    }
    Object mesh(std::string_view keyName, const Pose& pose, const Mesh& data,
                const Material& material) {
        Object object = add(keyName, pose, "mesh", {1.0, 1.0, 1.0}, material);
        if (Json* json = object.isValid() ? &objects_.back() : nullptr; json != nullptr) {
            Json vertices = Json::array();
            for (const double value : data.vertices) {
                vertices.push(Json(value));
            }
            Json indices = Json::array();
            for (const int value : data.indices) {
                indices.push(Json(value));
            }
            json->set("mesh", Json::object({{"vertices", std::move(vertices)},
                                            {"indices", std::move(indices)}}));
        }
        return object;
    }
    Object collider(std::string_view keyName, const Pose& pose, Vec3 size) {
        Object object = add(keyName, pose, "none", size, Material{});
        if (object.isValid()) {
            object.collision("box", size);
        }
        return object;
    }

    void region(std::string_view regionName, std::string_view kind, const Pose& pose, Vec3 size) {
        regions_.push(regionJson(regionName, kind, pose.position, size, pose.rotation));
    }
                                                                                   
                                                                       
    void joint(std::string_view jointName, std::string_view type, std::string_view parent,
               std::string_view child, Vec3 origin, Vec3 axis, double lower, double upper,
               double damping = 0.5, double friction = 0.05) {
        joints_.push(
            Json::object({{"name", Json(jointName)},
                          {"type", Json(type)},
                          {"parent", Json(parent)},
                          {"child", Json(child)},
                          {"origin", Json::array({Json(origin.x), Json(origin.y), Json(origin.z)})},
                          {"axis", Json::array({Json(axis.x), Json(axis.y), Json(axis.z)})},
                          {"lower", Json(lower)},
                          {"upper", Json(upper)},
                          {"damping", Json(damping)},
                          {"friction", Json(friction)}}));
    }
    void diagnostic(std::string_view severity, std::string_view code, std::string message,
                    std::string_view setting = {}, double requested = 0.0, double available = 0.0,
                    bool hasValues = false) {
        if (diagnostics_.items().size() >= 64U) {
            return;
        }
        Json item = Json::object({{"severity", Json(severity)},
                                  {"code", Json(code)},
                                  {"message", Json(std::move(message))}});
        if (!setting.empty()) {
            item.set("setting", Json(setting));
        }
        if (hasValues) {
            item.set("requested", Json(requested)).set("available", Json(available));
        }
        diagnostics_.push(std::move(item));
    }

    [[nodiscard]] Json reply() const {
        Json objects = Json::array();
        for (const Json& object : objects_) {
            objects.push(object);
        }
        return Json::object({{"output_version", Json(kOutputVersion)},
                             {"procgen", Json(kVersion)},
                             {"objects", std::move(objects)},
                             {"scene_regions", regions_},
                             {"joints", joints_},
                             {"diagnostics", diagnostics_}});
    }

    static Json regionJson(std::string_view regionName, std::string_view kind, Vec3 center,
                           Vec3 size, const Quat& rotation) {
        return Json::object(
            {{"name", Json(regionName)},
             {"kind", Json(kind)},
             {"center", Json::array({Json(center.x), Json(center.y), Json(center.z)})},
             {"rotation", Json::array({Json(rotation.w), Json(rotation.x), Json(rotation.y),
                                       Json(rotation.z)})},
             {"size", Json::array({Json(size.x), Json(size.y), Json(size.z)})}});
    }

  private:
    friend class Object;

    Object add(std::string_view keyName, const Pose& pose, std::string_view shape, Vec3 size,
               const Material& material) {
        if (objects_.size() >= kMaxObjects) {
            if (!isFull_) {
                diagnostic("error", "too_many_objects",
                           "The generator reached the 4096 object limit; later objects were "
                           "skipped");
                isFull_ = true;
            }
            return {nullptr, 0U};
        }
        if (!keys_.insert(std::string(keyName)).second) {
            diagnostic("error", "duplicate_key",
                       "Object key '" + std::string(keyName) + "' is used twice; skipped");
            return {nullptr, 0U};
        }
        const Vec3& p = pose.position;
        const Quat& q = pose.rotation;
        const double length = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        const double scale = length > 0.0 ? 1.0 / length : 1.0;
        objects_.push_back(
            Json::object({{"key", Json(keyName)},
                          {"shape", Json(shape)},
                          {"position", Json::array({Json(p.x), Json(p.y), Json(p.z)})},
                          {"rotation", Json::array({Json(q.w * scale), Json(q.x * scale),
                                                    Json(q.y * scale), Json(q.z * scale)})},
                          {"size", Json::array({Json(size.x), Json(size.y), Json(size.z)})},
                          {"color", Json::array({Json(material.color.x), Json(material.color.y),
                                                 Json(material.color.z)})},
                          {"roughness", Json(material.roughness)},
                          {"metallic", Json(material.metallic)}}));
        return {this, objects_.size() - 1U};
    }

    const Json& request_;
    Settings settings_;
    std::uint32_t seed_;
    std::vector<Json> objects_;
    Json regions_ = Json::array();
    Json joints_ = Json::array();
    Json diagnostics_ = Json::array();
    std::set<std::string> keys_;
    bool isFull_ = false;
};

inline Json* Object::json() {
    return scene_ == nullptr ? nullptr : &scene_->objects_[index_];
}

inline Object& Object::set(std::string field, Json value) {
    if (Json* target = json(); target != nullptr) {
        target->set(std::move(field), std::move(value));
    }
    return *this;
}

inline Object& Object::noCollision() {
    return set("collision", Json());
}

inline Object& Object::support(std::string_view surfaceName, double z,
                               const std::vector<Vec2>& polygon, double clearHeight) {
    Json* target = json();
    if (target == nullptr || polygon.size() < 3U || polygonArea(polygon) < 0.01) {
        return *this;
    }
    Json points = Json::array();
    for (const Vec2& point : polygon) {
        points.push(Json::array({Json(point.x), Json(point.y)}));
    }
    Json surfaces = (*target)["support_surfaces"];
    surfaces.push(Json::object({{"name", Json(surfaceName)},
                                {"z", Json(z)},
                                {"polygon", std::move(points)},
                                {"clear_height", Json(std::max(0.0, clearHeight))}}));
    target->set("support_surfaces", std::move(surfaces));
    return *this;
}

inline Object& Object::region(std::string_view regionName, std::string_view kind, Vec3 center,
                              Vec3 size, double yawRadians) {
    Json* target = json();
    if (target == nullptr) {
        return *this;
    }
    Json regions = (*target)["regions"];
    regions.push(Scene::regionJson(regionName, kind, center, size, Quat::yaw(yawRadians)));
    target->set("regions", std::move(regions));
    return *this;
}

inline double Settings::number(std::string_view name, double fallback, double minimum,
                               double maximum) {
    const Json& value = configuration_[name];
    if (value.isNull()) {
        return fallback;
    }
    const double number = value.asNumber(fallback);
    if (!value.isNumber() || number < minimum || number > maximum) {
        const double clamped = std::clamp(value.isNumber() ? number : fallback, minimum, maximum);
        scene_.diagnostic("warning", "setting_out_of_range",
                          "Setting '" + std::string(name) + "' must be between " +
                              std::to_string(minimum) + " and " + std::to_string(maximum) +
                              "; using " + std::to_string(clamped),
                          name, number, clamped, true);
        return clamped;
    }
    return number;
}

inline int Settings::integer(std::string_view name, int fallback, int minimum, int maximum) {
    return static_cast<int>(std::lround(number(name, fallback, minimum, maximum)));
}

                                                                         

                                                                     
inline Object shelfBoard(Scene& scene, std::string_view keyName, const Pose& center, double width,
                         double depth, double thickness, double clearHeight, double erosion,
                         const Material& material) {
    Object board = scene.box(keyName, center, {width, depth, thickness}, material);
    board.label("shelf_board");
    board.support("top", thickness * 0.5, rectangle(width, depth, erosion), clearHeight);
    return board;
}

                                                                               
inline int legs(Scene& scene, std::string_view keyPrefix, const Pose& floorCenter, double width,
                double depth, double height, double thickness, double margin, bool isCylinder,
                const Material& material, std::string_view group) {
    const double insetX = std::max(thickness * 0.5, width * 0.5 * (1.0 - margin * 2.0));
    const double insetY = std::max(thickness * 0.5, depth * 0.5 * (1.0 - margin * 2.0));
    const std::array<Vec2, 4> corners{
        {{-insetX, -insetY}, {insetX, -insetY}, {insetX, insetY}, {-insetX, insetY}}};
    int count = 0;
    for (const Vec2& corner : corners) {
        const Pose pose = floorCenter * Pose::at(corner.x, corner.y, height * 0.5);
        const std::string legKey = key(keyPrefix, count);
        Object leg = isCylinder ? scene.cylinder(legKey, pose, thickness, height, material)
                                : scene.box(legKey, pose, {thickness, thickness, height}, material);
        leg.label("leg").group(group);
        ++count;
    }
    return count;
}

                                                                                  
                                                                                  
                                                                                
                                                                        
inline bool wallWithDoor(Scene& scene, std::string_view keyPrefix, const Pose& frame, double length,
                         double height, double thickness, double doorWidth, double doorHeight,
                         double doorOffset, const Material& material, std::string_view group,
                         std::string_view doorSetting = "doorway_width") {
    const auto segment = [&](std::string_view suffix, double from, double to, double bottom,
                             double top) {
        if (to - from < 1.0e-3 || top - bottom < 1.0e-3) {
            return;
        }
        const Pose pose = frame * Pose::at((from + to) * 0.5, 0.0, (bottom + top) * 0.5);
        Object part = scene.box(std::string(keyPrefix) + std::string(suffix), pose,
                                {to - from, thickness, top - bottom}, material);
        part.label("wall");
        if (!group.empty()) {
            part.group(group);
        }
    };
    const double half = length * 0.5;
    if (doorWidth <= 0.0) {
        segment("", -half, half, 0.0, height);
        return true;
    }
    const double usable = length - 0.2;
    if (doorWidth > usable || doorHeight >= height + 1.0e-6 || doorHeight <= 0.0) {
        scene.diagnostic("error", "doorway_does_not_fit",
                         "A " + std::to_string(doorWidth) + " m doorway does not fit in a " +
                             std::to_string(length) + " m wall; the wall was built without it",
                         doorSetting, doorWidth, std::max(0.0, usable), true);
        segment("", -half, half, 0.0, height);
        return false;
    }
    const double centre =
        std::clamp(doorOffset, -half + 0.1 + doorWidth * 0.5, half - 0.1 - doorWidth * 0.5);
    segment("/left", -half, centre - doorWidth * 0.5, 0.0, height);
    segment("/right", centre + doorWidth * 0.5, half, 0.0, height);
    segment("/header", centre - doorWidth * 0.5, centre + doorWidth * 0.5, doorHeight, height);
    const Pose door = frame * Pose::at(centre, 0.0, doorHeight * 0.5);
    scene.region(std::string(keyPrefix) + "/doorway", "keepout", door,
                 {doorWidth, thickness + 1.2, doorHeight});
    return true;
}

enum class Side : std::uint8_t { North, South, East, West };

inline Side sideFromName(std::string_view name) {
    if (name == "north") {
        return Side::North;
    }
    if (name == "east") {
        return Side::East;
    }
    if (name == "west") {
        return Side::West;
    }
    return Side::South;
}

struct RoomSpec {
    double width = 12.0;
    double depth = 8.0;
    double height = 3.0;
    double wallThickness = 0.15;
    Side doorSide = Side::South;
    double doorWidth = 1.0;
    double doorHeight = 2.1;
    double doorOffset = 0.0;
};

                                                                                    
                                                                         
inline void roomShell(Scene& scene, std::string_view prefix, const RoomSpec& room,
                      const Material& floorMaterial, const Material& wallMaterial,
                      const Pose& frame = Pose{}) {
    const std::string root = std::string(prefix) + "floor";
    const double t = room.wallThickness;
    scene
        .box(root, frame * Pose::at(0.0, 0.0, -0.05),
             {room.width + 2.0 * t, room.depth + 2.0 * t, 0.1}, floorMaterial)
        .label("floor");
    const double outerX = room.width + 2.0 * t;
    struct WallPlacement {
        Side side;
        std::string_view name;
        Pose frame;
        double length;
    };
    const std::array<WallPlacement, 4> walls{{
        {Side::South, "south", Pose::at(0.0, -(room.depth + t) * 0.5, 0.0), outerX},
        {Side::North, "north", Pose::at(0.0, (room.depth + t) * 0.5, 0.0, kPi), outerX},
        {Side::East, "east", Pose::at((room.width + t) * 0.5, 0.0, 0.0, kPi * 0.5), room.depth},
        {Side::West, "west", Pose::at(-(room.width + t) * 0.5, 0.0, 0.0, -kPi * 0.5), room.depth},
    }};
    for (const WallPlacement& wall : walls) {
        const bool hasDoor = wall.side == room.doorSide;
        wallWithDoor(scene, std::string(prefix) + "wall/" + std::string(wall.name),
                     frame * wall.frame, wall.length, room.height, t,
                     hasDoor ? room.doorWidth : 0.0, room.doorHeight, room.doorOffset, wallMaterial,
                     root);
    }
}

                                                                                 
                                                                 
inline void handle(Scene& scene, std::string_view keyName, const Pose& center, double length,
                   double thickness, bool isVertical, const Material& material,
                   std::string_view group) {
    const Vec3 size =
        isVertical ? Vec3{thickness, thickness, length} : Vec3{length, thickness, thickness};
    scene.box(keyName, center, size, material).noCollision().label("handle").group(group);
}

                                                                                  
                                                                               
                                                                                 
                            
inline Object hingeDoor(Scene& scene, std::string_view keyName, std::string_view parentKey,
                        const Pose& closedCenter, double width, double height, double thickness,
                        bool isHingeLeft, double openRadians, const Material& material,
                        const Material& handleMaterial) {
    Object door = scene.box(keyName, closedCenter, {width, thickness, height}, material);
    door.label("door").group(parentKey).mass(std::max(0.5, width * height * thickness * 600.0));
    const double side = isHingeLeft ? -1.0 : 1.0;
    const Vec3 hinge = closedCenter.apply({side * width * 0.5, 0.0, 0.0});
    const Vec3 axis = closedCenter.rotation.rotate({0.0, 0.0, 1.0});
    scene.joint(std::string(keyName) + "/hinge", "revolute", parentKey, keyName, hinge, axis,
                isHingeLeft ? -openRadians : 0.0, isHingeLeft ? 0.0 : openRadians);
    const double handleLength = std::min(0.3, height * 0.5);
    handle(scene, std::string(keyName) + "/handle",
           closedCenter * Pose::at(-side * (width * 0.5 - 0.06), -thickness * 0.5 - 0.02, 0.0),
           handleLength, 0.02, true, handleMaterial, keyName);
    return door;
}

                                                                                   
                                                                                  
inline Object slideDrawer(Scene& scene, std::string_view keyName, std::string_view parentKey,
                          const Pose& frontCenter, double width, double height, double depth,
                          double thickness, double travel, const Material& material,
                          const Material& handleMaterial) {
    Object front = scene.box(keyName, frontCenter, {width, thickness, height}, material);
    front.label("drawer").group(parentKey).mass(std::max(0.5, width * depth * height * 50.0));
    const std::string prefix(keyName);
    const double innerWidth = std::max(0.02, width - 0.04);
    const double boxHeight = std::max(0.02, height - 0.04);
    const double boxDepth = std::max(0.02, depth - thickness);
    const Pose boxCenter = frontCenter * Pose::at(0.0, thickness * 0.5 + boxDepth * 0.5, 0.0);
    const auto part = [&](std::string_view suffix, Vec3 offset, Vec3 size) {
        scene
            .box(prefix + std::string(suffix), boxCenter * Pose::at(offset.x, offset.y, offset.z),
                 size, material)
            .label("drawer_part")
            .group(keyName);
    };
    part("/bottom", {0.0, 0.0, -boxHeight * 0.5 + 0.005}, {innerWidth, boxDepth, 0.01});
    part("/back", {0.0, boxDepth * 0.5 - 0.005, 0.0}, {innerWidth, 0.01, boxHeight});
    part("/left", {-innerWidth * 0.5 + 0.005, 0.0, 0.0}, {0.01, boxDepth - 0.02, boxHeight});
    part("/right", {innerWidth * 0.5 - 0.005, 0.0, 0.0}, {0.01, boxDepth - 0.02, boxHeight});
    const Vec3 axis = frontCenter.rotation.rotate({0.0, -1.0, 0.0});
    scene.joint(prefix + "/slide", "prismatic", parentKey, keyName, frontCenter.position, axis, 0.0,
                travel);
    handle(scene, prefix + "/handle", frontCenter * Pose::at(0.0, -thickness * 0.5 - 0.02, 0.0),
           std::min(0.3, width * 0.5), 0.02, false, handleMaterial, keyName);
    return front;
}

                                                              
inline std::vector<double> distribute(double start, double end, int count) {
    std::vector<double> centres;
    if (count <= 0) {
        return centres;
    }
    const double step = (end - start) / count;
    for (int index = 0; index < count; ++index) {
        centres.push_back(start + (index + 0.5) * step);
    }
    return centres;
}

                                                                           
inline std::vector<double> row(int count, double itemWidth, double gap, double start) {
    std::vector<double> centres;
    for (int index = 0; index < count; ++index) {
        centres.push_back(start + itemWidth * 0.5 + index * (itemWidth + gap));
    }
    return centres;
}

struct Spread {
    std::vector<double> centres;
    double needed = 0.0;
    double innerGap = 0.0;
    double edgeGap = 0.0;
    bool fits = true;
};

                                                                          
                                                                             
                                                                        
                                                                              
                                                                            
inline Spread spread(const std::vector<double>& sizes, double length, double minInnerGap,
                     double minEdgeGap, bool isSpread = true) {
    Spread result;
    double total = 0.0;
    for (const double size : sizes) {
        total += size;
    }
    const int count = static_cast<int>(sizes.size());
    const int innerGaps = std::max(0, count - 1);
    result.needed = total + innerGaps * minInnerGap + 2.0 * minEdgeGap;
    result.fits = length + 1.0e-9 >= result.needed;
    const double used = std::max(length, result.needed);
    const double extra = isSpread ? (used - result.needed) / (innerGaps + 2) : 0.0;
    result.innerGap = minInnerGap + extra;
    result.edgeGap = minEdgeGap + extra;
    const double span = total + innerGaps * result.innerGap;
    double cursor = -span * 0.5;
    for (const double size : sizes) {
        result.centres.push_back(cursor + size * 0.5);
        cursor += size + result.innerGap;
    }
    return result;
}

                                                                                       
inline Mesh extrude(const std::vector<Vec2>& outline, double height) {
    Mesh mesh;
    const int count = static_cast<int>(outline.size());
    if (count < 3) {
        return mesh;
    }
    for (const double z : {0.0, height}) {
        for (const Vec2& point : outline) {
            mesh.vertices.insert(mesh.vertices.end(), {point.x, point.y, z});
        }
    }
    std::vector<int> remaining(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        remaining[static_cast<std::size_t>(index)] = index;
    }
    const auto cross = [&outline](int a, int b, int c) {
        const Vec2 p = outline[static_cast<std::size_t>(a)];
        const Vec2 q = outline[static_cast<std::size_t>(b)];
        const Vec2 r = outline[static_cast<std::size_t>(c)];
        return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
    };
    std::vector<std::array<int, 3>> triangles;
    for (int guard = 0; remaining.size() > 3U && guard < count * count; ++guard) {
        const std::size_t size = remaining.size();
        bool clipped = false;
        for (std::size_t i = 0U; i < size && !clipped; ++i) {
            const int a = remaining[(i + size - 1U) % size];
            const int b = remaining[i];
            const int c = remaining[(i + 1U) % size];
            if (cross(a, b, c) <= 0.0) {
                continue;
            }
            bool isEar = true;
            for (const int other : remaining) {
                if (other != a && other != b && other != c && cross(a, b, other) >= 0.0 &&
                    cross(b, c, other) >= 0.0 && cross(c, a, other) >= 0.0) {
                    isEar = false;
                    break;
                }
            }
            if (isEar) {
                triangles.push_back({a, b, c});
                remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
                clipped = true;
            }
        }
        if (!clipped) {
            break;
        }
    }
    if (remaining.size() == 3U) {
        triangles.push_back({remaining[0], remaining[1], remaining[2]});
    }
    for (const auto& triangle : triangles) {
        mesh.indices.insert(mesh.indices.end(), {triangle[0], triangle[2], triangle[1]});
        mesh.indices.insert(mesh.indices.end(),
                            {triangle[0] + count, triangle[1] + count, triangle[2] + count});
    }
    for (int index = 0; index < count; ++index) {
        const int next = (index + 1) % count;
        mesh.indices.insert(mesh.indices.end(), {index, next, next + count});
        mesh.indices.insert(mesh.indices.end(), {index, next + count, index + count});
    }
    return mesh;
}

                                                                                
                                                                 
inline Mesh bevelBox(Vec3 size, double bevel) {
    const double hx = size.x * 0.5;
    const double hy = size.y * 0.5;
    const double b = std::clamp(bevel, 0.0, std::min(hx, hy) * 0.9);
    const std::vector<Vec2> outline{{-hx + b, -hy}, {hx - b, -hy}, {hx, -hy + b}, {hx, hy - b},
                                    {hx - b, hy},   {-hx + b, hy}, {-hx, hy - b}, {-hx, -hy + b}};
    Mesh mesh = extrude(outline, size.z);
    for (std::size_t index = 2U; index < mesh.vertices.size(); index += 3U) {
        mesh.vertices[index] -= size.z * 0.5;
    }
    return mesh;
}

                                                                                
                                                                           
inline Mesh sweep(const std::vector<Vec2>& profile, const std::vector<Vec3>& path) {
    Mesh mesh;
    const int ring = static_cast<int>(profile.size());
    if (ring < 3 || path.size() < 2U) {
        return mesh;
    }
    const auto normalize = [](Vec3 v) {
        const double length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        return length > 1.0e-12 ? v * (1.0 / length) : Vec3{0.0, 0.0, 1.0};
    };
    const auto cross = [](Vec3 a, Vec3 b) {
        return Vec3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
    };
    for (std::size_t index = 0U; index < path.size(); ++index) {
        const Vec3 previous = path[index == 0U ? 0U : index - 1U];
        const Vec3 next = path[index + 1U < path.size() ? index + 1U : index];
        const Vec3 tangent = normalize(next - previous);
        Vec3 up{0.0, 0.0, 1.0};
        if (std::abs(tangent.z) > 0.9) {
            up = {1.0, 0.0, 0.0};
        }
        const Vec3 side = normalize(cross(tangent, up));
        const Vec3 normal = cross(side, tangent);
        for (const Vec2& point : profile) {
            const Vec3 vertex = path[index] + side * point.x + normal * point.y;
            mesh.vertices.insert(mesh.vertices.end(), {vertex.x, vertex.y, vertex.z});
        }
    }
    for (int segment = 0; segment + 1 < static_cast<int>(path.size()); ++segment) {
        for (int corner = 0; corner < ring; ++corner) {
            const int a = segment * ring + corner;
            const int b = segment * ring + (corner + 1) % ring;
            const int c = a + ring;
            const int d = b + ring;
            mesh.indices.insert(mesh.indices.end(), {a, b, d, a, d, c});
        }
    }
    return mesh;
}

                                                                                  
inline Material wear(Material material, Rng& rng, double amount) {
    const double a = std::clamp(amount, 0.0, 1.0);
    const double darken = 1.0 - a * rng.uniform(0.1, 0.35);
    material.color = {
        std::clamp(material.color.x * darken * (1.0 + rng.uniform(-0.05, 0.05) * a), 0.0, 1.0),
        std::clamp(material.color.y * darken, 0.0, 1.0),
        std::clamp(material.color.z * darken * (1.0 + rng.uniform(-0.05, 0.05) * a), 0.0, 1.0)};
    material.roughness = std::clamp(material.roughness + a * 0.2, 0.0, 1.0);
    return material;
}

                                                                             

struct SurfaceRef {
    std::string ownerKey;
    std::string name;
    Pose frame;
    std::vector<Vec2> polygon;
    double clearHeight = 0.0;
};

struct PlaceOptions {
    int maxTries = 100;
    int maxTriesPerSurface = 10;
    double gap = 0.002;
    bool randomYaw = true;
};

struct Footprint {
    Vec2 center;
    Vec2 half;
    double yaw = 0.0;
};

inline std::array<Vec2, 4> footprintCorners(const Footprint& footprint) {
    const double c = std::cos(footprint.yaw);
    const double s = std::sin(footprint.yaw);
    std::array<Vec2, 4> corners{};
    const std::array<Vec2, 4> local{{{-footprint.half.x, -footprint.half.y},
                                     {footprint.half.x, -footprint.half.y},
                                     {footprint.half.x, footprint.half.y},
                                     {-footprint.half.x, footprint.half.y}}};
    for (std::size_t i = 0U; i < 4U; ++i) {
        corners[i] = {footprint.center.x + c * local[i].x - s * local[i].y,
                      footprint.center.y + s * local[i].x + c * local[i].y};
    }
    return corners;
}

inline bool footprintsOverlap(const Footprint& a, const Footprint& b, double gap) {
    const auto project = [](const std::array<Vec2, 4>& corners, Vec2 axis, double& low,
                            double& high) {
        low = high = corners[0].x * axis.x + corners[0].y * axis.y;
        for (const Vec2& corner : corners) {
            const double value = corner.x * axis.x + corner.y * axis.y;
            low = std::min(low, value);
            high = std::max(high, value);
        }
    };
    const auto cornersA = footprintCorners(a);
    const auto cornersB = footprintCorners(b);
    for (const double yaw : {a.yaw, a.yaw + kPi * 0.5, b.yaw, b.yaw + kPi * 0.5}) {
        const Vec2 axis{std::cos(yaw), std::sin(yaw)};
        double lowA = 0.0;
        double highA = 0.0;
        double lowB = 0.0;
        double highB = 0.0;
        project(cornersA, axis, lowA, highA);
        project(cornersB, axis, lowB, highB);
        if (highA + gap <= lowB || highB + gap <= lowA) {
            return false;
        }
    }
    return true;
}

                                                                                    
class Placer {
  public:
    explicit Placer(Scene& scene, PlaceOptions options = {}) : scene_(scene), options_(options) {}

                                                                                 
                                                                    
    bool place(std::string_view itemKey, Vec3 size, const std::vector<SurfaceRef>& surfaces,
               Pose& placed) {
        if (surfaces.empty()) {
            return false;
        }
        Rng rng = scene_.rng(itemKey);
        std::size_t surfaceIndex =
            static_cast<std::size_t>(rng.integer(0, static_cast<int>(surfaces.size()) - 1));
        int triesOnSurface = 0;
        for (int attempt = 0; attempt < options_.maxTries; ++attempt) {
            const SurfaceRef& surface = surfaces[surfaceIndex];
            if (++triesOnSurface > options_.maxTriesPerSurface) {
                triesOnSurface = 0;
                surfaceIndex = (surfaceIndex + 1U) % surfaces.size();
                continue;
            }
            if (surface.polygon.size() < 3U ||
                (surface.clearHeight > 0.0 && size.z + options_.gap > surface.clearHeight)) {
                triesOnSurface = options_.maxTriesPerSurface;
                continue;
            }
            Vec2 low = surface.polygon.front();
            Vec2 high = low;
            for (const Vec2& point : surface.polygon) {
                low = {std::min(low.x, point.x), std::min(low.y, point.y)};
                high = {std::max(high.x, point.x), std::max(high.y, point.y)};
            }
            Footprint footprint;
            footprint.center = {rng.uniform(low.x, high.x), rng.uniform(low.y, high.y)};
            footprint.half = {size.x * 0.5, size.y * 0.5};
            footprint.yaw = options_.randomYaw ? rng.uniform(-kPi, kPi) : 0.0;
            const auto corners = footprintCorners(footprint);
            const bool isInside =
                std::all_of(corners.begin(), corners.end(), [&surface](const Vec2& corner) {
                    return polygonContains(surface.polygon, corner);
                });
            if (!isInside) {
                continue;
            }
            std::vector<Footprint>& taken = placed_[surface.ownerKey + "#" + surface.name];
            const bool isFree = std::none_of(
                taken.begin(), taken.end(), [this, &footprint](const Footprint& other) {
                    return footprintsOverlap(footprint, other, options_.gap);
                });
            if (!isFree) {
                continue;
            }
            taken.push_back(footprint);
            placed = surface.frame *
                     Pose{{footprint.center.x, footprint.center.y, size.z * 0.5 + 0.0005},
                          Quat::yaw(footprint.yaw)};
            return true;
        }
        return false;
    }

                                                                              
                                                                                       
    int populate(std::string_view keyPrefix, const std::vector<SurfaceRef>& surfaces,
                 double density, Vec3 minimumSize, Vec3 maximumSize, const Material& material,
                 std::string_view label = "box") {
        double area = 0.0;
        for (const SurfaceRef& surface : surfaces) {
            area += polygonArea(surface.polygon);
        }
        const double averageFootprint =
            (minimumSize.x + maximumSize.x) * 0.5 * (minimumSize.y + maximumSize.y) * 0.5;
        const int wanted = averageFootprint > 0.0
                               ? static_cast<int>(std::floor(density * area / averageFootprint))
                               : 0;
        int placedCount = 0;
        int failures = 0;
        for (int index = 0; index < wanted && failures < 8; ++index) {
            const std::string itemKey = key(keyPrefix, index);
            Rng sizeRng = scene_.rng(itemKey + "#size");
            const Vec3 size = snapSize({sizeRng.uniform(minimumSize.x, maximumSize.x),
                                        sizeRng.uniform(minimumSize.y, maximumSize.y),
                                        sizeRng.uniform(minimumSize.z, maximumSize.z)},
                                       minimumSize, maximumSize);
            Pose pose;
            if (!place(itemKey, size, surfaces, pose)) {
                ++failures;
                continue;
            }
            Rng colorRng = scene_.rng(itemKey + "#color");
            scene_.box(itemKey, pose, size, vary(material, colorRng, 0.12))
                .label(label)
                .mass(std::max(0.1, size.x * size.y * size.z * 150.0));
            ++placedCount;
        }
        if (placedCount < wanted) {
            scene_.diagnostic("info", "populate_partial",
                              "Placed " + std::to_string(placedCount) + " of " +
                                  std::to_string(wanted) + " items for density " +
                                  std::to_string(density) + "; the surfaces are full",
                              "density", wanted, placedCount, true);
        }
        return placedCount;
    }

  private:
    Scene& scene_;
    PlaceOptions options_;
    struct PlacedMap {
        std::vector<std::pair<std::string, std::vector<Footprint>>> entries;
        std::vector<Footprint>& operator[](const std::string& name) {
            for (auto& entry : entries) {
                if (entry.first == name) {
                    return entry.second;
                }
            }
            entries.emplace_back(name, std::vector<Footprint>{});
            return entries.back().second;
        }
    };
    PlacedMap placed_;
};

                                                                              
inline SurfaceRef boardSurface(std::string_view ownerKey, const Pose& boardCenter, double width,
                               double depth, double thickness, double clearHeight, double erosion) {
    return {std::string(ownerKey), "top", boardCenter * Pose{{0.0, 0.0, thickness * 0.5}, Quat{}},
            rectangle(width, depth, erosion), clearHeight};
}

}                                

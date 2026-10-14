#include <robotlens/extension.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using robotlens::Json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDensity = 650.0;

struct Vec {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Vec operator+(Vec a, Vec b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec operator-(Vec a, Vec b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec operator*(Vec a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double length(Vec a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

constexpr Vec kX{1.0, 0.0, 0.0};
constexpr Vec kY{0.0, 1.0, 0.0};
constexpr Vec kZ{0.0, 0.0, 1.0};

struct Quat {
  double w = 1.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

Quat zTo(Vec direction) {
  const double norm = length(direction);
  if (norm < 1.0e-9)
    return {};
  const Vec d = direction * (1.0 / norm);
  if (d.z > 0.999999)
    return {};
  if (d.z < -0.999999)
    return {0.0, 1.0, 0.0, 0.0};
  const double half = std::acos(std::clamp(d.z, -1.0, 1.0)) * 0.5;
  const double axisNorm = std::sqrt(d.x * d.x + d.y * d.y);
  const double s = std::sin(half) / axisNorm;
  return {std::cos(half), -d.y * s, d.x * s, 0.0};
}

std::array<std::array<double, 3>, 3> matrix(Quat q) {
  return {{{1 - 2 * (q.y * q.y + q.z * q.z), 2 * (q.x * q.y - q.w * q.z),
            2 * (q.x * q.z + q.w * q.y)},
           {2 * (q.x * q.y + q.w * q.z), 1 - 2 * (q.x * q.x + q.z * q.z),
            2 * (q.y * q.z - q.w * q.x)},
           {2 * (q.x * q.z - q.w * q.y), 2 * (q.y * q.z + q.w * q.x),
            1 - 2 * (q.x * q.x + q.y * q.y)}}};
}

struct Color {
  double r = 0.7;
  double g = 0.7;
  double b = 0.7;
  bool operator==(const Color &) const = default;
};

constexpr Color kGraphite{0.2, 0.22, 0.25};
constexpr Color kSteel{0.55, 0.58, 0.62};
constexpr Color kRubber{0.07, 0.07, 0.08};
constexpr Color kWhite{0.92, 0.92, 0.9};
constexpr Color kOrange{0.95, 0.45, 0.1};
constexpr Color kLens{0.05, 0.08, 0.12};
constexpr Color kYellow{0.95, 0.78, 0.12};
constexpr std::array<Color, 6> kAccents{{{0.12, 0.45, 0.85},
                                         {0.1, 0.65, 0.6},
                                         {0.85, 0.25, 0.25},
                                         {0.55, 0.35, 0.8},
                                         {0.3, 0.7, 0.3},
                                         {0.95, 0.6, 0.1}}};

struct Shape {
  std::string kind = "box";
  Vec size{0.1, 0.1, 0.1};
  double radius = 0.05;
  double length = 0.1;
  Quat rotation;
  Vec at;
  Color color;
  bool colored = false;
  bool contact = false;

  Shape tint(Color value) const {
    Shape copy = *this;
    copy.color = value;
    copy.colored = true;
    return copy;
  }
  Shape touching() const {
    Shape copy = *this;
    copy.contact = true;
    return copy;
  }
};

Shape box(Vec size, Vec at = {}) {
  Shape shape;
  shape.size = size;
  shape.at = at;
  return shape;
}

Shape ball(double radius, Vec at = {}) {
  Shape shape;
  shape.kind = "sphere";
  shape.radius = radius;
  shape.at = at;
  return shape;
}

Shape rod(double radius, Vec from, Vec to, std::string kind = "capsule") {
  Shape shape;
  shape.kind = std::move(kind);
  shape.radius = radius;
  shape.length = std::max(0.01, length(to - from));
  shape.rotation = zTo(to - from);
  shape.at = (from + to) * 0.5;
  return shape;
}

Shape disc(double radius, double thickness, Vec axis, Vec at = {}) {
  Shape shape;
  shape.kind = "cylinder";
  shape.radius = radius;
  shape.length = thickness;
  shape.rotation = zTo(axis);
  shape.at = at;
  return shape;
}

struct Config {
  double scale = 1.0;
  double variation = 0.12;
  double reach = 0.0;
  double limb = 1.0;
  double limitScale = 1.0;
  double effortScale = 1.0;
  double chassisLength = 0.0;
  double chassisWidth = 0.0;
  double chassisHeight = 0.0;
  double wheelRadius = 0.0;
  double clearance = 0.0;
  int segments = 0;
  bool gripper = true;
  double gripperOpening = 0.035;
  bool lidar = true;
  bool camera = true;
  bool imu = true;
  bool customColors = false;
  Color body{0.2, 0.22, 0.25};
  Color accent{0.12, 0.45, 0.85};
  Color arm{0.95, 0.45, 0.1};
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

Color readColor(const Json &value, Color fallback) {
  if (value.items().size() < 3)
    return fallback;
  return {std::clamp(value[0].asNumber(fallback.r), 0.0, 1.0),
          std::clamp(value[1].asNumber(fallback.g), 0.0, 1.0),
          std::clamp(value[2].asNumber(fallback.b), 0.0, 1.0)};
}

double readNumber(const Json &config, std::string_view name, double fallback,
                  double low, double high) {
  return std::clamp(config[name].asNumber(fallback), low, high);
}

Config readConfig(const Json &config) {
  Config result;
  result.scale = readNumber(config, "scale", 1.0, 0.25, 3.0);
  result.variation = readNumber(config, "variation", 0.12, 0.0, 0.3);
  result.reach = readNumber(config, "reach", 0.0, 0.0, 2.5);
  result.limb = readNumber(config, "limb_length", 1.0, 0.5, 2.0);
  result.limitScale = readNumber(config, "joint_limit_scale", 1.0, 0.1, 1.0);
  result.effortScale = readNumber(config, "effort_scale", 1.0, 0.1, 10.0);
  result.chassisLength = readNumber(config, "chassis_length", 0.0, 0.0, 3.0);
  result.chassisWidth = readNumber(config, "chassis_width", 0.0, 0.0, 2.0);
  result.chassisHeight = readNumber(config, "chassis_height", 0.0, 0.0, 1.0);
  result.wheelRadius = readNumber(config, "wheel_radius", 0.0, 0.0, 0.5);
  result.clearance = readNumber(config, "ground_clearance", 0.0, 0.0, 0.5);
  result.segments = static_cast<int>(readNumber(config, "segments", 0.0, 0.0, 60.0));
  result.gripper = config["gripper"].asBool(true);
  result.gripperOpening = readNumber(config, "gripper_opening", 0.035, 0.005, 0.15);
  result.lidar = config["lidar"].asBool(true);
  result.camera = config["camera"].asBool(true);
  result.imu = config["imu"].asBool(true);
  result.customColors = config["custom_colors"].asBool(false);
  result.body = readColor(config["body_color"], result.body);
  result.accent = readColor(config["accent_color"], result.accent);
  result.arm = readColor(config["arm_color"], result.arm);
  result.x = readNumber(config, "x", 0.0, -100.0, 100.0);
  result.y = readNumber(config, "y", 0.0, -100.0, 100.0);
  result.yaw = readNumber(config, "yaw", 0.0, -180.0, 180.0);
  return result;
}

struct JointSpec {
  std::string type = "fixed";
  Vec axis = kZ;
  double lower = 0.0;
  double upper = 0.0;
  std::string actuator;
  double effort = 0.0;
};

JointSpec fixed() { return {}; }

JointSpec revolute(Vec axis, double lower, double upper, double effort,
                   bool driven = true) {
  return {"revolute", axis, lower, upper, driven ? "position" : "", effort};
}

JointSpec continuous(Vec axis, double effort, bool driven = true) {
  return {"continuous", axis, 0.0, 0.0, driven ? "velocity" : "", effort};
}

JointSpec prismatic(Vec axis, double lower, double upper, double effort) {
  return {"prismatic", axis, lower, upper, "position", effort};
}

class Random {
public:
  Random(std::uint64_t seed, double strength)
      : state_(seed + 0x9E3779B97F4A7C15ULL), strength_(strength) {}

  double unit() {
    std::uint64_t z = (state_ += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30U)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27U)) * 0x94D049BB133111EBULL;
    z ^= z >> 31U;
    return static_cast<double>(z >> 11U) / 9007199254740992.0;
  }
  double vary(double amount) {
    return 1.0 + amount * strength_ * (2.0 * unit() - 1.0);
  }
  int range(int low, int high) {
    return low + static_cast<int>(unit() * (high - low + 1)) % (high - low + 1);
  }

private:
  std::uint64_t state_;
  double strength_;
};

double shapeVolume(const Shape &shape) {
  if (shape.kind == "box")
    return shape.size.x * shape.size.y * shape.size.z;
  const double r = shape.radius;
  const double sphere = 4.0 / 3.0 * kPi * r * r * r;
  if (shape.kind == "sphere")
    return sphere;
  const double cylinder = kPi * r * r * shape.length;
  return shape.kind == "capsule" ? cylinder + sphere : cylinder;
}

std::array<double, 3> centralInertia(const Shape &shape, double mass) {
  if (shape.kind == "box") {
    const Vec s = shape.size;
    return {mass * (s.y * s.y + s.z * s.z) / 12.0,
            mass * (s.x * s.x + s.z * s.z) / 12.0,
            mass * (s.x * s.x + s.y * s.y) / 12.0};
  }
  const double r = shape.radius;
  if (shape.kind == "sphere") {
    const double value = 0.4 * mass * r * r;
    return {value, value, value};
  }
  const double l =
      shape.kind == "capsule" ? shape.length + 1.5 * r : shape.length;
  const double lateral = mass * (3.0 * r * r + l * l) / 12.0;
  return {lateral, lateral, 0.5 * mass * r * r};
}

double halfHeight(const Shape &shape) {
  const auto m = matrix(shape.rotation);
  if (shape.kind == "box")
    return 0.5 * (std::abs(m[2][0]) * shape.size.x +
                  std::abs(m[2][1]) * shape.size.y +
                  std::abs(m[2][2]) * shape.size.z);
  if (shape.kind == "sphere")
    return shape.radius;
  const double dz = std::abs(m[2][2]);
  if (shape.kind == "capsule")
    return dz * shape.length * 0.5 + shape.radius;
  return dz * shape.length * 0.5 +
         shape.radius * std::sqrt(std::max(0.0, 1.0 - dz * dz));
}

Json vector3(Vec value) {
  return Json::array({Json(value.x), Json(value.y), Json(value.z)});
}

Json pose(Vec position, Quat rotation = {}) {
  return Json::object({{"position", vector3(position)},
                       {"rotation", Json::array({Json(rotation.w),
                                                 Json(rotation.x),
                                                 Json(rotation.y),
                                                 Json(rotation.z)})}});
}

class Builder {
public:
  Builder(std::string id, std::string name, Config config)
      : id_(std::move(id)), name_(std::move(name)), scale_(config.scale),
        config_(config) {}

  const Config &config() const { return config_; }

  int link(const std::string &name, int parent, Vec at, JointSpec joint,
           std::vector<Shape> shapes, Color color) {
    for (Shape &shape : shapes) {
      shape.at = shape.at * scale_;
      shape.size = shape.size * scale_;
      shape.radius *= scale_;
      shape.length *= scale_;
      if (!shape.colored)
        shape.color = color;
      shape.color = recolor(shape.color);
    }
    if (joint.type == "prismatic") {
      joint.lower *= scale_;
      joint.upper *= scale_;
    }
    if (joint.type == "revolute") {
      joint.lower *= config_.limitScale;
      joint.upper *= config_.limitScale;
    }
    joint.effort *= scale_ * scale_ * config_.effortScale;
    const Vec world = parent < 0 ? Vec{} : links_[parent].world + at * scale_;
    links_.push_back({next("link"), name, parent, at * scale_, world,
                      std::move(joint), std::move(shapes)});
    return static_cast<int>(links_.size()) - 1;
  }

  Json finish(bool floating) {
    double lowest = 0.0;
    for (const LinkData &item : links_)
      for (const Shape &shape : item.shapes)
        lowest = std::min(lowest, item.world.z + shape.at.z - halfHeight(shape));
    Json links = Json::array();
    Json joints = Json::array();
    Json actuators = Json::array();
    for (std::size_t index = 0; index < links_.size(); ++index) {
      const LinkData &item = links_[index];
      Json visuals = Json::array();
      Json collisions = Json::array();
      double mass = 0.0;
      std::array<double, 3> inertia{0.0, 0.0, 0.0};
      for (const Shape &shape : item.shapes) {
        visuals.push(primitive(shape, item.name, "visual"));
        collisions.push(primitive(collision(shape), item.name, "collision"));
        const double shapeMass = std::max(0.005, shapeVolume(shape) * kDensity);
        const auto central = centralInertia(shape, shapeMass);
        const auto m = matrix(shape.rotation);
        const Vec a = shape.at;
        const std::array<double, 3> offset{a.y * a.y + a.z * a.z,
                                           a.x * a.x + a.z * a.z,
                                           a.x * a.x + a.y * a.y};
        for (int row = 0; row < 3; ++row) {
          for (int column = 0; column < 3; ++column)
            inertia[row] += m[row][column] * m[row][column] * central[column];
          inertia[row] += shapeMass * offset[row];
        }
        mass += shapeMass;
      }
      for (double &value : inertia)
        value = std::max(value, 1.0e-6);
      links.push(Json::object(
          {{"id", item.id},
           {"name", item.name},
           {"local_transform",
            item.parent < 0 ? pose({config_.x, config_.y, -lowest}, placement()) : pose({})},
           {"visuals", std::move(visuals)},
           {"collisions", std::move(collisions)},
           {"mass", Json(std::max(0.02, mass))},
           {"inertia", vector3({inertia[0], inertia[1], inertia[2]})}}));
      if (item.parent < 0)
        continue;
      const std::string jointId = next("joint");
      Json joint = Json::object({{"id", jointId},
                                 {"name", item.name + " joint"},
                                 {"parent", links_[item.parent].id},
                                 {"child", item.id},
                                 {"type", item.joint.type},
                                 {"parent_frame", pose(item.at)},
                                 {"child_frame", pose({})},
                                 {"axis", vector3(item.joint.axis)}});
      if (item.joint.type == "revolute" || item.joint.type == "prismatic")
        joint.set("limits", Json::array({Json(item.joint.lower),
                                         Json(item.joint.upper)}));
      joints.push(std::move(joint));
      if (!item.joint.actuator.empty() && item.joint.type != "fixed")
        actuators.push(Json::object(
            {{"id", next("actuator")},
             {"name", item.name + " actuator"},
             {"joint", jointId},
             {"type", item.joint.actuator},
             {"effort_limit", Json(std::max(0.1, item.joint.effort))}}));
    }
    return Json::object(
        {{"schema_version", Json(1)},
         {"robot", Json::object({{"id", id_},
                                 {"name", name_},
                                 {"root_link_id", links_.front().id},
                                 {"metadata", Json::object({{"base", floating ? "floating" : "fixed"}})},
                                 {"next_id", Json(static_cast<int>(counter_ + 1))},
                                 {"links", std::move(links)},
                                 {"joints", std::move(joints)},
                                 {"actuators", std::move(actuators)}})}});
  }

private:
  struct LinkData {
    std::string id;
    std::string name;
    int parent = -1;
    Vec at;
    Vec world;
    JointSpec joint;
    std::vector<Shape> shapes;
  };

  std::string next(std::string_view kind) {
    return id_ + "/" + std::string(kind) + "/" + std::to_string(++counter_);
  }

  Quat placement() const {
    const double half = config_.yaw * kPi / 360.0;
    return {std::cos(half), 0.0, 0.0, std::sin(half)};
  }

  Color recolor(Color color) const {
    if (!config_.customColors)
      return color;
    if (color == kGraphite)
      return config_.body;
    if (color == kOrange)
      return config_.arm;
    return color;
  }

  static Shape collision(const Shape &shape) {
    if (shape.contact)
      return shape;
    Shape copy = shape;
    if (copy.kind == "box")
      copy.size = copy.size * 0.9;
    copy.radius *= 0.9;
    copy.length *= 0.85;
    return copy;
  }

  Json primitive(const Shape &shape, const std::string &linkName,
                 std::string_view role) {
    const bool isBox = shape.kind == "box";
    const Vec size = isBox ? shape.size
                           : Vec{2.0 * shape.radius, 2.0 * shape.radius,
                                 shape.kind == "sphere" ? 2.0 * shape.radius
                                                        : shape.length};
    const double radius =
        isBox ? 0.5 * std::min({size.x, size.y, size.z}) : shape.radius;
    return Json::object(
        {{"id", next(role)},
         {"name", linkName + " " + std::string(role)},
         {"shape", shape.kind},
         {"pose", pose(shape.at, shape.rotation)},
         {"size", vector3(size)},
         {"radius", Json(radius)},
         {"length", Json(isBox ? size.z : shape.length)},
         {"color", Json::array({Json(shape.color.r), Json(shape.color.g),
                                Json(shape.color.b), Json(1)})}});
  }

  std::string id_;
  std::string name_;
  double scale_;
  Config config_;
  std::uint64_t counter_ = 0;
  std::vector<LinkData> links_;
};

int wheel(Builder &b, int parent, const std::string &name, Vec at,
          double radius, double width) {
  return b.link(name, parent, at, continuous(kY, 30.0),
                {disc(radius, width, kY).touching().tint(kRubber),
                 disc(radius * 0.55, width * 1.08, kY).tint(kSteel)},
                kRubber);
}

int caster(Builder &b, int parent, const std::string &name, Vec at,
           double radius) {
  return b.link(name, parent, at, fixed(),
                {ball(radius).touching(),
                 disc(radius * 0.8, radius * 0.6, kZ, {0, 0, radius * 1.1})},
                kSteel);
}

int lidar(Builder &b, int parent, Vec at) {
  if (!b.config().lidar)
    return -1;
  return b.link("lidar_link", parent, at, fixed(),
                {disc(0.038, 0.02, kZ, {0, 0, 0.01}).tint(kGraphite),
                 disc(0.042, 0.045, kZ, {0, 0, 0.0425}).tint(kLens)},
                kGraphite);
}

int camera(Builder &b, int parent, Vec at) {
  if (!b.config().camera)
    return -1;
  return b.link("camera_link", parent, at, fixed(),
                {box({0.035, 0.16, 0.035}),
                 disc(0.012, 0.006, kX, {0.0185, 0.04, 0}).tint(kLens),
                 disc(0.012, 0.006, kX, {0.0185, -0.04, 0}).tint(kLens)},
                kGraphite);
}

int imu(Builder &b, int parent, Vec at) {
  if (!b.config().imu)
    return -1;
  return b.link("imu_link", parent, at, fixed(), {box({0.03, 0.03, 0.012})},
                {0.1, 0.4, 0.2});
}

int gripper(Builder &b, int parent, Vec at, Color body, Color fingers) {
  const double width = 0.06 + 2.0 * b.config().gripperOpening;
  const int palm = b.link("gripper palm", parent, at, fixed(),
                          {box({0.04, width, 0.05}, {0.02, 0, 0})}, body);
  b.link("left finger", palm, {0.04, 0.025, 0}, prismatic(kY, 0.0, b.config().gripperOpening, 25),
         {box({0.07, 0.012, 0.03}, {0.035, 0.006, 0})}, fingers);
  b.link("right finger", palm, {0.04, -0.025, 0},
         prismatic({0, -1, 0}, 0.0, b.config().gripperOpening, 25),
         {box({0.07, 0.012, 0.03}, {0.035, -0.006, 0})}, fingers);
  return palm;
}

int arm(Builder &b, int parent, Vec mount, double reach, Color links,
        Color housings, bool redundant = false) {
  const double upper = 0.45 * reach;
  const double fore = 0.4 * reach;
  const int base =
      b.link("arm base", parent, mount, revolute(kZ, -3.05, 3.05, 90),
             {disc(0.075, 0.12, kZ, {0, 0, 0.06}).tint(housings)}, links);
  const double lower = redundant ? upper * 0.5 : upper;
  int shoulder = b.link(
      "upper arm", base, {0, 0, 0.12}, revolute(kY, -1.9, 1.9, 120),
      {disc(0.065, 0.13, kY).tint(housings), rod(0.045, {}, {0, 0, lower}, "cylinder")},
      links);
  if (redundant)
    shoulder = b.link("upper arm roll", shoulder, {0, 0, lower},
                      revolute(kZ, -3.0, 3.0, 50),
                      {disc(0.05, 0.05, kZ, {0, 0, 0.025}).tint(housings),
                       rod(0.042, {}, {0, 0, upper - lower}, "cylinder")},
                      links);
  const int elbow = b.link(
      "forearm", shoulder, {0, 0, upper - (redundant ? lower : 0.0)},
      revolute(kY, -2.5, 2.5, 70),
      {disc(0.055, 0.11, kY).tint(housings), rod(0.038, {}, {fore, 0, 0}, "cylinder")},
      links);
  const int wrist =
      b.link("wrist roll", elbow, {fore, 0, 0}, revolute(kX, -3.1, 3.1, 25),
             {disc(0.04, 0.07, kX, {0.035, 0, 0}).tint(housings)}, links);
  const int pitch = b.link(
      "wrist pitch", wrist, {0.07, 0, 0}, revolute(kY, -2.0, 2.0, 20),
      {disc(0.04, 0.07, kY).tint(housings), disc(0.032, 0.05, kX, {0.04, 0, 0})},
      links);
  return b.link("tool flange", pitch, {0.07, 0, 0}, revolute(kX, -3.1, 3.1, 12),
                {disc(0.035, 0.02, kX, {0.01, 0, 0}).tint(housings)}, links);
}

void articulated(Builder &b, Random &random, Color accent, bool cobot,
                 bool surgical) {
  const double reach = b.config().reach > 0.0
                           ? b.config().reach
                           : (cobot ? 0.9 : 1.0) * random.vary(0.12);
  if (surgical) {
    const int cart = b.link(
        "cart", -1, {}, fixed(),
        {box({0.6, 0.5, 0.6}), box({0.62, 0.52, 0.04}, {0, 0, 0.32}).tint(accent),
         rod(0.06, {-0.15, 0, 0.3}, {-0.15, 0, 0.95}, "cylinder").tint(kWhite)},
        kWhite);
    for (int index = 0; index < 4; ++index)
      caster(b, cart, "caster " + std::to_string(index + 1),
             {index < 2 ? -0.24 : 0.24, index % 2 ? -0.19 : 0.19, -0.34}, 0.04);
    const int flange = arm(b, cart, {-0.15, 0, 0.95},
                           b.config().reach > 0.0 ? b.config().reach
                                                  : 0.7 * random.vary(0.1),
                           kWhite, accent);
    b.link("instrument", flange, {0.02, 0, 0}, prismatic(kX, 0.0, 0.12, 15),
           {disc(0.02, 0.06, kX, {0.03, 0, 0}),
            rod(0.006, {0.06, 0, 0}, {0.4, 0, 0}, "cylinder").tint(kSteel)},
           kGraphite);
    b.link("endoscope camera", cart, {-0.05, 0, 1.0}, fixed(),
           {rod(0.02, {}, {0.3, 0, 0}, "cylinder").tint(kSteel),
            ball(0.04, {0.32, 0, 0}).tint(kLens)},
           kSteel);
    return;
  }
  const int base = b.link(
      "pedestal", -1, {}, fixed(),
      {disc(cobot ? 0.1 : 0.16, cobot ? 0.08 : 0.2, kZ, {0, 0, cobot ? 0.04 : 0.1})
           .touching()},
      kGraphite);
  if (cobot)
    b.link("status ring", base, {0, 0, 0.08}, fixed(),
           {disc(0.105, 0.015, kZ, {0, 0, 0.0075})}, accent);
  const int flange = arm(b, base, {0, 0, cobot ? 0.095 : 0.2}, reach,
                         cobot ? kWhite : kOrange, cobot ? accent : kGraphite);
  if (b.config().gripper)
    gripper(b, flange, {0.02, 0, 0}, kGraphite, kSteel);
  else
    b.link("tool", flange, {0.02, 0, 0}, fixed(),
           {disc(0.03, 0.08, kX, {0.04, 0, 0}),
            rod(0.01, {0.08, 0, 0}, {0.16, 0, 0}, "cylinder").tint(kSteel)},
           kGraphite);
}

void scara(Builder &b, Random &random, Color accent) {
  const double inner = 0.4 * random.vary(0.12);
  const double outer = 0.32 * random.vary(0.12);
  const int base = b.link(
      "base", -1, {}, fixed(),
      {box({0.3, 0.3, 0.1}, {0, 0, 0.05}).touching(),
       rod(0.08, {0, 0, 0.1}, {0, 0, 0.55}, "cylinder")},
      kGraphite);
  const int first = b.link(
      "inner arm", base, {0, 0, 0.55}, revolute(kZ, -2.5, 2.5, 60),
      {disc(0.07, 0.1, kZ, {0, 0, 0.05}).tint(kGraphite),
       box({inner, 0.12, 0.08}, {inner * 0.5, 0, 0.05})},
      accent);
  const int second = b.link(
      "outer arm", first, {inner, 0, 0}, revolute(kZ, -2.6, 2.6, 40),
      {disc(0.06, 0.1, kZ, {0, 0, 0.13}).tint(kGraphite),
       box({outer, 0.1, 0.07}, {outer * 0.5, 0, 0.13})},
      accent);
  const int quill = b.link("quill", second, {outer, 0, 0},
                           prismatic(kZ, -0.15, 0.0, 80),
                           {rod(0.02, {0, 0, -0.12}, {0, 0, 0.2}, "cylinder")},
                           kSteel);
  b.link("tool roll", quill, {0, 0, -0.12}, revolute(kZ, -3.1, 3.1, 8),
         {disc(0.03, 0.04, kZ, {0, 0, -0.02})}, kGraphite);
}

void delta(Builder &b, Random &random, Color accent) {
  const double height = 0.95 * random.vary(0.08);
  std::vector<Shape> frame{disc(0.22, 0.04, kZ).tint(accent)};
  for (int index = 0; index < 3; ++index) {
    const double angle = 2.0 * kPi * index / 3.0 + kPi / 3.0;
    const Vec top{0.36 * std::cos(angle), 0.36 * std::sin(angle), 0.0};
    frame.push_back(rod(0.025, top, top + Vec{0, 0, -height}).touching());
    frame.push_back(rod(0.02, {0.2 * std::cos(angle), 0.2 * std::sin(angle), 0}, top));
  }
  const int plate = b.link("frame", -1, {}, fixed(), frame, kSteel);
  const double platform = height - 0.3;
  int firstForearm = -1;
  Vec firstEnd;
  for (int index = 0; index < 3; ++index) {
    const double angle = 2.0 * kPi * index / 3.0;
    const Vec radial{std::cos(angle), std::sin(angle), 0.0};
    const Vec tangent{-std::sin(angle), std::cos(angle), 0.0};
    const Vec pivot = radial * 0.16 + Vec{0, 0, -0.03};
    const Vec elbow = radial * 0.2 + Vec{0, 0, -0.08};
    const Vec target = radial * 0.06 + Vec{0, 0, -platform};
    const Vec reach = target - pivot - elbow;
    const int upper = b.link("upper arm " + std::to_string(index + 1), plate,
                             pivot, revolute(tangent, -1.2, 1.2, 40),
                             {disc(0.03, 0.06, tangent).tint(kGraphite),
                              rod(0.022, {}, elbow, "cylinder")},
                             accent);
    const int forearm = b.link(
        "forearm " + std::to_string(index + 1), upper, elbow,
        revolute(tangent, -1.5, 1.5, 1, false),
        {ball(0.025).tint(kGraphite), rod(0.012, {}, reach, "cylinder")}, kSteel);
    if (index == 0) {
      firstForearm = forearm;
      firstEnd = reach;
    }
  }
  const Vec center = Vec{-0.06, 0, 0};
  b.link("effector", firstForearm, firstEnd, fixed(),
         {disc(0.08, 0.025, kZ, center),
          rod(0.015, center, center + Vec{0, 0, -0.08}, "cylinder")},
         kGraphite);
}

int cartesian(Builder &b, Random &random, Color accent) {
  const double span = 1.0 * random.vary(0.15);
  const double depth = 0.7 * random.vary(0.15);
  const double height = 0.65;
  std::vector<Shape> frame;
  for (int index = 0; index < 4; ++index) {
    const Vec foot{index < 2 ? -span * 0.5 : span * 0.5,
                   index % 2 ? -depth * 0.5 : depth * 0.5, 0.0};
    frame.push_back(box({0.06, 0.06, height}, foot + Vec{0, 0, height * 0.5}).touching());
  }
  for (int side = -1; side <= 1; side += 2)
    frame.push_back(box({span + 0.06, 0.06, 0.06}, {0, side * depth * 0.5, height + 0.03}).tint(accent));
  const int base = b.link("gantry frame", -1, {}, fixed(), frame, kSteel);
  const int bridge = b.link(
      "X bridge", base, {0, 0, height + 0.09},
      prismatic(kX, -span * 0.4, span * 0.4, 200),
      {box({0.08, depth + 0.1, 0.06})}, kGraphite);
  const int carriage = b.link("Y carriage", bridge, {0, 0, -0.06},
                              prismatic(kY, -depth * 0.4, depth * 0.4, 150),
                              {box({0.14, 0.14, 0.06})}, accent);
  return b.link("Z ram", carriage, {0, 0, -0.03}, prismatic(kZ, -0.3, 0.0, 120),
         {box({0.05, 0.05, 0.35}, {0, 0, 0.0}), ball(0.03, {0, 0, -0.19}).tint(kGraphite)},
         kSteel);
}

struct BaseSize {
  double length;
  double width;
  double height;
  double wheel;
  double clearance;
};

BaseSize baseSize(const Config &config, BaseSize fallback) {
  return {config.chassisLength > 0.0 ? config.chassisLength : fallback.length,
          config.chassisWidth > 0.0 ? config.chassisWidth : fallback.width,
          config.chassisHeight > 0.0 ? config.chassisHeight : fallback.height,
          config.wheelRadius > 0.0 ? config.wheelRadius : fallback.wheel,
          config.clearance > 0.0 ? config.clearance : fallback.clearance};
}

void mobileBase(Builder &b, Random &random, Color accent,
                std::string_view family) {
  const Config &config = b.config();
  if (family == "amr") {
    const BaseSize s = baseSize(config, {0.7 * random.vary(0.1), 0.5, 0.22, 0.1, 0.05});
    const double bottom = -(s.height * 0.5 + s.clearance);
    const int chassis = b.link(
        "chassis", -1, {}, fixed(),
        {box({s.length, s.width, s.height}),
         box({s.length + 0.02, s.width + 0.01, 0.04}).tint(accent),
         box({s.length * 0.43, s.width * 0.6, 0.01}, {0, 0, s.height * 0.5 + 0.005}).tint(kSteel)},
        kGraphite);
    wheel(b, chassis, "left wheel", {0, s.width * 0.5 + 0.03, bottom + s.wheel}, s.wheel, 0.05);
    wheel(b, chassis, "right wheel", {0, -s.width * 0.5 - 0.03, bottom + s.wheel}, s.wheel, 0.05);
    const double roller = std::max(0.02, s.clearance);
    caster(b, chassis, "front caster", {s.length * 0.37, 0, bottom + roller}, roller);
    caster(b, chassis, "rear caster", {-s.length * 0.37, 0, bottom + roller}, roller);
    lidar(b, chassis, {s.length * 0.36, 0, s.height * 0.5});
    camera(b, chassis, {s.length * 0.5 + 0.02, 0, 0.0});
    imu(b, chassis, {0, 0, 0});
    return;
  }
  if (family == "agv") {
    const BaseSize s = baseSize(config, {0.9, 0.6, 0.15, 0.07, 0.07});
    const double bottom = -(s.height * 0.5 + s.clearance);
    const double x = s.length * 0.5;
    const double y = s.width * 0.5;
    const int chassis = b.link(
        "chassis", -1, {}, fixed(),
        {box({s.length, s.width, s.height}),
         box({0.06, 0.04, 0.04}, {x, y - 0.02, s.height * 0.2}).tint(kYellow),
         box({0.06, 0.04, 0.04}, {x, -y + 0.02, s.height * 0.2}).tint(kYellow),
         disc(0.03, 0.06, kZ, {-x + 0.1, y - 0.08, s.height * 0.5 + 0.03}).tint(accent)},
        kGraphite);
    for (int index = 0; index < 4; ++index)
      wheel(b, chassis, "wheel " + std::to_string(index + 1),
            {index < 2 ? x - 0.15 : -x + 0.15, index % 2 ? -(y - 0.06) : y - 0.06,
             bottom + s.wheel},
            s.wheel, 0.05);
    b.link("lift deck", chassis, {0, 0, s.height * 0.5}, prismatic(kZ, 0.0, 0.08, 400),
           {box({s.length - 0.2, s.width - 0.1, 0.03}, {0, 0, 0.015})}, accent);
    b.link("tow pin", chassis, {-x - 0.02, 0, 0.0}, fixed(),
           {rod(0.02, {0, 0, -0.04}, {0, 0, 0.06}, "cylinder")}, kSteel);
    lidar(b, chassis, {x - 0.07, 0, s.height * 0.5});
    imu(b, chassis, {0, 0, 0});
    return;
  }
  if (family == "ugv") {
    const BaseSize s = baseSize(config, {0.8, 0.46, 0.2, 0.13, 0.12});
    const double rockerZ = -s.height * 0.25;
    const double axle = -(s.height * 0.5 + s.clearance) + s.wheel - rockerZ;
    const int chassis = b.link(
        "chassis", -1, {}, fixed(),
        {box({s.length, s.width, s.height}),
         box({s.length * 0.62, s.width * 0.65, 0.04}, {-0.05, 0, s.height * 0.6}).tint(accent),
         rod(0.02, {s.length * 0.31, 0, s.height * 0.5}, {s.length * 0.31, 0, s.height * 0.5 + 0.3},
             "cylinder")
             .tint(kSteel)},
        kGraphite);
    for (int side = -1; side <= 1; side += 2) {
      const std::string label = side > 0 ? "left" : "right";
      const int rocker = b.link(
          label + " rocker", chassis, {0, side * (s.width * 0.5 + 0.04), rockerZ},
          revolute(kY, -0.35, 0.35, 1, false),
          {box({s.length * 0.78, 0.04, 0.05}), disc(0.035, 0.06, kY).tint(kSteel)}, accent);
      wheel(b, rocker, label + " front wheel", {s.length * 0.39, side * 0.07, axle}, s.wheel, 0.08);
      wheel(b, rocker, label + " rear wheel", {-s.length * 0.39, side * 0.07, axle}, s.wheel, 0.08);
    }
    camera(b, chassis, {s.length * 0.31, 0, s.height * 0.5 + 0.32});
    lidar(b, chassis, {-s.length * 0.25, 0, s.height * 0.5 + 0.04});
    imu(b, chassis, {0, 0, 0});
    return;
  }
  if (family == "domestic") {
    const int base = b.link(
        "base", -1, {}, fixed(),
        {disc(0.22, 0.12, kZ), disc(0.225, 0.02, kZ, {0, 0, 0.0}).tint(accent)},
        kWhite);
    wheel(b, base, "left wheel", {0, 0.16, -0.04}, 0.05, 0.03);
    wheel(b, base, "right wheel", {0, -0.16, -0.04}, 0.05, 0.03);
    caster(b, base, "front caster", {0.15, 0, -0.065}, 0.025);
    caster(b, base, "rear caster", {-0.15, 0, -0.065}, 0.025);
    const int mast = b.link(
        "mast", base, {0, 0, 0.06}, prismatic(kZ, 0.0, 0.25, 150),
        {rod(0.03, {}, {0, 0, 0.5}, "cylinder"), disc(0.14, 0.015, kZ, {0, 0, 0.3}).tint(kWhite)},
        kSteel);
    const int pan = b.link("head pan", mast, {0, 0, 0.5},
                           revolute(kZ, -2.8, 2.8, 6),
                           {disc(0.04, 0.04, kZ, {0, 0, 0.02})}, kGraphite);
    const int head = b.link("head tilt", pan, {0, 0, 0.1},
                            revolute(kY, -0.6, 0.8, 4),
                            {box({0.16, 0.26, 0.15}),
                             box({0.01, 0.2, 0.1}, {0.081, 0, 0}).tint(kLens)},
                            kWhite);
    camera(b, head, {0.06, 0, 0.1});
    lidar(b, base, {0.14, 0, 0.06});
    imu(b, base, {0, 0, 0});
    return;
  }
  const BaseSize s = baseSize(config, {0.8, 0.6, 0.25, 0.11, 0.1});
  const double x = s.length * 0.5;
  const double y = s.width * 0.5;
  const double top = s.height * 0.5;
  const int chassis = b.link(
      "chassis", -1, {}, fixed(),
      {box({s.length, s.width, s.height}),
       box({s.length * 0.62, 0.01, s.height * 0.48}, {0, y, 0}).tint(accent),
       box({s.length * 0.62, 0.01, s.height * 0.48}, {0, -y, 0}).tint(accent),
       box({0.01, s.width * 0.67, s.height * 0.48}, {x, 0, 0}).tint(accent)},
      kGraphite);
  for (int index = 0; index < 4; ++index)
    wheel(b, chassis, std::string(index % 2 ? "right" : "left") +
                          (index < 2 ? " front wheel" : " rear wheel"),
          {index < 2 ? x - s.wheel - 0.04 : -x + s.wheel + 0.04,
           index % 2 ? -(y + 0.04) : y + 0.04, -(top + s.clearance) + s.wheel},
          s.wheel, 0.06);
  lidar(b, chassis, {x - 0.08, 0, top});
  imu(b, chassis, {0, 0, 0});
  if (family == "mobile_manipulator") {
    const int flange = arm(b, chassis, {0, 0, top},
                           config.reach > 0.0 ? config.reach : 0.8 * random.vary(0.08),
                           kOrange, kWhite);
    if (config.gripper)
      gripper(b, flange, {0.02, 0, 0}, kGraphite, kWhite);
    if (config.camera) {
      const int mast = b.link("camera mast", chassis, {-0.14, 0, top}, fixed(),
                              {rod(0.015, {}, {0, 0, 0.42}, "cylinder")}, kSteel);
      camera(b, mast, {0.02, 0, 0.44});
    }
  }
}

void drone(Builder &b, Random &random, Color accent, int rotors = 4) {
  const double arm = (rotors > 4 ? 0.36 : 0.3) * random.vary(0.12);
  std::vector<Shape> body{box({0.22, 0.22, 0.08}),
                          box({0.18, 0.14, 0.03}, {0, 0, 0.055}).tint(accent)};
  for (int side = -1; side <= 1; side += 2) {
    body.push_back(rod(0.01, {0.06, side * 0.08, -0.04}, {0.08, side * 0.12, -0.16}, "cylinder").tint(kSteel));
    body.push_back(rod(0.01, {-0.06, side * 0.08, -0.04}, {-0.08, side * 0.12, -0.16}, "cylinder").tint(kSteel));
    body.push_back(rod(0.012, {-0.16, side * 0.12, -0.17}, {0.16, side * 0.12, -0.17}).touching().tint(kSteel));
  }
  const int frame = b.link("body", -1, {}, fixed(), body, kGraphite);
  for (int index = 0; index < rotors; ++index) {
    const double angle = kPi / rotors + 2.0 * kPi * index / rotors;
    const Vec tip{arm * std::cos(angle), arm * std::sin(angle), 0.0};
    const int boom = b.link("arm " + std::to_string(index + 1), frame, {}, fixed(),
                            {rod(0.012, {}, tip, "cylinder"),
                             disc(0.025, 0.04, kZ, tip + Vec{0, 0, 0.02}).tint(kGraphite)},
                            kGraphite);
    b.link("rotor " + std::to_string(index + 1), boom, tip + Vec{0, 0, 0.045},
           continuous(kZ, 2.0),
           {disc(0.12, 0.006, kZ), disc(0.012, 0.02, kZ).tint(kSteel)},
           index % 2 ? accent : kWhite);
  }
  camera(b, frame, {0.12, 0, -0.03});
  imu(b, frame, {0, 0, 0});
}

void auv(Builder &b, Random &random, Color accent) {
  const double hull = 0.9 * random.vary(0.15);
  const int body = b.link(
      "hull", -1, {}, fixed(),
      {rod(0.1, {-hull * 0.5, 0, 0}, {hull * 0.5, 0, 0}),
       disc(0.102, 0.04, kX, {hull * 0.2, 0, 0}).tint(accent),
       box({0.12, 0.05, 0.03}, {hull * 0.35, 0, -0.1}).tint(kGraphite)},
      kYellow);
  b.link("thruster", body, {-hull * 0.5 - 0.12, 0, 0}, continuous(kX, 5.0),
         {disc(0.07, 0.07, kX).tint(kGraphite), disc(0.02, 0.1, kX).tint(kSteel)},
         kGraphite);
  for (int side = -1; side <= 1; side += 2)
    b.link(side > 0 ? "left dive plane" : "right dive plane", body,
           {hull * 0.25, side * 0.1, 0}, revolute(kY, -0.5, 0.5, 4),
           {box({0.1, 0.12, 0.012}, {0, side * 0.06, 0})}, kGraphite);
  b.link("rudder", body, {-hull * 0.38, 0, 0.09}, revolute(kZ, -0.5, 0.5, 4),
         {box({0.1, 0.012, 0.12}, {0, 0, 0.06})}, kGraphite);
  b.link("elevator", body, {-hull * 0.38, 0, -0.09}, revolute(kZ, -0.5, 0.5, 4),
         {box({0.1, 0.012, 0.1}, {0, 0, -0.05})}, kGraphite);
  camera(b, body, {hull * 0.5 + 0.08, 0, -0.03});
  imu(b, body, {0, 0, 0});
}

int quadruped(Builder &b, Random &random, Color accent) {
  const double leg = 0.24 * b.config().limb * random.vary(0.12);
  const int body = b.link(
      "body", -1, {}, fixed(),
      {box({0.6, 0.28, 0.14}), box({0.4, 0.2, 0.03}, {0, 0, 0.085}).tint(accent),
       box({0.08, 0.16, 0.08}, {0.32, 0, 0.0}).tint(kGraphite)},
      kWhite);
  for (int index = 0; index < 4; ++index) {
    const double x = index < 2 ? 0.24 : -0.24;
    const double side = index % 2 ? -1.0 : 1.0;
    const std::string name = std::string(index < 2 ? "front " : "rear ") +
                             (side > 0 ? "left" : "right");
    const int hip = b.link(name + " hip", body, {x, side * 0.14, 0},
                           revolute(kX, -0.8, 0.8, 30),
                           {disc(0.045, 0.07, kX)}, kGraphite);
    const int thigh = b.link(name + " thigh", hip, {0, side * 0.06, 0},
                             revolute(kY, -1.6, 1.6, 40),
                             {disc(0.04, 0.06, kY).tint(kGraphite),
                              rod(0.035, {}, {0, 0, -leg})},
                             accent);
    b.link(name + " shin", thigh, {0, 0, -leg}, revolute(kY, -2.6, 0.1, 40),
           {rod(0.022, {}, {0, 0, -leg}), ball(0.03, {0, 0, -leg}).touching().tint(kRubber)},
           kWhite);
  }
  camera(b, body, {0.36, 0, 0.02});
  imu(b, body, {0, 0, 0});
  return body;
}

void hexapod(Builder &b, Random &random, Color accent, int perSide = 3) {
  const double femur = 0.14 * b.config().limb * random.vary(0.15);
  const double tibia = 0.24 * b.config().limb * random.vary(0.15);
  const int body = b.link("body", -1, {}, fixed(),
                          {box({0.5, 0.26, 0.09}), disc(0.1, 0.03, kZ, {0, 0, 0.06}).tint(accent)},
                          kGraphite);
  const double span = perSide > 3 ? 0.42 : 0.36;
  for (int index = 0; index < perSide * 2; ++index) {
    const double x = -span * 0.5 + span * (index / 2) / (perSide - 1);
    const double side = index % 2 ? -1.0 : 1.0;
    const std::string name = "leg " + std::to_string(index + 1);
    const int coxa = b.link(name + " coxa", body, {x, side * 0.13, 0},
                            revolute(kZ, -0.8, 0.8, 12),
                            {rod(0.025, {}, {0, side * 0.07, 0}, "cylinder")}, kGraphite);
    const int upper = b.link(name + " femur", coxa, {0, side * 0.07, 0},
                             revolute(kX, -1.4, 1.4, 15),
                             {rod(0.02, {}, {0, side * femur, 0.06})}, accent);
    b.link(name + " tibia", upper, {0, side * femur, 0.06},
           revolute(kX, -2.0, 2.0, 15),
           {rod(0.016, {}, {0, side * 0.04, -tibia}),
            ball(0.022, {0, side * 0.04, -tibia}).touching().tint(kRubber)},
           kSteel);
  }
  camera(b, body, {0.26, 0, 0.02});
  imu(b, body, {0, 0, 0});
}

void snake(Builder &b, Random &random, Color accent) {
  const int count =
      b.config().segments > 0 ? b.config().segments : random.range(30, 40);
  int parent = b.link("head", -1, {}, fixed(),
                      {box({0.14, 0.11, 0.08}), ball(0.015, {0.06, 0.035, 0.03}).tint(kLens),
                       ball(0.015, {0.06, -0.035, 0.03}).tint(kLens)},
                      accent);
  camera(b, parent, {0.08, 0, 0});
  Vec at{-0.09, 0, 0};
  for (int index = 0; index < count; ++index) {
    const Vec axis = index % 2 ? kY : kZ;
    parent = b.link("segment " + std::to_string(index + 1), parent, at,
                    revolute(axis, -1.0, 1.0, 8),
                    {disc(0.045, 0.03, kX).tint(kGraphite),
                     rod(0.04, {-0.03, 0, 0}, {-0.12, 0, 0}).touching()},
                    index % 4 < 2 ? kWhite : accent);
    at = {-0.15, 0, 0};
  }
}

void soft(Builder &b, Random &random, Color accent) {
  const int count =
      b.config().segments > 0 ? b.config().segments : random.range(9, 13);
  int parent = b.link("base", -1, {}, fixed(),
                      {disc(0.12, 0.08, kZ, {0, 0, 0.04}).touching()}, kGraphite);
  Vec at{0, 0, 0.08};
  for (int index = 0; index < count; ++index) {
    const double taper = 1.0 - 0.04 * index;
    parent = b.link("segment " + std::to_string(index + 1), parent, at,
                    revolute(index % 2 ? kX : kY, -0.35, 0.35, 6 * taper),
                    {disc(0.06 * taper, 0.015, kZ, {0, 0, 0.0075}).tint(accent),
                     rod(0.025 * taper, {0, 0, 0.015}, {0, 0, 0.075}, "cylinder")},
                    kWhite);
    at = {0, 0, 0.08};
  }
  b.link("tip", parent, at, fixed(), {ball(0.03, {0, 0, 0.02})}, accent);
}

int leg(Builder &b, int parent, const std::string &side, double sign,
        Vec hip, double thigh, double shin, Color color, bool brace) {
  const int upper = b.link(side + " thigh", parent, hip,
                           revolute(kY, -1.6, 1.2, 80),
                           {ball(0.065).tint(kGraphite),
                            brace ? box({0.04, 0.03, thigh}, {0, sign * 0.07, -thigh * 0.5})
                                  : rod(0.06, {}, {0, 0, -thigh})},
                           color);
  const int lower = b.link(side + " shin", upper, {0, 0, -thigh},
                           revolute(kY, 0.0, 2.4, 60),
                           {ball(0.05).tint(kGraphite),
                            brace ? box({0.035, 0.03, shin}, {0, sign * 0.07, -shin * 0.5})
                                  : rod(0.05, {}, {0, 0, -shin})},
                           color);
  return b.link(side + " foot", lower, {0, 0, -shin},
                revolute(kY, -0.8, 0.8, 30, !brace),
                {box({0.24, 0.11, 0.05}, {0.05, 0, -0.05}).touching()}, kGraphite);
}

void humanArms(Builder &b, int torso, double y, double z);

void humanoid(Builder &b, Random &random, Color accent, std::string_view family) {
  const bool exo = family == "exoskeleton";
  const bool companion = family == "companion";
  if (companion) {
    const int body = b.link("body", -1, {}, fixed(),
                            {ball(0.16), disc(0.17, 0.03, kZ, {0, 0, -0.06}).tint(accent)},
                            kWhite);
    wheel(b, body, "left wheel", {0, 0.12, -0.13}, 0.05, 0.03);
    wheel(b, body, "right wheel", {0, -0.12, -0.13}, 0.05, 0.03);
    caster(b, body, "front caster", {0.1, 0, -0.155}, 0.025);
    caster(b, body, "rear caster", {-0.1, 0, -0.155}, 0.025);
    const int pan = b.link("neck", body, {0, 0, 0.16}, revolute(kZ, -2.0, 2.0, 3),
                           {disc(0.04, 0.04, kZ, {0, 0, 0.02})}, kGraphite);
    const int head = b.link("head", pan, {0, 0, 0.12}, revolute(kY, -0.5, 0.6, 2),
                            {ball(0.12 * random.vary(0.1)),
                             ball(0.025, {0.1, 0.045, 0.02}).tint(kLens),
                             ball(0.025, {0.1, -0.045, 0.02}).tint(kLens)},
                            kWhite);
    for (int side = -1; side <= 1; side += 2) {
      b.link(side > 0 ? "left antenna" : "right antenna", head,
             {0, side * 0.06, 0.1}, revolute(kX, -0.6, 0.6, 0.5),
             {rod(0.008, {}, {0, side * 0.03, 0.12}, "cylinder"),
              ball(0.02, {0, side * 0.03, 0.12}).tint(accent)},
             kSteel);
      b.link(side > 0 ? "left flipper" : "right flipper", body,
             {0, side * 0.16, 0.03}, revolute(kY, -1.5, 1.5, 2),
             {rod(0.03, {}, {0, side * 0.02, -0.12})}, accent);
    }
    camera(b, head, {0.11, 0, 0.07});
    imu(b, body, {0, 0, 0});
    return;
  }
  const double tall = b.config().limb * random.vary(0.08);
  const int pelvis = b.link("pelvis", -1, {}, fixed(),
                            {box({0.22, exo ? 0.42 : 0.3, exo ? 0.08 : 0.12})},
                            exo ? kGraphite : kWhite);
  leg(b, pelvis, "left", 1.0, {0, exo ? 0.2 : 0.1, -0.06}, 0.4 * tall, 0.38 * tall,
      exo ? accent : kWhite, exo);
  leg(b, pelvis, "right", -1.0, {0, exo ? -0.2 : -0.1, -0.06}, 0.4 * tall, 0.38 * tall,
      exo ? accent : kWhite, exo);
  if (exo) {
    b.link("back frame", pelvis, {-0.14, 0, 0.04}, fixed(),
           {box({0.04, 0.3, 0.5}, {0, 0, 0.25}),
            box({0.08, 0.2, 0.25}, {-0.05, 0, 0.18}).tint(kGraphite),
            box({0.22, 0.04, 0.04}, {0.11, 0.17, 0.48}).tint(accent),
            box({0.22, 0.04, 0.04}, {0.11, -0.17, 0.48}).tint(accent)},
           kSteel);
    imu(b, pelvis, {0, 0, 0});
    return;
  }
  const int torso = b.link("torso", pelvis, {0, 0, 0.06}, revolute(kZ, -1.0, 1.0, 60),
                           {box({0.22, 0.36, 0.42}, {0, 0, 0.23}),
                            box({0.02, 0.24, 0.18}, {0.11, 0, 0.28}).tint(accent)},
                           kWhite);
  const int neck = b.link("neck", torso, {0, 0, 0.44}, revolute(kZ, -1.4, 1.4, 6),
                          {rod(0.03, {}, {0, 0, 0.06}, "cylinder").tint(kGraphite)},
                          kGraphite);
  const int head = b.link("head", neck, {0, 0, 0.06}, revolute(kY, -0.5, 0.6, 4),
                          {ball(0.11, {0, 0, 0.1}),
                           box({0.03, 0.16, 0.05}, {0.095, 0, 0.11}).tint(kLens)},
                          kWhite);
  camera(b, head, {0.1, 0, 0.17});
  humanArms(b, torso, 0.23, 0.38);
  imu(b, torso, {0, 0, 0.2});
}

void humanArms(Builder &b, int torso, double y, double z) {
  for (int side = -1; side <= 1; side += 2) {
    const std::string label = side > 0 ? "left" : "right";
    const int shoulder = b.link(label + " upper arm", torso, {0, side * y, z},
                                revolute(kY, -3.0, 1.0, 40),
                                {ball(0.06).tint(kGraphite), rod(0.045, {}, {0, 0, -0.26})},
                                kWhite);
    const int fore = b.link(label + " forearm", shoulder, {0, 0, -0.28},
                            revolute(kY, -2.4, 0.0, 25),
                            {ball(0.045).tint(kGraphite), rod(0.04, {}, {0, 0, -0.24})},
                            kWhite);
    b.link(label + " hand", fore, {0, 0, -0.28}, revolute(kX, -1.5, 1.5, 4),
           {box({0.05, 0.08, 0.1}, {0, 0, -0.05})}, kGraphite);
  }
}

int wheelOn(Builder &b, int parent, const std::string &name, Vec at,
            double radius, double width, Vec axis) {
  return b.link(name, parent, at, continuous(axis, 30.0),
                {disc(radius, width, axis).touching().tint(kRubber),
                 disc(radius * 0.55, width * 1.08, axis).tint(kSteel)},
                kRubber);
}

int wheeledChassis(Builder &b, BaseSize s, int axles, bool outside,
                   std::vector<Shape> extra, Color body, double width) {
  std::vector<Shape> shapes{box({s.length, s.width, s.height})};
  for (Shape &shape : extra)
    shapes.push_back(std::move(shape));
  const int chassis = b.link("chassis", -1, {}, fixed(), shapes, body);
  const double x = s.length * 0.5 - s.wheel - 0.04;
  const double y = outside ? s.width * 0.5 + width * 0.5 + 0.01
                           : s.width * 0.5 - width * 0.5 - 0.02;
  const double z = -(s.height * 0.5 + s.clearance) + s.wheel;
  for (int axle = 0; axle < axles; ++axle) {
    const double at = axles == 1 ? 0.0 : x - 2.0 * x * axle / (axles - 1);
    for (int side = -1; side <= 1; side += 2)
      wheel(b, chassis,
            std::string(side > 0 ? "left" : "right") + " wheel " + std::to_string(axle + 1),
            {at, side * y, z}, s.wheel, width);
  }
  return chassis;
}

void pedestalArm(Builder &b, Random &random, Color accent, double reach,
                 bool redundant, double pedestal) {
  const int base = b.link("pedestal", -1, {}, fixed(),
                          {disc(pedestal, pedestal * 0.8, kZ, {0, 0, pedestal * 0.4}).touching()},
                          kGraphite);
  const double length = b.config().reach > 0.0 ? b.config().reach : reach * random.vary(0.1);
  const int flange = arm(b, base, {0, 0, pedestal * 0.8}, length, kWhite, accent, redundant);
  if (b.config().gripper)
    gripper(b, flange, {0.02, 0, 0}, kGraphite, kSteel);
}

void dualArm(Builder &b, Random &random, Color accent) {
  const int base = b.link("pedestal", -1, {}, fixed(),
                          {disc(0.25, 0.08, kZ, {0, 0, 0.04}).touching(),
                           rod(0.07, {0, 0, 0.08}, {0, 0, 0.75}, "cylinder")},
                          kGraphite);
  const int torso = b.link("torso", base, {0, 0, 0.75}, revolute(kZ, -2.5, 2.5, 80),
                           {box({0.24, 0.5, 0.3}, {0, 0, 0.15}),
                            box({0.12, 0.08, 0.06}, {0, 0.27, 0.15}).tint(kGraphite),
                            box({0.12, 0.08, 0.06}, {0, -0.27, 0.15}).tint(kGraphite)},
                           kWhite);
  const int head = b.link("head", torso, {0, 0, 0.3}, revolute(kZ, -1.5, 1.5, 5),
                          {box({0.14, 0.2, 0.12}, {0, 0, 0.08}),
                           box({0.01, 0.16, 0.05}, {0.071, 0, 0.09}).tint(kLens)},
                          kWhite);
  camera(b, head, {0.08, 0, 0.16});
  const double reach = b.config().reach > 0.0 ? b.config().reach : 0.7 * random.vary(0.1);
  for (int side = -1; side <= 1; side += 2) {
    const int flange = arm(b, torso, {0, side * 0.31, 0.12}, reach, kWhite, accent);
    if (b.config().gripper)
      gripper(b, flange, {0.02, 0, 0}, kGraphite, kSteel);
  }
  imu(b, torso, {0, 0, 0.1});
}

void palletizer(Builder &b, Random &random, Color accent) {
  const double total = b.config().reach > 0.0 ? b.config().reach : 1.8 * random.vary(0.1);
  const double upper = total * 0.5;
  const double fore = total * 0.5;
  const int base = b.link("base", -1, {}, fixed(),
                          {box({0.6, 0.6, 0.15}, {0, 0, 0.075}).touching(),
                           disc(0.25, 0.1, kZ, {0, 0, 0.2})},
                          kGraphite);
  const int turret = b.link("turret", base, {0, 0, 0.25}, revolute(kZ, -3.0, 3.0, 400),
                            {box({0.4, 0.35, 0.25}, {0, 0, 0.125})}, kOrange);
  const int main = b.link("main arm", turret, {0.05, 0, 0.25}, revolute(kY, -1.2, 1.0, 500),
                          {disc(0.11, 0.3, kY).tint(kGraphite),
                           box({0.12, 0.16, upper}, {0, 0, upper * 0.5})},
                          kOrange);
  const int forearm = b.link("forearm", main, {0, 0, upper}, revolute(kY, -1.5, 1.5, 300),
                             {disc(0.09, 0.24, kY).tint(kGraphite),
                              box({fore, 0.12, 0.1}, {fore * 0.5, 0, 0})},
                             kOrange);
  const int wrist = b.link("wrist", forearm, {fore, 0, 0}, revolute(kZ, -3.1, 3.1, 60),
                           {disc(0.07, 0.1, kZ, {0, 0, -0.05}).tint(kGraphite)}, kOrange);
  std::vector<Shape> plate{box({0.3, 0.3, 0.03}, {0, 0, -0.015})};
  for (int index = 0; index < 4; ++index)
    plate.push_back(disc(0.04, 0.02, kZ, {index < 2 ? -0.09 : 0.09, index % 2 ? -0.09 : 0.09, -0.04})
                        .tint(accent));
  b.link("suction plate", wrist, {0, 0, -0.1}, fixed(), plate, kSteel);
  imu(b, base, {0, 0, 0.1});
}

void panTilt(Builder &b, Random &, Color accent) {
  const int base = b.link("base", -1, {}, fixed(),
                          {box({0.14, 0.14, 0.06}, {0, 0, 0.03}).touching()}, kGraphite);
  const int pan = b.link("pan", base, {0, 0, 0.06}, revolute(kZ, -3.0, 3.0, 4),
                         {disc(0.05, 0.03, kZ, {0, 0, 0.015}),
                          box({0.04, 0.02, 0.12}, {0, 0.12, 0.08}),
                          box({0.04, 0.02, 0.12}, {0, -0.12, 0.08}),
                          box({0.04, 0.26, 0.02}, {0, 0, 0.03})},
                         accent);
  const int tilt = b.link("tilt", pan, {0, 0, 0.11}, revolute(kY, -1.2, 1.2, 3),
                          {box({0.12, 0.2, 0.08}),
                           disc(0.03, 0.01, kX, {0.065, 0.05, 0}).tint(kLens),
                           disc(0.03, 0.01, kX, {0.065, -0.05, 0}).tint(kLens)},
                          kGraphite);
  camera(b, tilt, {0.06, 0, 0.06});
  lidar(b, tilt, {0, 0, 0.04});
  imu(b, base, {0, 0, 0.03});
}

void linearRig(Builder &b, Random &random, Color accent) {
  const double travel = 0.4 * random.vary(0.15);
  const int rail = b.link("rail", -1, {}, fixed(),
                          {box({travel * 2.0 + 0.3, 0.12, 0.06}, {0, 0, 0.03}).touching(),
                           box({0.04, 0.16, 0.1}, {travel + 0.17, 0, 0.05}).tint(accent),
                           box({0.04, 0.16, 0.1}, {-travel - 0.17, 0, 0.05}).tint(accent)},
                          kSteel);
  const int carriage = b.link("carriage", rail, {0, 0, 0.06}, prismatic(kX, -travel, travel, 150),
                              {box({0.15, 0.14, 0.06}, {0, 0, 0.03}),
                               box({0.06, 0.06, 0.45}, {-0.05, 0, 0.285})},
                              kGraphite);
  const int slide = b.link("Z slide", carriage, {0.01, 0, 0.12}, prismatic(kZ, 0.0, 0.3, 80),
                           {box({0.05, 0.1, 0.08}, {0, 0, 0.04})}, accent);
  b.link("suction cup", slide, {0.05, 0, 0.0}, fixed(),
         {rod(0.01, {}, {0.08, 0, 0}, "cylinder"), disc(0.03, 0.02, kX, {0.09, 0, 0}).tint(kRubber)},
         kSteel);
}

void gantryPicker(Builder &b, Random &random, Color accent) {
  const int ram = cartesian(b, random, accent);
  const int roll = b.link("tool roll", ram, {0, 0, -0.2}, revolute(kZ, -3.1, 3.1, 10),
                          {disc(0.04, 0.04, kZ, {0, 0, -0.02})}, kGraphite);
  b.link("suction cup", roll, {0, 0, -0.04}, fixed(),
         {disc(0.05, 0.02, kZ, {0, 0, -0.01}).tint(kRubber)}, kRubber);
}

void hand(Builder &b, Random &random, Color accent) {
  const double size = random.vary(0.08);
  const int wrist = b.link("wrist", -1, {}, fixed(),
                           {disc(0.045, 0.06, kZ, {0, 0, 0.03}).touching()}, kGraphite);
  const int palm = b.link("palm", wrist, {0, 0, 0.06}, revolute(kY, -0.8, 0.8, 8),
                          {box({0.09, 0.03, 0.1}, {0, 0, 0.05})}, kWhite);
  const std::array<double, 4> lengths{0.04, 0.045, 0.042, 0.035};
  for (int finger = 0; finger < 4; ++finger) {
    int parent = palm;
    Vec at{-0.033 + 0.022 * finger, 0, 0.1};
    for (int phalanx = 0; phalanx < 3; ++phalanx) {
      const double length = lengths[finger] * size * (1.0 - 0.22 * phalanx);
      parent = b.link("finger " + std::to_string(finger + 1) + " phalanx " +
                          std::to_string(phalanx + 1),
                      parent, at, revolute(kX, 0.0, 1.6, 2),
                      {ball(0.011).tint(accent), rod(0.009, {}, {0, 0, length})}, kWhite);
      at = {0, 0, length};
    }
  }
  const int thumb = b.link("thumb base", palm, {-0.045, -0.01, 0.03}, revolute(kZ, -0.3, 1.4, 2),
                           {ball(0.013).tint(accent), rod(0.01, {}, {-0.03, 0, 0.02})}, kWhite);
  const int thumbMid = b.link("thumb middle", thumb, {-0.03, 0, 0.02}, revolute(kY, -1.2, 0.2, 2),
                              {ball(0.011).tint(accent), rod(0.009, {}, {-0.015, 0, 0.03})}, kWhite);
  b.link("thumb tip", thumbMid, {-0.015, 0, 0.03}, revolute(kY, -1.2, 0.2, 1),
         {ball(0.01).tint(accent), rod(0.008, {}, {-0.008, 0, 0.025})}, kWhite);
}

void forklift(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {1.0, 0.6, 0.3, 0.12, 0.05});
  const double top = s.height * 0.5;
  const double front = s.length * 0.5;
  const double bottom = -(top + s.clearance);
  const int chassis = wheeledChassis(
      b, s, 2, true,
      {box({0.25, s.width, 0.25}, {-front + 0.125, 0, top + 0.125}).tint(kGraphite),
       rod(0.02, {front - 0.05, s.width * 0.4, top}, {front - 0.05, s.width * 0.4, top + 1.0}, "cylinder").tint(kSteel),
       rod(0.02, {front - 0.05, -s.width * 0.4, top}, {front - 0.05, -s.width * 0.4, top + 1.0}, "cylinder").tint(kSteel),
       rod(0.02, {-front + 0.3, s.width * 0.4, top}, {-front + 0.3, s.width * 0.4, top + 1.0}, "cylinder").tint(kSteel),
       rod(0.02, {-front + 0.3, -s.width * 0.4, top}, {-front + 0.3, -s.width * 0.4, top + 1.0}, "cylinder").tint(kSteel),
       box({s.length - 0.25, s.width * 0.85, 0.03}, {0.075, 0, top + 1.0}).tint(kGraphite),
       box({0.06, 0.08, 1.2}, {front + 0.03, s.width * 0.3, top + 0.6 - s.height}).tint(kSteel),
       box({0.06, 0.08, 1.2}, {front + 0.03, -s.width * 0.3, top + 0.6 - s.height}).tint(kSteel)},
      kYellow, 0.08);
  b.link("fork carriage", chassis, {front + 0.08, 0, bottom + 0.02}, prismatic(kZ, 0.0, 0.9, 800),
         {box({0.04, s.width * 0.8, 0.4}, {0, 0, 0.2}),
          box({0.6, 0.08, 0.03}, {0.3, s.width * 0.22, 0.015}),
          box({0.6, 0.08, 0.03}, {0.3, -s.width * 0.22, 0.015})},
         kGraphite);
  b.link("beacon", chassis, {-front + 0.4, 0, top + 1.0}, fixed(),
         {disc(0.04, 0.06, kZ, {0, 0, 0.03})}, accent);
  lidar(b, chassis, {-front + 0.15, 0, top + 1.015});
  camera(b, chassis, {front - 0.02, 0, top + 0.95});
  imu(b, chassis, {0, 0, 0});
}

void tugger(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.7, 0.5, 0.35, 0.1, 0.05});
  const int chassis = wheeledChassis(
      b, s, 2, true,
      {box({s.length + 0.01, s.width + 0.01, 0.05}, {0, 0, -s.height * 0.2}).tint(accent),
       rod(0.025, {-s.length * 0.5, 0, -s.height * 0.2}, {-s.length * 0.5 - 0.12, 0, -s.height * 0.2}, "cylinder").tint(kSteel)},
      kGraphite, 0.06);
  b.link("hitch pin", chassis, {-s.length * 0.5 - 0.12, 0, -s.height * 0.2}, fixed(),
         {rod(0.015, {0, 0, -0.06}, {0, 0, 0.06}, "cylinder")}, kSteel);
  b.link("beacon", chassis, {-s.length * 0.3, 0, s.height * 0.5}, fixed(),
         {disc(0.035, 0.07, kZ, {0, 0, 0.035})}, kYellow);
  lidar(b, chassis, {s.length * 0.38, 0, s.height * 0.5});
  camera(b, chassis, {s.length * 0.5 + 0.02, 0, 0.05});
  imu(b, chassis, {0, 0, 0});
}

void omniBase(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.6, 0.5, 0.15, 0.08, 0.04});
  const int chassis = wheeledChassis(
      b, s, 2, true, {box({s.length * 0.8, s.width * 0.8, 0.02}, {0, 0, s.height * 0.5 + 0.01}).tint(accent)},
      kGraphite, 0.06);
  lidar(b, chassis, {s.length * 0.35, 0, s.height * 0.5 + 0.02});
  camera(b, chassis, {s.length * 0.5 + 0.02, 0, 0});
  imu(b, chassis, {0, 0, 0});
}

int trackedBase(Builder &b, Random &, Color accent, BaseSize fallback) {
  const BaseSize s = baseSize(b.config(), fallback);
  const double bottom = -(s.height * 0.5 + s.clearance);
  const double trackHeight = s.wheel * 2.2;
  std::vector<Shape> shapes{box({s.length, s.width, s.height}),
                            box({s.length * 0.7, s.width * 0.7, 0.03}, {0, 0, s.height * 0.5 + 0.015}).tint(accent)};
  for (int side = -1; side <= 1; side += 2)
    shapes.push_back(box({s.length + 0.1, 0.12, trackHeight},
                         {0, side * (s.width * 0.5 + 0.07), bottom + trackHeight * 0.5})
                         .touching()
                         .tint(kRubber));
  const int chassis = b.link("chassis", -1, {}, fixed(), shapes, kGraphite);
  for (int side = -1; side <= 1; side += 2)
    for (int end = -1; end <= 1; end += 2)
      wheelOn(b, chassis,
              std::string(side > 0 ? "left" : "right") + (end > 0 ? " front" : " rear") + " sprocket",
              {end * (s.length * 0.5 - s.wheel * 0.6), side * (s.width * 0.5 + 0.07), bottom + trackHeight * 0.5},
              s.wheel, 0.13, kY);
  imu(b, chassis, {0, 0, 0});
  return chassis;
}

void tracked(Builder &b, Random &random, Color accent) {
  const int chassis = trackedBase(b, random, accent, {0.8, 0.5, 0.25, 0.1, 0.08});
  lidar(b, chassis, {0.25, 0, 0.14});
  camera(b, chassis, {0.42, 0, 0.0});
}

void inspectionCrawler(Builder &b, Random &random, Color accent) {
  const int chassis = trackedBase(b, random, accent, {0.45, 0.3, 0.12, 0.06, 0.03});
  const int mast = b.link("mast", chassis, {0.05, 0, 0.06}, prismatic(kZ, 0.0, 0.25, 40),
                          {rod(0.015, {}, {0, 0, 0.3}, "cylinder")}, kSteel);
  const int pan = b.link("camera pan", mast, {0, 0, 0.3}, revolute(kZ, -3.0, 3.0, 2),
                         {disc(0.03, 0.03, kZ, {0, 0, 0.015})}, kGraphite);
  const int tilt = b.link("camera tilt", pan, {0, 0, 0.05}, revolute(kY, -1.4, 1.4, 2),
                          {box({0.08, 0.12, 0.06}), disc(0.015, 0.01, kZ, {0, 0.045, 0.035}).tint(kYellow),
                           disc(0.015, 0.01, kZ, {0, -0.045, 0.035}).tint(kYellow)},
                          accent);
  camera(b, tilt, {0.05, 0, 0});
  lidar(b, chassis, {-0.12, 0, 0.06});
}

void sixWheel(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.9, 0.55, 0.25, 0.13, 0.15});
  const int chassis = wheeledChassis(
      b, s, 3, true,
      {box({s.length * 0.7, s.width * 0.8, 0.04}, {0, 0, s.height * 0.5 + 0.02}).tint(accent),
       rod(0.01, {-s.length * 0.4, s.width * 0.35, s.height * 0.5}, {-s.length * 0.4, s.width * 0.35, s.height * 0.5 + 0.5}, "cylinder").tint(kSteel),
       rod(0.02, {s.length * 0.35, 0, s.height * 0.5}, {s.length * 0.35, 0, s.height * 0.5 + 0.35}, "cylinder").tint(kSteel)},
      kGraphite, 0.09);
  camera(b, chassis, {s.length * 0.35, 0, s.height * 0.5 + 0.37});
  lidar(b, chassis, {0, 0, s.height * 0.5 + 0.04});
  imu(b, chassis, {0, 0, 0});
}

void lawnMower(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.6, 0.45, 0.2, 0.14, 0.04});
  const double bottom = -(s.height * 0.5 + s.clearance);
  const int chassis = b.link(
      "chassis", -1, {}, fixed(),
      {box({s.length, s.width, s.height}), box({0.04, s.width + 0.02, s.height * 0.6}, {s.length * 0.5, 0, 0}).tint(accent),
       rod(0.008, {-s.length * 0.3, 0, s.height * 0.5}, {-s.length * 0.3, 0, s.height * 0.5 + 0.15}, "cylinder").tint(kSteel),
       ball(0.03, {-s.length * 0.3, 0, s.height * 0.5 + 0.17}).tint(kWhite)},
      {0.25, 0.55, 0.25});
  for (int side = -1; side <= 1; side += 2) {
    wheel(b, chassis, side > 0 ? "left wheel" : "right wheel",
          {-s.length * 0.3, side * (s.width * 0.5 + 0.04), bottom + s.wheel}, s.wheel, 0.06);
    caster(b, chassis, side > 0 ? "left caster" : "right caster",
           {s.length * 0.35, side * s.width * 0.35, bottom + 0.035}, 0.035);
  }
  b.link("blade", chassis, {0.05, 0, -s.height * 0.5}, continuous(kZ, 10.0),
         {disc(std::min(0.18, s.width * 0.4), 0.008, kZ, {0, 0, -std::min(0.02, s.clearance * 0.5)}).tint(kSteel)},
         kSteel);
  camera(b, chassis, {s.length * 0.5 + 0.03, 0, 0.04});
  imu(b, chassis, {0, 0, 0});
}

void vacuum(Builder &b, Random &, Color accent) {
  const int body = b.link("body", -1, {}, fixed(),
                          {disc(0.17, 0.08, kZ), disc(0.172, 0.03, kZ).tint(accent)}, kWhite);
  wheel(b, body, "left wheel", {0, 0.11, -0.015}, 0.035, 0.02);
  wheel(b, body, "right wheel", {0, -0.11, -0.015}, 0.035, 0.02);
  caster(b, body, "front caster", {0.12, 0, -0.04}, 0.01);
  b.link("side brush", body, {0.12, 0.1, -0.035}, continuous(kZ, 1.0),
         {disc(0.045, 0.005, kZ)}, accent);
  lidar(b, body, {0.05, 0, 0.04});
  imu(b, body, {0, 0, 0});
}

void delivery(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.7, 0.55, 0.25, 0.08, 0.08});
  const double top = s.height * 0.5;
  const int chassis = wheeledChassis(
      b, s, 3, true,
      {box({s.length * 0.9, s.width * 0.9, 0.35}, {0, 0, top + 0.175}).tint(kWhite),
       box({0.01, s.width * 0.6, 0.08}, {s.length * 0.45, 0, top + 0.25}).tint(kLens),
       rod(0.006, {-s.length * 0.4, s.width * 0.35, top + 0.35}, {-s.length * 0.4, s.width * 0.35, top + 1.0}, "cylinder").tint(kSteel),
       box({0.12, 0.005, 0.08}, {-s.length * 0.4 + 0.06, s.width * 0.35, top + 0.96}).tint(accent)},
      kGraphite, 0.05);
  b.link("lid", chassis, {-s.length * 0.45, 0, top + 0.35}, revolute(kY, -1.8, 0.0, 15),
         {box({s.length * 0.9, s.width * 0.9, 0.03}, {s.length * 0.45, 0, 0.015})}, accent);
  lidar(b, chassis, {s.length * 0.3, 0, top + 0.38});
  camera(b, chassis, {s.length * 0.46, 0, top + 0.15});
  imu(b, chassis, {0, 0, 0});
}

void telepresence(Builder &b, Random &random, Color accent) {
  const double height = 1.1 * random.vary(0.08);
  const int base = b.link("base", -1, {}, fixed(),
                          {disc(0.2, 0.1, kZ), disc(0.202, 0.02, kZ).tint(accent),
                           rod(0.025, {0, 0, 0.05}, {0, 0, height}, "cylinder").tint(kSteel)},
                          kWhite);
  wheel(b, base, "left wheel", {0, 0.15, -0.05}, 0.05, 0.03);
  wheel(b, base, "right wheel", {0, -0.15, -0.05}, 0.05, 0.03);
  caster(b, base, "front caster", {0.14, 0, -0.08}, 0.02);
  caster(b, base, "rear caster", {-0.14, 0, -0.08}, 0.02);
  const int screen = b.link("screen", base, {0, 0, height + 0.05}, revolute(kY, -0.5, 0.5, 3),
                            {box({0.03, 0.32, 0.22}), box({0.005, 0.28, 0.18}, {0.016, 0, 0}).tint(kLens)},
                            kGraphite);
  camera(b, screen, {0.02, 0, 0.13});
  lidar(b, base, {0.13, 0, 0.05});
  imu(b, base, {0, 0, 0});
}

void warehousePicker(Builder &b, Random &random, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.9, 0.7, 0.3, 0.1, 0.05});
  const double top = s.height * 0.5;
  std::vector<Shape> frame;
  for (int level = 0; level < 3; ++level)
    frame.push_back(box({0.3, s.width * 0.9, 0.02}, {-s.length * 0.3, 0, top + 0.25 + 0.3 * level}).tint(kSteel));
  frame.push_back(rod(0.015, {-s.length * 0.45, s.width * 0.42, top}, {-s.length * 0.45, s.width * 0.42, top + 0.9}, "cylinder").tint(kSteel));
  frame.push_back(rod(0.015, {-s.length * 0.45, -s.width * 0.42, top}, {-s.length * 0.45, -s.width * 0.42, top + 0.9}, "cylinder").tint(kSteel));
  const int chassis = wheeledChassis(b, s, 2, false, frame, kGraphite, 0.06);
  const int lift = b.link("lift", chassis, {s.length * 0.15, 0, top}, prismatic(kZ, 0.0, 0.6, 300),
                          {rod(0.05, {}, {0, 0, 0.3}, "cylinder"), box({0.16, 0.16, 0.06}, {0, 0, 0.33})},
                          accent);
  const int flange = arm(b, lift, {0, 0, 0.36},
                         b.config().reach > 0.0 ? b.config().reach : 0.7 * random.vary(0.08),
                         kOrange, kWhite);
  if (b.config().gripper)
    gripper(b, flange, {0.02, 0, 0}, kGraphite, kWhite);
  lidar(b, chassis, {s.length * 0.4, 0, top});
  camera(b, chassis, {s.length * 0.5 + 0.02, 0, 0});
  imu(b, chassis, {0, 0, 0});
}

void pipeCrawler(Builder &b, Random &random, Color accent) {
  const double length = 0.4 * random.vary(0.12);
  const int body = b.link("body", -1, {}, fixed(),
                          {rod(0.06, {-length * 0.5, 0, 0}, {length * 0.5, 0, 0}),
                           disc(0.062, 0.03, kX).tint(accent)},
                          kYellow);
  for (int end = -1; end <= 1; end += 2)
    for (int index = 0; index < 3; ++index) {
      const double angle = -kPi / 2.0 + 2.0 * kPi * index / 3.0;
      const Vec radial{0.0, std::cos(angle), std::sin(angle)};
      const Vec axis{0.0, -std::sin(angle), std::cos(angle)};
      const Vec mount{end * length * 0.35, 0, 0};
      const int strut = b.link("strut " + std::to_string(index + 1) + (end > 0 ? " front" : " rear"),
                               body, mount, fixed(), {rod(0.012, {}, radial * 0.09, "cylinder")}, kSteel);
      wheelOn(b, strut, "wheel " + std::to_string(index + 1) + (end > 0 ? " front" : " rear"),
              radial * 0.11, 0.04, 0.03, axis);
    }
  camera(b, body, {length * 0.5 + 0.08, 0, 0});
  imu(b, body, {0, 0, 0});
}

void wallClimber(Builder &b, Random &, Color accent) {
  const int body = b.link("body", -1, {}, fixed(),
                          {box({0.4, 0.3, 0.06}), box({0.3, 0.2, 0.04}, {0, 0, 0.05}).tint(accent)},
                          kGraphite);
  wheel(b, body, "left wheel", {0, 0.17, 0.0}, 0.05, 0.03);
  wheel(b, body, "right wheel", {0, -0.17, 0.0}, 0.05, 0.03);
  for (int index = 0; index < 4; ++index)
    b.link("suction cup " + std::to_string(index + 1), body,
           {index < 2 ? 0.15 : -0.15, index % 2 ? -0.1 : 0.1, -0.03}, prismatic(kZ, -0.02, 0.0, 30),
           {disc(0.045, 0.02, kZ, {0, 0, -0.01}).touching()}, kRubber);
  camera(b, body, {0.21, 0, 0.0});
  imu(b, body, {0, 0, 0});
}

void biped(Builder &b, Random &random, Color accent) {
  const double tall = b.config().limb * random.vary(0.08);
  const int pelvis = b.link("pelvis", -1, {}, fixed(),
                            {box({0.2, 0.28, 0.1}), box({0.18, 0.26, 0.22}, {0, 0, 0.16}).tint(accent)},
                            kWhite);
  leg(b, pelvis, "left", 1.0, {0, 0.1, -0.05}, 0.36 * tall, 0.34 * tall, kWhite, false);
  leg(b, pelvis, "right", -1.0, {0, -0.1, -0.05}, 0.36 * tall, 0.34 * tall, kWhite, false);
  const int head = b.link("sensor head", pelvis, {0, 0, 0.27}, revolute(kZ, -1.5, 1.5, 4),
                          {box({0.12, 0.18, 0.08}, {0, 0, 0.04})}, kGraphite);
  camera(b, head, {0.07, 0, 0.04});
  imu(b, pelvis, {0, 0, 0});
}

void wheelLegged(Builder &b, Random &random, Color accent) {
  const double length = 0.18 * b.config().limb * random.vary(0.1);
  const int body = b.link("body", -1, {}, fixed(),
                          {box({0.3, 0.36, 0.18}), box({0.2, 0.3, 0.03}, {0, 0, 0.105}).tint(accent)},
                          kGraphite);
  for (int side = -1; side <= 1; side += 2) {
    const std::string label = side > 0 ? "left" : "right";
    const Vec knee{0.12, 0, -length};
    const Vec foot{-0.12, 0, -length};
    const int thigh = b.link(label + " thigh", body, {0, side * 0.22, -0.02}, revolute(kY, -1.2, 1.2, 40),
                             {disc(0.04, 0.05, kY).tint(kSteel), rod(0.025, {}, knee)}, kWhite);
    const int shin = b.link(label + " shin", thigh, knee, revolute(kY, -2.0, 2.0, 40),
                            {ball(0.03).tint(kSteel), rod(0.022, {}, foot)}, kWhite);
    wheel(b, shin, label + " wheel", foot + Vec{0, side * 0.04, 0}, 0.09, 0.05);
  }
  camera(b, body, {0.16, 0, 0.03});
  imu(b, body, {0, 0, 0});
}

void quadrupedArm(Builder &b, Random &random, Color accent) {
  const int body = quadruped(b, random, accent);
  const int flange = arm(b, body, {0.08, 0, 0.1},
                         b.config().reach > 0.0 ? b.config().reach : 0.6 * random.vary(0.08),
                         kGraphite, accent);
  if (b.config().gripper)
    gripper(b, flange, {0.02, 0, 0}, kGraphite, kSteel);
}

void wheeledHumanoid(Builder &b, Random &, Color accent) {
  const BaseSize s = baseSize(b.config(), {0.55, 0.45, 0.25, 0.08, 0.05});
  const int chassis = wheeledChassis(b, s, 2, false,
                                     {box({s.length + 0.01, s.width + 0.01, 0.04}).tint(accent)},
                                     kGraphite, 0.05);
  const int spine = b.link("spine lift", chassis, {0, 0, s.height * 0.5}, prismatic(kZ, 0.0, 0.3, 300),
                           {rod(0.05, {}, {0, 0, 0.4}, "cylinder")}, kSteel);
  const int torso = b.link("torso", spine, {0, 0, 0.4}, revolute(kZ, -1.5, 1.5, 60),
                           {box({0.22, 0.36, 0.42}, {0, 0, 0.21}),
                            box({0.02, 0.24, 0.18}, {0.11, 0, 0.26}).tint(accent)},
                           kWhite);
  const int head = b.link("head", torso, {0, 0, 0.44}, revolute(kZ, -1.4, 1.4, 6),
                          {ball(0.11, {0, 0, 0.1}), box({0.03, 0.16, 0.05}, {0.095, 0, 0.11}).tint(kLens)},
                          kWhite);
  camera(b, head, {0.1, 0, 0.17});
  humanArms(b, torso, 0.23, 0.38);
  lidar(b, chassis, {s.length * 0.4, 0, s.height * 0.5});
  imu(b, torso, {0, 0, 0.2});
}

void fixedWing(Builder &b, Random &random, Color accent) {
  const double span = 1.6 * random.vary(0.1);
  std::vector<Shape> body{rod(0.07, {-0.6, 0, 0}, {0.5, 0, 0}),
                          box({0.25, span, 0.02}, {0.05, 0, 0.05}),
                          box({0.15, 0.5, 0.015}, {-0.55, 0, 0.04}),
                          box({0.15, 0.012, 0.18}, {-0.52, 0, 0.16}).tint(accent)};
  for (int side = -1; side <= 1; side += 2) {
    body.push_back(rod(0.008, {0.1, side * 0.12, -0.05}, {0.1, side * 0.18, -0.2}, "cylinder").tint(kSteel));
    body.push_back(ball(0.03, {0.1, side * 0.18, -0.22}).touching().tint(kRubber));
  }
  body.push_back(rod(0.008, {-0.5, 0, -0.05}, {-0.5, 0, -0.2}, "cylinder").tint(kSteel));
  body.push_back(ball(0.025, {-0.5, 0, -0.225}).touching().tint(kRubber));
  const int fuselage = b.link("fuselage", -1, {}, fixed(), body, kWhite);
  for (int side = -1; side <= 1; side += 2)
    b.link(side > 0 ? "left aileron" : "right aileron", fuselage, {-0.075, side * span * 0.35, 0.05},
           revolute(kY, -0.5, 0.5, 2), {box({0.08, span * 0.3, 0.012}, {-0.04, 0, 0})}, accent);
  b.link("elevator", fuselage, {-0.625, 0, 0.04}, revolute(kY, -0.5, 0.5, 2),
         {box({0.06, 0.5, 0.01}, {-0.03, 0, 0})}, accent);
  b.link("rudder", fuselage, {-0.6, 0, 0.07}, revolute(kZ, -0.5, 0.5, 2),
         {box({0.08, 0.012, 0.18}, {-0.04, 0, 0.09})}, accent);
  b.link("propeller", fuselage, {0.6, 0, 0}, continuous(kX, 3.0),
         {disc(0.025, 0.04, kX), box({0.012, 0.35, 0.03})}, kGraphite);
  camera(b, fuselage, {0.45, 0, -0.08});
  imu(b, fuselage, {0, 0, 0});
}

void rov(Builder &b, Random &, Color accent) {
  std::vector<Shape> frame{box({0.5, 0.35, 0.15}), box({0.6, 0.45, 0.12}, {0, 0, 0.14}).tint(kYellow),
                           rod(0.02, {-0.3, 0.2, -0.13}, {0.3, 0.2, -0.13}, "cylinder").touching().tint(kSteel),
                           rod(0.02, {-0.3, -0.2, -0.13}, {0.3, -0.2, -0.13}, "cylinder").touching().tint(kSteel),
                           ball(0.03, {0.27, 0.13, 0.03}).tint(kWhite), ball(0.03, {0.27, -0.13, 0.03}).tint(kWhite)};
  const int body = b.link("frame", -1, {}, fixed(), frame, kGraphite);
  for (int index = 0; index < 4; ++index)
    b.link("horizontal thruster " + std::to_string(index + 1), body,
           {index < 2 ? 0.28 : -0.28, index % 2 ? -0.24 : 0.24, 0}, continuous(kX, 5.0),
           {disc(0.045, 0.08, kX), disc(0.015, 0.1, kX).tint(kSteel)}, accent);
  for (int side = -1; side <= 1; side += 2)
    b.link(side > 0 ? "left vertical thruster" : "right vertical thruster", body, {0, side * 0.15, 0.22},
           continuous(kZ, 5.0), {disc(0.045, 0.06, kZ), disc(0.015, 0.08, kZ).tint(kSteel)}, accent);
  const int manipulator = b.link("manipulator", body, {0.27, 0, -0.06}, revolute(kY, -1.0, 1.0, 20),
                                 {disc(0.03, 0.05, kY), rod(0.018, {}, {0.18, 0, 0}, "cylinder")}, kSteel);
  if (b.config().gripper)
    gripper(b, manipulator, {0.18, 0, 0}, kGraphite, kSteel);
  camera(b, body, {0.27, 0, 0.0});
  imu(b, body, {0, 0, 0});
}

void boat(Builder &b, Random &random, Color accent) {
  const double length = 1.2 * random.vary(0.12);
  std::vector<Shape> hull{rod(0.1, {-length * 0.5, 0.3, 0}, {length * 0.5, 0.3, 0}).touching(),
                          rod(0.1, {-length * 0.5, -0.3, 0}, {length * 0.5, -0.3, 0}).touching(),
                          box({length * 0.75, 0.7, 0.04}, {0, 0, 0.14}).tint(kGraphite),
                          box({0.4, 0.4, 0.2}, {-0.05, 0, 0.26}).tint(kWhite),
                          rod(0.015, {0.1, 0, 0.36}, {0.1, 0, 0.7}, "cylinder").tint(kSteel),
                          ball(0.04, {-0.15, 0.12, 0.4}).tint(kWhite)};
  const int body = b.link("hull", -1, {}, fixed(), hull, accent);
  for (int side = -1; side <= 1; side += 2)
    b.link(side > 0 ? "left propeller" : "right propeller", body, {-length * 0.5 - 0.06, side * 0.3, -0.03},
           continuous(kX, 8.0), {disc(0.06, 0.02, kX), disc(0.015, 0.06, kX).tint(kSteel)}, kSteel);
  lidar(b, body, {0.1, 0, 0.7});
  camera(b, body, {0.17, 0, 0.3});
  imu(b, body, {0, 0, 0.14});
}

using Recipe = void (*)(Builder &, Random &, Color);

struct Family {
  std::string_view key;
  std::string_view title;
  Recipe build;
  bool floating;
};

void articulatedArm(Builder &b, Random &r, Color a) { articulated(b, r, a, false, false); }
void cobotArm(Builder &b, Random &r, Color a) { articulated(b, r, a, true, false); }
void surgicalCart(Builder &b, Random &r, Color a) { articulated(b, r, a, false, true); }
void amrBase(Builder &b, Random &r, Color a) { mobileBase(b, r, a, "amr"); }
void agvBase(Builder &b, Random &r, Color a) { mobileBase(b, r, a, "agv"); }
void ugvBase(Builder &b, Random &r, Color a) { mobileBase(b, r, a, "ugv"); }
void domesticBase(Builder &b, Random &r, Color a) { mobileBase(b, r, a, "domestic"); }
void manipulatorBase(Builder &b, Random &r, Color a) { mobileBase(b, r, a, "mobile_manipulator"); }
void quadrotor(Builder &b, Random &r, Color a) { drone(b, r, a, 4); }
void hexacopter(Builder &b, Random &r, Color a) { drone(b, r, a, 6); }
void quadrupedOnly(Builder &b, Random &r, Color a) { quadruped(b, r, a); }
void hexapodSix(Builder &b, Random &r, Color a) { hexapod(b, r, a, 3); }
void spider(Builder &b, Random &r, Color a) { hexapod(b, r, a, 4); }
void humanoidBody(Builder &b, Random &r, Color a) { humanoid(b, r, a, "humanoid"); }
void companionBody(Builder &b, Random &r, Color a) { humanoid(b, r, a, "companion"); }
void exoskeletonFrame(Builder &b, Random &r, Color a) { humanoid(b, r, a, "exoskeleton"); }
void cartesianGantry(Builder &b, Random &r, Color a) { cartesian(b, r, a); }
void sevenDof(Builder &b, Random &r, Color a) { pedestalArm(b, r, a, 0.95, true, 0.12); }
void desktopArm(Builder &b, Random &r, Color a) { pedestalArm(b, r, a, 0.45, false, 0.07); }

constexpr std::array<Family, 50> kFamilies{{
    {"articulated", "Articulated arm", articulatedArm, false},
    {"scara", "SCARA arm", scara, false},
    {"delta", "Delta robot", delta, false},
    {"cartesian", "Cartesian gantry", cartesianGantry, false},
    {"cobot", "Collaborative arm", cobotArm, false},
    {"amr", "Autonomous mobile robot", amrBase, true},
    {"agv", "Guided vehicle", agvBase, true},
    {"drone", "Quadrotor", quadrotor, true},
    {"ugv", "Rocker rover", ugvBase, true},
    {"auv", "Underwater vehicle", auv, true},
    {"quadruped", "Quadruped", quadrupedOnly, true},
    {"hexapod", "Hexapod", hexapodSix, true},
    {"snake", "Serpentine robot", snake, true},
    {"humanoid", "Humanoid", humanoidBody, true},
    {"soft_arm", "Segmented continuum arm", soft, false},
    {"surgical", "Surgical cart", surgicalCart, false},
    {"domestic", "Domestic robot", domesticBase, true},
    {"companion", "Companion robot", companionBody, true},
    {"exoskeleton", "Lower-limb exoskeleton", exoskeletonFrame, true},
    {"mobile_manipulator", "Mobile manipulator", manipulatorBase, true},
    {"arm_7dof", "7-DOF arm", sevenDof, false},
    {"dual_arm", "Dual-arm torso", dualArm, false},
    {"palletizer", "Palletizer", palletizer, false},
    {"desktop_arm", "Desktop arm", desktopArm, false},
    {"pan_tilt", "Pan-tilt unit", panTilt, false},
    {"linear_rig", "Linear actuator rig", linearRig, false},
    {"gantry_picker", "Gantry picker", gantryPicker, false},
    {"hand", "Robotic hand", hand, false},
    {"forklift", "Forklift AGV", forklift, true},
    {"tugger", "Tugger", tugger, true},
    {"omni_base", "Omnidirectional base", omniBase, true},
    {"tracked", "Tracked rover", tracked, true},
    {"six_wheel", "Six-wheel rover", sixWheel, true},
    {"lawn_mower", "Lawn mower", lawnMower, true},
    {"vacuum", "Robot vacuum", vacuum, true},
    {"delivery", "Delivery robot", delivery, true},
    {"telepresence", "Telepresence robot", telepresence, true},
    {"warehouse_picker", "Warehouse picker", warehousePicker, true},
    {"inspection_crawler", "Inspection crawler", inspectionCrawler, true},
    {"pipe_crawler", "Pipe crawler", pipeCrawler, true},
    {"wall_climber", "Wall climber", wallClimber, true},
    {"biped", "Biped walker", biped, true},
    {"spider", "Spider robot", spider, true},
    {"wheel_legged", "Wheel-legged robot", wheelLegged, true},
    {"quadruped_arm", "Quadruped with arm", quadrupedArm, true},
    {"wheeled_humanoid", "Wheeled humanoid", wheeledHumanoid, true},
    {"hexacopter", "Hexacopter", hexacopter, true},
    {"fixed_wing", "Fixed-wing drone", fixedWing, true},
    {"rov", "Underwater ROV", rov, true},
    {"boat", "Autonomous boat", boat, true},
}};

Json generate(const Json &message) {
  if (message["event"].asString() != "generate" ||
      message["robot_id"].asString().empty())
    return Json::object({{"error", "invalid_request"}});
  const Json &config = message["configuration"];
  const std::string_view key = config["family"].asString("articulated");
  const auto family = std::find_if(kFamilies.begin(), kFamilies.end(),
                                   [key](const Family &item) { return item.key == key; });
  if (family == kFamilies.end())
    return Json::object({{"error", "unknown_family"}});
  const Config settings = readConfig(config);
  Random random(static_cast<std::uint64_t>(message["seed"].asNumber(0.0)),
                settings.variation / 0.12);
  const Color seeded = kAccents[static_cast<std::size_t>(random.range(0, 5))];
  const Color accent = settings.customColors ? settings.accent : seeded;
  Builder builder(std::string(message["robot_id"].asString()), std::string(family->title),
                  settings);
  family->build(builder, random, accent);
  return builder.finish(family->floating);
}

} // namespace

ROBOTLENS_MODULE(generate)

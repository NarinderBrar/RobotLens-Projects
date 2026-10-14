#pragma once

#include <robotlens/procgen.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace roomkit {

using robotlens::Json;
namespace pg = robotlens::procgen;

inline double radians(double degrees) {
    return degrees * pg::kPi / 180.0;
}

inline pg::Material colored(pg::Material material, pg::Vec3 color) {
    material.color = color;
    return material;
}

inline void buildRoom(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    pg::RoomSpec room;
    room.width = settings.number("width", 12.0, 1.0, 100.0);
    room.depth = settings.number("depth", 8.0, 1.0, 100.0);
    room.height = settings.number("height", 3.0, 1.0, 10.0);
    room.wallThickness = settings.number("wall_thickness", 0.15, 0.05, 1.0);
    const std::string side = settings.text("doorway_side", "south");
    room.doorSide = pg::sideFromName(side);
    room.doorWidth = side == "none" ? 0.0 : settings.number("doorway_width", 1.0, 0.5, 10.0);
    room.doorHeight = settings.number("doorway_height", 2.1, 1.0, 5.0);
    room.doorOffset = settings.number("doorway_offset", 0.0, -50.0, 50.0);
    const pg::Material floor =
        colored(pg::palette("floor"), settings.color("floor_color", {0.55, 0.55, 0.52}));
    const pg::Material wall =
        colored(pg::palette("wall"), settings.color("wall_color", {0.86, 0.85, 0.8}));
    pg::roomShell(scene, "", room, floor, wall);
}

inline void buildWall(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double length = settings.number("length", 4.0, 0.2, 100.0);
    const double height = settings.number("height", 2.5, 0.2, 10.0);
    const double thickness = settings.number("thickness", 0.15, 0.02, 1.0);
    const pg::Pose frame = pg::Pose::at(settings.number("x", 0.0, -100.0, 100.0),
                                        settings.number("y", 3.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const pg::Material material =
        colored(pg::palette("wall"), settings.color("color", {0.86, 0.85, 0.8}));
    pg::wallWithDoor(scene, "wall", frame, length, height, thickness,
                     settings.number("doorway_width", 0.0, 0.0, 10.0),
                     settings.number("doorway_height", 2.1, 0.5, 5.0),
                     settings.number("doorway_offset", 0.0, -50.0, 50.0), material, "");
}

inline void buildDoorway(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double width = settings.number("width", 1.0, 0.5, 5.0);
    const double height = settings.number("height", 2.1, 1.0, 5.0);
    const double frameSize = settings.number("frame", 0.1, 0.02, 0.5);
    const double depth = settings.number("depth", 0.15, 0.02, 1.0);
    const pg::Pose frame = pg::Pose::at(settings.number("x", 0.0, -100.0, 100.0),
                                        settings.number("y", 3.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const pg::Material material = pg::palette("doorway");
    const double postX = (width + frameSize) * 0.5;
    scene
        .box("frame/header", frame * pg::Pose::at(0.0, 0.0, height + frameSize * 0.5),
             {width + 2.0 * frameSize, depth, frameSize}, material)
        .label("doorway");
    for (const double side : {-1.0, 1.0}) {
        scene
            .box(side < 0.0 ? "frame/left" : "frame/right",
                 frame * pg::Pose::at(side * postX, 0.0, height * 0.5), {frameSize, depth, height},
                 material)
            .label("doorway")
            .group("frame/header");
    }
    scene.region("frame/opening", "keepout", frame * pg::Pose::at(0.0, 0.0, height * 0.5),
                 {width, depth + 1.2, height});
}

inline void buildTable(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double width = settings.number("width", 1.2, 0.2, 5.0);
    const double depth = settings.number("depth", 0.8, 0.2, 5.0);
    const double height = settings.number("height", 0.75, 0.2, 2.0);
    const double top = settings.number("top_thickness", 0.04, 0.01, 0.2);
    const double leg = settings.number("leg_thickness", 0.05, 0.01, 0.3);
    const double margin = settings.number("leg_margin", 0.05, 0.0, 0.4);
    const bool isRound = settings.flag("cylindrical_legs", false);
    const pg::Pose floor = pg::Pose::at(settings.number("x", 2.0, -100.0, 100.0),
                                        settings.number("y", 2.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    if (top >= height) {
        scene.diagnostic("error", "top_too_thick",
                         "The table top is thicker than the table is tall", "top_thickness", top,
                         height, true);
        return;
    }
    const double bevel = settings.number("bevel", 0.0, 0.0, 0.2);
    const pg::Pose topPose = floor * pg::Pose::at(0.0, 0.0, height - top * 0.5);
    pg::Object tableTop =
        bevel > 0.0 ? scene.mesh("table", topPose, pg::bevelBox({width, depth, top}, bevel),
                                 pg::palette("table"))
                    : scene.box("table", topPose, {width, depth, top}, pg::palette("table"));
    tableTop.collision("box", {width, depth, top})
        .label("table_top")
        .support("top", top * 0.5, pg::rectangle(width, depth, 0.02), 1.0);
    pg::legs(scene, "table/leg", floor, width, depth, height - top, leg, margin, isRound,
             pg::palette("leg"), "table");
}

inline void buildObstacles(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const std::string shape = settings.text("shape", "box");
    const double width = settings.number("width", 0.5, 0.05, 10.0);
    const double depth = settings.number("depth", 0.5, 0.05, 10.0);
    const double height = settings.number("height", 0.8, 0.05, 5.0);
    const int count = settings.integer("count", 5, 1, 200);
    const double areaWidth = settings.number("area_width", 8.0, 0.1, 100.0);
    const double areaDepth = settings.number("area_depth", 6.0, 0.1, 100.0);
    const pg::Vec2 centre{settings.number("x", 0.0, -100.0, 100.0),
                          settings.number("y", 0.0, -100.0, 100.0)};
    const double clearRadius = settings.number("clear_radius", 1.0, 0.0, 10.0);
    const double spacing = settings.number("spacing", 0.3, 0.0, 5.0);
    const double footprintRadius = std::sqrt(width * width + depth * depth) * 0.5;

    std::vector<pg::Footprint> placed;
    int built = 0;
    for (int index = 0; index < count; ++index) {
        const std::string key = pg::key("obstacle", index);
        pg::Rng rng = scene.rng(key);
        for (int attempt = 0; attempt < 100; ++attempt) {
            pg::Footprint footprint;
            footprint.center = {centre.x + rng.uniform(-areaWidth, areaWidth) * 0.5,
                                centre.y + rng.uniform(-areaDepth, areaDepth) * 0.5};
            footprint.half = {width * 0.5, depth * 0.5};
            footprint.yaw = rng.uniform(-pg::kPi, pg::kPi);
            const double distance = std::sqrt(footprint.center.x * footprint.center.x +
                                              footprint.center.y * footprint.center.y);
            if (distance < clearRadius + footprintRadius) {
                continue;
            }
            bool isFree = true;
            for (const pg::Footprint& other : placed) {
                if (pg::footprintsOverlap(footprint, other, spacing)) {
                    isFree = false;
                    break;
                }
            }
            if (!isFree) {
                continue;
            }
            placed.push_back(footprint);
            const pg::Pose pose = pg::Pose{{footprint.center.x, footprint.center.y, height * 0.5},
                                           pg::Quat::yaw(footprint.yaw)};
            const pg::Material material = pg::vary(pg::palette("obstacle"), rng, 0.15);
            if (shape == "cylinder") {
                scene.cylinder(key, pose, width, height, material).label("obstacle");
            } else if (shape == "l_shape") {
                scene
                    .box(key, pose * pg::Pose::at(0.0, -depth * 0.375, 0.0),
                         {width, depth * 0.25, height}, material)
                    .label("obstacle");
                scene
                    .box(key + "/arm", pose * pg::Pose::at(-width * 0.375, depth * 0.125, 0.0),
                         {width * 0.25, depth * 0.75, height}, material)
                    .label("obstacle")
                    .group(key);
            } else {
                scene.box(key, pose, {width, depth, height}, material).label("obstacle");
            }
            ++built;
            break;
        }
    }
    if (built < count) {
        scene.diagnostic("warning", "obstacles_do_not_fit",
                         "Placed " + std::to_string(built) + " of " + std::to_string(count) +
                             " obstacles; enlarge the area or reduce the count or gap",
                         "count", count, built, true);
    }
}

inline void buildDoor(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double width = settings.number("width", 0.9, 0.5, 3.0);
    const double height = settings.number("height", 2.05, 1.5, 3.0);
    const double thickness = settings.number("thickness", 0.04, 0.02, 0.1);
    const double frameSize = settings.number("frame", 0.08, 0.03, 0.3);
    const double frameDepth = settings.number("frame_depth", 0.12, 0.05, 0.5);
    const bool isHingeLeft = settings.text("hinge_side", "left") != "right";
    const double openAngle = radians(settings.number("open_angle", 100.0, 10.0, 180.0));
    const pg::Pose frame = pg::Pose::at(settings.number("x", 0.0, -100.0, 100.0),
                                        settings.number("y", 3.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const pg::Material frameMaterial = pg::palette("doorway");
    scene
        .box("door", frame * pg::Pose::at(0.0, 0.0, height + frameSize * 0.5),
             {width + 2.0 * frameSize, frameDepth, frameSize}, frameMaterial)
        .label("doorway");
    for (const double side : {-1.0, 1.0}) {
        scene
            .box(side < 0.0 ? "door/post/left" : "door/post/right",
                 frame * pg::Pose::at(side * (width + frameSize) * 0.5, 0.0, height * 0.5),
                 {frameSize, frameDepth, height}, frameMaterial)
            .label("doorway")
            .group("door");
    }
    pg::hingeDoor(scene, "door/leaf", "door", frame * pg::Pose::at(0.0, 0.0, height * 0.5 + 0.005),
                  width - 0.01, height - 0.01, thickness, isHingeLeft, openAngle,
                  pg::palette("door"), pg::palette("handle"));
    scene.region("door/swing", "keepout",
                 frame * pg::Pose::at(0.0, -width * 0.5 - frameDepth * 0.5, height * 0.5),
                 {width, width, height});
}

inline void buildWindow(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double length = settings.number("length", 3.0, 0.5, 50.0);
    const double height = settings.number("height", 2.5, 1.0, 10.0);
    const double thickness = settings.number("thickness", 0.15, 0.05, 1.0);
    const double windowWidth = settings.number("window_width", 1.2, 0.2, 10.0);
    const double windowHeight = settings.number("window_height", 1.0, 0.2, 5.0);
    const double sill = settings.number("sill_height", 0.9, 0.0, 5.0);
    const double offset = settings.number("offset", 0.0, -25.0, 25.0);
    const pg::Pose frame = pg::Pose::at(settings.number("x", 0.0, -100.0, 100.0),
                                        settings.number("y", 3.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const pg::Material wall = pg::palette("wall");
    if (windowWidth > length - 0.2 || sill + windowHeight > height - 0.05) {
        scene.diagnostic("error", "window_does_not_fit",
                         "The window does not fit in the wall; the wall was built without it",
                         "window_width", windowWidth, std::max(0.0, length - 0.2), true);
        scene
            .box("wall", frame * pg::Pose::at(0.0, 0.0, height * 0.5), {length, thickness, height},
                 wall)
            .label("wall");
        return;
    }
    const double half = length * 0.5;
    const double centre =
        std::clamp(offset, -half + 0.1 + windowWidth * 0.5, half - 0.1 - windowWidth * 0.5);
    const double left = centre - windowWidth * 0.5;
    const double right = centre + windowWidth * 0.5;
    const double top = sill + windowHeight;
    const auto piece = [&](const std::string& key, double x0, double x1, double z0, double z1,
                           const std::string& group) {
        if (x1 - x0 < 1.0e-3 || z1 - z0 < 1.0e-3) {
            return;
        }
        pg::Object part =
            scene.box(key, frame * pg::Pose::at((x0 + x1) * 0.5, 0.0, (z0 + z1) * 0.5),
                      {x1 - x0, thickness, z1 - z0}, wall);
        part.label("wall");
        if (!group.empty()) {
            part.group(group);
        }
    };
    piece("wall", -half, left, 0.0, height, "");
    piece("wall/right", right, half, 0.0, height, "wall");
    piece("wall/below", left, right, 0.0, sill, "wall");
    piece("wall/above", left, right, top, height, "wall");
    const double bar = 0.05;
    const pg::Material frameMaterial = pg::palette("window_frame");
    const auto framePart = [&](const std::string& key, double x, double z, double sx, double sz) {
        scene.box(key, frame * pg::Pose::at(x, 0.0, z), {sx, thickness * 0.6, sz}, frameMaterial)
            .label("window_frame")
            .group("wall");
    };
    framePart("window/frame/bottom", centre, sill + bar * 0.5, windowWidth, bar);
    framePart("window/frame/top", centre, top - bar * 0.5, windowWidth, bar);
    framePart("window/frame/left", left + bar * 0.5, sill + windowHeight * 0.5, bar,
              windowHeight - 2.0 * bar);
    framePart("window/frame/right", right - bar * 0.5, sill + windowHeight * 0.5, bar,
              windowHeight - 2.0 * bar);
    scene
        .box("window/glass", frame * pg::Pose::at(centre, 0.0, sill + windowHeight * 0.5),
             {windowWidth - 2.0 * bar, 0.01, windowHeight - 2.0 * bar}, pg::palette("glass"))
        .label("glass")
        .group("wall");
}

inline Json handleWith(void (*build)(pg::Scene&), const Json& message) {
    if (message["event"].asString() != "generate") {
        return Json();
    }
    pg::Scene scene(message);
    build(scene);
    return scene.reply();
}

}                     

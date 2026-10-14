#pragma once

#include <robotlens/procgen.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace kitchenkit {

using robotlens::Json;
namespace pg = robotlens::procgen;

                                                                                   
                                       
struct Dimensions {
    double counterHeight = 0.9;
    double counterThickness = 0.03;
    double depth = 0.6;
    double plinth = 0.1;
    double panel = 0.018;
    double front = 0.019;
    double gap = 0.003;
    double wallCabinetZ = 1.45;
    double wallCabinetHeight = 0.7;
    double wallCabinetDepth = 0.35;
    double doorOpen = 1.9;
};

enum class FrontStyle { Doors, Drawers, Mixed };

inline FrontStyle frontStyle(const std::string& name) {
    if (name == "drawers") {
        return FrontStyle::Drawers;
    }
    if (name == "doors") {
        return FrontStyle::Doors;
    }
    return FrontStyle::Mixed;
}

struct Carcass {
    bool hasTop = true;
    bool hasShelf = true;
    std::string label = "cabinet";
    bool hasPlinth = true;
};

                                                                     
inline void carcass(pg::Scene& scene, const std::string& root, const pg::Pose& floor, double width,
                    double depth, double height, double bottom, const Dimensions& d,
                    const Carcass& options) {
    const pg::Material body = pg::palette("cabinet");
    const double p = d.panel;
    const auto part = [&](const std::string& key, pg::Vec3 centre, pg::Vec3 size) {
        return scene.box(key, floor * pg::Pose::at(centre.x, centre.y, centre.z), size, body);
    };
    part(root, {0.0, depth * 0.5 - p * 0.5, bottom + height * 0.5}, {width, p, height})
        .label(options.label)
        .mass(std::max(5.0, width * height * 20.0));
    for (const double side : {-1.0, 1.0}) {
        part(root + (side < 0.0 ? "/side/left" : "/side/right"),
             {side * (width * 0.5 - p * 0.5), -p * 0.5, bottom + height * 0.5},
             {p, depth - p, height})
            .label("cabinet_panel")
            .group(root);
    }
    const double innerWidth = width - 2.0 * p;
    const double innerDepth = depth - p;
    const double clear = height - (options.hasTop ? 2.0 : 1.0) * p;
    part(root + "/bottom", {0.0, -p * 0.5, bottom + p * 0.5}, {innerWidth, innerDepth, p})
        .label("shelf_board")
        .group(root)
        .support("inside", p * 0.5, pg::rectangle(innerWidth, innerDepth, 0.02),
                 options.hasShelf ? clear * 0.5 - p : clear);
    if (options.hasTop) {
        part(root + "/top", {0.0, -p * 0.5, bottom + height - p * 0.5}, {innerWidth, innerDepth, p})
            .label("cabinet_panel")
            .group(root);
    }
    if (options.hasShelf && height > 0.4) {
        part(root + "/shelf", {0.0, -p * 0.5, bottom + height * 0.5}, {innerWidth, innerDepth, p})
            .label("shelf_board")
            .group(root)
            .support("top", p * 0.5, pg::rectangle(innerWidth, innerDepth, 0.02), clear * 0.5 - p);
    }
    if (options.hasPlinth && bottom > 0.02) {
        part(root + "/plinth", {0.0, -depth * 0.5 + p * 0.5 + 0.05, bottom * 0.5},
             {innerWidth, p, bottom})
            .label("cabinet_panel")
            .group(root);
    }
}

                                                                           
                                                             
inline void dropDoor(pg::Scene& scene, const std::string& key, const std::string& parent,
                     const pg::Pose& closedCenter, double width, double height, double thickness,
                     const pg::Material& material) {
    scene.box(key, closedCenter, {width, thickness, height}, material)
        .label("door")
        .group(parent)
        .mass(std::max(1.0, width * height * thickness * 800.0));
    scene.joint(key + "/hinge", "revolute", parent, key,
                closedCenter.apply({0.0, 0.0, -height * 0.5}),
                closedCenter.rotation.rotate({1.0, 0.0, 0.0}), 0.0, pg::kPi * 0.5);
    pg::handle(scene, key + "/handle",
               closedCenter * pg::Pose::at(0.0, -thickness * 0.5 - 0.02, height * 0.5 - 0.06),
               width * 0.7, 0.02, false, pg::palette("handle"), key);
}

                                                                                     
inline void fronts(pg::Scene& scene, const std::string& root, const pg::Pose& floor, double width,
                   double depth, double height, double bottom, FrontStyle style,
                   const Dimensions& d) {
    const pg::Material front = pg::palette("cabinet_front");
    const pg::Material handle = pg::palette("handle");
    const double y = -depth * 0.5 - d.front * 0.5;
    double top = bottom + height;
    if (style == FrontStyle::Drawers || style == FrontStyle::Mixed) {
        const int count = style == FrontStyle::Mixed ? 1 : (height > 0.6 ? 3 : 2);
        const double drawerHeight =
            style == FrontStyle::Mixed ? std::min(0.18, height * 0.3) : height / count;
        for (int index = 0; index < count; ++index) {
            const double z = top - drawerHeight * (index + 0.5);
            pg::slideDrawer(scene, root + "/drawer/" + std::to_string(index), root,
                            floor * pg::Pose::at(0.0, y, z), width - 2.0 * d.gap,
                            drawerHeight - 2.0 * d.gap, depth - d.panel - 0.05, d.front,
                            (depth - d.panel) * 0.8, front, handle);
        }
        top -= drawerHeight * count;
    }
    if (style == FrontStyle::Drawers || top - bottom < 0.1) {
        return;
    }
    const double doorHeight = top - bottom - 2.0 * d.gap;
    const double z = bottom + (top - bottom) * 0.5;
    if (width > 0.55) {
        const double doorWidth = width * 0.5 - 2.0 * d.gap;
        for (const double side : {-1.0, 1.0}) {
            pg::hingeDoor(scene, root + (side < 0.0 ? "/door/left" : "/door/right"), root,
                          floor * pg::Pose::at(side * width * 0.25, y, z), doorWidth, doorHeight,
                          d.front, side < 0.0, d.doorOpen, front, handle);
        }
    } else {
        pg::hingeDoor(scene, root + "/door", root, floor * pg::Pose::at(0.0, y, z),
                      width - 2.0 * d.gap, doorHeight, d.front, true, d.doorOpen, front, handle);
    }
}

inline void baseCabinet(pg::Scene& scene, const std::string& root, const pg::Pose& floor,
                        double width, FrontStyle style, const Dimensions& d) {
    const double height = d.counterHeight - d.counterThickness - d.plinth;
    carcass(scene, root, floor, width, d.depth, height, d.plinth, d, Carcass{});
    fronts(scene, root, floor, width, d.depth, height, d.plinth, style, d);
}

inline void wallCabinet(pg::Scene& scene, const std::string& root, const pg::Pose& floor,
                        double width, const Dimensions& d) {
    const pg::Pose back = floor * pg::Pose::at(0.0, d.depth * 0.5 - d.wallCabinetDepth * 0.5, 0.0);
    carcass(scene, root, back, width, d.wallCabinetDepth, d.wallCabinetHeight, d.wallCabinetZ, d,
            Carcass{true, true, "cabinet", false});
    fronts(scene, root, back, width, d.wallCabinetDepth, d.wallCabinetHeight, d.wallCabinetZ,
           FrontStyle::Doors, d);
}

                                                                           
inline void sinkCabinet(pg::Scene& scene, const std::string& root, const pg::Pose& floor,
                        double width, const Dimensions& d) {
    const double height = d.counterHeight - d.counterThickness - d.plinth;
    carcass(scene, root, floor, width, d.depth, height, d.plinth, d,
            Carcass{false, false, "cabinet"});
    fronts(scene, root, floor, width, d.depth, height, d.plinth, FrontStyle::Doors, d);
    const double openingWidth = width - 0.2;
    const double openingDepth = 0.4;
    const double basinDepth = 0.18;
    const double wall = 0.01;
    const double top = d.counterHeight - d.counterThickness;
    const pg::Material steel = pg::palette("sink");
    const pg::Pose centre = floor * pg::Pose::at(0.0, -0.02, top - basinDepth * 0.5);
    const auto part = [&](const std::string& key, pg::Vec3 offset, pg::Vec3 size) {
        return scene.box(root + key, centre * pg::Pose::at(offset.x, offset.y, offset.z), size,
                         steel);
    };
    part("/basin", {0.0, 0.0, -basinDepth * 0.5 + wall * 0.5},
         {openingWidth - 2.0 * wall, openingDepth - 2.0 * wall, wall})
        .label("sink")
        .group(root)
        .support("basin", wall * 0.5,
                 pg::rectangle(openingWidth - 2.0 * wall, openingDepth - 2.0 * wall, 0.01),
                 basinDepth - wall);
    for (const double side : {-1.0, 1.0}) {
        part(side < 0.0 ? "/basin/front" : "/basin/back",
             {0.0, side * (openingDepth * 0.5 - wall * 0.5), 0.0}, {openingWidth, wall, basinDepth})
            .label("sink")
            .group(root);
        part(side < 0.0 ? "/basin/left" : "/basin/right",
             {side * (openingWidth * 0.5 - wall * 0.5), 0.0, 0.0},
             {wall, openingDepth - 2.0 * wall, basinDepth})
            .label("sink")
            .group(root);
    }
}

                                                                                 
                             
inline void countertop(pg::Scene& scene, const std::string& key, const pg::Pose& run, double x0,
                       double x1, const std::vector<std::pair<double, double>>& sinks,
                       const Dimensions& d) {
    const pg::Material stone = pg::palette("counter");
    const double t = d.counterThickness;
    const double z = d.counterHeight - t * 0.5;
    const double yFront = -d.depth * 0.5 - 0.02;
    const double yBack = d.depth * 0.5;
    int piece = 0;
    const auto slab = [&](double from, double to, double y0, double y1) {
        if (to - from < 0.01 || y1 - y0 < 0.01) {
            return;
        }
        const std::string pieceKey = piece == 0 ? key : key + "/piece/" + std::to_string(piece);
        pg::Object slabObject =
            scene.box(pieceKey, run * pg::Pose::at((from + to) * 0.5, (y0 + y1) * 0.5, z),
                      {to - from, y1 - y0, t}, stone);
        slabObject.label("counter").support("top", t * 0.5, pg::rectangle(to - from, y1 - y0, 0.03),
                                            1.0);
        if (piece > 0) {
            slabObject.group(key);
        }
        ++piece;
    };
    double cursor = x0;
    for (const auto& [sinkX, sinkWidth] : sinks) {
        const double half = (sinkWidth - 0.2) * 0.5;
        const double openingFront = -0.02 - 0.2;
        const double openingBack = -0.02 + 0.2;
        slab(cursor, sinkX - half, yFront, yBack);
        slab(sinkX - half, sinkX + half, yFront, openingFront);
        slab(sinkX - half, sinkX + half, openingBack, yBack);
        cursor = sinkX + half;
        scene
            .cylinder(key + "/faucet/" + std::to_string(piece),
                      run * pg::Pose::at(sinkX, d.depth * 0.5 - 0.08, d.counterHeight + 0.15), 0.03,
                      0.3, pg::palette("metal"))
            .noCollision()
            .label("faucet")
            .group(key);
        std::vector<pg::Vec2> tube;
        for (int step = 0; step < 8; ++step) {
            const double angle = 2.0 * pg::kPi * step / 8.0;
            tube.push_back({0.012 * std::cos(angle), 0.012 * std::sin(angle)});
        }
        std::vector<pg::Vec3> arc;
        for (int step = 0; step <= 8; ++step) {
            const double angle = pg::kPi * step / 8.0;
            arc.push_back({0.0, -0.09 + 0.09 * std::cos(angle), 0.09 * std::sin(angle)});
        }
        scene
            .mesh(key + "/spout/" + std::to_string(piece),
                  run * pg::Pose::at(sinkX, d.depth * 0.5 - 0.08, d.counterHeight + 0.3),
                  pg::sweep(tube, arc), pg::palette("metal"))
            .label("faucet")
            .group(key);
    }
    slab(cursor, x1, yFront, yBack);
}

inline void refrigerator(pg::Scene& scene, const std::string& root, const pg::Pose& unit,
                         double width, const Dimensions& d, double height = 1.75) {
    const double depth = 0.7;
    const double freezer = 0.5;
    const pg::Pose floor = unit * pg::Pose::at(0.0, d.depth * 0.5 - depth * 0.5, 0.0);
    carcass(scene, root, floor, width, depth, height - 0.05, 0.05, d,
            Carcass{true, false, "refrigerator"});
    const double p = d.panel;
    const double innerWidth = width - 2.0 * p;
    const double innerDepth = depth - p;
    const pg::Material body = pg::palette("appliance");
    const double divider = height - freezer;
    scene
        .box(root + "/divider", floor * pg::Pose::at(0.0, -p * 0.5, divider),
             {innerWidth, innerDepth, p}, body)
        .label("cabinet_panel")
        .group(root);
    const std::vector<double> shelves = pg::distribute(0.05 + p, divider - p * 0.5, 3);
    for (std::size_t index = 1U; index < shelves.size(); ++index) {
        scene
            .box(root + "/shelf/" + std::to_string(index),
                 floor * pg::Pose::at(0.0, -p * 0.5 - 0.05, shelves[index]),
                 {innerWidth, innerDepth - 0.1, 0.01}, pg::palette("glass"))
            .label("shelf_board")
            .group(root)
            .support("top", 0.005, pg::rectangle(innerWidth, innerDepth - 0.1, 0.02), 0.25);
    }
    const double y = -depth * 0.5 - 0.025;
    const double mainHeight = divider - 0.05 - 2.0 * d.gap;
    pg::hingeDoor(scene, root + "/door/main", root,
                  floor * pg::Pose::at(0.0, y, 0.05 + (divider - 0.05) * 0.5), width - 2.0 * d.gap,
                  mainHeight, 0.05, false, d.doorOpen, body, pg::palette("handle"));
    pg::hingeDoor(scene, root + "/door/freezer", root,
                  floor * pg::Pose::at(0.0, y, divider + freezer * 0.5), width - 2.0 * d.gap,
                  freezer - 2.0 * d.gap, 0.05, false, d.doorOpen, body, pg::palette("handle"));
}

inline void range(pg::Scene& scene, const std::string& root, const pg::Pose& floor, double width,
                  const Dimensions& d) {
    const double height = d.counterHeight - d.counterThickness - d.plinth;
    carcass(scene, root, floor, width, d.depth, height, d.plinth, d, Carcass{false, true, "range"});
    const pg::Material steel = pg::palette("appliance");
    const double y = -d.depth * 0.5 - d.front * 0.5;
    const double panelHeight = 0.1;
    scene
        .box(root + "/controls",
             floor * pg::Pose::at(0.0, y, d.plinth + height - panelHeight * 0.5 - d.gap),
             {width - 2.0 * d.gap, d.front, panelHeight}, steel)
        .label("cabinet_panel")
        .group(root);
    const double doorHeight = height - panelHeight - 3.0 * d.gap;
    dropDoor(scene, root + "/oven_door", root,
             floor * pg::Pose::at(0.0, y, d.plinth + d.gap + doorHeight * 0.5), width - 2.0 * d.gap,
             doorHeight, d.front, steel);
    scene
        .box(root + "/cooktop",
             floor * pg::Pose::at(0.0, -0.01, d.counterHeight - d.counterThickness * 0.5),
             {width, d.depth + 0.02, d.counterThickness}, pg::palette("counter"))
        .label("cooktop")
        .group(root)
        .support("top", d.counterThickness * 0.5, pg::rectangle(width, d.depth, 0.05), 1.0);
    int burner = 0;
    for (const double bx : {-width * 0.22, width * 0.22}) {
        for (const double by : {-d.depth * 0.2, d.depth * 0.2}) {
            scene
                .cylinder(root + "/burner/" + std::to_string(burner++),
                          floor * pg::Pose::at(bx, by, d.counterHeight + 0.005), 0.18, 0.01,
                          pg::palette("metal"))
                .noCollision()
                .label("burner")
                .group(root);
        }
    }
}

inline void rangeHood(pg::Scene& scene, const std::string& root, const pg::Pose& floor,
                      double width, const Dimensions& d, double ceiling) {
    const pg::Material steel = pg::palette("metal");
    const double hoodDepth = 0.5;
    const double bottom = d.wallCabinetZ + 0.1;
    scene
        .box(root, floor * pg::Pose::at(0.0, d.depth * 0.5 - hoodDepth * 0.5, bottom + 0.125),
             {width, hoodDepth, 0.25}, steel)
        .label("range_hood");
    const double chimneyHeight = std::max(0.05, ceiling - bottom - 0.25);
    scene
        .box(root + "/chimney",
             floor * pg::Pose::at(0.0, d.depth * 0.5 - 0.125, bottom + 0.25 + chimneyHeight * 0.5),
             {0.3, 0.25, chimneyHeight}, steel)
        .label("range_hood")
        .group(root);
}

inline void dishwasher(pg::Scene& scene, const std::string& root, const pg::Pose& floor,
                       double width, const Dimensions& d) {
    const double height = d.counterHeight - d.counterThickness - d.plinth;
    carcass(scene, root, floor, width, d.depth, height, d.plinth, d,
            Carcass{true, false, "dishwasher"});
    const pg::Material steel = pg::palette("appliance");
    const double y = -d.depth * 0.5 - d.front * 0.5;
    dropDoor(scene, root + "/door", root, floor * pg::Pose::at(0.0, y, d.plinth + height * 0.5),
             width - 2.0 * d.gap, height - 2.0 * d.gap, d.front, steel);
    const double innerWidth = width - 2.0 * d.panel - 0.02;
    const double rackDepth = d.depth - d.panel - 0.06;
    const pg::Pose rack = floor * pg::Pose::at(0.0, -d.panel * 0.5, d.plinth + height * 0.35);
    scene.box(root + "/rack", rack, {innerWidth, rackDepth, 0.02}, pg::palette("metal"))
        .label("dishwasher_rack")
        .group(root)
        .mass(2.0)
        .support("top", 0.01, pg::rectangle(innerWidth, rackDepth, 0.02), 0.25);
    scene.joint(root + "/rack/slide", "prismatic", root, root + "/rack", rack.position,
                floor.rotation.rotate({0.0, -1.0, 0.0}), 0.0, rackDepth * 0.8);
}

inline void microwave(pg::Scene& scene, const std::string& root, const pg::Pose& counterTop,
                      const Dimensions& d) {
    const double width = 0.5;
    const double depth = 0.38;
    const double height = 0.3;
    const pg::Pose floor = counterTop * pg::Pose::at(0.0, 0.0, 0.0);
    carcass(scene, root, floor, width, depth, height, 0.0, d, Carcass{true, false, "microwave"});
    pg::hingeDoor(scene, root + "/door", root,
                  floor * pg::Pose::at(0.0, -depth * 0.5 - 0.01, height * 0.5), width - 0.01,
                  height - 0.01, 0.02, true, d.doorOpen, pg::palette("glass"),
                  pg::palette("handle"));
}

                                                                             

enum class Unit { Cabinet, Sink, Dishwasher, Range, Refrigerator, Filler };

struct Placed {
    Unit unit;
    double x0;
    double width;
};

inline bool hasCounter(Unit unit) {
    return unit == Unit::Cabinet || unit == Unit::Sink || unit == Unit::Dishwasher;
}

                                                                                    
                                                 
inline std::vector<Placed> planRun(pg::Rng& rng, double length, bool isMain) {
    std::vector<std::pair<Unit, double>> core;
    if (isMain) {
        std::vector<std::pair<Unit, double>> middle{{Unit::Sink, 0.9}, {Unit::Range, 0.76}};
        if (rng.chance(0.5)) {
            std::swap(middle[0], middle[1]);
        }
        const bool hasFridge = length >= 0.76 + 0.9 + 0.76 + 0.4;
        const bool hasDishwasher = length >= 0.76 + 0.9 + 0.76 + 0.6 + 0.8;
        const bool isFridgeFirst = rng.chance(0.5);
        if (hasFridge && isFridgeFirst) {
            core.emplace_back(Unit::Refrigerator, 0.76);
        }
        for (const auto& item : middle) {
            core.push_back(item);
            core.emplace_back(Unit::Cabinet, 0.0);
            if (item.first == Unit::Sink && hasDishwasher) {
                core.emplace_back(Unit::Dishwasher, 0.6);
            }
        }
        if (hasFridge && !isFridgeFirst) {
            core.emplace_back(Unit::Refrigerator, 0.76);
        }
    }
    double fixed = 0.0;
    for (const auto& item : core) {
        fixed += item.second;
    }
    std::vector<Placed> placed;
    double remaining = length - fixed;
    std::vector<double> fillers;
    const std::vector<double> widths{0.4, 0.5, 0.6, 0.8};
    while (remaining >= 0.4) {
        std::vector<double> fitting;
        for (const double width : widths) {
            if (width <= remaining + 1.0e-9) {
                fitting.push_back(width);
            }
        }
        const double width = rng.pick(fitting);
        fillers.push_back(width);
        remaining -= width;
    }
    std::vector<std::pair<Unit, double>> sequence;
    std::size_t nextFiller = 0U;
    const std::size_t slots = std::count_if(core.begin(), core.end(), [](const auto& item) {
        return item.first == Unit::Cabinet && item.second == 0.0;
    });
    for (const auto& item : core) {
        if (item.first == Unit::Cabinet && item.second == 0.0) {
            const std::size_t share = slots == 0U ? 0U : (fillers.size() + slots - 1U) / slots;
            for (std::size_t n = 0U; n < share && nextFiller < fillers.size(); ++n) {
                sequence.emplace_back(Unit::Cabinet, fillers[nextFiller++]);
            }
        } else {
            sequence.push_back(item);
        }
    }
    while (nextFiller < fillers.size()) {
        sequence.emplace_back(Unit::Cabinet, fillers[nextFiller++]);
    }
    if (remaining > 0.005) {
        sequence.emplace_back(Unit::Filler, remaining);
    }
    double x = 0.0;
    for (const auto& [unit, width] : sequence) {
        placed.push_back({unit, x, width});
        x += width;
    }
    return placed;
}

                                                                                    
                                                                               
inline void buildRun(pg::Scene& scene, const std::string& prefix, const pg::Pose& run,
                     const std::vector<Placed>& units, FrontStyle style, bool hasWallCabinets,
                     double ceiling, const Dimensions& d, bool hasMicrowave) {
    std::vector<std::pair<double, double>> sinks;
    double segmentStart = -1.0;
    int segment = 0;
    bool isMicrowavePlaced = !hasMicrowave;
    const auto closeSegment = [&](double end) {
        if (segmentStart >= 0.0) {
            countertop(scene, prefix + "/counter/" + std::to_string(segment++), run, segmentStart,
                       end, sinks, d);
            sinks.clear();
            segmentStart = -1.0;
        }
    };
    int index = 0;
    for (const Placed& placed : units) {
        const std::string key = prefix + "/unit/" + std::to_string(index++);
        const double centre = placed.x0 + placed.width * 0.5;
        const pg::Pose floor = run * pg::Pose::at(centre, 0.0, 0.0);
        if (hasCounter(placed.unit)) {
            if (segmentStart < 0.0) {
                segmentStart = placed.x0;
            }
        } else {
            closeSegment(placed.x0);
        }
        switch (placed.unit) {
        case Unit::Cabinet:
            baseCabinet(scene, key, floor, placed.width, style, d);
            if (!isMicrowavePlaced && placed.width >= 0.6) {
                microwave(scene, key + "/microwave",
                          run * pg::Pose::at(centre, d.depth * 0.5 - 0.19 - 0.05, d.counterHeight),
                          d);
                isMicrowavePlaced = true;
            }
            break;
        case Unit::Sink:
            sinkCabinet(scene, key, floor, placed.width, d);
            sinks.emplace_back(centre, placed.width);
            break;
        case Unit::Dishwasher:
            dishwasher(scene, key, floor, placed.width, d);
            break;
        case Unit::Range:
            range(scene, key, floor, placed.width, d);
            if (hasWallCabinets) {
                rangeHood(scene, key + "/hood", floor, placed.width, d, ceiling);
            }
            break;
        case Unit::Refrigerator:
            refrigerator(scene, key, floor, placed.width, d);
            break;
        case Unit::Filler:
            scene
                .box(key,
                     floor *
                         pg::Pose::at(0.0, -d.panel, (d.counterHeight - d.counterThickness) * 0.5),
                     {placed.width, d.depth - 2.0 * d.panel, d.counterHeight - d.counterThickness},
                     pg::palette("cabinet"))
                .label("filler");
            break;
        }
        const bool hasWallCabinet = hasWallCabinets && placed.unit != Unit::Range &&
                                    placed.unit != Unit::Refrigerator && placed.width >= 0.3;
        if (hasWallCabinet) {
            wallCabinet(scene, key + "/wall", floor, placed.width, d);
        }
    }
    closeSegment(units.empty() ? 0.0 : units.back().x0 + units.back().width);
}

inline double runLength(const std::vector<Placed>& units) {
    return units.empty() ? 0.0 : units.back().x0 + units.back().width;
}

inline void buildKitchen(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const std::string layout = settings.text("layout", "l_shaped");
    const double width = settings.number("room_width", 5.0, 3.0, 20.0);
    const double depthRoom = settings.number("room_depth", 6.0, 3.0, 20.0);
    const double ceiling = settings.number("room_height", 2.7, 2.2, 5.0);
    const double secondLength = settings.number("second_run_length", 2.4, 0.8, 10.0);
    const FrontStyle style = frontStyle(settings.text("front_style", "mixed"));
    const bool hasWallCabinets = settings.flag("wall_cabinets", true);
    const bool hasRoom = settings.flag("room", true);
    Dimensions d;
    d.counterHeight = settings.number("counter_height", 0.9, 0.7, 1.1);
    const double walkway = settings.number("walkway", 1.2, 0.6, 3.0);
    const bool isSpread = settings.text("spacing", "spread") != "compact";
    const double mainRunSetting = settings.number("main_run_length", 3.6, 0.0, 20.0);
    pg::Rng rng = scene.rng("layout");

    const double halfWidth = width * 0.5;
    const double halfDepth = depthRoom * 0.5;
    const double cornerGap = 0.05;
    const double fridgeFront = 0.7 + 0.05;
    const double runDepthWithFronts = std::max(d.depth + d.front, fridgeFront) + cornerGap;
    const bool isFullWall = mainRunSetting <= 0.0 || layout == "u_shaped" || layout == "peninsula";
    const double mainLength = isFullWall ? width : std::min(mainRunSetting, width);
    const bool isCornerRun = isFullWall || layout == "l_shaped" || !isSpread;
    const double mainStart = isCornerRun ? -halfWidth : -mainLength * 0.5;
    const double mainCentre = mainStart + mainLength * 0.5;
    const pg::Pose northRun = pg::Pose::at(mainStart, halfDepth - d.depth * 0.5, 0.0);
    const std::vector<Placed> mainUnits = planRun(rng, mainLength, true);
    buildRun(scene, "run/main", northRun, mainUnits, style, hasWallCabinets, ceiling, d, true);
    const double mainFront = halfDepth - d.depth - 0.02;

    pg::Side door = pg::Side::South;
    double doorOffset = 0.0;
    const auto sideRun = [&](const std::string& name, bool isWest, double length) {
        const double available = depthRoom - runDepthWithFronts - 0.6;
        const double used = std::min(length, available);
        if (used < 0.4) {
            scene.diagnostic("warning", "run_does_not_fit",
                             "The " + name + " run does not fit in the room depth",
                             "second_run_length", length, std::max(0.0, available), true);
            return;
        }
        const std::vector<Placed> units = planRun(rng, used, false);
        const double end = halfDepth - runDepthWithFronts;
        const pg::Pose frame =
            isWest ? pg::Pose::at(-halfWidth + d.depth * 0.5, end - runLength(units), 0.0,
                                  pg::kPi * 0.5)
                   : pg::Pose::at(halfWidth - d.depth * 0.5, end, 0.0, -pg::kPi * 0.5);
        buildRun(scene, "run/" + name, frame, units, style, hasWallCabinets, ceiling, d, false);
    };
    if (layout == "galley") {
        const std::vector<Placed> units = planRun(rng, mainLength, false);
        buildRun(scene, "run/south",
                 pg::Pose::at(mainStart + mainLength, -halfDepth + d.depth * 0.5, 0.0, pg::kPi),
                 units, style, hasWallCabinets, ceiling, d, false);
        const double southFront = -halfDepth + d.depth + 0.02;
        scene.region("aisle/galley", "aisle",
                     pg::Pose::at(mainCentre, (mainFront + southFront) * 0.5, 1.0),
                     {mainLength, mainFront - southFront, 2.0});
        door = pg::Side::East;
    } else if (layout == "l_shaped") {
        sideRun("west", true, secondLength);
        doorOffset = width * 0.25;
    } else if (layout == "u_shaped") {
        sideRun("west", true, secondLength);
        sideRun("east", false, secondLength);
    } else if (layout == "peninsula") {
        const std::vector<Placed> units =
            planRun(rng, std::min(secondLength, depthRoom * 0.5), false);
        const double x = halfWidth - walkway - d.depth * 0.5;
        buildRun(scene, "run/peninsula",
                 pg::Pose::at(x, halfDepth - runDepthWithFronts, 0.0, -pg::kPi * 0.5), units, style,
                 false, ceiling, d, false);
        doorOffset = -width * 0.25;
    } else if (layout == "island") {
        const double islandLength = std::min(2.4, width - 2.0 * walkway);
        if (islandLength >= 0.8) {
            const std::vector<Placed> units = planRun(rng, islandLength, false);
            const double freeDepth = mainFront + halfDepth - d.depth - 0.04;
            const double walkwayUsed = isSpread ? std::max(walkway, freeDepth * 0.5) : walkway;
            double centreY = mainFront - walkwayUsed - d.depth * 0.5 - 0.02;
            const double originClear = d.depth * 0.5 + 0.02 + 0.45;
            if (isSpread && std::abs(centreY) < originClear) {
                const bool hasNorthRoom = mainFront - originClear - d.depth * 0.5 - 0.02 >= walkway;
                centreY = centreY >= 0.0 && hasNorthRoom ? originClear : -originClear;
            }
            if (!isSpread && centreY < originClear) {
                scene.diagnostic(
                    "warning", "island_moved",
                    "The island was moved to keep the robot start at the origin clear; "
                    "the walkway is narrower than requested",
                    "walkway", walkway, mainFront - originClear - d.depth * 0.5, true);
                centreY = originClear;
            }
            buildRun(scene, "run/island", pg::Pose::at(islandLength * 0.5, centreY, 0.0, pg::kPi),
                     units, style, false, ceiling, d, false);
            const double islandFront = centreY + d.depth * 0.5 + 0.02;
            scene.region("aisle/island", "aisle",
                         pg::Pose::at(0.0, (mainFront + islandFront) * 0.5, 1.0),
                         {islandLength, mainFront - islandFront, 2.0});
        } else {
            scene.diagnostic("warning", "island_does_not_fit",
                             "The room is too narrow for an island with this walkway", "walkway",
                             2.0 * walkway + 0.8, width, true);
        }
    }
    scene.region("goal/main_counter", "goal", pg::Pose::at(mainCentre, mainFront - 0.6, 0.5),
                 {0.4, 0.4, 1.0});
    if (hasRoom) {
        pg::RoomSpec room;
        room.width = width;
        room.depth = depthRoom;
        room.height = ceiling;
        room.wallThickness = 0.12;
        room.doorSide = door;
        room.doorWidth = 0.9;
        room.doorHeight = std::min(2.05, ceiling - 0.1);
        room.doorOffset = doorOffset;
        pg::roomShell(scene, "room/", room, pg::palette("floor"), pg::palette("wall"));
    }
}

inline void buildCabinetModule(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const std::string type = settings.text("type", "base");
    const double width = settings.number("width", 0.6, 0.3, 1.2);
    Dimensions d;
    d.counterHeight = settings.number("counter_height", 0.9, 0.7, 1.1);
    const pg::Pose floor = pg::Pose::at(
        settings.number("x", 0.0, -100.0, 100.0), settings.number("y", 2.0, -100.0, 100.0), 0.0,
        settings.number("yaw", 0.0, -180.0, 180.0) * pg::kPi / 180.0);
    const FrontStyle style = frontStyle(settings.text("front_style", "mixed"));
    if (type == "wall") {
        wallCabinet(scene, "cabinet", floor, width, d);
        return;
    }
    if (type == "sink") {
        sinkCabinet(scene, "cabinet", floor, width, d);
    } else {
        baseCabinet(scene, "cabinet", floor, width, style, d);
    }
    if (settings.flag("counter", true)) {
        const pg::Pose run = floor * pg::Pose::at(-width * 0.5, 0.0, 0.0);
        std::vector<std::pair<double, double>> sinks;
        if (type == "sink") {
            sinks.emplace_back(width * 0.5, width);
        }
        countertop(scene, "counter", run, 0.0, width, sinks, d);
    }
}

inline void buildApplianceModule(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const std::string kind = settings.text("kind", "refrigerator");
    const double width = settings.number("width", 0.76, 0.4, 1.2);
    Dimensions d;
    const pg::Pose floor = pg::Pose::at(
        settings.number("x", 0.0, -100.0, 100.0), settings.number("y", 2.0, -100.0, 100.0), 0.0,
        settings.number("yaw", 0.0, -180.0, 180.0) * pg::kPi / 180.0);
    if (kind == "range") {
        range(scene, "appliance", floor, width, d);
    } else if (kind == "dishwasher") {
        dishwasher(scene, "appliance", floor, width, d);
    } else if (kind == "microwave") {
        microwave(scene, "appliance", floor * pg::Pose::at(0.0, 0.0, 0.0), d);
    } else {
        refrigerator(scene, "appliance", floor, width, d,
                     settings.number("height", 1.75, 1.2, 2.2));
    }
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

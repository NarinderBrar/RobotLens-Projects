#pragma once

#include <robotlens/procgen.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace warehousekit {

using robotlens::Json;
namespace pg = robotlens::procgen;

inline double radians(double degrees) {
    return degrees * pg::kPi / 180.0;
}

struct RackSpec {
    double width = 1.2;
    double depth = 0.6;
    double height = 1.8;
    int levels = 5;
    double board = 0.02;
    double upright = 0.03;
    double bottom = 0.1;
    double topClearance = 0.5;
};

inline RackSpec rackPreset(const std::string& name) {
    if (name == "pallet_rack") {
        return {2.7, 1.1, 3.0, 3, 0.05, 0.1, 0.15, 1.2};
    }
    if (name == "cubby") {
        return {0.8, 0.4, 1.2, 4, 0.02, 0.02, 0.05, 0.3};
    }
    if (name == "gondola") {
        return {1.2, 0.5, 1.6, 4, 0.02, 0.03, 0.12, 0.3};
    }
    return {};
}

                                                                                     
                                                                            
inline std::vector<pg::SurfaceRef> buildRack(pg::Scene& scene, const std::string& root,
                                             const pg::Pose& floorPose, const RackSpec& rack,
                                             double wearAmount = 0.0) {
    pg::Rng wearRng = scene.rng(root + "#wear");
    const double erosion = rack.upright + 0.02;
    const double spacing = rack.levels > 1 ? (rack.height - rack.board * 0.5 - rack.bottom) /
                                                 static_cast<double>(rack.levels - 1)
                                           : 0.0;
    std::vector<pg::SurfaceRef> surfaces;
    const pg::Material boardMaterial =
        wearAmount > 0.0 ? pg::wear(pg::palette("shelf_board"), wearRng, wearAmount)
                         : pg::palette("shelf_board");
    for (int level = 0; level < rack.levels; ++level) {
        const double z = rack.bottom + spacing * level;
        const double clear = level + 1 < rack.levels ? spacing - rack.board : rack.topClearance;
        const std::string key = level == 0 ? root : root + "/level/" + std::to_string(level);
        const pg::Pose pose = floorPose * pg::Pose::at(0.0, 0.0, z);
        pg::Object board = pg::shelfBoard(scene, key, pose, rack.width, rack.depth, rack.board,
                                          clear, erosion, boardMaterial);
        if (level > 0) {
            board.group(root);
        }
        if (clear > 0.02) {
            board.region("slot", "containment", {0.0, 0.0, rack.board * 0.5 + clear * 0.5},
                         {rack.width - 2.0 * erosion, rack.depth - 2.0 * erosion, clear});
            surfaces.push_back(
                pg::boardSurface(key, pose, rack.width, rack.depth, rack.board, clear, erosion));
        }
    }
    const double hx = rack.width * 0.5 - rack.upright * 0.5;
    const double hy = rack.depth * 0.5 - rack.upright * 0.5;
    const pg::Material uprightMaterial = wearAmount > 0.0
                                             ? pg::wear(pg::palette("upright"), wearRng, wearAmount)
                                             : pg::palette("upright");
    int index = 0;
    for (const double sx : {-1.0, 1.0}) {
        for (const double sy : {-1.0, 1.0}) {
            scene
                .box(pg::key(root + "/upright", index++),
                     floorPose * pg::Pose::at(sx * hx, sy * hy, rack.height * 0.5),
                     {rack.upright, rack.upright, rack.height}, uprightMaterial)
                .label("upright")
                .group(root);
        }
    }
    return surfaces;
}

inline void fillShelves(pg::Scene& scene, pg::Placer& placer, const std::string& prefix,
                        const std::vector<pg::SurfaceRef>& surfaces, const RackSpec& rack,
                        double density, double tallShare = 0.0, double wearAmount = 0.0) {
    pg::Rng wearRng = scene.rng(prefix + "#wear");
    const pg::Material boxMaterial =
        wearAmount > 0.0 ? pg::wear(pg::palette("box"), wearRng, wearAmount) : pg::palette("box");
    if (density <= 0.0 || surfaces.empty()) {
        return;
    }
    double clear = surfaces.front().clearHeight;
    for (const pg::SurfaceRef& surface : surfaces) {
        clear = std::min(clear, surface.clearHeight);
    }
    const double usableWidth = rack.width - 2.0 * (rack.upright + 0.02);
    const double usableDepth = rack.depth - 2.0 * (rack.upright + 0.02);
    const double maxHeight = std::min(0.4, clear * 0.8);
    if (maxHeight < 0.03 || usableWidth < 0.1 || usableDepth < 0.1) {
        scene.diagnostic("info", "shelves_too_small",
                         "The shelves are too small for boxes; none were placed", "box_density",
                         density, 0.0, true);
        return;
    }
    const pg::Vec3 maxSize{std::min(0.45, usableWidth * 0.8), std::min(0.4, usableDepth * 0.9),
                           maxHeight};
    const pg::Vec3 minSize{std::min(0.15, maxSize.x), std::min(0.12, maxSize.y),
                           std::min(0.1, maxHeight * 0.6)};
    const double tall = std::clamp(tallShare, 0.0, 1.0);
    if (tall > 0.0 && clear * 0.9 > maxHeight) {
        const pg::Vec3 tallMax{maxSize.x, maxSize.y, clear * 0.9};
        const pg::Vec3 tallMin{minSize.x, minSize.y, std::max(minSize.z, clear * 0.7)};
        placer.populate(prefix + "/tall", surfaces, density * tall, tallMin, tallMax, boxMaterial);
    }
    placer.populate(prefix, surfaces, density * (1.0 - tall), minSize, maxSize, boxMaterial);
}

inline void buildRackModule(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const std::string preset = settings.text("preset", "wire_rack");
    RackSpec rack = rackPreset(preset);
    if (preset == "custom") {
        rack.width = settings.number("width", 1.2, 0.3, 6.0);
        rack.depth = settings.number("depth", 0.6, 0.2, 3.0);
        rack.height = settings.number("height", 1.8, 0.3, 12.0);
        rack.levels = settings.integer("levels", 5, 1, 12);
    }
    const pg::Pose floor = pg::Pose::at(settings.number("x", 2.0, -100.0, 100.0),
                                        settings.number("y", 2.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const auto surfaces = buildRack(scene, "rack", floor, rack);
    pg::Placer placer(scene);
    fillShelves(scene, placer, "rack/box", surfaces, rack,
                settings.number("box_density", 0.4, 0.0, 0.9));
}

inline void buildPallet(pg::Scene& scene, pg::Placer& placer, const std::string& root,
                        const pg::Pose& floorPose, bool isEuro, double loadDensity) {
    const double length = isEuro ? 1.2 : 1.219;
    const double width = isEuro ? 0.8 : 1.016;
    const double height = isEuro ? 0.144 : 0.14;
    const double deck = 0.022;
    const pg::Pose centre = floorPose * pg::Pose::at(0.0, 0.0, height * 0.5);
    const pg::Material wood = pg::palette("pallet");
    scene.collider(root, centre, {length, width, height})
        .label("pallet")
        .mass(isEuro ? 25.0 : 22.0)
        .support("top", height * 0.5, pg::rectangle(length, width, 0.02), 1.5);
    const auto visual = [&](const std::string& key, pg::Vec3 position, pg::Vec3 size) {
        scene
            .box(root + "/" + key, floorPose * pg::Pose::at(position.x, position.y, position.z),
                 size, wood)
            .noCollision()
            .label("pallet_part")
            .group(root);
    };
    const std::vector<double> deckY = pg::distribute(-width * 0.5, width * 0.5, 5);
    for (std::size_t i = 0U; i < deckY.size(); ++i) {
        visual("deck/" + std::to_string(i), {0.0, deckY[i], height - deck * 0.5},
               {length, width / 5.0 * 0.8, deck});
    }
    const std::vector<double> blockX = pg::distribute(-length * 0.5, length * 0.5, 3);
    const std::vector<double> blockY = pg::distribute(-width * 0.5, width * 0.5, 3);
    const double blockHeight = height - 2.0 * deck;
    int block = 0;
    for (const double bx : blockX) {
        for (const double by : blockY) {
            visual("block/" + std::to_string(block++), {bx, by, deck + blockHeight * 0.5},
                   {0.145, 0.145, blockHeight});
        }
    }
    for (std::size_t i = 0U; i < blockY.size(); ++i) {
        visual("runner/" + std::to_string(i), {0.0, blockY[i], deck * 0.5}, {length, 0.145, deck});
    }
    if (loadDensity > 0.0) {
        const pg::SurfaceRef top{root, "top",
                                 centre * pg::Pose{{0.0, 0.0, height * 0.5}, pg::Quat{}},
                                 pg::rectangle(length, width, 0.02), 1.5};
        placer.populate(root + "/load", {top}, loadDensity, {0.25, 0.2, 0.15}, {0.5, 0.4, 0.4},
                        pg::palette("box"));
    }
}

inline void buildPalletModule(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const bool isEuro = settings.text("standard", "euro") != "us";
    const int count = settings.integer("count", 2, 1, 50);
    const double density = settings.number("load_density", 0.5, 0.0, 0.9);
    const double spacing = settings.number("spacing", 0.2, 0.0, 5.0);
    const pg::Pose start = pg::Pose::at(settings.number("x", -3.0, -100.0, 100.0),
                                        settings.number("y", 2.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const double length = isEuro ? 1.2 : 1.219;
    pg::Placer placer(scene);
    for (int index = 0; index < count; ++index) {
        buildPallet(scene, placer, pg::key("pallet", index),
                    start * pg::Pose::at(index * (length + spacing), 0.0, 0.0), isEuro, density);
    }
}

inline void buildBinModule(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double width = settings.number("width", 0.6, 0.1, 3.0);
    const double depth = settings.number("depth", 0.4, 0.1, 3.0);
    const double height = settings.number("height", 0.3, 0.05, 2.0);
    const double thickness = settings.number("thickness", 0.01, 0.002, 0.1);
    const int items = settings.integer("items", 3, 0, 50);
    const pg::Pose floor = pg::Pose::at(settings.number("x", -2.0, -100.0, 100.0),
                                        settings.number("y", -2.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    if (2.0 * thickness >= std::min(width, depth) || thickness >= height) {
        scene.diagnostic("error", "walls_too_thick", "The bin walls leave no space inside",
                         "thickness", thickness, std::min(width, depth) * 0.5, true);
        return;
    }
    const pg::Material material = pg::palette("bin");
    const double inside = height - thickness;
    const pg::Pose bottomPose = floor * pg::Pose::at(0.0, 0.0, thickness * 0.5);
    scene.box("bin", bottomPose, {width, depth, thickness}, material)
        .label("bin")
        .mass(1.0)
        .support("inside", thickness * 0.5,
                 pg::rectangle(width - 2.0 * thickness, depth - 2.0 * thickness, 0.005), inside)
        .region("inside", "containment", {0.0, 0.0, thickness * 0.5 + inside * 0.5},
                {width - 2.0 * thickness, depth - 2.0 * thickness, inside});
    const auto wall = [&](const std::string& key, pg::Vec3 position, pg::Vec3 size) {
        scene
            .box("bin/" + key, floor * pg::Pose::at(position.x, position.y, position.z), size,
                 material)
            .label("bin_wall")
            .group("bin");
    };
    const double wallZ = thickness + inside * 0.5;
    wall("front", {0.0, -(depth - thickness) * 0.5, wallZ}, {width, thickness, inside});
    wall("back", {0.0, (depth - thickness) * 0.5, wallZ}, {width, thickness, inside});
    wall("left", {-(width - thickness) * 0.5, 0.0, wallZ},
         {thickness, depth - 2.0 * thickness, inside});
    wall("right", {(width - thickness) * 0.5, 0.0, wallZ},
         {thickness, depth - 2.0 * thickness, inside});

    const pg::SurfaceRef surface{
        "bin", "inside", bottomPose * pg::Pose{{0.0, 0.0, thickness * 0.5}, pg::Quat{}},
        pg::rectangle(width - 2.0 * thickness, depth - 2.0 * thickness, 0.005), inside};
    pg::Placer placer(scene);
    const double itemMax =
        std::min({0.2, (width - 2.0 * thickness) * 0.45, (depth - 2.0 * thickness) * 0.45});
    int placed = 0;
    for (int index = 0; index < items; ++index) {
        const std::string key = pg::key("bin/item", index);
        pg::Rng rng = scene.rng(key + "#size");
        const double itemHeight = std::min(inside * 0.9, 0.15);
        const pg::Vec3 size =
            pg::snapSize({rng.uniform(itemMax * 0.5, itemMax), rng.uniform(itemMax * 0.5, itemMax),
                          std::min(inside * 0.9, rng.uniform(0.04, 0.15))},
                         {itemMax * 0.5, itemMax * 0.5, std::min(0.04, itemHeight)},
                         {itemMax, itemMax, itemHeight});
        pg::Pose pose;
        if (!placer.place(key, size, {surface}, pose)) {
            continue;
        }
        scene.box(key, pose, size, pg::vary(pg::palette("box"), rng, 0.2)).label("box").mass(0.3);
        ++placed;
    }
    if (placed < items) {
        scene.diagnostic("info", "bin_full",
                         "Placed " + std::to_string(placed) + " of " + std::to_string(items) +
                             " items; the bin is full",
                         "items", items, placed, true);
    }
}

inline void buildBoxModule(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const pg::Vec3 size{settings.number("width", 0.4, 0.05, 3.0),
                        settings.number("depth", 0.3, 0.05, 3.0),
                        settings.number("height", 0.3, 0.05, 3.0)};
    const int columns = settings.integer("columns", 2, 1, 10);
    const int rows = settings.integer("rows", 1, 1, 10);
    const int layers = settings.integer("layers", 2, 1, 10);
    const pg::Pose floor = pg::Pose::at(settings.number("x", 3.0, -100.0, 100.0),
                                        settings.number("y", -2.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const double gap = 0.005;
    for (int layer = 0; layer < layers; ++layer) {
        for (int row = 0; row < rows; ++row) {
            for (int column = 0; column < columns; ++column) {
                const std::string key = "box/" + std::to_string(column) + "/" +
                                        std::to_string(row) + "/" + std::to_string(layer);
                pg::Rng rng = scene.rng(key);
                const double x = (column - (columns - 1) * 0.5) * (size.x + gap);
                const double y = (row - (rows - 1) * 0.5) * (size.y + gap);
                const double z = layer * (size.z + 0.0005) + size.z * 0.5;
                scene
                    .box(key, floor * pg::Pose::at(x, y, z), size,
                         pg::vary(pg::palette("box"), rng, 0.12))
                    .label("box")
                    .mass(std::max(0.2, size.x * size.y * size.z * 150.0));
            }
        }
    }
}

struct AisleBand {
    double centreY = 0.0;
    double width = 0.0;
};

                                                                                
                                                  
inline double placeClutter(pg::Scene& scene, const pg::Pose& origin,
                           const std::vector<AisleBand>& aisles, double rowLength, int count,
                           double minPassage) {
    if (count <= 0 || aisles.empty()) {
        return aisles.empty() ? 0.0 : aisles.front().width;
    }
    std::vector<std::vector<std::pair<double, double>>> taken(aisles.size());
    double narrowest = aisles.front().width;
    for (const AisleBand& aisle : aisles) {
        narrowest = std::min(narrowest, aisle.width);
    }
    int placed = 0;
    for (int index = 0; index < count; ++index) {
        const std::string key = pg::key("clutter", index);
        pg::Rng rng = scene.rng(key);
        for (int attempt = 0; attempt < 30; ++attempt) {
            const auto band =
                static_cast<std::size_t>(rng.integer(0, static_cast<int>(aisles.size()) - 1));
            const AisleBand& aisle = aisles[band];
            const double maxDepth = std::min(0.6, aisle.width - minPassage);
            if (maxDepth < 0.15) {
                continue;
            }
            const double depth = rng.uniform(0.15, maxDepth);
            const double length = rng.uniform(0.3, std::min(1.0, rowLength * 0.5));
            const double height = rng.uniform(0.3, 1.2);
            const double x =
                rng.uniform(-rowLength * 0.5 + length * 0.5, rowLength * 0.5 - length * 0.5);
            const double side = rng.chance(0.5) ? 1.0 : -1.0;
            const double y = aisle.centreY + side * (aisle.width * 0.5 - depth * 0.5);
            if (std::sqrt(x * x + y * y) < 1.2) {
                continue;
            }
            const auto overlaps = [&](const std::pair<double, double>& interval) {
                return x - length * 0.5 - 0.3 < interval.second &&
                       interval.first < x + length * 0.5 + 0.3;
            };
            if (std::any_of(taken[band].begin(), taken[band].end(), overlaps)) {
                continue;
            }
            taken[band].emplace_back(x - length * 0.5, x + length * 0.5);
            scene
                .box(key, origin * pg::Pose::at(x, y, height * 0.5), {length, depth, height},
                     pg::vary(pg::palette("obstacle"), rng, 0.15))
                .label("obstacle");
            narrowest = std::min(narrowest, aisle.width - depth);
            ++placed;
            break;
        }
    }
    if (placed < count) {
        scene.diagnostic("warning", "clutter_partial",
                         "Placed " + std::to_string(placed) + " of " + std::to_string(count) +
                             " clutter objects without going below the minimum passage",
                         "clutter_count", count, placed, true);
    }
    return narrowest;
}

inline void buildWarehouse(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const RackSpec rack = rackPreset(settings.text("rack_preset", "wire_rack"));
    const int rows = settings.integer("rows", 4, 1, 12);
    const int perRow = settings.integer("racks_per_row", 5, 1, 30);
    const double aisle = settings.number("aisle_width", 1.6, 0.3, 8.0);
    const double endAisle = settings.number("end_aisle_width", 1.5, 0.3, 8.0);
    const double density = settings.number("box_density", 0.35, 0.0, 0.9);
    const double wallHeight = settings.number("wall_height", 4.0, 1.0, 15.0);
    const double doorWidth = settings.number("doorway_width", 2.0, 0.0, 10.0);
    const bool hasRoom = settings.flag("room", true);
    const pg::Pose origin = pg::Pose::at(settings.number("offset_x", 0.0, -200.0, 200.0),
                                         settings.number("offset_y", 0.0, -200.0, 200.0), 0.0);
    const int clutter = settings.integer("clutter_count", 0, 0, 100);
    const double minPassage = settings.number("min_passage", 0.9, 0.3, 4.0);
    const double tallShare = settings.number("tall_box_share", 0.0, 0.0, 1.0);
    const double wearAmount = settings.number("wear", 0.0, 0.0, 1.0);
    const bool isSpread = settings.text("spacing", "spread") != "compact";
    const double rackGap = 0.05;

    const int northRows = (rows + 1) / 2;
    const double neededWidth = perRow * rack.width + (perRow - 1) * rackGap + 2.0 * endAisle;
    const double neededDepth = 2.0 * (northRows * rack.depth + (northRows + 0.5) * aisle);
    const auto roomSize = [&scene, &settings](const char* setting, double needed) {
        const double requested = settings.number(setting, 0.0, 0.0, 200.0);
        if (requested <= 0.0) {
            return needed;
        }
        if (requested + 1.0e-6 < needed) {
            scene.diagnostic("error", "room_too_small",
                             std::string("The racks and aisles need ") + std::to_string(needed) +
                                 " m but " + setting + " is " + std::to_string(requested) +
                                 " m; set it to 0 to fit the room automatically",
                             setting, needed, requested, true);
        }
        return requested;
    };
    pg::RoomSpec room;
    room.width = roomSize("room_width", neededWidth);
    room.depth = roomSize("room_depth", neededDepth);
    room.height = wallHeight;
    room.wallThickness = 0.2;
    room.doorSide = pg::Side::South;
    room.doorWidth = doorWidth;
    room.doorHeight = std::min(2.5, wallHeight - 0.2);
    if (hasRoom) {
        pg::roomShell(scene, "room/", room, pg::palette("floor"), pg::palette("wall"), origin);
    }

    const std::vector<double> columnX =
        pg::spread(std::vector<double>(perRow, rack.width), room.width, rackGap, endAisle, isSpread)
            .centres;
    const double rowLength = columnX.back() - columnX.front() + rack.width;
    const double rowGap = isSpread && room.depth > neededDepth
                              ? (room.depth * 0.5 - northRows * rack.depth) / (northRows + 0.5)
                              : aisle;
    std::vector<double> rowY;
    for (int row = rows - northRows - 1; row >= 0; --row) {
        rowY.push_back(-(rowGap * 0.5 + rack.depth * 0.5 + row * (rack.depth + rowGap)));
    }
    for (int row = 0; row < northRows; ++row) {
        rowY.push_back(rowGap * 0.5 + rack.depth * 0.5 + row * (rack.depth + rowGap));
    }

    pg::Placer placer(scene);
    for (int row = 0; row < rows; ++row) {
        for (int column = 0; column < perRow; ++column) {
            const double x = columnX[column];
            const std::string root = "rack/" + std::to_string(row) + "/" + std::to_string(column);
            const auto surfaces =
                buildRack(scene, root, origin * pg::Pose::at(x, rowY[row], 0.0), rack, wearAmount);
            fillShelves(scene, placer, root + "/box", surfaces, rack, density, tallShare,
                        wearAmount);
        }
    }

    const double aisleHeight = 2.0;
    const auto aisleRegion = [&scene, &origin, aisleHeight](const std::string& name, double cx,
                                                            double cy, double sx, double sy) {
        if (sx > 0.05 && sy > 0.05) {
            scene.region(name, "aisle", origin * pg::Pose::at(cx, cy, aisleHeight * 0.5),
                         {sx, sy, aisleHeight});
        }
    };
    std::vector<AisleBand> bands;
    for (int row = 0; row + 1 < rows; ++row) {
        const double low = rowY[row] + rack.depth * 0.5;
        const double high = rowY[row + 1] - rack.depth * 0.5;
        aisleRegion("aisle/" + std::to_string(row), 0.0, (low + high) * 0.5, rowLength, high - low);
        bands.push_back({(low + high) * 0.5, high - low});
    }
    const double southEdge = -room.depth * 0.5;
    const double northEdge = room.depth * 0.5;
    const double firstLow = rowY.front() - rack.depth * 0.5;
    const double lastHigh = rowY.back() + rack.depth * 0.5;
    aisleRegion("aisle/south", 0.0, (southEdge + firstLow) * 0.5, rowLength, firstLow - southEdge);
    aisleRegion("aisle/north", 0.0, (lastHigh + northEdge) * 0.5, rowLength, northEdge - lastHigh);
    const double westEdge = -room.width * 0.5;
    const double eastEdge = room.width * 0.5;
    aisleRegion("aisle/west", (westEdge - rowLength * 0.5) * 0.5, 0.0, -rowLength * 0.5 - westEdge,
                room.depth);
    aisleRegion("aisle/east", (rowLength * 0.5 + eastEdge) * 0.5, 0.0, eastEdge - rowLength * 0.5,
                room.depth);
    for (const double side : {-1.0, 1.0}) {
        const double x =
            side * (rowLength * 0.5 + std::min(endAisle, room.width * 0.5 - rowLength * 0.5) * 0.5);
        scene.region(side < 0.0 ? "goal/west" : "goal/east", "goal",
                     origin * pg::Pose::at(x, 0.0, 0.5), {0.4, 0.4, 1.0});
    }
    if (clutter > 0) {
        const double narrowest = placeClutter(scene, origin, bands, rowLength, clutter, minPassage);
        scene.diagnostic("info", "difficulty",
                         "Clutter leaves a narrowest aisle passage of " +
                             std::to_string(narrowest) + " m (minimum requested " +
                             std::to_string(minPassage) + " m); tall box share " +
                             std::to_string(tallShare),
                         "min_passage", minPassage, narrowest, true);
    }
    scene.diagnostic("info", "layout",
                     "Built " + std::to_string(rows) + " rows of " + std::to_string(perRow) +
                         " racks in a " + std::to_string(room.width) + " x " +
                         std::to_string(room.depth) + " m room with " + std::to_string(rowGap) +
                         " m aisles (" + (isSpread ? "spread" : "compact") +
                         "); the layout origin is inside an aisle");
}

inline void buildConveyor(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double length = settings.number("length", 4.0, 0.5, 30.0);
    const double width = settings.number("width", 0.6, 0.2, 2.0);
    const double height = settings.number("height", 0.8, 0.2, 2.0);
    const int items = settings.integer("items", 4, 0, 60);
    const pg::Pose floor = pg::Pose::at(settings.number("x", -3.0, -100.0, 100.0),
                                        settings.number("y", -2.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const double belt = 0.06;
    const pg::Pose top = floor * pg::Pose::at(0.0, 0.0, height - belt * 0.5);
    scene.box("conveyor", top, {length, width, belt}, pg::palette("conveyor"))
        .label("conveyor")
        .support("belt", belt * 0.5, pg::rectangle(length, width, 0.03), 1.0);
    for (const double side : {-1.0, 1.0}) {
        scene
            .box(side < 0.0 ? "conveyor/rail/left" : "conveyor/rail/right",
                 floor * pg::Pose::at(0.0, side * (width * 0.5 + 0.015), height + 0.02),
                 {length, 0.03, 0.1}, pg::palette("metal"))
            .label("conveyor_rail")
            .group("conveyor");
    }
    const int pairs = std::max(2, static_cast<int>(std::ceil(length / 1.5)) + 1);
    const std::vector<double> xs = pg::distribute(-length * 0.5, length * 0.5, pairs);
    int leg = 0;
    for (const double x : xs) {
        for (const double side : {-1.0, 1.0}) {
            scene
                .box(pg::key("conveyor/leg", leg++),
                     floor * pg::Pose::at(x, side * (width * 0.5 - 0.03), (height - belt) * 0.5),
                     {0.05, 0.05, height - belt}, pg::palette("metal"))
                .label("leg")
                .group("conveyor");
        }
    }
    if (items > 0) {
        pg::Placer placer(scene);
        const pg::SurfaceRef surface{"conveyor", "belt",
                                     top * pg::Pose{{0.0, 0.0, belt * 0.5}, pg::Quat{}},
                                     pg::rectangle(length, width, 0.03), 1.0};
        const double itemMax = std::min(0.45, width * 0.8);
        int placed = 0;
        for (int index = 0; index < items; ++index) {
            const std::string key = pg::key("conveyor/item", index);
            pg::Rng rng = scene.rng(key + "#size");
            const pg::Vec3 size = pg::snapSize(
                {rng.uniform(0.15, itemMax), rng.uniform(0.15, itemMax), rng.uniform(0.1, 0.4)},
                {std::min(0.15, itemMax), std::min(0.15, itemMax), 0.1}, {itemMax, itemMax, 0.4});
            pg::Pose pose;
            if (placer.place(key, size, {surface}, pose)) {
                scene.box(key, pose, size, pg::vary(pg::palette("box"), rng, 0.15))
                    .label("box")
                    .mass(std::max(0.2, size.x * size.y * size.z * 150.0));
                ++placed;
            }
        }
        if (placed < items) {
            scene.diagnostic("info", "conveyor_full",
                             "Placed " + std::to_string(placed) + " of " + std::to_string(items) +
                                 " items on the belt",
                             "items", items, placed, true);
        }
    }
}

inline void buildBarrier(pg::Scene& scene) {
    pg::Settings& settings = scene.settings();
    const double length = settings.number("length", 5.0, 0.5, 100.0);
    const double height = settings.number("height", 1.0, 0.3, 3.0);
    const double spacing = settings.number("post_spacing", 1.5, 0.3, 10.0);
    const pg::Pose floor = pg::Pose::at(settings.number("x", 0.0, -100.0, 100.0),
                                        settings.number("y", -3.0, -100.0, 100.0), 0.0,
                                        radians(settings.number("yaw", 0.0, -180.0, 180.0)));
    const pg::Material material = pg::palette("barrier");
    const int posts = std::max(2, static_cast<int>(std::ceil(length / spacing)) + 1);
    for (int index = 0; index < posts; ++index) {
        const double x = -length * 0.5 + length * index / (posts - 1);
        pg::Object post =
            scene.cylinder(index == 0 ? std::string("barrier") : pg::key("barrier/post", index),
                           floor * pg::Pose::at(x, 0.0, height * 0.5), 0.08, height, material);
        post.label("barrier");
        if (index > 0) {
            post.group("barrier");
        }
    }
    for (const double z : {height * 0.5, height - 0.03}) {
        scene
            .box(z < height * 0.75 ? "barrier/rail/mid" : "barrier/rail/top",
                 floor * pg::Pose::at(0.0, 0.0, z), {length, 0.04, 0.06}, material)
            .label("barrier")
            .group("barrier");
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

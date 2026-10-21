import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]


def numbers(element, name, default):
    return tuple(float(value) for value in element.get(name, default).split())


def compose(parent, element):
    px, py, pz, heading = parent
    x, y, z = numbers(element, "pos", "0 0 0")
    w, qx, qy, qz = numbers(element, "quat", "1 0 0 0")
    if any(element.get(key) for key in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Map geometry requires planar quaternion rotations")
    if abs(qx) > 1e-6 or abs(qy) > 1e-6 or abs(w * w + qz * qz - 1) > 1e-5:
        raise ValueError("Map geometry requires normalized planar quaternions")
    cosine, sine = math.cos(heading), math.sin(heading)
    return (px + cosine * x - sine * y, py + sine * x + cosine * y,
            pz + z, heading + 2 * math.atan2(qz, w))


def collision_boxes(world):
    root = ET.parse(world).getroot()
    if root.find("include") is not None or root.find("default") is not None:
        raise ValueError("Expand included geometry and defaults before generating the map")
    boxes = []

    def visit(element, parent):
        for child in element:
            if child.tag == "body":
                if child.find("freejoint") is not None or child.find("joint") is not None:
                    raise ValueError("The global map requires static bodies")
                visit(child, compose(parent, child))
            elif child.tag == "geom":
                if not int(child.get("contype", "1")) and not int(child.get("conaffinity", "1")):
                    continue
                if child.get("type") != "box" or child.get("fromto"):
                    raise ValueError("The global map currently supports collision boxes only")
                size = numbers(child, "size", "")
                if len(size) != 3 or any(value <= 0 for value in size):
                    raise ValueError("Collision boxes require three positive half sizes")
                box = (*compose(parent, child), *size)
                if not all(math.isfinite(value) for value in box):
                    raise ValueError("Map geometry must be finite")
                boxes.append(box)

    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("No worldbody found")
    visit(worldbody, (0, 0, 0, 0))
    return boxes


def build_map(world, resolution=0.05, robot_height=0.5):
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("Resolution must be positive and finite")
    if not math.isfinite(robot_height) or robot_height <= 0.05:
        raise ValueError("Robot height must exceed the floor tolerance")
    boxes = collision_boxes(world)
    floors = [box for box in boxes if box[2] + box[6] <= 0.05]
    if not floors:
        raise ValueError("No floor collision found for map bounds")
    floor = max(floors, key=lambda box: box[4] * box[5])
    x, y, _, yaw, sx, sy, _ = floor
    if abs(math.sin(yaw)) > 1e-6:
        raise ValueError("Floor bounds must be axis-aligned")
    origin = (x - sx, y - sy)
    width = math.ceil(2 * sx / resolution)
    height = math.ceil(2 * sy / resolution)
    if width * height > 1_000_000:
        raise ValueError("Map exceeds the navigation stack limit of 1000000 cells")
    occupied = [0] * (width * height)
    padding = resolution * math.sqrt(2) / 2
    for x, y, z, yaw, sx, sy, sz in boxes:
        if z + sz <= 0.05 or z - sz >= robot_height:
            continue
        cosine, sine = math.cos(yaw), math.sin(yaw)
        radius_x = abs(cosine) * sx + abs(sine) * sy + padding
        radius_y = abs(sine) * sx + abs(cosine) * sy + padding
        left = max(0, math.floor((x - radius_x - origin[0]) / resolution))
        right = min(width, math.ceil((x + radius_x - origin[0]) / resolution))
        bottom = max(0, math.floor((y - radius_y - origin[1]) / resolution))
        top = min(height, math.ceil((y + radius_y - origin[1]) / resolution))
        for row in range(bottom, top):
            dy = origin[1] + (row + 0.5) * resolution - y
            for column in range(left, right):
                dx = origin[0] + (column + 0.5) * resolution - x
                if (abs(cosine * dx + sine * dy) <= sx + padding
                        and abs(-sine * dx + cosine * dy) <= sy + padding):
                    occupied[row * width + column] = 100
    return {"contract": 1, "frame": "map", "width": width, "height": height,
            "resolution": resolution, "origin": {"x": origin[0], "y": origin[1], "yaw": 0},
            "data": occupied}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, default=PROJECT / "worlds/warehouse.xml")
    parser.add_argument("--output", type=Path, default=PROJECT / "maps/warehouse.json")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--robot-height", type=float, default=0.5)
    args = parser.parse_args()
    grid = build_map(args.world, args.resolution, args.robot_height)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(grid, separators=(",", ":")) + "\n")
    print(f"Generated {grid['width']} x {grid['height']} global map: {args.output}", flush=True)


if __name__ == "__main__":
    main()

import math
from pathlib import Path
import xml.etree.ElementTree as ET


PROJECT = Path(__file__).resolve().parents[1]
RESOLUTION = 0.05


def values(element, name, default):
    return tuple(float(value) for value in element.get(name, default).split())


def compose(parent, element):
    px, py, pz, heading = parent
    x, y, z = values(element, 'pos', '0 0 0')
    w, qx, qy, qz = values(element, 'quat', '1 0 0 0')
    if abs(qx) > 1e-6 or abs(qy) > 1e-6 or element.get('euler') or element.get('axisangle'):
        raise ValueError('The warehouse map requires planar quaternion rotations')
    yaw = 2 * math.atan2(qz, w)
    c, s = math.cos(heading), math.sin(heading)
    return px + c * x - s * y, py + s * x + c * y, pz + z, heading + yaw


def collision_boxes(world):
    root = ET.parse(world).getroot()
    boxes = []

    def visit(element, parent):
        for child in element:
            if child.tag == 'body':
                if child.find('freejoint') is not None or child.find('joint') is not None:
                    raise ValueError('The warehouse map must contain only static bodies')
                visit(child, compose(parent, child))
            elif child.tag == 'geom':
                if not int(child.get('contype', '1')) and not int(child.get('conaffinity', '1')):
                    continue
                if child.get('type', 'sphere') != 'box':
                    raise ValueError('Unsupported warehouse collision geometry')
                boxes.append((*compose(parent, child), *values(child, 'size', '')))

    visit(root.find('worldbody'), (0, 0, 0, 0))
    return boxes


def build_map(world, resolution=RESOLUTION, robot_height=1.8):
    boxes = collision_boxes(world)
    floors = [box for box in boxes if box[2] + box[6] <= 0.05]
    if not floors:
        raise ValueError('No floor collision found for map bounds')
    floor = max(floors, key=lambda box: box[4] * box[5])
    x, y, _, yaw, sx, sy, _ = floor
    if abs(math.sin(yaw)) > 1e-6:
        raise ValueError('Warehouse floor bounds must be axis-aligned')
    origin = (x - sx, y - sy)
    width = math.ceil(2 * sx / resolution)
    height = math.ceil(2 * sy / resolution)
    occupied = bytearray(width * height)
    padding = resolution * math.sqrt(2) / 2
    for x, y, z, yaw, sx, sy, sz in boxes:
        if z + sz <= 0.05 or z - sz >= robot_height:
            continue
        c, s = math.cos(yaw), math.sin(yaw)
        rx, ry = abs(c) * sx + abs(s) * sy + padding, abs(s) * sx + abs(c) * sy + padding
        left = max(0, math.floor((x - rx - origin[0]) / resolution))
        right = min(width, math.ceil((x + rx - origin[0]) / resolution))
        bottom = max(0, math.floor((y - ry - origin[1]) / resolution))
        top = min(height, math.ceil((y + ry - origin[1]) / resolution))
        for row in range(bottom, top):
            dy = origin[1] + (row + 0.5) * resolution - y
            for column in range(left, right):
                dx = origin[0] + (column + 0.5) * resolution - x
                if abs(c * dx + s * dy) <= sx + padding and abs(-s * dx + c * dy) <= sy + padding:
                    occupied[row * width + column] = 1
    return width, height, origin, resolution, occupied


def write_map(project=PROJECT):
    width, height, origin, resolution, occupied = build_map(project / 'worlds/warehouse.xml')
    folder = project / 'nav2_config'
    image = folder / 'warehouse_navigation_map.pgm'
    metadata = folder / 'warehouse_navigation_map.yaml'
    image.write_text(f'P2\n{width} {height}\n255\n' + '\n'.join(
        ' '.join('0' if occupied[row * width + column] else '254' for column in range(width))
        for row in reversed(range(height))) + '\n')
    metadata.write_text(f'image: {image.name}\nmode: trinary\nresolution: {resolution}\n'
                        f'origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\n'
                        'occupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return metadata


if __name__ == '__main__':
    print(write_map())

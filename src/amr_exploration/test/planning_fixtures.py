"""Shared fixture loaders for planning-equivalence/speed tests (root-authored)."""
import base64
import json
import math
import random
import zlib
from pathlib import Path
from types import SimpleNamespace

FIXTURES = Path(__file__).resolve().parent / "fixtures"


def _decode(blob, offset):
    return [value - offset for value in zlib.decompress(base64.b64decode(blob))]


def _info(entry, width_key, height_key):
    return SimpleNamespace(
        **{width_key: entry["width"], height_key: entry["height"]},
        resolution=entry["resolution"],
        origin=SimpleNamespace(
            position=SimpleNamespace(x=entry["origin_x"], y=entry["origin_y"], z=0.0),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


def grid_message(width, height, resolution, origin_x, origin_y, data):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=_info({"width": width, "height": height, "resolution": resolution,
                    "origin_x": origin_x, "origin_y": origin_y}, "width", "height"),
        data=list(data))


def costmap_message(width, height, resolution, origin_x, origin_y, data):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        metadata=_info({"width": width, "height": height, "resolution": resolution,
                        "origin_x": origin_x, "origin_y": origin_y}, "size_x", "size_y"),
        data=list(data))


def hospital(name):
    entry = json.loads((FIXTURES / "hospital_planning_fixtures.json").read_text())[name]
    m, c = entry["map"], entry["costmap"]
    grid = grid_message(m["width"], m["height"], m["resolution"], m["origin_x"],
                        m["origin_y"], _decode(m["data"], 1))
    costmap = costmap_message(c["width"], c["height"], c["resolution"], c["origin_x"],
                              c["origin_y"], _decode(c["data"], 0))
    return grid, costmap, tuple(entry["pose"])


def synthetic(seed, width=90, height=70, resolution=0.1):
    """Rooms, walls with doors, pillars, pockets and unknown regions."""
    rng = random.Random(seed)
    grid = [0] * (width * height)
    cost = [0] * (width * height)

    def put(x, y, g, c):
        if 0 <= x < width and 0 <= y < height:
            grid[y * width + x] = g
            cost[y * width + x] = c

    for x in range(width):
        for y in (0, height - 1):
            put(x, y, 100, 254)
    for y in range(height):
        for x in (0, width - 1):
            put(x, y, 100, 254)
    for _ in range(rng.randint(2, 4)):                # interior walls with doors
        if rng.random() < 0.5:
            x = rng.randint(20, width - 20)
            door = rng.randint(8, height - 20)
            for y in range(height):
                if not door <= y < door + rng.randint(12, 20):
                    put(x, y, 100, 254)
        else:
            y = rng.randint(15, height - 15)
            door = rng.randint(8, width - 25)
            for x in range(width):
                if not door <= x < door + rng.randint(12, 22):
                    put(x, y, 100, 254)
    for _ in range(rng.randint(3, 8)):                # pillars
        px, py = rng.randint(5, width - 6), rng.randint(5, height - 6)
        for dx in range(rng.randint(1, 4)):
            for dy in range(rng.randint(1, 4)):
                put(px + dx, py + dy, 100, 254)
    for _ in range(rng.randint(1, 3)):                # unknown regions
        ux, uy = rng.randint(5, width - 25), rng.randint(5, height - 20)
        for x in range(ux, ux + rng.randint(8, 20)):
            for y in range(uy, uy + rng.randint(6, 15)):
                if grid[y * width + x] == 0:
                    put(x, y, -1, 255)
    for i in range(width * height):                   # simple inflation band
        if grid[i] == 0:
            x, y = i % width, i // width
            near = any(
                0 <= x + dx < width and 0 <= y + dy < height
                and cost[(y + dy) * width + x + dx] >= 254
                for dx in range(-4, 5) for dy in range(-4, 5)
                if dx * dx + dy * dy <= 16)
            if near:
                cost[i] = 253 if any(
                    0 <= x + dx < width and 0 <= y + dy < height
                    and cost[(y + dy) * width + x + dx] >= 254
                    for dx in range(-2, 3) for dy in range(-2, 3)) else 128
    free = []
    for clearance in (12, 10, 9, 8, 7, 6, 5, 4):
        free = [
            (x, y)
            for y in range(clearance, height - clearance)
            for x in range(clearance, width - clearance)
            if all(cost[(y + dy) * width + x + dx] < 253
                   for dx in range(-clearance, clearance + 1)
                   for dy in range(-clearance, clearance + 1))]
        if free:
            break
    start = rng.choice(free)
    pose = ((start[0] + 0.5) * resolution, (start[1] + 0.5) * resolution,
            rng.uniform(-math.pi, math.pi))
    return (grid_message(width, height, resolution, 0.0, 0.0, grid),
            costmap_message(width, height, resolution, 0.0, 0.0, cost), pose)

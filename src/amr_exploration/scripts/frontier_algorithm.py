"""Pure frontier extraction helpers used by the online exploration node."""

from collections import deque
import heapq
from math import hypot
import math
from numbers import Integral, Real


NAVIGATION_FOOTPRINT = (
    (0.61, 0.41), (0.61, -0.41),
    (-0.61, -0.41), (-0.61, 0.41))


def _strict_integral(value):
    """Return an actual integral value, without coercing booleans/floats."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        return None
    return int(value)


def _strict_finite_real(value):
    """Return a finite real as float, without coercing text/booleans."""
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _strict_dimension(value):
    dimension = _strict_integral(value)
    if dimension is None or dimension <= 0:
        return None
    return dimension


def _strict_origin_geometry(origin):
    try:
        position = origin.position
        orientation = origin.orientation
        values = (
            _strict_finite_real(position.x),
            _strict_finite_real(position.y),
            _strict_finite_real(position.z),
            _strict_finite_real(orientation.x),
            _strict_finite_real(orientation.y),
            _strict_finite_real(orientation.z),
            _strict_finite_real(orientation.w))
        if any(value is None for value in values):
            return None
        origin_x, origin_y, _origin_z, qx, qy, qz, qw = values
        quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(quaternion_norm) or quaternion_norm <= 0.0:
            return None
        qx /= quaternion_norm
        qy /= quaternion_norm
        qz /= quaternion_norm
        qw /= quaternion_norm

        # OccupancyGrid geometry is planar: yaw is allowed, roll and pitch are
        # not.  A non-unit quaternion is normalized before this check.
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy))
        sin_pitch = 2.0 * (qw * qy - qz * qx)
        if abs(sin_pitch) > 1.0 + 1.0e-9:
            return None
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        if abs(roll) > 1.0e-6 or abs(pitch) > 1.0e-6:
            return None
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        return origin_x, origin_y, math.cos(yaw), math.sin(yaw)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _strict_grid_geometry(info, width_name, height_name):
    try:
        width = _strict_dimension(getattr(info, width_name))
        height = _strict_dimension(getattr(info, height_name))
        resolution = _strict_finite_real(info.resolution)
        if (width is None or height is None or resolution is None
                or resolution <= 0.0):
            return None
        origin = _strict_origin_geometry(info.origin)
        if origin is None:
            return None
        return (width, height, resolution) + origin
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _occupancy_value(value):
    occupancy = _strict_integral(value)
    if (occupancy is None
            or (occupancy != -1 and not 0 <= occupancy <= 100)):
        return None
    return occupancy


def occupancy_grid_geometry(grid):
    """Return validated map geometry, or ``None`` for invalid map evidence."""
    try:
        if grid.header.frame_id != "map":
            return None
        geometry = _strict_grid_geometry(grid.info, "width", "height")
        if geometry is None:
            return None
        data = grid.data
        if len(data) != geometry[0] * geometry[1]:
            return None
        if any(_occupancy_value(value) is None for value in data):
            return None
        return geometry
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _dimension(value):
    try:
        dimension = int(value)
        if dimension <= 0 or dimension != value:
            return None
        return dimension
    except (TypeError, ValueError, OverflowError):
        return None


def _origin_geometry(origin, allow_identity=False):
    try:
        position = origin.position
        origin_x = float(position.x)
        origin_y = float(position.y)
        origin_z = float(getattr(position, "z", 0.0))
        if not all(math.isfinite(value) for value in (origin_x, origin_y, origin_z)):
            return None

        orientation = getattr(origin, "orientation", None)
        if orientation is None:
            if not allow_identity:
                return None
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
        else:
            qx = float(orientation.x)
            qy = float(orientation.y)
            qz = float(orientation.z)
            qw = float(orientation.w)
            if not all(math.isfinite(value) for value in (qx, qy, qz, qw)):
                return None

        quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not math.isfinite(quaternion_norm) or quaternion_norm <= 0.0:
            return None
        qx /= quaternion_norm
        qy /= quaternion_norm
        qz /= quaternion_norm
        qw /= quaternion_norm

        # Costmap metadata is planar.  A yaw-bearing quaternion is valid;
        # roll or pitch would make the 2-D cell-to-world mapping ambiguous.
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy))
        sin_pitch = 2.0 * (qw * qy - qz * qx)
        if abs(sin_pitch) > 1.0 + 1.0e-9:
            return None
        pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
        if abs(roll) > 1.0e-6 or abs(pitch) > 1.0e-6:
            return None
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))
        return origin_x, origin_y, math.cos(yaw), math.sin(yaw)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _grid_geometry(info, allow_identity=False):
    try:
        width_value = getattr(info, "width", None)
        height_value = getattr(info, "height", None)
        if width_value is None:
            width_value = info.size_x
        if height_value is None:
            height_value = info.size_y
        width = _dimension(width_value)
        height = _dimension(height_value)
        resolution = float(info.resolution)
        if (width is None or height is None
                or not math.isfinite(resolution) or resolution <= 0.0):
            return None
        origin = _origin_geometry(info.origin, allow_identity=allow_identity)
        if origin is None:
            return None
        return (width, height, resolution) + origin
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def costmap_geometry(costmap):
    """Return validated map-frame costmap geometry, or ``None``."""
    try:
        if costmap.header.frame_id != "map":
            return None
        geometry = _strict_grid_geometry(
            costmap.metadata, "size_x", "size_y")
        if geometry is None:
            return None
        data = costmap.data
        if len(data) != geometry[0] * geometry[1]:
            return None
        for value in data:
            cost = _strict_integral(value)
            if cost is None or not 0 <= cost <= 255:
                return None
        return geometry
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _cell_world(geometry, cell):
    width, height, resolution, origin_x, origin_y, cos_yaw, sin_yaw = geometry
    x, y = cell
    if not (0 <= x < width and 0 <= y < height):
        return None
    local_x = (x + 0.5) * resolution
    local_y = (y + 0.5) * resolution
    return (
        origin_x + cos_yaw * local_x - sin_yaw * local_y,
        origin_y + sin_yaw * local_x + cos_yaw * local_y)


def frontier_cell_world(grid, cell):
    """Return a map-grid cell center in world coordinates, or ``None``."""
    try:
        geometry = _grid_geometry(grid.info, allow_identity=True)
        if geometry is None:
            return None
        return _cell_world(geometry, cell)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def frontier_world_cell(grid, world):
    """Return the map-grid cell containing a world point, or ``None``."""
    try:
        geometry = _grid_geometry(grid.info, allow_identity=True)
        if geometry is None:
            return None
        return _costmap_cell(geometry, world)
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def _costmap_cell(geometry, world):
    width, height, resolution, origin_x, origin_y, cos_yaw, sin_yaw = geometry
    world_x, world_y = world
    if not all(math.isfinite(float(value)) for value in (world_x, world_y)):
        return None
    dx = world_x - origin_x
    dy = world_y - origin_y
    local_x = cos_yaw * dx + sin_yaw * dy
    local_y = -sin_yaw * dx + cos_yaw * dy

    def cell_index(local):
        scaled = local / resolution
        nearest = round(scaled)
        if abs(scaled - nearest) <= 1.0e-9 * max(1.0, abs(scaled)):
            scaled = float(nearest)
        return math.floor(scaled)

    costmap_x = cell_index(local_x)
    costmap_y = cell_index(local_y)
    if not (0 <= costmap_x < width and 0 <= costmap_y < height):
        return None
    return costmap_x, costmap_y


def _endpoint_cost(map_geometry, costmap_geometry_value, data, cell):
    world = _cell_world(map_geometry, cell)
    if world is None:
        return None
    costmap_cell = _costmap_cell(costmap_geometry_value, world)
    if costmap_cell is None:
        return None
    costmap_x, costmap_y = costmap_cell
    try:
        return int(data[costmap_y * costmap_geometry_value[0] + costmap_x])
    except (IndexError, TypeError, ValueError, OverflowError):
        return None


def _strict_footprint(footprint):
    """Return a finite, non-degenerate, convex footprint polygon."""
    try:
        points = []
        for point in footprint:
            if len(point) != 2:
                return None
            x = _strict_finite_real(point[0])
            y = _strict_finite_real(point[1])
            if x is None or y is None:
                return None
            points.append((x, y))
        if len(points) < 3:
            return None

        twice_area = 0.0
        turns = []
        for index, point in enumerate(points):
            next_point = points[(index + 1) % len(points)]
            twice_area += point[0] * next_point[1]
            twice_area -= next_point[0] * point[1]
            previous_point = points[index - 1]
            cross = (
                (point[0] - previous_point[0])
                * (next_point[1] - point[1])
                - (point[1] - previous_point[1])
                * (next_point[0] - point[0]))
            if not math.isfinite(cross) or abs(cross) <= 1.0e-12:
                return None
            turns.append(cross > 0.0)
        if not math.isfinite(twice_area) or abs(twice_area) <= 1.0e-12:
            return None
        if any(turn != turns[0] for turn in turns[1:]):
            return None
        return tuple(points)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return None


def _polygon_intersects_cell(polygon, cell_min_x, cell_min_y,
                             cell_max_x, cell_max_y):
    """Use convex-polygon SAT; touching a lethal cell counts as collision."""
    cell = (
        (cell_min_x, cell_min_y), (cell_max_x, cell_min_y),
        (cell_max_x, cell_max_y), (cell_min_x, cell_max_y))
    for shape in (polygon, cell):
        for index, point in enumerate(shape):
            next_point = shape[(index + 1) % len(shape)]
            edge_x = next_point[0] - point[0]
            edge_y = next_point[1] - point[1]
            axis = (-edge_y, edge_x)
            polygon_projection = [
                vertex[0] * axis[0] + vertex[1] * axis[1]
                for vertex in polygon]
            cell_projection = [
                vertex[0] * axis[0] + vertex[1] * axis[1]
                for vertex in cell]
            if (max(polygon_projection) < min(cell_projection) - 1.0e-12
                    or max(cell_projection)
                    < min(polygon_projection) - 1.0e-12):
                return False
    return True


def _footprint_costmap_clear(costmap_geometry_value, data, world, footprint):
    """Return whether the identity-yaw footprint overlaps no lethal cell."""
    width, height, resolution, origin_x, origin_y, cos_yaw, sin_yaw = (
        costmap_geometry_value)
    world_x, world_y = world
    polygon = []
    for footprint_x, footprint_y in footprint:
        point_x = world_x + footprint_x
        point_y = world_y + footprint_y
        delta_x = point_x - origin_x
        delta_y = point_y - origin_y
        polygon.append((
            cos_yaw * delta_x + sin_yaw * delta_y,
            -sin_yaw * delta_x + cos_yaw * delta_y))

    max_local_x = width * resolution
    max_local_y = height * resolution
    if any(
            point_x < -1.0e-12 or point_x > max_local_x + 1.0e-12
            or point_y < -1.0e-12 or point_y > max_local_y + 1.0e-12
            for point_x, point_y in polygon):
        return False

    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    first_x = max(0, math.floor(min_x / resolution) - 1)
    last_x = min(width - 1, math.floor(max_x / resolution) + 1)
    first_y = max(0, math.floor(min_y / resolution) - 1)
    last_y = min(height - 1, math.floor(max_y / resolution) + 1)
    for cell_y in range(first_y, last_y + 1):
        for cell_x in range(first_x, last_x + 1):
            cost = int(data[cell_y * width + cell_x])
            if cost >= 253 and _polygon_intersects_cell(
                    polygon,
                    cell_x * resolution,
                    cell_y * resolution,
                    (cell_x + 1) * resolution,
                    (cell_y + 1) * resolution):
                return False
    return True


def _neighbors(x, y, width, height):
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if not dx and not dy:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield nx, ny


def frontier_clusters(width, height, data):
    """Return deterministic clusters of free cells adjacent to unknown space.

    Each item is ``(candidate, cells)`` where both the candidate and cells are
    grid coordinates ``(x, y)``.  The candidate is always a known-free cell;
    this leaves final collision/reachability checking to Nav2's costmap.
    """
    width = _strict_dimension(width)
    height = _strict_dimension(height)
    if width is None or height is None:
        return None
    try:
        if len(data) != width * height:
            return None
        validated_data = tuple(_occupancy_value(value) for value in data)
    except (TypeError, ValueError, OverflowError):
        return None
    if any(value is None for value in validated_data):
        return None

    def value(x, y):
        return validated_data[y * width + x]

    frontier = set()
    for y in range(height):
        for x in range(width):
            if value(x, y) != 0:
                continue
            if any(value(nx, ny) == -1 for nx, ny in _neighbors(x, y, width, height)):
                frontier.add((x, y))

    frontier_heap = [(point[1], point[0]) for point in frontier]
    heapq.heapify(frontier_heap)
    clusters = []
    while frontier:
        while frontier_heap:
            seed_y, seed_x = heapq.heappop(frontier_heap)
            seed = (seed_x, seed_y)
            if seed in frontier:
                break
        else:  # pragma: no cover - every frontier cell enters the heap
            return None
        frontier.remove(seed)
        queue = deque([seed])
        cells = [seed]
        while queue:
            point = queue.popleft()
            for neighbor in _neighbors(point[0], point[1], width, height):
                if neighbor in frontier:
                    frontier.remove(neighbor)
                    queue.append(neighbor)
                    cells.append(neighbor)
        cx = sum(point[0] for point in cells) / len(cells)
        cy = sum(point[1] for point in cells) / len(cells)
        candidate = min(cells, key=lambda point: (hypot(point[0] - cx, point[1] - cy), point[1], point[0]))
        clusters.append((candidate, tuple(sorted(cells, key=lambda point: (point[1], point[0])))))

    return sorted(clusters, key=lambda item: (-len(item[1]), item[0][1], item[0][0]))


def costmap_frontier_candidates(
        clusters, grid, costmap, blacklist=(), robot_world=None,
        min_goal_distance=0.0, footprint=None):
    """Select map-grid endpoints admissible in costmap and goal distance."""
    map_geometry = None
    try:
        map_geometry = _grid_geometry(grid.info, allow_identity=True)
        if (map_geometry is None
                or len(grid.data) != map_geometry[0] * map_geometry[1]):
            return None
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None

    costmap_geometry_value = costmap_geometry(costmap)
    if costmap_geometry_value is None:
        return None
    try:
        data = costmap.data
        if len(data) != costmap_geometry_value[0] * costmap_geometry_value[1]:
            return None
        validated_footprint = None
        if footprint is not None:
            validated_footprint = _strict_footprint(footprint)
            if validated_footprint is None:
                return None
        min_goal_distance = float(min_goal_distance)
        if not math.isfinite(min_goal_distance) or min_goal_distance < 0.0:
            return None
        if robot_world is not None:
            robot_x, robot_y = (float(value) for value in robot_world)
            if not all(math.isfinite(value) for value in (robot_x, robot_y)):
                return None
        blacklisted = set(blacklist)
        footprint_cache_miss = object()
        footprint_clear_cache = {}

        def footprint_clear(cell, world):
            if validated_footprint is None:
                return True
            if world is None:
                return False
            cached = footprint_clear_cache.get(cell, footprint_cache_miss)
            if cached is footprint_cache_miss:
                cached = _footprint_costmap_clear(
                    costmap_geometry_value, data, world, validated_footprint)
                footprint_clear_cache[cell] = cached
            return cached

        candidates = []
        for representative, cells in clusters:
            eligible = []
            for cell in cells:
                if cell in blacklisted:
                    continue
                if robot_world is None:
                    eligible.append(cell)
                    continue
                world = _cell_world(map_geometry, cell)
                if (world is not None and
                        (world[0] - robot_x) ** 2 + (world[1] - robot_y) ** 2
                        >= min_goal_distance ** 2):
                    eligible.append(cell)
            if not eligible:
                continue
            representative_cost = _endpoint_cost(
                map_geometry, costmap_geometry_value, data, representative)
            representative_world = _cell_world(map_geometry, representative)
            representative_clear = (
                representative_cost is not None
                and representative_cost < 253
                and footprint_clear(representative, representative_world))
            if representative in eligible and representative_clear:
                candidates.append(representative)
                continue

            if validated_footprint is None:
                admissible = []
                for cell in eligible:
                    cost = _endpoint_cost(
                        map_geometry, costmap_geometry_value, data, cell)
                    world = _cell_world(map_geometry, cell)
                    if cost is not None and cost < 253:
                        distance = hypot(
                            cell[0] - representative[0],
                            cell[1] - representative[1])
                        admissible.append((cost, distance, cell[1], cell[0], cell))
                if admissible:
                    candidates.append(min(admissible)[-1])
                continue

            ranked = []
            for cell in eligible:
                cost = _endpoint_cost(
                    map_geometry, costmap_geometry_value, data, cell)
                if cost is None or cost >= 253:
                    continue
                world = _cell_world(map_geometry, cell)
                if world is None:
                    continue
                distance = hypot(
                    cell[0] - representative[0],
                    cell[1] - representative[1])
                ranked.append((cost, distance, cell[1], cell[0], cell, world))
            ranked.sort(key=lambda item: item[:5])
            for _cost, _distance, _y, _x, cell, world in ranked:
                if footprint_clear(cell, world):
                    candidates.append(cell)
                    break
        return candidates
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def available_candidates(clusters, blacklist):
    """Return candidates not present in the failed-goal blacklist."""
    return [candidate for candidate, _cells in clusters if candidate not in blacklist]

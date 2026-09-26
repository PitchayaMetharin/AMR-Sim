"""Pure frontier extraction helpers used by the online exploration node."""

from collections import deque
import heapq
from math import hypot
import math
from numbers import Integral, Real


NAVIGATION_FOOTPRINT = (
    (0.61, 0.41), (0.61, -0.41),
    (-0.61, -0.41), (-0.61, 0.41))


# Keep exactly one immutable costmap/footprint mask snapshot.  Routes and
# candidate eligibility remain per-call because they depend on live pose,
# heading, map content, and blacklist state.
_FOOTPRINT_MASK_CACHE = None


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


def _footprint_costmap_clear(
        costmap_geometry_value, data, world, footprint, yaw=0.0):
    """Return whether a footprint at ``world`` overlaps no lethal cell."""
    width, height, resolution, origin_x, origin_y, cos_yaw, sin_yaw = (
        costmap_geometry_value)
    world_x, world_y = world
    footprint_cos = math.cos(yaw)
    footprint_sin = math.sin(yaw)
    polygon = []
    for footprint_x, footprint_y in footprint:
        point_x = (
            world_x + footprint_cos * footprint_x
            - footprint_sin * footprint_y)
        point_y = (
            world_y + footprint_sin * footprint_x
            + footprint_cos * footprint_y)
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
            if cost >= 254 and _polygon_intersects_cell(
                    polygon,
                    cell_x * resolution,
                    cell_y * resolution,
                    (cell_x + 1) * resolution,
                    (cell_y + 1) * resolution):
                return False
    return True


def _footprint_collision_mask(
        costmap_geometry_value, lethal_rows, width, height, footprint,
        world_yaw, center_offset=(0.0, 0.0)):
    """Return a bit-mask row for footprint/lethal overlap at each cell.

    The mask is exact for footprint poses at the sampled cell or movement
    midpoint.  Bit operations keep the heading-aware search bounded without
    calling polygon SAT for every state transition.
    """
    _width, _height, resolution, _origin_x, _origin_y, cos_yaw, sin_yaw = (
        costmap_geometry_value)
    costmap_yaw = math.atan2(sin_yaw, cos_yaw)
    relative_yaw = world_yaw - costmap_yaw
    footprint_cos = math.cos(relative_yaw)
    footprint_sin = math.sin(relative_yaw)
    polygon = tuple(
        (
            footprint_cos * footprint_x - footprint_sin * footprint_y,
            footprint_sin * footprint_x + footprint_cos * footprint_y)
        for footprint_x, footprint_y in footprint)
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    offset_x, offset_y = center_offset
    offset_radius_x = math.ceil(
        max(abs(min_x), abs(max_x)) / resolution) + 2
    offset_radius_y = math.ceil(
        max(abs(min_y), abs(max_y)) / resolution) + 2
    overlap_offsets = {}
    for cell_y in range(-offset_radius_y, offset_radius_y + 1):
        offsets_x = []
        for cell_x in range(-offset_radius_x, offset_radius_x + 1):
            cell_min_x = (cell_x - 0.5 - offset_x) * resolution
            cell_min_y = (cell_y - 0.5 - offset_y) * resolution
            if _polygon_intersects_cell(
                    polygon, cell_min_x, cell_min_y,
                    cell_min_x + resolution, cell_min_y + resolution):
                offsets_x.append(cell_x)
        if offsets_x:
            overlap_offsets[cell_y] = tuple(offsets_x)

    width_mask = (1 << width) - 1
    blocked_rows = []
    safe_first_x = max(
        0, math.ceil(-0.5 - offset_x - min_x / resolution - 1.0e-12))
    safe_last_x = min(
        width - 1,
        math.floor(width - 0.5 - offset_x - max_x / resolution + 1.0e-12))
    if safe_first_x <= safe_last_x:
        safe_x_mask = (
            (1 << (safe_last_x - safe_first_x + 1)) - 1) << safe_first_x
    else:
        safe_x_mask = 0

    for center_y in range(height):
        center_local_y = (center_y + 0.5 + offset_y) * resolution
        y_inside = (
            center_local_y + min_y >= -1.0e-12
            and center_local_y + max_y <= height * resolution + 1.0e-12)
        blocked = width_mask ^ safe_x_mask if y_inside else width_mask
        if y_inside:
            for offset_y_index, offsets_x in overlap_offsets.items():
                source_y = center_y + offset_y_index
                if not 0 <= source_y < height:
                    continue
                source = lethal_rows[source_y]
                for offset_x_index in offsets_x:
                    if offset_x_index >= 0:
                        blocked |= source >> offset_x_index
                    else:
                        blocked |= source << -offset_x_index
        blocked_rows.append(blocked & width_mask)
    return tuple(blocked_rows)


def _first_step_headings(
        costmap_geometry_value, data, robot_world, footprint,
        robot_yaw=0.0):
    """Return first-step headings legal from one costmap pose.

    This is the first transition of ``_reachable_costmap_cells`` expressed as
    a pure set-valued proof.  It keeps the same center-cost, diagonal-corner,
    exact-footprint, and midpoint checks as the route graph.
    """
    try:
        width, height, _resolution, _origin_x, _origin_y, cos_yaw, sin_yaw = (
            costmap_geometry_value)
        if len(data) != width * height:
            return frozenset()
        validated_footprint = _strict_footprint(footprint)
        if validated_footprint is None:
            return frozenset()
        robot_yaw = float(robot_yaw)
        if not math.isfinite(robot_yaw):
            return frozenset()
        start = _costmap_cell(costmap_geometry_value, robot_world)
        if start is None:
            return frozenset()

        def cost(cell):
            cell_x, cell_y = cell
            return int(data[cell_y * width + cell_x])

        if cost(start) >= 253:
            return frozenset()
        costmap_yaw = math.atan2(sin_yaw, cos_yaw)
        heading_angles = tuple(
            costmap_yaw + index * math.pi / 4.0 for index in range(8))
        directions = ((1, 0), (1, 1), (0, 1), (-1, 1),
                      (-1, 0), (-1, -1), (0, -1), (1, -1))
        legal = set()
        for move_heading, (dx, dy) in enumerate(directions):
            next_cell = (start[0] + dx, start[1] + dy)
            if (not 0 <= next_cell[0] < width
                    or not 0 <= next_cell[1] < height
                    or cost(next_cell) >= 253):
                continue
            if dx and dy and (
                    cost((start[0] + dx, start[1])) >= 253
                    or cost((start[0], start[1] + dy)) >= 253):
                continue
            next_world = _cell_world(costmap_geometry_value, next_cell)
            if next_world is None:
                continue
            move_yaw = heading_angles[move_heading]
            turn_delta = math.atan2(
                math.sin(move_yaw - robot_yaw),
                math.cos(move_yaw - robot_yaw))
            turn_samples = max(
                1, int(math.ceil(abs(turn_delta) / (math.pi / 16.0))))
            turn_clear = all(
                _footprint_costmap_clear(
                    costmap_geometry_value, data, robot_world,
                    validated_footprint,
                    yaw=robot_yaw + turn_delta * sample / turn_samples)
                for sample in range(turn_samples + 1))
            midpoint = (
                (robot_world[0] + next_world[0]) / 2.0,
                (robot_world[1] + next_world[1]) / 2.0)
            if (
                    turn_clear
                    and _footprint_costmap_clear(
                        costmap_geometry_value, data, robot_world,
                        validated_footprint, yaw=move_yaw)
                    and _footprint_costmap_clear(
                        costmap_geometry_value, data, midpoint,
                        validated_footprint, yaw=move_yaw)
                    and _footprint_costmap_clear(
                        costmap_geometry_value, data, next_world,
                        validated_footprint, yaw=move_yaw)):
                legal.add(move_heading)
        return frozenset(legal)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return frozenset()


def _goal_distance_is_valid(robot_world, goal_world, min_goal_distance):
    """Return whether a goal satisfies the existing minimum distance gate."""
    try:
        robot_x, robot_y = (float(value) for value in robot_world)
        goal_x, goal_y = (float(value) for value in goal_world)
        minimum = float(min_goal_distance)
        if (not all(math.isfinite(value)
                    for value in (robot_x, robot_y, goal_x, goal_y, minimum))
                or minimum < 0.0):
            return False
        return ((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2
                >= minimum ** 2)
    except (TypeError, ValueError, OverflowError):
        return False


def _route_start_proof(
        costmap_geometry_value, data, footprint,
        old_world, old_yaw, new_world, new_yaw):
    """Prove a refreshed pose preserves the selected costmap route start."""
    try:
        old_world = tuple(float(value) for value in old_world)
        new_world = tuple(float(value) for value in new_world)
        old_yaw = float(old_yaw)
        new_yaw = float(new_yaw)
        if (len(old_world) != 2 or len(new_world) != 2
                or not all(math.isfinite(value)
                           for value in old_world + new_world)
                or not all(
                    math.isfinite(value) for value in (old_yaw, new_yaw))):
            return False
        width, height = costmap_geometry_value[:2]
        if len(data) != width * height:
            return False
        validated_footprint = _strict_footprint(footprint)
        if validated_footprint is None:
            return False
        old_start = _costmap_cell(costmap_geometry_value, old_world)
        new_start = _costmap_cell(costmap_geometry_value, new_world)
        if old_start is None or old_start != new_start:
            return False
        start_index = old_start[1] * width + old_start[0]
        if int(data[start_index]) >= 253:
            return False
        if not _footprint_costmap_clear(
                costmap_geometry_value, data, old_world,
                validated_footprint, yaw=old_yaw):
            return False
        if not _footprint_costmap_clear(
                costmap_geometry_value, data, new_world,
                validated_footprint, yaw=new_yaw):
            return False
        old_headings = _first_step_headings(
            costmap_geometry_value, data, old_world, validated_footprint,
            robot_yaw=old_yaw)
        new_headings = _first_step_headings(
            costmap_geometry_value, data, new_world, validated_footprint,
            robot_yaw=new_yaw)
        return bool(old_headings) and old_headings.issubset(new_headings)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return False


def _reachable_costmap_cells(
        costmap_geometry_value, data, robot_world, robot_yaw,
        target_cells, footprint, stop_after_first=False):
    """Return target cells reachable through heading-aware safe costmap space.

    Unknown and lethal cells are not traversable, diagonal moves cannot cut an
    occupied corner, and heading/turn/midpoint footprint masks are checked
    during the search.  Nav2's planner and smoother remain authoritative.
    """
    width, height, resolution, _origin_x, _origin_y, cos_yaw, sin_yaw = (
        costmap_geometry_value)
    target_cells = tuple(dict.fromkeys(target_cells))
    if not target_cells:
        return set()
    target_cell_set = set(target_cells)
    start = _costmap_cell(costmap_geometry_value, robot_world)
    if start is None:
        return set()

    def cost(cell):
        cell_x, cell_y = cell
        return int(data[cell_y * width + cell_x])

    if cost(start) >= 253:
        return set()
    if footprint is None:
        directions = ((1, 0), (1, 1), (0, 1), (-1, 1),
                      (-1, 0), (-1, -1), (0, -1), (1, -1))
        queue = deque([start])
        parents = {start: None}
        remaining = set(target_cell_set)
        while queue and remaining:
            cell_x, cell_y = queue.popleft()
            remaining.discard((cell_x, cell_y))
            for dx, dy in directions:
                next_cell = (cell_x + dx, cell_y + dy)
                if (not 0 <= next_cell[0] < width
                        or not 0 <= next_cell[1] < height
                        or next_cell in parents
                        or cost(next_cell) >= 253):
                    continue
                if dx and dy and (
                        cost((cell_x + dx, cell_y)) >= 253
                        or cost((cell_x, cell_y + dy)) >= 253):
                    continue
                parents[next_cell] = (cell_x, cell_y)
                queue.append(next_cell)
        return target_cell_set.intersection(parents)

    costmap_yaw = math.atan2(sin_yaw, cos_yaw)
    mask_cache_key = (
        costmap_geometry_value, bytes(data), tuple(footprint))
    global _FOOTPRINT_MASK_CACHE
    cached_masks = _FOOTPRINT_MASK_CACHE
    if cached_masks is not None and cached_masks[0] == mask_cache_key:
        lethal_rows, heading_masks, movement_masks, goal_mask = cached_masks[1:]
    else:
        lethal_rows = []
        for cell_y in range(height):
            row = 0
            row_offset = cell_y * width
            for cell_x, value in enumerate(data[row_offset:row_offset + width]):
                if int(value) >= 254:
                    row |= 1 << cell_x
            lethal_rows.append(row)

        heading_angles = tuple(
            costmap_yaw + index * math.pi / 4.0 for index in range(8))
        heading_masks = tuple(
            _footprint_collision_mask(
                costmap_geometry_value, lethal_rows, width, height, footprint,
                costmap_yaw + index * math.pi / 16.0)
            for index in range(32))
        movement_masks = tuple(
            _footprint_collision_mask(
                costmap_geometry_value, lethal_rows, width, height, footprint,
                heading_angles[index], center_offset=(dx / 2.0, dy / 2.0))
            for index, (dx, dy) in enumerate(
                ((1, 0), (1, 1), (0, 1), (-1, 1),
                 (-1, 0), (-1, -1), (0, -1), (1, -1))))
        goal_mask = _footprint_collision_mask(
            costmap_geometry_value, lethal_rows, width, height, footprint, 0.0)
        _FOOTPRINT_MASK_CACHE = (
            mask_cache_key, lethal_rows, heading_masks, movement_masks,
            goal_mask)

    heading_angles = tuple(
        costmap_yaw + index * math.pi / 4.0 for index in range(8))

    def blocked(mask, cell):
        return bool(mask[cell[1]] & (1 << cell[0]))

    def turn_clear(cell, arrival_heading, move_heading):
        if arrival_heading < 0:
            return True
        arrival_index = arrival_heading * 4
        target_index = move_heading * 4
        delta = (target_index - arrival_index) % 32
        if delta > 16:
            delta -= 32
        step = 1 if delta >= 0 else -1
        for index in range(0, abs(delta) + 1):
            heading_index = (arrival_index + step * index) % 32
            if blocked(heading_masks[heading_index], cell):
                return False
        return True

    def goal_turn_clear(cell, arrival_heading):
        if arrival_heading < 0:
            return not blocked(goal_mask, cell)
        arrival_angle = heading_angles[arrival_heading]
        goal_delta = math.atan2(
            math.sin(-arrival_angle), math.cos(-arrival_angle))
        samples = max(1, int(math.ceil(abs(goal_delta) / (math.pi / 16.0))))
        for sample in range(samples + 1):
            angle = arrival_angle + goal_delta * sample / samples
            heading_index = int(round(
                (angle - costmap_yaw) / (math.pi / 16.0))) % 32
            if blocked(heading_masks[heading_index], cell):
                return False
        return not blocked(goal_mask, cell)

    directions = ((1, 0), (1, 1), (0, 1), (-1, 1),
                  (-1, 0), (-1, -1), (0, -1), (1, -1))
    queue = deque([(start[0], start[1], -1)])
    visited = {(start[0], start[1], -1)}
    remaining = set(target_cell_set)
    reachable = set()
    footprint_cache_miss = object()
    exact_clear_cache = {}

    def exact_clear(world, yaw):
        key = (world, round(yaw, 12))
        cached = exact_clear_cache.get(key, footprint_cache_miss)
        if cached is footprint_cache_miss:
            cached = _footprint_costmap_clear(
                costmap_geometry_value, data, world, footprint, yaw=yaw)
            exact_clear_cache[key] = cached
        return cached

    if not exact_clear(robot_world, robot_yaw):
        return set()

    first_step_headings = _first_step_headings(
        costmap_geometry_value, data, robot_world, footprint,
        robot_yaw=robot_yaw)

    first_target = target_cells[0] if stop_after_first else None
    while queue:
        cell_x, cell_y, arrival_heading = queue.popleft()
        current_cell = (cell_x, cell_y)
        if current_cell in remaining and goal_turn_clear(
                current_cell, arrival_heading):
            remaining.remove(current_cell)
            reachable.add(current_cell)
            if stop_after_first and current_cell == first_target:
                return {current_cell}

        for move_heading, (dx, dy) in enumerate(directions):
            next_cell = (cell_x + dx, cell_y + dy)
            next_state = (next_cell[0], next_cell[1], move_heading)
            if (next_state in visited
                    or not 0 <= next_cell[0] < width
                    or not 0 <= next_cell[1] < height
                    or cost(next_cell) >= 253):
                continue
            if dx and dy and (
                    cost((cell_x + dx, cell_y)) >= 253
                    or cost((cell_x, cell_y + dy)) >= 253):
                continue

            if arrival_heading < 0:
                if move_heading not in first_step_headings:
                    continue
            elif (not turn_clear(current_cell, arrival_heading, move_heading)
                  or blocked(movement_masks[move_heading], current_cell)
                  or blocked(heading_masks[move_heading * 4], next_cell)):
                continue

            visited.add(next_state)
            queue.append(next_state)

    if stop_after_first:
        for target in target_cells:
            if target in reachable:
                return {target}
        return set()
    return reachable


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
        min_goal_distance=0.0, footprint=None, robot_yaw=0.0,
        require_path_clear=False, return_diagnostics=False):
    """Select endpoints admissible in costmap, distance, and optional path.

    The default return value remains the candidate-cell list.  When
    ``return_diagnostics`` is true, return ``(candidates, diagnostics)``;
    each diagnostic records whether the cluster has an admissible endpoint,
    whether the heading-aware route proof reached one, and the resulting
    safety/route classification.  The diagnostics are observation-only and
    use the same gates as the normal selector.
    """
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
        if type(require_path_clear) is not bool:
            return None
        if robot_world is not None:
            robot_x, robot_y = (float(value) for value in robot_world)
            if not all(math.isfinite(value) for value in (robot_x, robot_y)):
                return None
        if not math.isfinite(float(robot_yaw)):
            return None
        robot_yaw = float(robot_yaw)
        if require_path_clear and robot_world is None:
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
        candidate_records = []
        diagnostics = []
        for cluster_index, (representative, cells) in enumerate(clusters):
            diagnostic = {
                "cluster_index": cluster_index,
                "classification": "BLOCKED_SAFETY",
                "safe_endpoint_count": 0,
                "reachable_endpoint_count": 0,
            }
            diagnostics.append(diagnostic)
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
            representative_selected = (
                representative in eligible and representative_clear)
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
            safe_ranked = [
                item for item in ranked if footprint_clear(item[-2], item[-1])]
            diagnostic["safe_endpoint_count"] = len(safe_ranked)
            if not require_path_clear:
                if representative_selected:
                    candidates.append(representative)
                    diagnostic["reachable_endpoint_count"] = 1
                elif safe_ranked:
                    candidates.append(safe_ranked[0][-2])
                    diagnostic["reachable_endpoint_count"] = 1
                if diagnostic["reachable_endpoint_count"]:
                    diagnostic["classification"] = "REACHABLE"
                continue

            if representative_selected:
                candidate_records.append((
                    cluster_index, representative, representative_world))
            for _cost, _distance, _y, _x, cell, world in safe_ranked:
                if not representative_selected or cell != representative:
                    candidate_records.append((cluster_index, cell, world))
        if not require_path_clear:
            return (candidates, diagnostics) if return_diagnostics else candidates

        target_cells = []
        target_records = []
        for cluster_index, candidate, world in candidate_records:
            target_cell = (
                None if world is None
                else _costmap_cell(costmap_geometry_value, world))
            if target_cell is not None:
                target_cells.append(target_cell)
                target_records.append((cluster_index, candidate, target_cell))
        reachable = _reachable_costmap_cells(
            costmap_geometry_value, data, (robot_x, robot_y), robot_yaw,
            target_cells,
            validated_footprint, stop_after_first=True)
        for cluster_index, candidate, target_cell in target_records:
            if target_cell in reachable:
                candidates.append(candidate)
                diagnostics[cluster_index]["reachable_endpoint_count"] += 1
        for diagnostic in diagnostics:
            if diagnostic["reachable_endpoint_count"]:
                diagnostic["classification"] = "REACHABLE"
            elif diagnostic["safe_endpoint_count"]:
                diagnostic["classification"] = "BLOCKED_ROUTE"
        if return_diagnostics:
            return candidates, diagnostics
        return candidates
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def available_candidates(clusters, blacklist):
    """Return candidates not present in the failed-goal blacklist."""
    return [candidate for candidate, _cells in clusters if candidate not in blacklist]

"""Pure frontier extraction helpers used by the online exploration node."""

from array import array
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

# Byte translation tables: exact threshold tests over validated costs (0..255)
# and occupancy values (-1 is stored as 255 in the signed-byte view).
_LETHAL_FLAGS = bytes(1 if value >= 254 else 0 for value in range(256))
_BARRIER_FLAGS = bytes(1 if value >= 253 else 0 for value in range(256))
_UNKNOWN_FLAGS = bytes(1 if value == 255 else 0 for value in range(256))
_FREE_FLAGS = bytes(1 if value == 0 else 0 for value in range(256))
_BIT_CHARS = bytes.maketrans(b"\x00\x01", b"01")
_MOVE_DIRECTIONS = (
    (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1), (0, -1), (1, -1))


def _threshold_rows(width, height, raw, flags):
    """Return one bit mask per row: bit x of row y is set when flagged.

    ``raw`` is a bytes-like buffer of ``width * height`` values and ``flags``
    a 256-entry translation table mapping each value to 0 or 1.
    """
    marked = raw.translate(flags)
    return [
        int(marked[row * width:(row + 1) * width][::-1].translate(_BIT_CHARS), 2)
        for row in range(height)]


def _union_rows(masks):
    """Return the per-row OR of several row-mask sequences."""
    union = [0] * len(masks[0])
    for mask in masks:
        for row_index, row in enumerate(mask):
            union[row_index] |= row
    return tuple(union)


def _turn_heading_sequence(arrival_heading, move_heading):
    """Heading-mask indices swept by a turn from arrival to move heading."""
    arrival_index = arrival_heading * 4
    target_index = move_heading * 4
    delta = (target_index - arrival_index) % 32
    if delta > 16:
        delta -= 32
    step = 1 if delta >= 0 else -1
    return tuple(
        (arrival_index + step * index) % 32
        for index in range(0, abs(delta) + 1))


_TURN_SEQUENCES = tuple(
    tuple(_turn_heading_sequence(arrival, move) for move in range(8))
    for arrival in range(8))


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
        # Fast path: all values are plain ints in range, which the per-value
        # validation below would accept unchanged.
        if set(map(type, data)) <= {int} and min(data) >= 0 and max(data) <= 255:
            return geometry
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
        costmap_geometry_value, data, world, footprint, yaw=0.0,
        collect=None):
    """Return whether a footprint at ``world`` overlaps no lethal cell.

    Rows of the window without a lethal cell are skipped in one comparison.
    When ``collect`` is a list, every overlapping cell ``(x, y, cost)`` is
    appended and the scan does not stop at the first one (observation only;
    the return value is unchanged).
    """
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
    # A lethal cell whose centre is clearly outside an edge plane cannot
    # overlap, and one whose centre is clearly inside overlaps.  Only the
    # remaining boundary cells need the exact separating-axis test.
    planes = None
    outside_bound = _CELL_HALF_DIAGONAL * resolution + _KERNEL_MARGIN
    for cell_y in range(first_y, last_y + 1):
        row_start = cell_y * width
        if isinstance(data, bytes):
            row = data[row_start + first_x:row_start + last_x + 1]
        else:
            row = [int(data[row_start + cell_x])
                   for cell_x in range(first_x, last_x + 1)]
        if not row or max(row) < 254:
            continue
        centre_y = (cell_y + 0.5) * resolution
        for offset, value in enumerate(row):
            if value < 254:
                continue
            cell_x = first_x + offset
            if planes is None:
                planes = _edge_planes(polygon)
            centre_class = _centre_class(
                planes, (cell_x + 0.5) * resolution, centre_y, outside_bound)
            if centre_class < 0:
                continue
            if centre_class > 0 or _polygon_intersects_cell(
                    polygon,
                    cell_x * resolution,
                    cell_y * resolution,
                    (cell_x + 1) * resolution,
                    (cell_y + 1) * resolution):
                if collect is None:
                    return False
                collect.append((cell_x, cell_y, int(value)))
    return not collect if collect is not None else True


_KERNEL_MEMO = {}
_KERNEL_MEMO_LIMIT = 1024
# Upper bound on a unit-normal extent of a grid cell (sqrt(2) / 2, rounded up).
_CELL_HALF_DIAGONAL = 0.7072
# Classification margin for the centre tests; far above float rounding.
_KERNEL_MARGIN = 1.0e-9


def _edge_planes(polygon):
    """Return ``(a, b, c)`` per edge: ``a*x + b*y + c`` is the signed distance.

    Positive values lie on the polygon's interior side of the edge line.
    """
    count = len(polygon)
    twice_area = 0.0
    for index, point in enumerate(polygon):
        next_point = polygon[(index + 1) % count]
        twice_area += point[0] * next_point[1] - next_point[0] * point[1]
    orientation = 1.0 if twice_area > 0.0 else -1.0
    planes = []
    for index, point in enumerate(polygon):
        next_point = polygon[(index + 1) % count]
        edge_x = next_point[0] - point[0]
        edge_y = next_point[1] - point[1]
        length = math.hypot(edge_x, edge_y)
        plane_a = -orientation * edge_y / length
        plane_b = orientation * edge_x / length
        planes.append(
            (plane_a, plane_b, -(plane_a * point[0] + plane_b * point[1])))
    return tuple(planes)


def _centre_class(planes, centre_x, centre_y, outside_bound):
    """Classify a cell centre against the polygon's edge planes.

    Returns -1 when the cell certainly misses the polygon, 1 when it certainly
    overlaps, and 0 when the exact separating-axis test is needed.
    """
    certainly_inside = True
    for plane_a, plane_b, plane_c in planes:
        signed = plane_a * centre_x + plane_b * centre_y + plane_c
        if signed < -outside_bound:
            return -1
        if signed <= _KERNEL_MARGIN:
            certainly_inside = False
    return 1 if certainly_inside else 0


def _overlap_kernel(polygon, resolution, offset_x, offset_y):
    """Return ``{cell_y: (cell_x, ...)}`` for cells whose box meets the polygon.

    The result equals testing every cell of the symmetric window with
    ``_polygon_intersects_cell``.  Cells whose centre lies clearly inside every
    edge plane overlap, and cells clearly outside one edge plane do not; only the
    remaining boundary cells need the exact separating-axis test.  The kernel
    depends only on the polygon, resolution and centre offset, so it is memoised.
    """
    key = (resolution, polygon, offset_x, offset_y)
    cached = _KERNEL_MEMO.get(key)
    if cached is not None:
        return cached
    min_x = min(point[0] for point in polygon)
    max_x = max(point[0] for point in polygon)
    min_y = min(point[1] for point in polygon)
    max_y = max(point[1] for point in polygon)
    offset_radius_x = math.ceil(max(abs(min_x), abs(max_x)) / resolution) + 2
    offset_radius_y = math.ceil(max(abs(min_y), abs(max_y)) / resolution) + 2
    planes = _edge_planes(polygon)
    outside_bound = _CELL_HALF_DIAGONAL * resolution + _KERNEL_MARGIN
    overlap_offsets = {}
    for cell_y in range(-offset_radius_y, offset_radius_y + 1):
        centre_y = (cell_y - offset_y) * resolution
        offsets_x = []
        for cell_x in range(-offset_radius_x, offset_radius_x + 1):
            centre_x = (cell_x - offset_x) * resolution
            centre_class = _centre_class(
                planes, centre_x, centre_y, outside_bound)
            if centre_class < 0:
                continue
            if centre_class > 0:
                offsets_x.append(cell_x)
                continue
            cell_min_x = (cell_x - 0.5 - offset_x) * resolution
            cell_min_y = (cell_y - 0.5 - offset_y) * resolution
            if _polygon_intersects_cell(
                    polygon, cell_min_x, cell_min_y,
                    cell_min_x + resolution, cell_min_y + resolution):
                offsets_x.append(cell_x)
        if offsets_x:
            overlap_offsets[cell_y] = tuple(offsets_x)
    if len(_KERNEL_MEMO) >= _KERNEL_MEMO_LIMIT:
        _KERNEL_MEMO.clear()
    _KERNEL_MEMO[key] = overlap_offsets
    return overlap_offsets


def _mask_pad(footprint, resolution):
    """Return the padding (cells) that covers every overlap-kernel offset."""
    radius = max(math.hypot(point[0], point[1]) for point in footprint)
    return int(math.ceil(radius / resolution)) + 4


def _lethal_planes(width, height, lethal_rows, pad):
    """Pack the lethal map into one integer for whole-map dilation.

    Row ``r`` occupies slot ``r + pad`` with its columns shifted by ``pad``, and
    each slot is ``stride`` bits wide, so out-of-map rows and columns read as
    zero.  ``levels[k]`` is the OR of the packed map shifted by ``0 .. 2**k - 1``.
    """
    stride = (width + 4 * pad + 16 + 7) // 8 * 8
    row_bytes = stride // 8
    zero_row = bytes(row_bytes)
    slots = []
    for slot in range(height + 2 * pad):
        row = slot - pad
        if 0 <= row < height and lethal_rows[row]:
            slots.append(
                (lethal_rows[row] << pad).to_bytes(row_bytes, "little"))
        else:
            slots.append(zero_row)
    levels = [int.from_bytes(b"".join(slots), "little")]
    for level in range((2 * pad).bit_length() - 1):
        levels.append(levels[-1] | (levels[-1] >> (1 << level)))
    return stride, pad, tuple(levels)


def _accumulate_kernel(overlap_offsets, planes):
    """OR the lethal map over every kernel cell, for all centre rows at once.

    Each run of consecutive kernel columns in a row is one interval of shifts,
    covered by two overlapping doubling levels.
    """
    stride, pad, levels = planes
    accumulated = 0
    for offset_y, offsets_x in overlap_offsets.items():
        row_shift = (offset_y + pad) * stride + pad
        runs = []
        run_first = run_last = offsets_x[0]
        for offset in offsets_x[1:]:
            if offset == run_last + 1:
                run_last = offset
            else:
                runs.append((run_first, run_last))
                run_first = run_last = offset
        runs.append((run_first, run_last))
        for first, last in runs:
            length = last - first + 1
            level = length.bit_length() - 1
            plane = levels[level]
            shift = row_shift + first
            accumulated |= (
                (plane >> shift)
                | (plane >> (shift + length - (1 << level))))
    return accumulated


def _footprint_collision_mask(
        costmap_geometry_value, lethal_rows, width, height, footprint,
        world_yaw, center_offset=(0.0, 0.0), planes=None):
    """Return a bit-mask row for footprint/lethal overlap at each cell.

    The mask is exact for footprint poses at the sampled cell or movement
    midpoint.  ``planes`` is an optional ``_lethal_planes`` packing of
    ``lethal_rows``; pass one when building several masks for the same map.
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
    overlap_offsets = _overlap_kernel(polygon, resolution, offset_x, offset_y)
    if planes is None:
        planes = _lethal_planes(
            width, height, lethal_rows, _mask_pad(footprint, resolution))

    width_mask = (1 << width) - 1
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

    dilated = _accumulate_kernel(overlap_offsets, planes)
    row_bytes = planes[0] // 8
    dilated_bytes = dilated.to_bytes((dilated.bit_length() + 7) // 8, "little")
    blocked_rows = []
    for center_y in range(height):
        center_local_y = (center_y + 0.5 + offset_y) * resolution
        y_inside = (
            center_local_y + min_y >= -1.0e-12
            and center_local_y + max_y <= height * resolution + 1.0e-12)
        if not y_inside:
            blocked_rows.append(width_mask)
            continue
        contribution = int.from_bytes(
            dilated_bytes[center_y * row_bytes:(center_y + 1) * row_bytes],
            "little")
        blocked_rows.append(
            ((width_mask ^ safe_x_mask) | contribution) & width_mask)
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
        legal = set()
        for move_heading, (dx, dy) in enumerate(_MOVE_DIRECTIONS):
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


def robot_start_check(costmap, robot_world, robot_yaw):
    """Classify the robot start as ``(category, cells)``.

    ``"inadmissible"``: invalid input, pose outside the costmap, or a start
    cell costing >= 253 (``cells`` is ``[]``).  ``"collision"``: the footprint
    rotated by ``robot_yaw`` overlaps lethal cells (``cells`` lists the
    overlapping ``(x, y, cost)`` found by the exact footprint test).
    ``"clear"`` otherwise.  Observation only; it does not relax the route
    proof.
    """
    try:
        geometry = costmap_geometry(costmap)
        if geometry is None:
            return "inadmissible", []
        data = bytes(costmap.data)
        world = (float(robot_world[0]), float(robot_world[1]))
        yaw = float(robot_yaw)
        if not all(math.isfinite(value) for value in world + (yaw,)):
            return "inadmissible", []
        cell = _costmap_cell(geometry, world)
        if cell is None or data[cell[1] * geometry[0] + cell[0]] >= 253:
            return "inadmissible", []
        cells = []
        if _footprint_costmap_clear(
                geometry, data, world, NAVIGATION_FOOTPRINT, yaw=yaw,
                collect=cells):
            return "clear", []
        return "collision", cells
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
        return "inadmissible", []


def robot_start_collision_free(costmap, robot_world, robot_yaw):
    """Return whether the robot footprint at its pose is collision-free."""
    return robot_start_check(costmap, robot_world, robot_yaw)[0] == "clear"


def _reachable_costmap_cells(
        costmap_geometry_value, data, robot_world, robot_yaw,
        target_cells, footprint, stop_after_first=False):
    """Return target cells reachable through heading-aware safe costmap space.

    Unknown and lethal cells are not traversable, diagonal moves cannot cut an
    occupied corner, and heading/turn/midpoint footprint masks are checked
    during the search.  Nav2's planner and smoother remain authoritative.

    With ``stop_after_first`` the result holds one target: the first-listed
    target as soon as it is reached, otherwise (when the search ends) the
    first target in listed order that was reached.  The exhaustive mode
    returns every reachable target.
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
        queue = deque([start])
        parents = {start: None}
        remaining = set(target_cell_set)
        while queue and remaining:
            cell_x, cell_y = queue.popleft()
            remaining.discard((cell_x, cell_y))
            for dx, dy in _MOVE_DIRECTIONS:
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
    heading_angles = tuple(
        costmap_yaw + index * math.pi / 4.0 for index in range(8))
    raw_costs = bytes(data)
    mask_cache_key = (costmap_geometry_value, raw_costs, tuple(footprint))
    global _FOOTPRINT_MASK_CACHE
    cached_masks = _FOOTPRINT_MASK_CACHE
    if cached_masks is not None and cached_masks[0] == mask_cache_key:
        (lethal_rows, heading_masks, movement_masks, goal_mask,
         heading_union) = cached_masks[1:]
    else:
        lethal_rows = _threshold_rows(width, height, raw_costs, _LETHAL_FLAGS)
        planes = _lethal_planes(
            width, height, lethal_rows, _mask_pad(footprint, resolution))
        heading_masks = tuple(
            _footprint_collision_mask(
                costmap_geometry_value, lethal_rows, width, height, footprint,
                costmap_yaw + index * math.pi / 16.0, planes=planes)
            for index in range(32))
        movement_masks = tuple(
            _footprint_collision_mask(
                costmap_geometry_value, lethal_rows, width, height, footprint,
                heading_angles[index], center_offset=(dx / 2.0, dy / 2.0),
                planes=planes)
            for index, (dx, dy) in enumerate(_MOVE_DIRECTIONS))
        goal_mask = _footprint_collision_mask(
            costmap_geometry_value, lethal_rows, width, height, footprint, 0.0,
            planes=planes)
        heading_union = _union_rows(heading_masks)
        _FOOTPRINT_MASK_CACHE = (
            mask_cache_key, lethal_rows, heading_masks, movement_masks,
            goal_mask, heading_union)

    def blocked(mask, cell):
        return bool(mask[cell[1]] & (1 << cell[0]))

    def goal_turn_clear(cell, arrival_heading):
        if arrival_heading < 0:
            return not blocked(goal_mask, cell)
        # A cell clear at every heading needs no per-heading sweep.
        if blocked(heading_union, cell):
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

    if not _footprint_costmap_clear(
            costmap_geometry_value, data, robot_world, footprint,
            yaw=robot_yaw):
        return set()

    first_step_headings = _first_step_headings(
        costmap_geometry_value, data, robot_world, footprint,
        robot_yaw=robot_yaw)

    # Breadth-first search over (cell, arrival heading) states.  Targets are
    # resolved in pop order.  Cells live in a padded grid whose border is a
    # barrier, so moves off the map need no bounds checks.  In
    # ``stop_after_first`` mode the first-listed target returns at once when
    # reached; the search otherwise ends when every target is resolved.
    padded_width = width + 2
    padded_size = (height + 2) * padded_width
    barrier = bytearray(b"\x01") * padded_size
    raw_barrier = raw_costs.translate(_BARRIER_FLAGS)
    for cell_y in range(height):
        row_start = (cell_y + 1) * padded_width + 1
        barrier[row_start:row_start + width] = (
            raw_barrier[cell_y * width:(cell_y + 1) * width])
    target_flag = bytearray(padded_size)
    for target_x, target_y in target_cell_set:
        if 0 <= target_x < width and 0 <= target_y < height:
            target_flag[(target_y + 1) * padded_width + target_x + 1] = 1
    visited = bytearray(8 * padded_size)
    move_table = tuple(
        (move_heading, dx, dy, dy * padded_width + dx, dy * padded_width,
         bool(dx and dy))
        for move_heading, (dx, dy) in enumerate(_MOVE_DIRECTIONS))
    queue = deque([(start[0], start[1], -1)])
    first_target = target_cells[0] if stop_after_first else None
    remaining = set(target_cell_set)
    reachable = set()
    while queue and remaining:
        cell_x, cell_y, arrival_heading = queue.popleft()
        cell_pidx = (cell_y + 1) * padded_width + cell_x + 1
        if target_flag[cell_pidx]:
            current_cell = (cell_x, cell_y)
            if current_cell in remaining and goal_turn_clear(
                    current_cell, arrival_heading):
                remaining.remove(current_cell)
                reachable.add(current_cell)
                if current_cell == first_target:
                    return {current_cell}

        turn_free = not ((heading_union[cell_y] >> cell_x) & 1)
        for move_heading, dx, dy, delta, row_delta, diagonal in move_table:
            next_pidx = cell_pidx + delta
            if barrier[next_pidx]:
                continue
            state_slot = next_pidx * 8 + move_heading
            if visited[state_slot]:
                continue
            if diagonal and (
                    barrier[cell_pidx + dx]
                    or barrier[cell_pidx + row_delta]):
                continue
            next_x = cell_x + dx
            next_y = cell_y + dy
            if arrival_heading < 0:
                if move_heading not in first_step_headings:
                    continue
            else:
                if not turn_free and any(
                        (heading_masks[index][cell_y] >> cell_x) & 1
                        for index in _TURN_SEQUENCES[
                            arrival_heading][move_heading]):
                    continue
                if (movement_masks[move_heading][cell_y] >> cell_x) & 1:
                    continue
                if (heading_masks[move_heading * 4][next_y] >> next_x) & 1:
                    continue
            visited[state_slot] = 1
            queue.append((next_x, next_y, move_heading))
    if stop_after_first:
        for target in target_cells:
            if target in reachable:
                return {target}
        return set()
    return reachable


def goal_heading(costmap, robot_world, goal_world, require_turn_clear=False):
    """Travel heading from the robot to the goal, or 0.0 when unusable.

    The heading is used only when the finite inputs are distinct and the
    navigation footprint is clear at the goal at that heading; otherwise the
    previously proven yaw 0.0 is kept.  With ``require_turn_clear`` the
    circumscribed disk must also be clear at the goal so the robot can rotate
    in place there.  Malformed points raise ``TypeError``.
    """
    try:
        values = (robot_world[0], robot_world[1], goal_world[0], goal_world[1])
        values = tuple(float(v) for v in values)
        if not all(math.isfinite(v) for v in values):
            return 0.0
        dx = values[2] - values[0]
        dy = values[3] - values[1]
        if math.hypot(dx, dy) <= 1e-6:
            return 0.0
        heading = math.atan2(dy, dx)
        geometry = costmap_geometry(costmap)
        if geometry is None:
            return 0.0
        goal_point = (values[2], values[3])
        if require_turn_clear and not _turn_clearance_checker(
                geometry, costmap.data, NAVIGATION_FOOTPRINT)(goal_point):
            return 0.0
        if _footprint_costmap_clear(
                geometry, costmap.data, goal_point,
                NAVIGATION_FOOTPRINT, yaw=heading):
            return heading
        return 0.0
    except (AttributeError, ValueError, OverflowError, IndexError):
        return 0.0


def _neighbors(x, y, width, height):
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            if not dx and not dy:
                continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                yield nx, ny


def _validated_occupancy_data(data):
    """Return occupancy values as ints in {-1} U [0, 100], or ``None``."""
    # Fast path: plain ints in range are accepted by _occupancy_value unchanged.
    if set(map(type, data)) <= {int} and min(data) >= -1 and max(data) <= 100:
        return list(data)
    validated = [_occupancy_value(value) for value in data]
    if any(value is None for value in validated):
        return None
    return validated


def _frontier_cells(width, height, values):
    """Return free cells ``(x, y)`` with an 8-neighbour in unknown space.

    Row bit masks evaluate the neighbourhood exactly: a free cell is a frontier
    cell when an unknown cell sits in its own or an adjacent column of the same
    or a vertically adjacent row.
    """
    raw = array("b", values).tobytes()
    unknown_rows = _threshold_rows(width, height, raw, _UNKNOWN_FLAGS)
    free_rows = _threshold_rows(width, height, raw, _FREE_FLAGS)
    width_mask = (1 << width) - 1
    frontier = set()
    for y in range(height):
        free = free_rows[y]
        if not free:
            continue
        near = unknown_rows[y]
        if y > 0:
            near |= unknown_rows[y - 1]
        if y + 1 < height:
            near |= unknown_rows[y + 1]
        near = (near | (near << 1) | (near >> 1)) & width_mask
        hits = free & near
        while hits:
            lowest = hits & -hits
            frontier.add((lowest.bit_length() - 1, y))
            hits ^= lowest
    return frontier


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
        validated_data = _validated_occupancy_data(data)
    except (TypeError, ValueError, OverflowError):
        return None
    if validated_data is None:
        return None

    frontier = _frontier_cells(width, height, validated_data)
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


def _strict_clearance_radius(value):
    """Return ``value`` as a finite, positive real radius in metres, or None."""
    if isinstance(value, bool) or not isinstance(value, Real):
        return None
    try:
        radius = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    if not math.isfinite(radius) or radius <= 0.0:
        return None
    return radius


def _turn_clearance_checker(
        costmap_geometry_value, data, footprint, radius=None):
    """Return ``clear(world)``: a disk around the robot is usable.

    The disk must lie inside the costmap and touch no cell with cost >= 254,
    so the robot can rotate in place there.  Its radius is the circumscribed
    radius of ``footprint`` (farthest vertex from the origin) unless ``radius``
    (metres) is given; ``footprint`` may then be None.  Candidate cell offsets
    are computed once, and each offset row is scanned only over its lethal
    cells.
    """
    width, height, resolution = costmap_geometry_value[:3]
    if radius is None:
        radius = max(math.hypot(x, y) for x, y in footprint)
    radius = radius / resolution
    reach = int(math.ceil(radius)) + 1
    radius_sq = radius * radius
    # Offsets whose cell can touch the disk for some centre inside the
    # centre cell (centre cell spans [0, 1) in each axis).
    offsets = []
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            gap_x = max(dx - 1.0, -(dx + 1.0), 0.0)
            gap_y = max(dy - 1.0, -(dy + 1.0), 0.0)
            if gap_x * gap_x + gap_y * gap_y <= radius_sq:
                offsets.append((dx, dy))
    offset_set = frozenset(offsets)
    row_spans = {}
    for off_x, off_y in offsets:
        row_spans[off_y] = max(row_spans.get(off_y, 0), abs(off_x))
    row_spans = tuple(sorted(row_spans.items()))
    lethal_rows = _threshold_rows(width, height, bytes(data), _LETHAL_FLAGS)

    def clear(world):
        cell = _costmap_cell(costmap_geometry_value, world)
        if cell is None:
            return False
        cell_x, cell_y = cell
        cos_yaw, sin_yaw = costmap_geometry_value[5:7]
        dx = world[0] - costmap_geometry_value[3]
        dy = world[1] - costmap_geometry_value[4]
        local_x = (cos_yaw * dx + sin_yaw * dy) / resolution
        local_y = (-sin_yaw * dx + cos_yaw * dy) / resolution
        if (local_x - radius < 0.0 or local_y - radius < 0.0
                or local_x + radius > width or local_y + radius > height):
            return False
        for off_y, span in row_spans:
            cy = cell_y + off_y
            if not 0 <= cy < height:
                continue
            first_x = max(0, cell_x - span)
            last_x = min(width - 1, cell_x + span)
            lethal_bits = (
                (lethal_rows[cy] >> first_x)
                & ((1 << (last_x - first_x + 1)) - 1))
            while lethal_bits:
                lowest = lethal_bits & -lethal_bits
                cx = first_x + lowest.bit_length() - 1
                lethal_bits ^= lowest
                if (cx - cell_x, off_y) not in offset_set:
                    continue
                nearest_x = min(max(local_x, cx), cx + 1.0)
                nearest_y = min(max(local_y, cy), cy + 1.0)
                if ((nearest_x - local_x) ** 2 + (nearest_y - local_y) ** 2
                        > radius_sq):
                    continue
                return False
        return True

    return clear


APPROACH_STRIDE_CELLS = 2
APPROACH_ENDPOINT_LIMIT = 8


def _approach_endpoints(
        map_geometry, map_data, cells, approach_radius, blacklisted, avoided,
        robot_world, min_goal_distance, costmap_geometry_value, costmap_data,
        footprint_clear, exclude, turn_clear=None):
    """Rank known-free footprint-clear cells near a frontier cluster.

    Returns up to ``APPROACH_ENDPOINT_LIMIT`` items
    ``(distance, cost, y, x, cell, world)`` sorted ascending; footprints are
    checked lazily in ranked order.  Each candidate's costmap cost uses the same
    arithmetic as ``_endpoint_cost``, written inline for speed.
    """
    width, height, resolution, origin_x, origin_y, cos_yaw, sin_yaw = (
        map_geometry)
    cluster_items = []
    for cell in cells:
        world = _cell_world(map_geometry, cell)
        if world is not None:
            cluster_items.append((cell, world))
    if not cluster_items:
        return []
    reach = int(math.ceil(approach_radius / resolution))
    radius_sq = (approach_radius + 1.0e-9) ** 2
    min_goal_sq = min_goal_distance ** 2
    stride = APPROACH_STRIDE_CELLS
    # Stride-aligned global lattice keeps sampling deterministic.  Samples are
    # row-major indices ``y * width + x``; each window is a contiguous x-run.
    sampled = set()
    for fx, fy in cells:
        x0 = max(0, fx - reach)
        x1 = min(width - 1, fx + reach)
        y0 = max(0, fy - reach)
        y1 = min(height - 1, fy + reach)
        start_x = x0 + (-x0) % stride
        start_y = y0 + (-y0) % stride
        if start_x > x1:
            continue
        for y in range(start_y, y1 + 1, stride):
            row_base = y * width
            sampled.update(
                range(row_base + start_x, row_base + x1 + 1, stride))
    # Buckets are in lattice-cell units.  Every cluster cell within ``reach``
    # cells (the approach radius) of a sample lies in the sample's 3x3 bucket
    # neighbourhood when a bucket spans ``reach + 1`` cells, so the minimum over
    # that neighbourhood equals the minimum over the whole cluster whenever it
    # is within the radius.
    bucket_cells = reach + 1
    key_scale = 1 << 40
    buckets = {}
    for cell, world in cluster_items:
        bucket_key = (cell[0] // bucket_cells) * key_scale + cell[1] // bucket_cells
        buckets.setdefault(bucket_key, []).append(world)
    neighbour_offsets = tuple(
        dx * key_scale + dy for dx in (-1, 0, 1) for dy in (-1, 0, 1))
    exclude_keys = {cell[1] * width + cell[0] for cell in exclude}
    cost_width, cost_height = costmap_geometry_value[0:2]
    cost_resolution = costmap_geometry_value[2]
    cost_origin_x, cost_origin_y, cost_cos, cost_sin = (
        costmap_geometry_value[3:7])
    ranked = []
    for index in sampled:
        if exclude_keys and index in exclude_keys:
            continue
        if map_data[index] != 0:
            continue
        y, x = divmod(index, width)
        if blacklisted and (x, y) in blacklisted:
            continue
        local_x = (x + 0.5) * resolution
        local_y = (y + 0.5) * resolution
        world_x = origin_x + cos_yaw * local_x - sin_yaw * local_y
        world_y = origin_y + sin_yaw * local_x + cos_yaw * local_y
        world = (world_x, world_y)
        if avoided(world):
            continue
        if robot_world is not None and (
                (world_x - robot_world[0]) ** 2
                + (world_y - robot_world[1]) ** 2 < min_goal_sq):
            continue
        base_key = (x // bucket_cells) * key_scale + y // bucket_cells
        nearest_sq = math.inf
        for offset in neighbour_offsets:
            for cluster_x, cluster_y in buckets.get(base_key + offset, ()):
                distance_sq = (
                    (world_x - cluster_x) ** 2 + (world_y - cluster_y) ** 2)
                if distance_sq < nearest_sq:
                    nearest_sq = distance_sq
        if nearest_sq > radius_sq:
            continue
        # Costmap cell of this centre, as in _costmap_cell (same operations).
        delta_x = world_x - cost_origin_x
        delta_y = world_y - cost_origin_y
        costmap_local_x = cost_cos * delta_x + cost_sin * delta_y
        costmap_local_y = -cost_sin * delta_x + cost_cos * delta_y
        scaled_x = costmap_local_x / cost_resolution
        nearest_x = round(scaled_x)
        if abs(scaled_x - nearest_x) <= 1.0e-9 * max(1.0, abs(scaled_x)):
            scaled_x = float(nearest_x)
        scaled_y = costmap_local_y / cost_resolution
        nearest_y = round(scaled_y)
        if abs(scaled_y - nearest_y) <= 1.0e-9 * max(1.0, abs(scaled_y)):
            scaled_y = float(nearest_y)
        costmap_x = math.floor(scaled_x)
        costmap_y = math.floor(scaled_y)
        if not (0 <= costmap_x < cost_width and 0 <= costmap_y < cost_height):
            continue
        cost = int(costmap_data[costmap_y * cost_width + costmap_x])
        if cost >= 253:
            continue
        ranked.append((math.sqrt(nearest_sq), cost, y, x, (x, y), world))
    ranked.sort(key=lambda item: item[:4])
    safe = []
    for item in ranked:
        if footprint_clear(item[-2], item[-1]) and (
                turn_clear is None or turn_clear(item[-1])):
            safe.append(item)
            if len(safe) >= APPROACH_ENDPOINT_LIMIT:
                break
    return safe


def costmap_frontier_candidates(
        clusters, grid, costmap, blacklist=(), robot_world=None,
        min_goal_distance=0.0, footprint=None, robot_yaw=0.0,
        require_path_clear=False, return_diagnostics=False,
        approach_radius=0.0, avoid_worlds=(), avoid_radius=0.0,
        goal_clearance_radius=None, return_tiers=False):
    """Select endpoints admissible in costmap, distance, and optional path.

    The default return value remains the candidate-cell list.  When
    ``return_diagnostics`` is true, return ``(candidates, diagnostics)``;
    each diagnostic records whether the cluster has an admissible endpoint,
    whether the heading-aware route proof reached one, and the resulting
    safety/route classification.  The diagnostics are observation-only and
    use the same gates as the normal selector.

    With ``approach_radius > 0`` known-free map cells within that distance of
    a cluster are also endpoints (ranked after the cluster's frontier cells)
    when their footprint is clear.  Endpoints within ``avoid_radius`` of any
    ``avoid_worlds`` point are excluded.

    With ``goal_clearance_radius`` (metres, > 0) approach endpoints must keep
    a disk of that radius free of lethal cells in place of the circumscribed
    radius; frontier-cell endpoints (representative and alternatives) are not
    affected.  ``None`` keeps the circumscribed gate.  An invalid radius
    returns ``None``.

    With ``return_tiers`` (requires ``return_diagnostics``) return
    ``(candidates, diagnostics, tiers)`` where ``tiers`` maps each candidate
    cell to ``"frontier_cell"`` or ``"approach"``, the gate that admitted it.
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
        data = bytes(costmap.data)
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
        for radius in (approach_radius, avoid_radius):
            if (isinstance(radius, bool)
                    or not isinstance(radius, (int, float))
                    or not math.isfinite(radius) or radius < 0.0):
                return None
        goal_radius = None
        if goal_clearance_radius is not None:
            goal_radius = _strict_clearance_radius(goal_clearance_radius)
            if goal_radius is None:
                return None
        approach_radius = float(approach_radius)
        avoid_radius = float(avoid_radius)
        if avoid_worlds is None:
            return None
        avoid_points = []
        for point in avoid_worlds:
            if len(point) != 2:
                return None
            point_x, point_y = float(point[0]), float(point[1])
            if not (math.isfinite(point_x) and math.isfinite(point_y)):
                return None
            avoid_points.append((point_x, point_y))
        avoid_sq = avoid_radius ** 2

        def avoided(world):
            if not avoid_points or avoid_radius <= 0.0 or world is None:
                return False
            return any(
                (world[0] - ax) ** 2 + (world[1] - ay) ** 2 < avoid_sq
                for ax, ay in avoid_points)

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

        turn_clear = None
        if goal_radius is not None:
            turn_clear = _turn_clearance_checker(
                costmap_geometry_value, data, validated_footprint,
                radius=goal_radius)
        elif validated_footprint is not None and approach_radius > 0.0:
            turn_clear = _turn_clearance_checker(
                costmap_geometry_value, data, validated_footprint)

        if return_tiers and not return_diagnostics:
            return None
        candidates = []
        candidate_records = []
        diagnostics = []
        tiers = {}

        def result():
            if return_tiers:
                return candidates, diagnostics, tiers
            return (candidates, diagnostics) if return_diagnostics else candidates

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
                if avoided(_cell_world(map_geometry, cell)):
                    continue
                if robot_world is None:
                    eligible.append(cell)
                    continue
                world = _cell_world(map_geometry, cell)
                if (world is not None and
                        (world[0] - robot_x) ** 2 + (world[1] - robot_y) ** 2
                        >= min_goal_distance ** 2):
                    eligible.append(cell)
            if not eligible and approach_radius <= 0.0:
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
                item for item in ranked
                if footprint_clear(item[-2], item[-1])]
            approach_ranked = []
            if approach_radius > 0.0:
                approach_ranked = _approach_endpoints(
                    map_geometry, grid.data, cells, approach_radius,
                    blacklisted, avoided,
                    robot_world if robot_world is not None else None,
                    min_goal_distance, costmap_geometry_value, data,
                    footprint_clear, set(item[-2] for item in safe_ranked),
                    turn_clear)
            diagnostic["safe_endpoint_count"] = (
                len(safe_ranked) + len(approach_ranked))
            if not require_path_clear:
                if representative_selected:
                    candidates.append(representative)
                    tiers[representative] = "frontier_cell"
                    diagnostic["reachable_endpoint_count"] = 1
                elif safe_ranked:
                    candidates.append(safe_ranked[0][-2])
                    tiers[safe_ranked[0][-2]] = "frontier_cell"
                    diagnostic["reachable_endpoint_count"] = 1
                elif approach_ranked:
                    candidates.append(approach_ranked[0][-2])
                    tiers[approach_ranked[0][-2]] = "approach"
                    diagnostic["reachable_endpoint_count"] = 1
                if diagnostic["reachable_endpoint_count"]:
                    diagnostic["classification"] = "REACHABLE"
                continue

            if representative_selected:
                candidate_records.append((
                    cluster_index, representative, representative_world, 1))
            for _cost, _distance, _y, _x, cell, world in safe_ranked:
                if not representative_selected or cell != representative:
                    candidate_records.append((cluster_index, cell, world, 1))
            for _distance, _cost, _y, _x, cell, world in approach_ranked:
                candidate_records.append((cluster_index, cell, world, 2))
        if not require_path_clear:
            return result()

        target_cells = []
        target_records = []
        record_tiers = {
            (record[0], record[1]): "frontier_cell" if record[3] == 1
            else "approach"
            for record in candidate_records}
        for cluster_index, candidate, world, _tier in candidate_records:
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
                tiers.setdefault(candidate, record_tiers.get(
                    (cluster_index, candidate), "frontier_cell"))
                diagnostics[cluster_index]["reachable_endpoint_count"] += 1
        for diagnostic in diagnostics:
            if diagnostic["reachable_endpoint_count"]:
                diagnostic["classification"] = "REACHABLE"
            elif diagnostic["safe_endpoint_count"]:
                diagnostic["classification"] = "BLOCKED_ROUTE"
        return result()
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def patrol_candidates(
        grid, costmap, robot_world, robot_yaw=0.0, footprint=None,
        min_goal_distance=0.0, sample_spacing=1.0, avoid_worlds=(),
        avoid_radius=0.5, visited_worlds=(), goal_clearance_radius=None):
    """Rank known-free, reachable map cells for continuous patrol.

    Returns map cells ``(x, y)`` best-first, ``[]`` when none qualify, and
    ``None`` for invalid input.  A cell qualifies only when it is known free
    in the map, below the costmap collision threshold, footprint clear (when
    a footprint is given), far enough from the robot, away from the avoided
    worlds, and proven reachable by the same heading-aware route search that
    frontier selection uses.  Cells farthest from the robot and the visited
    worlds rank first; ties break by ``(y, x)`` ascending.  With
    ``goal_clearance_radius`` (metres, > 0) the cell must also keep that disk
    free of lethal cells; an invalid radius returns ``None``.
    """
    try:
        map_geometry = _grid_geometry(grid.info, allow_identity=True)
        if (map_geometry is None
                or len(grid.data) != map_geometry[0] * map_geometry[1]):
            return None
        costmap_geometry_value = costmap_geometry(costmap)
        if costmap_geometry_value is None:
            return None
        data = costmap.data
        if len(data) != costmap_geometry_value[0] * costmap_geometry_value[1]:
            return None
        validated_footprint = None
        if footprint is not None:
            validated_footprint = _strict_footprint(footprint)
            if validated_footprint is None:
                return None
        robot_x, robot_y = (float(value) for value in robot_world)
        robot_yaw = float(robot_yaw)
        sample_spacing = float(sample_spacing)
        min_goal_distance = float(min_goal_distance)
        avoid_radius = float(avoid_radius)
        if (not all(math.isfinite(value)
                    for value in (robot_x, robot_y, robot_yaw))
                or not math.isfinite(sample_spacing) or sample_spacing <= 0.0
                or not math.isfinite(min_goal_distance)
                or min_goal_distance < 0.0
                or not math.isfinite(avoid_radius) or avoid_radius < 0.0):
            return None
        avoid_points = []
        for world in avoid_worlds:
            point = tuple(float(value) for value in world)
            if len(point) != 2 or not all(math.isfinite(v) for v in point):
                return None
            avoid_points.append(point)
        visited_points = []
        for world in visited_worlds:
            point = tuple(float(value) for value in world)
            if len(point) != 2 or not all(math.isfinite(v) for v in point):
                return None
            visited_points.append(point)

        goal_radius = None
        if goal_clearance_radius is not None:
            goal_radius = _strict_clearance_radius(goal_clearance_radius)
            if goal_radius is None:
                return None
        width, height, resolution = map_geometry[:3]
        stride = max(1, int(round(sample_spacing / resolution)))
        if goal_radius is not None:
            turn_clear = _turn_clearance_checker(
                costmap_geometry_value, data, validated_footprint,
                radius=goal_radius)
        else:
            turn_clear = (
                None if validated_footprint is None
                else _turn_clearance_checker(
                    costmap_geometry_value, data, validated_footprint))
        records = []
        for cell_y in range(0, height, stride):
            for cell_x in range(0, width, stride):
                if grid.data[cell_y * width + cell_x] != 0:
                    continue
                cell = (cell_x, cell_y)
                cost = _endpoint_cost(
                    map_geometry, costmap_geometry_value, data, cell)
                if cost is None or cost >= 253:
                    continue
                world = _cell_world(map_geometry, cell)
                if world is None:
                    continue
                if hypot(world[0] - robot_x, world[1] - robot_y) < min_goal_distance:
                    continue
                if any(hypot(world[0] - point[0], world[1] - point[1])
                       <= avoid_radius for point in avoid_points):
                    continue
                if (validated_footprint is not None
                        and not _footprint_costmap_clear(
                            costmap_geometry_value, data, world,
                            validated_footprint)):
                    continue
                if turn_clear is not None and not turn_clear(world):
                    continue
                target = _costmap_cell(costmap_geometry_value, world)
                if target is None:
                    continue
                records.append((cell, world, target))
        if not records:
            return []
        reachable = _reachable_costmap_cells(
            costmap_geometry_value, data, (robot_x, robot_y), robot_yaw,
            [target for _cell, _world, target in records],
            validated_footprint, stop_after_first=False)
        ranked = []
        for cell, world, target in records:
            if target not in reachable:
                continue
            score = min(
                hypot(world[0] - point[0], world[1] - point[1])
                for point in [(robot_x, robot_y)] + visited_points)
            ranked.append((-score, cell[1], cell[0], cell))
        ranked.sort(key=lambda item: item[:3])
        return [item[3] for item in ranked]
    except (AttributeError, TypeError, ValueError, OverflowError):
        return None


def available_candidates(clusters, blacklist):
    """Return candidates not present in the failed-goal blacklist."""
    return [candidate for candidate, _cells in clusters if candidate not in blacklist]

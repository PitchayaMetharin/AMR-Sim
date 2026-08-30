"""Pure frontier extraction helpers used by the online exploration node."""

from collections import deque
from math import hypot


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
    if width <= 0 or height <= 0 or len(data) != width * height:
        return []

    def value(x, y):
        return data[y * width + x]

    frontier = set()
    for y in range(height):
        for x in range(width):
            if value(x, y) != 0:
                continue
            if any(value(nx, ny) == -1 for nx, ny in _neighbors(x, y, width, height)):
                frontier.add((x, y))

    clusters = []
    while frontier:
        seed = min(frontier, key=lambda point: (point[1], point[0]))
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


def available_candidates(clusters, blacklist):
    """Return candidates not present in the failed-goal blacklist."""
    return [candidate for candidate, _cells in clusters if candidate not in blacklist]

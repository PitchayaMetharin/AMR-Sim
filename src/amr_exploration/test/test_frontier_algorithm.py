from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from frontier_algorithm import available_candidates, frontier_clusters


def test_frontier_clusters_are_free_and_deterministic():
    # One free cell beside unknown, with an occupied cell excluded.
    grid = [100, -1, -1, 0, 0, 100, 0, 0, 0]
    clusters = frontier_clusters(3, 3, grid)
    assert clusters
    candidate, cells = clusters[0]
    assert candidate in cells
    assert all(grid[y * 3 + x] == 0 for x, y in cells)
    assert candidate == (0, 1)


def test_blacklist_removes_failed_frontier_candidate():
    clusters = [((1, 2), ((1, 2),)), ((4, 2), ((4, 2),))]
    assert available_candidates(clusters, {(1, 2)}) == [(4, 2)]


def test_invalid_grid_has_no_frontiers():
    assert frontier_clusters(2, 2, [0]) == []

# planner/utils.py

from typing import Tuple
import math

def manhattan(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """
    Compute the Manhattan (L1) distance between two grid cells.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def euclidean(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """
    Compute the Euclidean (L2) distance between two grid cells.
    """
    return math.hypot(a[0] - b[0], a[1] - b[1])


def chebyshev(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """
    Compute the Chebyshev (L∞) distance (good for 8-connected grids).
    """
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def zero_heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    """
    A trivial heuristic that always returns 0 (reduces A* to Dijkstra’s algorithm).
    """
    return 0.0
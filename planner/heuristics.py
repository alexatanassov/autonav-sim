# planner/heuristics.py
"""
Common heuristics for grid-based A*.

Pick the one that matches connectivity and move costs:

- 4-connected with unit-cost moves  -> manhattan
- 8-connected with (diag=√2) costs  -> octile
- Euclidean is admissible for either but can expand more nodes than octile on 8-connected grids.
"""

from __future__ import annotations
import math
from typing import Tuple, Callable

Cell = Tuple[int, int]
Heuristic = Callable[[Cell, Cell], float]

def manhattan(a: Cell, b: Cell) -> float:
    """L1 distance: |di| + |dj|. Best for 4-connected, unit-cost moves."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def euclidean(a: Cell, b: Cell) -> float:
    """L2 distance: sqrt(di^2 + dj^2). Best for 4- or 8-connected."""
    di = a[0] - b[0]
    dj = a[1] - b[1]
    return math.hypot(di, dj)

def chebyshev(a: Cell, b: Cell) -> float:
    """L∞ distance: max(|di|, |dj|). Best for 8-connected when diag cost==1."""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

def octile(a: Cell, b: Cell) -> float:
    """
    Octile metric for 8-connected grids where diagonal step cost = sqrt(2) and
    orthogonal step cost = 1. This matches the true shortest path on an empty grid.
    """
    di = abs(a[0] - b[0])
    dj = abs(a[1] - b[1])
    return (di + dj) - (math.sqrt(2) - 1.0) * min(di, dj)

# Optional: Weighted A*
def weighted(h: Heuristic, epsilon: float) -> Heuristic:
    """
    Scale a heuristic for Weighted A*: you get faster and
    greedier searches that may produce suboptimal paths (bounded by ε).
    """
    if epsilon < 1.0:
        raise ValueError("epsilon must be >= 1.0 for Weighted A*")
    def h_w(a: Cell, b: Cell) -> float:
        return epsilon * h(a, b)
    return h_w

# Registry to select by name
REGISTRY = {
    "manhattan": manhattan,
    "euclidean": euclidean,
    "chebyshev": chebyshev,
    "octile": octile,
}
# planner/costs.py

"""
Cost utilities for A*.

- compute_obstacle_distance: multi-source BFS distance-to-nearest-obstacle (in cells).
- make_weighted_cost: step cost (4/8-connected) scaled by proximity penalty.
"""

from __future__ import annotations
import math
from collections import deque
from typing import Callable, Tuple
import numpy as np

from .grid_map import GridMap


def compute_obstacle_distance(gm: GridMap, connectivity: int = 4) -> np.ndarray:
    """
    Distance transform (in grid cells) to nearest occupied cell.
    Obstacles get 0; free cells grow by 1 per layer (Manhattan/BFS).

    connectivity: 4 or 8 (4 is typical for inflation penalties).
    """
    H, W = gm.height, gm.width
    dist = np.full((H, W), np.inf, dtype=float)
    q: deque[Tuple[int, int]] = deque()

    # Seed queue with all occupied cells
    for i in range(H):
        for j in range(W):
            if gm.is_occupied((i, j)):
                dist[i, j] = 0.0
                q.append((i, j))

    # Neighbor steps for BFS growth
    steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    if connectivity == 8:
        steps += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

    # Multi-source BFS
    while q:
        i, j = q.popleft()
        for di, dj in steps:
            ni, nj = i + di, j + dj
            if 0 <= ni < H and 0 <= nj < W:
                nd = dist[i, j] + 1.0
                if nd < dist[ni, nj]:
                    dist[ni, nj] = nd
                    q.append((ni, nj))

    return dist


def make_weighted_cost(
    gm: GridMap,
    *,
    base_step_cost_4: float = 1.0,
    base_step_cost_diag: float = math.sqrt(2.0),
    penalty: float = 5.0,
    cutoff_cells: int = 3,
    falloff: str = "linear",
    distance_map: np.ndarray | None = None,
) -> Callable[[Tuple[int, int], Tuple[int, int]], float]:
    """
    Returns a cost_fn(u, v) that increases cost near obstacles.

    step cost:
      - 4-connected move -> base_step_cost_4
      - diagonal move    -> base_step_cost_diag

    proximity penalty:
      - uses min(dist(u), dist(v)) where dist = distance-to-obstacle (cells)
      - if dist >= cutoff_cells -> no penalty
      - otherwise penalty * weight(dist), weight in [0,1]

    falloff:
      - "linear": weight = (cutoff - d) / cutoff
      - "quadratic": weight = ((cutoff - d) / cutoff)^2
    """
    if distance_map is None:
        distance_map = compute_obstacle_distance(gm, connectivity=4)

    cutoff = float(cutoff_cells)

    def weight_from_dist(d: float) -> float:
        if d >= cutoff:
            return 0.0
        x = max(0.0, (cutoff - float(d)) / cutoff)
        return x if falloff == "linear" else x * x

    def step_cost(u: Tuple[int, int], v: Tuple[int, int]) -> float:
        di = abs(u[0] - v[0])
        dj = abs(u[1] - v[1])
        return base_step_cost_diag if di == 1 and dj == 1 else base_step_cost_4

    def cost_fn(u: Tuple[int, int], v: Tuple[int, int]) -> float:
        d_u = distance_map[u[0], u[1]]
        d_v = distance_map[v[0], v[1]]
        prox_w = weight_from_dist(min(d_u, d_v))
        return step_cost(u, v) * (1.0 + penalty * prox_w)

    return cost_fn
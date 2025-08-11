# planner/a_star.py

import heapq
import itertools
from typing import Callable, Dict, List, Optional, Tuple
from .grid_map import GridMap

Cell = Tuple[int, int]
Heuristic = Callable[[Cell, Cell], float]
CostFn = Callable[[Cell, Cell], float]


def reconstruct_path(
    came_from: Dict[Cell, Cell],
    current: Cell
) -> List[Cell]:
    """Rebuild path by walking back from goal to start."""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def a_star(
    grid_map: GridMap,
    start: Cell,
    goal: Cell,
    heuristic: Heuristic,
    cost_fn: CostFn,
    connectivity: int = 4,
    max_expansions: Optional[int] = None,
) -> Optional[List[Cell]]:
    """
    A* search on a GridMap.

    Parameters
    ----------
    grid_map : GridMap
        Provides `get_neighbors(cell, connectivity)` and collision checks.
    start, goal : (i, j)
        Grid indices for start and target.
    heuristic : callable(u, v) -> float
        Admissible/consistent estimate from u to v.
    cost_fn : callable(u, v) -> float
        Transition cost from u to v.
    connectivity : int
        4 or 8 neighbor connectivity.
    max_expansions : int | None
        Optional cap on node expansions to avoid runaway searches.

    Returns
    -------
    list[(i, j)] or None
        Path from start to goal (inclusive), or None if unreachable.
    """
    # Check for blocked endpoints
    if not grid_map.is_free(start) or not grid_map.is_free(goal):
        return None

    # Min-heap entries: (f, g, tie, cell)
    open_heap: List[Tuple[float, float, int, Cell]] = []
    tie = itertools.count()
    g_score: Dict[Cell, float] = {start: 0.0}
    came_from: Dict[Cell, Cell] = {}
    closed_set: set[Cell] = set()

    h0 = heuristic(start, goal)
    heapq.heappush(open_heap, (h0, 0.0, next(tie), start))

    expansions = 0

    while open_heap:
        f_curr, g_curr, _, current = heapq.heappop(open_heap)

        if current in closed_set:
            continue
        closed_set.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        expansions += 1
        if max_expansions is not None and expansions > max_expansions:
            return None

        for nbr in grid_map.get_neighbors(current, connectivity):
            if nbr in closed_set:
                continue

            tentative_g = g_curr + cost_fn(current, nbr)
            if tentative_g < g_score.get(nbr, float("inf")):
                g_score[nbr] = tentative_g
                came_from[nbr] = current
                f = tentative_g + heuristic(nbr, goal)
                heapq.heappush(open_heap, (f, tentative_g, next(tie), nbr))

    return None
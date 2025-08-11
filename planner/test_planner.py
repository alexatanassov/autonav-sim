"""
test_planner.py

Provides sanity check for map methods and ensures that
GridMap and A* are working correctly.
"""

from .grid_map import GridMap
from .a_star import a_star
from typing import List, Tuple, Optional
from .heuristics import manhattan, octile

# -------------------
# Original tests
# -------------------

def test_grid_map():
    gm = GridMap(width=5, height=5, resolution=1.0, origin=(0,0))

    # in_bounds / is_occupied / clear_cell / set_obstacle tests
    assert gm.in_bounds((0,0))
    assert not gm.is_occupied((2,2))
    gm.set_obstacle((2,2))
    assert gm.is_occupied((2,2))
    gm.clear_cell((2,2))
    assert not gm.is_occupied((2,2))

    # coordinate round trips
    assert gm.grid_to_world((0,0)) == (0.5, 0.5)  # grid -> world
    assert gm.world_to_grid(0.5, 0.5) == (0,0)    # world -> grid

    neighbors = gm.get_neighbors((1,1), connectivity=4)
    assert (0,1) in neighbors
    assert (1,0) in neighbors


def test_a_star():
    gm = GridMap(width=10, height=10, resolution=1.0, origin=(0,0))

    for y in range(3,7):
        gm.set_obstacle((5, y))
    
    start = (2, 2)
    goal = (8,8)
    path: Optional[List[Tuple[int, int]]] = a_star(
        grid_map=gm,
        start=start,
        goal=goal,
        heuristic=manhattan,
        cost_fn=lambda u, v: 1.0,
        connectivity=4
    )

    assert path is not None, "A* failed to find a path when one exists"
    assert path[0] == start, "Path does not start at the start cell"
    assert path[-1] == goal, "Path does not end at the goal cell"

    cols = [cell[1] for cell in path]
    assert any(c < 5 for c in cols) or any(c > 5 for c in cols), (
        "Path did not route around the obstacle wall at column 5"
    )

# ------------------------
# Additional tests
# ------------------------

import math

def _euclidean(a, b):
    di = a[0] - b[0]
    dj = a[1] - b[1]
    return math.hypot(di, dj)

def _cost4(u, v):
    return 1.0

def _cost8(u, v):
    di = abs(u[0] - v[0])
    dj = abs(u[1] - v[1])
    return math.sqrt(2.0) if di == 1 and dj == 1 else 1.0


def test_blocked_start_or_goal_returns_none():
    gm = GridMap(width=6, height=6, resolution=1.0)
    start, goal = (1, 1), (4, 4)

    gm.set_obstacle(start)
    assert a_star(gm, start, goal, heuristic=_euclidean, cost_fn=_cost4, connectivity=4) is None

    gm.clear_cell(start)
    gm.set_obstacle(goal)
    assert a_star(gm, start, goal, heuristic=_euclidean, cost_fn=_cost4, connectivity=4) is None


def test_8_connectivity_shorter_or_equal_than_4():
    """
    In an empty map, an 8-connected planner should need
    fewer or equal *steps* to reach a diagonal goal than a 4-connected planner.
    """
    gm = GridMap(width=20, height=20, resolution=1.0)
    start, goal = (0, 0), (19, 19)

    p4 = a_star(gm, start, goal, heuristic=manhattan, cost_fn=_cost4, connectivity=4)
    p8 = a_star(gm, start, goal, heuristic=octile,   cost_fn=_cost8, connectivity=8)

    assert p4 is not None and p8 is not None
    steps4 = len(p4) - 1
    steps8 = len(p8) - 1
    assert steps8 <= steps4, f"Expected 8-conn path to be no longer in steps (got {steps8} vs {steps4})"

def test_diagonal_corner_cut_prevention():
    """
    With obstacles at (1,0) and (0,1), moving diagonally (0,0)->(1,1)
    should be disallowed when both orthogonal adjacents are blocked.
    On this tiny map, that means no path exists.
    """
    gm = GridMap(width=3, height=3, resolution=1.0)
    start, goal = (0, 0), (1, 1)
    gm.set_obstacle((1, 0))
    gm.set_obstacle((0, 1))

    path = a_star(gm, start, goal, heuristic=octile, cost_fn=_cost8, connectivity=8)
    assert path is None, "Diagonal corner-cut should be blocked; expected no path"


def test_inflation_blocks_narrow_corridor():
    """
    A 1-cell corridor should close when we inflate obstacles by 1 cell.
    """
    gm = GridMap(width=7, height=5, resolution=1.0)
    # Two walls with a single-cell gap at column 3 on rows 1 and 3
    for j in range(7):
        if j != 3:
            gm.set_obstacle((1, j))
            gm.set_obstacle((3, j))

    start, goal = (0, 3), (4, 3)
    # Sanity check
    p_no_infl = a_star(gm, start, goal, heuristic=manhattan, cost_fn=_cost4, connectivity=4)
    assert p_no_infl is not None, "Path should exist before inflation"

    # Inflate obstacles -> corridor closes
    gm.inflate(radius_cells=1)
    p_infl = a_star(gm, start, goal, heuristic=manhattan, cost_fn=_cost4, connectivity=4)
    assert p_infl is None, "Inflation should have closed the corridor"
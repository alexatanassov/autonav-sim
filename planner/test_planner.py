"""
test_planner.py

Provides sanity for check for map methods and ensures that
GridMap and A* are working correcty 
"""

from .grid_map import GridMap
from .a_star import a_star
from typing import List, Tuple, Optional
from .utils import manhattan

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
    # grid (0,0) -> world (0.5,0.5)
    assert gm.grid_to_world((0,0)) == (0.5, 0.5)
    # world (0.5,0.5) -> grid (0,0)
    assert gm.world_to_grid(0.5, 0.5) == (0,0)

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

if __name__ == "__main__":
    # When you run this file directly (test_planner.py),
    # execute both tests and report success.
    test_grid_map()
    test_a_star()
    print("All planner tests passed!")
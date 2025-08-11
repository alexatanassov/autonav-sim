"""
test_costs.py

Checks that the weighted cost function from planner.costs
produces paths that avoid obstacles more than the uniform cost.
"""

from planner.grid_map import GridMap
from planner.a_star import a_star
from planner.heuristics import octile
from planner.costs import make_weighted_cost

def test_weighted_path_is_longer_or_equal():
    # Map: 20x20 with a vertical wall at col=10, gap at (10,10)
    gm = GridMap(20, 20, resolution=1.0)
    for i in range(20):
        gm.set_obstacle((i, 10))
    gm.clear_cell((10, 10))

    start, goal = (18, 2), (2, 18)

    # Uniform cost path
    p_uniform = a_star(
        gm, start, goal,
        heuristic=octile,
        cost_fn=lambda u, v: 1.0,
        connectivity=8
    )
    assert p_uniform is not None, "Uniform cost path should exist"
    steps_uniform = len(p_uniform) - 1

    # Weighted cost path
    cost_fn = make_weighted_cost(gm, penalty=8.0, cutoff_cells=3, falloff="linear")
    p_weighted = a_star(
        gm, start, goal,
        heuristic=octile,
        cost_fn=cost_fn,
        connectivity=8
    )
    assert p_weighted is not None, "Weighted cost path should exist"
    steps_weighted = len(p_weighted) - 1

    # Weighted path should be at least as long as uniform
    assert steps_weighted >= steps_uniform, (
        f"Expected weighted path length >= uniform ({steps_weighted} vs {steps_uniform})"
    )

if __name__ == "__main__":
    test_weighted_path_is_longer_or_equal()
    print("test_weighted_path_is_longer_or_equal passed.")
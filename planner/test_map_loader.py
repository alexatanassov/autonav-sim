# planner/test_map_loader.py
"""
Tiny synthetic ROS YAML+image loader test.
Writes a 5x5 PNG + YAML in a temp dir and verifies:
- YAML fields parsed
- image->GridMap conversion with vertical flip
- A* runs on the loaded map
"""

from pathlib import Path
import tempfile
import numpy as np
from PIL import Image

from .io.map_loader import load_ros_yaml_map
from .grid_map import GridMap
from .a_star import a_star
from .heuristics import octile
import math

def _cost8(u, v):
    di = abs(u[0] - v[0]); dj = abs(u[1] - v[1])
    return math.sqrt(2.0) if di == 1 and dj == 1 else 1.0

def test_load_ros_yaml_map_and_plan():
    # Image (top-left origin):
    # 5x5, with a single black obstacle at (row=1, col=2).
    # White = free (255), Black = occupied (0) when negate=0.
    img_arr = np.full((5, 5), 255, dtype=np.uint8)
    img_arr[1, 2] = 0  # obstacle in image coords (top-left)

    with tempfile.TemporaryDirectory() as tmpdir:
        tmp = Path(tmpdir)
        img_path = tmp / "map.png"
        yaml_path = tmp / "map.yaml"

        # Write PNG
        Image.fromarray(img_arr, mode="L").save(img_path)

        # YAML: resolution 0.5 m/cell, origin at (0,0,0)
        yaml_text = f"""image: {img_path.name}
resolution: 0.5
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
        yaml_path.write_text(yaml_text)

        gm: GridMap = load_ros_yaml_map(yaml_path)

        # Basic metadata
        assert gm.width == 5 and gm.height == 5
        assert gm.resolution == 0.5
        assert gm.origin == (0.0, 0.0)

        # Verify vertical flip: image obstacle at (1,2) -> grid (H-1-1 = 3, 2)
        assert gm.is_occupied((3, 2)), "Obstacle should appear at (3,2) after vertical flip"

        # Quick A* sanity: plan from bottom-left to top-right avoiding the (3,2) cell if needed
        start = (0, 0)            # bottom-left in GridMap indexing
        goal  = (4, 4)            # top-right
        path = a_star(gm, start, goal, heuristic=octile, cost_fn=_cost8, connectivity=8)
        assert path is not None and path[0] == start and path[-1] == goal
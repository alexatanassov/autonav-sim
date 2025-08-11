# scripts/run_on_map.py
import sys, math
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from planner.a_star import a_star
from planner.heuristics import manhattan, octile
from planner.io.map_loader import load_ros_yaml_map
from planner.grid_map import GridMap

def cost4(u, v): return 1.0
def cost8(u, v):
    di = abs(u[0]-v[0]); dj = abs(u[1]-v[1])
    return math.sqrt(2.0) if di==1 and dj==1 else 1.0

def print_ascii(gm: GridMap, path, start, goal):
    H, W = gm.height, gm.width
    canvas = [['#' if gm.is_occupied((i,j)) else '.' for j in range(W)] for i in range(H)]
    if path:
        for (i,j) in path: canvas[i][j] = '*'
    si,sj = start; gi,gj = goal
    canvas[si][sj] = 'S'; canvas[gi][gj] = 'G'
    for row in canvas[::-1]:
        print(''.join(row))

def main():
    if len(sys.argv) < 6:
        print("Usage: python scripts/run_on_map.py <map.yaml> <start_x> <start_y> <goal_x> <goal_y>")
        sys.exit(1)

    yaml_path = sys.argv[1]
    sx, sy = float(sys.argv[2]), float(sys.argv[3])
    gx, gy = float(sys.argv[4]), float(sys.argv[5])

    gm = load_ros_yaml_map(yaml_path)
    start = gm.world_to_grid(sx, sy)
    goal  = gm.world_to_grid(gx, gy)

    print(f"Map: {yaml_path}  size={gm.width}x{gm.height} res={gm.resolution} origin={gm.origin}")
    print(f"Start: world=({sx:.2f},{sy:.2f}) -> grid={start}")
    print(f"Goal:  world=({gx:.2f},{gy:.2f}) -> grid={goal}")

    path = a_star(gm, start, goal, heuristic=octile, cost_fn=cost8, connectivity=8)
    if path is None:
        print("No path found.")
        sys.exit(2)

    print(f"Path length (cells): {len(path)}")
    print_ascii(gm, path, start, goal)

if __name__ == "__main__":
    main()
# scripts/print_path_ascii.py
import math
from planner.grid_map import GridMap
from planner.a_star import a_star

def manhattan(a, b): return abs(a[0]-b[0]) + abs(a[1]-b[1])
def octile(a, b):
    di, dj = abs(a[0]-b[0]), abs(a[1]-b[1])
    return (di + dj) - (math.sqrt(2) - 1) * min(di, dj)

def cost4(u, v): return 1.0
def cost8(u, v):
    di, dj = abs(u[0]-v[0]), abs(u[1]-v[1])
    return math.sqrt(2.0) if di == 1 and dj == 1 else 1.0

def print_ascii(gm: GridMap, path, start, goal):
    H, W = gm.height, gm.width
    canvas = [['.' for _ in range(W)] for _ in range(H)]
    for i in range(H):
        for j in range(W):
            if gm.is_occupied((i, j)):
                canvas[i][j] = '#'
    if path:
        for (i, j) in path:
            canvas[i][j] = '*'
    si, sj = start; gi, gj = goal
    canvas[si][sj] = 'S'; canvas[gi][gj] = 'G'
    for row in canvas:
        print(''.join(row))

def main():
    gm = GridMap(width=20, height=12, resolution=1.0)
    # make a wall with a gap
    col, gap = 10, 6
    for i in range(gm.height):
        if i != gap:
            gm.set_obstacle((i, col))
    start, goal = (10, 2), (1, 17)

    print("4-connected A*:")
    p4 = a_star(gm, start, goal, heuristic=manhattan, cost_fn=cost4, connectivity=4)
    print_ascii(gm, p4, start, goal)
    print("\n8-connected A*:")
    p8 = a_star(gm, start, goal, heuristic=octile, cost_fn=cost8, connectivity=8)
    print_ascii(gm, p8, start, goal)

if __name__ == "__main__":
    main()
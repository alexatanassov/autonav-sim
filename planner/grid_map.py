# planner/grid_map.py
"""
GridMap: 2D occupancy grid with coordinate transforms, obstacle marking,
neighbor lookup, and simple inflation. Values: 0=free, 1=occupied.
"""

from typing import Iterable, Tuple, List
import numpy as np

Cell = Tuple[int, int]


class GridMap:
    def __init__(
        self,
        width: int,
        height: int,
        resolution: float = 1.0,
        origin: Tuple[float, float] = (0.0, 0.0),
    ):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))  # world coords (x0, y0)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)

    # -------- basic queries / edits --------
    def in_bounds(self, cell: Cell) -> bool:
        i, j = cell
        return 0 <= i < self.height and 0 <= j < self.width

    def is_occupied(self, cell: Cell) -> bool:
        i, j = cell
        return bool(self.grid[i, j])

    def is_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and not self.is_occupied(cell)

    def set_obstacle(self, cell: Cell) -> None:
        if self.in_bounds(cell):
            i, j = cell
            self.grid[i, j] = 1

    def clear_cell(self, cell: Cell) -> None:
        if self.in_bounds(cell):
            i, j = cell
            self.grid[i, j] = 0

    def set_obstacles(self, cells: Iterable[Cell]) -> None:
        """Batch mark obstacles."""
        for c in cells:
            self.set_obstacle(c)

    # -------- coordinates --------
    def world_to_grid(self, x: float, y: float) -> Cell:
        """World (x,y) -> grid (i,j) (cell indices)."""
        i = int((y - self.origin[1]) / self.resolution)
        j = int((x - self.origin[0]) / self.resolution)
        return (i, j)

    def grid_to_world(self, cell: Cell) -> Tuple[float, float]:
        """Grid (i,j) -> world (x,y) at cell center."""
        i, j = cell
        x = j * self.resolution + self.origin[0] + 0.5 * self.resolution
        y = i * self.resolution + self.origin[1] + 0.5 * self.resolution
        return (x, y)

    # -------- neighbor lookup --------
    def get_neighbors(
        self,
        cell: Cell,
        connectivity: int = 4,
        allow_diagonal_through_walls: bool = False,
    ) -> List[Cell]:
        """
        Returns free neighbors. If connectivity==8, prevents 'corner cutting'
        unless allow_diagonal_through_walls=True.
        """
        i, j = cell
        steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if connectivity == 8:
            steps += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        nbrs: List[Cell] = []
        for di, dj in steps:
            nb = (i + di, j + dj)
            if not self.in_bounds(nb) or self.is_occupied(nb):
                continue

            # block diagonal 'corner cutting' through two touching obstacles
            if connectivity == 8 and (di != 0 and dj != 0) and not allow_diagonal_through_walls:
                adj1 = (i + di, j)   # step vertical
                adj2 = (i, j + dj)   # step horizontal
                if (not self.in_bounds(adj1) or self.is_occupied(adj1)) and \
                   (not self.in_bounds(adj2) or self.is_occupied(adj2)):
                    # both orthogonal sides blocked -> disallow diagonal
                    continue

            nbrs.append(nb)
        return nbrs

    # -------- safety margin (simple inflation) --------
    def inflate(self, radius_cells: int) -> None:
        """
        Inflate obstacles by Chebyshev radius in cell units (simple, no deps).
        """
        if radius_cells <= 0:
            return
        occ = np.argwhere(self.grid == 1)
        if occ.size == 0:
            return
        inflated = self.grid.copy()
        r = int(radius_cells)
        for (i, j) in occ:
            i0, i1 = max(0, i - r), min(self.height - 1, i + r)
            j0, j1 = max(0, j - r), min(self.width - 1, j + r)
            inflated[i0:i1 + 1, j0:j1 + 1] = 1
        self.grid = inflated
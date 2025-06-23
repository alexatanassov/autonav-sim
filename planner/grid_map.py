"""
grid_map.py

Defines GridMap: a 2D occupancy grid with utilities for coordinate transforms,
obstacle marking, and neighbor lookup. Used by the A* planner.
"""

import numpy as np
from typing import Tuple, List

class GridMap:

    def __init__(
            self,
            width: int,
            height: int,
            resolution: float = 1.0,
            origin: Tuple[float, float] = (0.0, 0.0),
    ):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin  # (x_origin, y_origin) in world coordinates
        self.grid = np.zeros((height, width), dtype=np.uint8)

    def in_bounds(self, cell: Tuple[int, int]) -> bool:
        """
        Return True if (i,j) is within [0, height-1] x [0, width-1]
        """
        i, j = cell
        return 0 <= i < self.height and 0 <= j < self.width
    
    def is_occupied(self, cell: Tuple[int, int]) -> bool:
        """
        Return True if the given cell is marked occupied (1)
        """
        i, j = cell
        return bool(self.grid[i, j])
    
    def set_obstacle(self, cell: Tuple[int, int]):
        """
        Mark the given cell as occupied (1)
        """
        i, j = cell
        if self.in_bounds(cell):
            self.grid[i, j] = 1
    
    def clear_cell(self, cell: Tuple[int, int]):
        """
        Mark the given cell as free (0)
        """
        i, j = cell
        if self.in_bounds(cell):
            self.grid[i, j] = 0

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates (x, y) into grid indices (i, j)
        """
        i = int((y - self.origin[1]) / self.resolution)
        j = int((x - self.origin[0]) / self.resolution)
        return (i, j)
    
    def grid_to_world(self, cell: Tuple[int, int]) -> Tuple[float, float]:
        """
        Convert grid indices (i, j) into world coordinates (x, y) at the center of that cell.
        """
        i, j = cell
        x = j * self.resolution + self.origin[0] + self.resolution / 2.0
        y = i * self.resolution + self.origin[1] + self.resolution / 2.0
        return (x, y)
    
    def get_neighbors(
        self,
        cell: Tuple[int, int],
        connectivity: int = 4
    ) -> List[Tuple[int, int]]:
        """
        Return a list of neighbor cells around `cell`.
        connectivity=4 uses N, S, E, W;
        connectivity=8 adds diagonals.
        """
        i, j = cell
        steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if connectivity == 8:
            steps += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        neighbors = []
        for di, dj in steps:
            nb = (i + di, j + dj)
            if self.in_bounds(nb) and not self.is_occupied(nb):
                neighbors.append(nb)
        return neighbors
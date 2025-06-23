# planner/a_star.py

import heapq
from typing import Callable, Dict, List, Optional, Tuple
from .grid_map import GridMap

def reconstruct_path(
    came_from: Dict[Tuple[int, int], Tuple[int, int]],
    current: Tuple[int, int]    
) -> List[Tuple[int, int]]:
    """
    Rebuild the path by walking backwards from the goal to star
    """
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return list(reversed(path))

def a_star(
    grid_map: GridMap,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    heuristic: Callable[[Tuple[int, int], Tuple[int, int]], float],
    cost_fn: Callable[[Tuple[int, int], Tuple[int, int]], float],
    connectivity: int = 4
) -> Optional[List[Tuple[int, int]]]: 
    """
    Perform A* search on a GridMap.

    Parameters
    ----------
    grid_map:
        An instance of GridMap
    start:
        (i, j) grid indices where search begins 
    end:
        (i, j) grid indices of target cell
    heuristic:
        A function h(cell1, cell2) that estimates cost from one cell to another
    cost_fn:
        Function of (u,v) giving cost to move from u to v
    connectivity:
        Can be 4 or 8, depending on if 4 or 8 neighbors

    Returns: 
        A list of (i, j) cells from the start to goal cell
        or None if no such path exists
    """
    # Defines open set as a min-heap of tuples: (f_score, g_score, cell)
    open_set: List[Tuple[float, float, Tuple[int,int]]] = []
    heapq.heappush(open_set, (heuristic(start, goal), 0.0, start))

    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    g_score: Dict[Tuple[int, int], float] = {start: 0.0}

    # Main search loop
    while open_set:
        # Pop the cell with smalles f_score
        f_current, g_current, current = heapq.heappop(open_set)

        # If goal is reached, reconstruct and return path
        if current == goal:
            return reconstruct_path(came_from, current)
        
        # If goal not reached, explore neighbors of current cell
        for neighbor in grid_map.get_neighbors(current, connectivity):
                # Compute cost to reach neigbor via current
                g_tentative = g_current + cost_fn(current, neighbor)
                
                # If this path to neighbor is better than any previous one
                if g_tentative < g_score.get(neighbor, float('inf')): # G score defaulted to infinity
                     came_from[neighbor] = current 
                     g_score[neighbor] = g_tentative
                     f_score = g_tentative + heuristic(neighbor, goal)
                     heapq.heappush(open_set, (f_score, g_tentative, neighbor))

    return None
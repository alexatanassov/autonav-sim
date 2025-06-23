# planner

Global path‐planning library: `GridMap` + parameterized A* search.

---

## Installation

From your project root, install dependencies:

```bash
pip install -r requirements.txt

## Planner Module

The `planner` package provides a lightweight, easy-to-use global path-planning toolkit for 2D grids. At its core is the `GridMap` class, which manages a fixed-resolution occupancy grid (mapping between world coordinates and cell indices, marking obstacles, and querying neighbors). On top of that, the `a_star` function performs A* search over the grid, with fully parameterizable heuristics (e.g. Manhattan, Euclidean) and move-cost functions, as well as selectable 4- or 8-connectivity. All components include comprehensive unit tests (`planner/test_planner.py`) to ensure correctness and make it simple to integrate into simulation or real-world ROS pipelines.  
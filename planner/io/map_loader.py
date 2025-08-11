# planner/io/map_loader.py
"""
Map loading utilities for GridMap.
Supports:
- ROS map_server YAML + image (PGM/PNG)
- NumPy binary format (.npy)
"""

from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

import numpy as np
from PIL import Image
import yaml

from planner.grid_map import GridMap

@dataclass
class RosMapMeta:
    image: str
    resolution: float
    origin: Tuple[float, float, float]
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196

def _load_yaml(yaml_path: Path) -> RosMapMeta:
    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f)
    return RosMapMeta(
        image=cfg["image"],
        resolution=float(cfg["resolution"]),
        origin=tuple(cfg["origin"]),
        negate=int(cfg.get("negate", 0)),
        occupied_thresh=float(cfg.get("occupied_thresh", 0.65)),
        free_thresh=float(cfg.get("free_thresh", 0.196)),
    )

def _read_gray_image(img_path: Path) -> np.ndarray:
    im = Image.open(img_path).convert("L")
    return np.array(im, dtype=np.uint8)

def load_ros_yaml_map(yaml_path: str | Path) -> GridMap:
    """
    Load ROS map_server YAML + image into GridMap.
    Applies thresholds & optional inversion (negate).
    Flips vertically so (0,0) = bottom-left in GridMap.
    """
    yaml_path = Path(yaml_path)
    meta = _load_yaml(yaml_path)
    img_path = (yaml_path.parent / meta.image).resolve()

    gray = _read_gray_image(img_path)
    if meta.negate == 1:
        gray = 255 - gray

    occ_cut = int(np.clip(meta.occupied_thresh * 255.0, 0, 255))
    free_cut = int(np.clip(meta.free_thresh * 255.0, 0, 255))

    occupied = gray <= occ_cut
    free = gray >= free_cut

    grid = np.zeros_like(gray, dtype=np.uint8)
    grid[occupied] = 1

    grid = np.flipud(grid)  # flip so bottom-left is (0,0)
    origin_xy = (float(meta.origin[0]), float(meta.origin[1]))
    gm = GridMap(width=grid.shape[1], height=grid.shape[0],
                 resolution=meta.resolution, origin=origin_xy)
    gm.grid = grid
    return gm

def load_npy_map(npy_path: str | Path, resolution: float, origin: Tuple[float, float]) -> GridMap:
    """
    Load .npy binary (H,W) where 1=obstacle, 0=free.
    """
    arr = np.load(npy_path).astype(np.uint8)
    gm = GridMap(width=arr.shape[1], height=arr.shape[0],
                 resolution=resolution, origin=origin)
    gm.grid = arr
    return gm
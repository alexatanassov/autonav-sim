# autonav-sim

**A full-stack autonomous navigation system in simulation.**

This project simulates a mobile robot capable of mapping, localization, motion planning, and perception using ROS and Gazebo. It demonstrates key concepts in SLAM, path planning, reinforcement learning, and computer vision.

## Features
- 2D SLAM using LiDAR or RGBD
- Global path planning with A*/RRT
- Local planning via PID or RL policy
- Object detection using YOLOv5
- ROS/Gazebo-based simulation with TurtleBot3

## Project Structure
- `slam/`: SLAM and localization
- `planner/`: Global and local planning modules
- `perception/`: Object detection, semantic mapping
- `control/`: Velocity control logic
- `simulation/`: URDF models, Gazebo world
- `launch/`: ROS launch files
- `notebooks/`: ML experiments, training logs
- `docs/`: Design notes, diagrams, and evaluation

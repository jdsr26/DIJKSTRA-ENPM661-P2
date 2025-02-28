# Dijkstra's Algorithm for Pathfinding

This project provides a Python implementation of Dijkstra's algorithm for pathfinding on a 2D map. It utilizes NumPy for efficient array operations and OpenCV for map handling and path visualization.

## Features

* **Dijkstra's Algorithm Implementation:** Implements the classic Dijkstra's algorithm to find the shortest path between two points.
* **Map Representation:** Uses NumPy arrays to represent the map, allowing for efficient obstacle representation and path calculations.
* **Obstacle Avoidance:** Handles obstacle avoidance by marking specific map cells as impassable.
* **Path Visualization:** Leverages OpenCV to visualize the map, obstacles, and the calculated shortest path.
* **Custom Input:** Accepts custom start and goal coordinates as input.

## Dependencies

Before running the project, ensure you have the following libraries installed:

```bash
pip install numpy
pip install opencv-python-headless
pip install matplotlib  # Optional, for additional plotting features

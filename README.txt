##Dijkstra's Algorithm for Pathfinding

This project provides a Python implementation of Dijkstra's Algorithm for pathfinding on a map. It efficiently utilizes NumPy and OpenCV to handle map representation, obstacle avoidance, and path visualization.

Features:
- Implements Dijkstra's Algorithm for shortest pathfinding
- Uses NumPy for numerical operations
- Employs OpenCV for map visualization and obstacle handling
- Supports custom start and goal coordinates
- Optional Matplotlib integration for enhanced plotting

Installation:

Ensure you have Python 3.x installed. Then, install the required dependencies:
```bash
pip install numpy
pip install opencv-python-headless
pip install matplotlib  # Optional, for additional plotting features
```
Usage:

Run the script and provide start and goal coordinates as input:
```bash
Enter start coordinates (x,y): 50 50
Enter goal coordinates (x,y): 1150 50
```
Example:
Input:
Enter start coordinates (x,y): 50 50
Enter goal coordinates (x,y): 1150 50

Output:
- The algorithm will compute the shortest path and visualize it on the map.
- The path avoids obstacles and follows the optimal route.

Repository:
GitHub Repo: https://github.com/jdsr26/DIJKSTRA-ENPM661-P2/blob/main/dijsktra_maincode

License:
This project is open-source and available under the MIT License.

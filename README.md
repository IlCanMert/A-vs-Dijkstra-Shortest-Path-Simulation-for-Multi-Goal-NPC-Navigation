# A* vs Dijkstra Shortest Path Simulation for Multi-Goal NPC Navigation

An interactive Python application that simulates and compares **A\*** and **Dijkstra** pathfinding algorithms in a grid-based environment. [cite_start]The project visualizes how non-player characters (NPCs) navigate through obstacles to reach multiple targets using the most efficient route[cite: 10, 11].

## 📖 Overview

[cite_start]This project was developed to visually demonstrate the differences between uninformed search (Dijkstra) and informed search (A* with Manhattan heuristic)[cite: 12]. [cite_start]Unlike standard single-target pathfinding, this tool supports **multi-goal navigation**, where the system calculates the optimal visiting order (TSP-style) to minimize total travel distance before executing the pathfinding algorithm[cite: 63, 64].

## 🚀 Key Features

* [cite_start]**Interactive Grid System:** Users can draw obstacles (walls), place a start point, and add multiple goal points directly on the grid[cite: 13, 14].
* **Algorithm Comparison:** Run both algorithms on the exact same scenario to compare:
    * [cite_start]**Dijkstra:** Guarantees the shortest path but explores the grid in a uniform "wavefront"[cite: 34, 338].
    * [cite_start]**A\* (A-Star):** Uses the Manhattan distance heuristic to guide the search towards the goal, exploring significantly fewer nodes[cite: 35, 339].
* [cite_start]**Multi-Goal Optimization:** If multiple goals are placed, the application calculates the best permutation (brute-force TSP) to visit all targets in the shortest possible total distance[cite: 218, 220].
* **Real-time Visualization:**
    * [cite_start]**🟦 Blue:** Current node being processed[cite: 172].
    * [cite_start]**🟨 Yellow:** Open set (Nodes to be explored)[cite: 170].
    * [cite_start]**⬜ Gray:** Closed set (Visited nodes)[cite: 171].
    * [cite_start]**Turquoise:** Final shortest path[cite: 173].
* [cite_start]**Performance Metrics:** Displays total path length, number of visited nodes, and execution time (in milliseconds) after every run[cite: 147, 151, 152].
* [cite_start]**No External Dependencies:** Built entirely with Python's standard libraries (`tkinter`, `heapq`, `itertools`)[cite: 81, 85].

## 🛠️ Installation & Usage

[cite_start]Since this project uses only standard Python libraries, no `pip install` is required[cite: 89].

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git](https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git)
    cd YOUR_REPO_NAME
    ```

2.  **Run the application:**
    ```bash
    python npc_shortest_path_simulation.py
    ```

## 🎮 Controls

[cite_start]You can control the simulation using the keyboard shortcuts below[cite: 237, 238, 239, 240, 241, 242, 243, 244]:

| Key | Action |
| :--- | :--- |
| **S** | Switch to **Start** placement mode |
| **G** | Switch to **Goal** placement mode (Multiple allowed) |
| **W** | Switch to **Wall** placement mode (Draw obstacles) |
| **A** | Select **A\*** Algorithm |
| **D** | Select **Dijkstra** Algorithm |
| **C** | Generate a **Random** Scenario (Walls & Goals) |
| **R** | **Reset** the grid |
| **SPACE** | **Run** the simulation |

## 📊 Performance Comparison

[cite_start]Based on project test results, while both algorithms always find the same optimal path length, their efficiency differs significantly in complex environments[cite: 337, 340].

**Example Scenario (30x30 Grid, 5 Goals):**
* [cite_start]**A\*:** Visited ~369 nodes (Faster, focused search)[cite: 520].
* [cite_start]**Dijkstra:** Visited ~2258 nodes (Slower, explores widely)[cite: 520].

## 👥 Authors

* Mertcan İLHAN
* Oğuzhan YAZICI
* Tunahan İÇÖZ
[cite_start][cite: 670]

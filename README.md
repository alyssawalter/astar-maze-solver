# Maze Solver
This program implements the A* search algorithm to solve a maze. The maze is represented as a grid where each cell can either be empty (0), a wall (1), an exit (2), visited (3), or part of the solution path (4). The program uses the Manhattan distance as the heuristic to guide the search.

## Features
- Utilizes the A* search algorithm to efficiently find the shortest path from the start to the exit in the maze.
- Calculates the heuristic cost using the Manhattan distance, providing an admissible and consistent heuristic for guiding the search.
- Generates and displays the solution path, including the moves taken from the start to the exit.
- Can evaluate the shortest path after disabling each of the four possible moves (left, right, up, down), allowing for analysis of each move's impact on the solution.

## Usage
### Prerequisites
- Python 3.x

### Installation
1. Clone this repository to your local machine:
   
```bash
git clone https://github.com/alyssawalter/astar-maze-solver.git
``` 
2. Navigate to the project directory:

```bash
cd astar-maze-solver
```

### Running the Game
Run the main.py script:
```bash
python main.py
```

## File Structure
- main.py: Main script containing the implementation of the A* algorithm and maze solving logic.
- maze2024.txt: Input file containing the maze layout. You can replace this file with your own maze configurations.

## Instructions
- The program will display the initial maze configuration.
- It will then calculate the shortest path after disabling each move (left, right, up, down).
- Finally, it will display the best move (the disabled move resulting in the shortest path) and its corresponding shortest path length.

## About the Maze
### Maze Constraints
The maze is represented as a grid with specific constraints assigned to different cell types. These constraints help in defining the behavior of the maze solver algorithm and provide clarity in understanding the maze's structure.

- SPACE (0): Represents an empty space where the solver can move freely without any obstructions.
- WALL (1): Represents a wall or obstacle blocking the path of the solver. The solver cannot pass through walls and must navigate around them.
- EXIT (2): Marks the exit point or goal of the maze. The solver's objective is to reach this point from the starting position.
- VISITED (3): Indicates that a cell has been visited by the solver during its search process. This helps in preventing the solver from revisiting already explored paths.
- PATH (4): Denotes cells that are part of the solution path from the starting point to the exit. These cells represent the shortest path discovered by the solver.
- START_MARK (5): Marks the starting position of the solver in the maze. The solver initiates its search from this point and navigates through the maze to reach the exit.

### Initial Maze
This is the maze from the text file included in the project: 'maze2024.txt'. 
```bash
[[1 1 1 0 0 0 0 0 0 1]
 [1 0 5 1 1 1 1 1 0 1]
 [1 0 1 1 1 1 1 0 2 1]
 [0 0 0 0 0 1 0 0 0 0]
 [1 1 0 1 0 1 0 1 0 0]
 [0 0 0 1 0 1 0 0 0 1]
 [1 1 1 1 0 0 0 1 0 1]
 [1 0 0 0 0 1 0 1 0 0]
 [1 1 1 1 0 0 0 1 0 1]
 [0 2 0 0 0 1 0 1 0 0]
 [1 1 1 0 1 1 0 1 1 0]
 [1 0 0 0 1 0 0 1 0 0]
 [0 0 1 0 1 1 0 0 0 1]]
```
For detailed results and analysis, please refer to the program output.

Feel free to experiment with your own maze configurations and analyze different move scenarios!

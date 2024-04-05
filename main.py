
"""
This program implements the astar search algorithm for solving a maze
(1 point cost for each move) and using the Manhattan distance as the heuristic.
"""

import numpy as np
import queue 
from heapq import heapify


class MazeState():
    """ Stores information about each visited state within the search """
    # Define constants
    SPACE = 0
    WALL = 1
    EXIT = 2
    VISITED = 3
    PATH = 4
    START_MARK = 5

    MAZE_FILE = 'maze2024.txt'
    maze = np.loadtxt(MAZE_FILE, dtype=np.int32)

    # Find the start position in the maze of the form (row, column)
    # in the given maze, that start = (1, 2) ie. second row down, third column from left
    start = tuple(np.array(np.where(maze == 5)).flatten())

    # find the positions of exit points in the maze
    # in the given maze ends = (array([2, 8]), array([8, 1]))
    ends = np.where(maze == 2)

    move_num = 0  # Used by show_path() to count moves in the solution path

    def reset_state(self):
        """ Reset the state to the initial state defined by the maze file """
        MazeState.maze = np.loadtxt(MazeState.MAZE_FILE, dtype=np.int32)
        MazeState.start = tuple(np.array(np.where(MazeState.maze == 5)).flatten())
        MazeState.ends = np.where(MazeState.maze == 2)
        MazeState.move_num = 0

    def __init__(self, conf=start, g=0, h=0, pred_state=None, pred_action=None):
        """ Initializes the state with information passed from the arguments """
        self.pos = conf  # Configuration of the state - current coordinates
        self.gcost = g  # Path cost
        self.hcost = h # heuristic cost
        self.fcost = g + h # cost-function value (gx + hx)
        self.pred = pred_state  # Predecesor/parent state
        self.action_from_pred = pred_action  # Action from predecesor/parent state to current state

    def __hash__(self):
        """ Returns a hash code so that it can be stored in a set data structure """
        return self.pos.__hash__()

    def is_goal(self):
        """ Returns true if current position is same as the exit position """
        return self.maze[self.pos] == MazeState.EXIT

    def __eq__(self, other):
        """ Checks for equality of states by positions only """
        return self.pos == other.pos

    def __lt__(self, other):
        """ Allows for ordering the states by the cost-function (f) value """
        return self.fcost < other.fcost

    def __str__(self):
        """ Returns the maze representation of the state """
        a = np.array(self.maze)
        a[self.start] = MazeState.START_MARK
        a[self.ends] = MazeState.EXIT
        return str(a)

    def show_path(self):
        """ Recursively outputs the list of moves and states along path """
        if self.pred is not None:
            self.pred.show_path()

        if MazeState.move_num == 0:
            print('START')
        else:
            print('Move', MazeState.move_num, 'ACTION:', self.action_from_pred)
        MazeState.move_num = MazeState.move_num + 1
        self.maze[self.pos] = MazeState.PATH

    def get_new_pos(self, move):
        """ Returns a new position from the current position and the specified move """
        # note: if we are moving up, the row number goes down (0 == top row),
        # if we are moving left, the column number goes down (0 == leftmost column)

        # remember, self.pos = (row, column)
        cur_row = self.pos[0]
        cur_col = self.pos[1]

        # shape is an attribute of NumPy Arrays that returns a tuple (# of rows, # of columns)
        total_rows = self.maze.shape[0]
        total_columns = self.maze.shape[1]

        # to enable wrapping, if we move up while on row 0, we need to be moved to the bottom row,
        # which can be achieved by -> (current row - 1 + total rows) % total rows
        # same logic applies to down, left, right
        if move == 'up':
            new_pos = (((cur_row - 1 + total_rows) % total_rows), cur_col)

        elif move == 'down':
            new_pos = (((cur_row + 1 + total_rows) % total_rows), self.pos[1])

        elif move == 'left':
            new_pos = (cur_row, ((cur_col - 1 + total_columns) % total_columns))

        elif move == 'right':
            new_pos = (cur_row, ((cur_col + 1 + total_columns) % total_columns))

        else:
            raise ('wrong direction for checking move')
        return new_pos

    def can_move(self, move):
        """ Returns true if agent can move in the given direction """
        new_pos = self.get_new_pos(move)
        if new_pos[0] < 0 or new_pos[0] >= self.maze.shape[0] or new_pos[1] < 0 or new_pos[1] >= self.maze.shape[1]:
            return False
        else:
            return self.maze[new_pos] != MazeState.WALL

    def gen_next_state(self, move):
        """ Generates a new MazeState object by taking move from current state """
        new_pos = self.get_new_pos(move)

        # calculate Manhattan distance heuristic
        hcost = self.calculate_manhattan_distance(new_pos)

        if self.maze[new_pos] != MazeState.EXIT:
            self.maze[new_pos] = MazeState.VISITED

        return MazeState(new_pos, self.gcost + 1, hcost,  self, move)

    def calculate_manhattan_distance(self, new_pos):
        """ Calculates Manhattan distance heuristic from current position to exit(s) """
        # Considers all exits in maze
        exit_positions = list(zip(*np.where(self.maze == MazeState.EXIT)))

        # Calculate Manhattan distance to the closest exit
        min_distance = float('inf')  # initiate at positive infinity
        for exit_pos in exit_positions:
            distance = abs(new_pos[0] - exit_pos[0]) + abs(new_pos[1] - exit_pos[1])
            min_distance = min(min_distance, distance) # Ensures you are checking against the closer exit

        return min_distance


def run_search(disabled_move=None):
    # load start state onto frontier priority queue
    frontier = queue.PriorityQueue()
    start_state = MazeState() # represents the starting configuration of the maze
    frontier.put(start_state) # states are ordered in the queue according to their fcost dictated by def __lt__

    # Keep a closed set of states to which optimal path was already found
    closed_set = set()

    # Expand state (up to 4 moves possible)
    possible_moves = ['left', 'right', 'down', 'up']

    # check if there is a disabled move
    if disabled_move in possible_moves:
        possible_moves.remove(disabled_move)

    num_states = 0

    next_state = None

    while not frontier.empty():
        # Choose state at front of priority queue (lowest fcost)
        next_state = frontier.get()
        num_states = num_states + 1

        # If goal then quit
        if next_state.is_goal():
            break

        # Add state chosen for expansion to closed_set
        closed_set.add(next_state)

        # Expanding the current state (node) by generating new states for each of the 4 possible moves
        for move in possible_moves:
            if next_state.can_move(move):
                neighbor = next_state.gen_next_state(move)
                if neighbor in closed_set:
                    continue
                # If the new state is not in the closed_set, it is added to the frontier priority queue
                if neighbor not in frontier.queue:
                    frontier.put(neighbor)
                # If it is already in the frontier queue, we compare the fcost of it now to previously
                # and update the queue accordingly
                else:
                    if neighbor.fcost < frontier.queue[frontier.queue.index(neighbor)].fcost:
                        frontier.queue[frontier.queue.index(neighbor)] = neighbor
                        heapify(frontier.queue) # ensures that the lowest fcost states have priority

    return print_solution(disabled_move, start_state, next_state, num_states, MazeState.move_num - 1)


def print_solution(disabled_move, start_state, next_state, num_states, move_path_length):
    if next_state.is_goal():
        print("next_state:")
        print(next_state)
        print("\n\nSOLUTION AFTER DISABLED MOVE: ", disabled_move)
        next_state.show_path()
        print(start_state)  # the ending state of the maze
        print('Number of states visited =', num_states)
        move_path_length = MazeState.move_num - 1
        print('Length of shortest path = ', move_path_length)
    else:
        print("\nSOLUTION AFTER DISABLED MOVE: ", disabled_move)
        print(start_state)  # the ending state of the maze
        print(f"No solution")
    path_length = move_path_length
    start_state.reset_state() # reset maze back to starting state
    return path_length


print('INITIAL MAZE')
print(MazeState())
print("\n\n")

disable_move = ['left', 'right', 'down', 'up']
move_dict = {}
for move in disable_move:
    move_dict[move] = run_search(move)

shortest_path = min((key for key, value in move_dict.items() if value >= 0), default=None)
print("\nBEST MOVE: disable ", shortest_path)
print("SHORTEST PATH LENGTH FOR BEST MOVE: ", move_dict[shortest_path])


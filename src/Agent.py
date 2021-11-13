# Import necessary things
from abc import ABC, abstractmethod

import numpy as np
import math

from constants import NUM_ROWS, NUM_COLS, STARTING_POSITION_OF_AGENT, X, Y, ACCURACY_TO_ACHIEVE, \
    FLAT_FALSE_NEGATIVE_RATE, HILLY_FALSE_NEGATIVE_RATE, FOREST_FALSE_NEGATIVE_RATE
from src.Cell import Cell
from helpers.helper import astar_search, check
from helpers.helper import compute_current_estimated_goal


# Agent class
class Agent(ABC):
    def __init__(self, algorithm: str = 'astar'):
        self.maze = list()
        # self.knowledge_base = list()
        # self.variable_to_constraint_dict = dict()
        # Compute four neighbors
        for row in range(NUM_ROWS):
            cell_list = list()
            for col in range(NUM_COLS):
                cell_list.append(Cell())
                for ind in range(len(X)):
                    neighbor = (row + X[ind], col + Y[ind])
                    if check(neighbor):
                        # cell_list[col].num_neighbor += 1
                        cell_list[col].four_neighbors.append(neighbor)
                for ind in range(-1, 2):
                    for ind2 in range(-1, 2):
                        neighbor = (row + ind, col + ind2)
                        if neighbor == (row, col):
                            continue
                        if check(neighbor):
                            cell_list[col].eight_neighbors.append(neighbor)
            self.maze.append(cell_list)

        # Initialize some variables
        self.algorithm = algorithm
        self.current_position = STARTING_POSITION_OF_AGENT
        self.num_cells_processed_while_planning = 0
        self.final_paths = list()
        self.num_backtracks = 0
        self.parents = dict()

        self.current_estimated_goal = list()

        self.num_confirmed_cells = 0
        self.num_confirmed_blocked_cells = 0

        self.num_astar_calls = 0
        self.num_bumps = 0
        self.num_early_termination = 0

        self.num_examinations = 0

        # set list of global threshold
        self.global_threshold = list()
        self.global_threshold.append(math.ceil(math.log(ACCURACY_TO_ACHIEVE) / math.log(FLAT_FALSE_NEGATIVE_RATE)))
        self.global_threshold.append(math.ceil(math.log(ACCURACY_TO_ACHIEVE) / math.log(HILLY_FALSE_NEGATIVE_RATE)))
        self.global_threshold.append(math.ceil(math.log(ACCURACY_TO_ACHIEVE) / math.log(FOREST_FALSE_NEGATIVE_RATE)))

    def pre_planning(self, agent_num=6):
        self.current_estimated_goal = compute_current_estimated_goal(self.maze, self.current_position, agent_num)

    # General method for planning
    def planning(self, goal_pos):
        # Choose which algorithm you want to use for search
        if self.algorithm == 'astar':
            self.parents, num_explored_nodes = astar_search(self.maze, self.current_position, goal_pos)[:2]
            self.num_cells_processed_while_planning += num_explored_nodes
            # print(self.num_cells_processed_while_planning)
        # elif self.algorithm == 'bfs':
        #     parents, num_explored_nodes = bfs_search(maze, start_pos, goal_pos)
        else:
            raise Exception("algorithm should be either astar or bfs")

    # reset method
    def reset(self):
        # self.knowledge_base = list()
        # self.variable_to_constraint_dict = dict()
        self.current_position = STARTING_POSITION_OF_AGENT
        self.num_cells_processed_while_planning = 0
        self.final_paths = list()
        self.num_backtracks = 0
        self.parents = dict()
        self.current_estimated_goal = list()
        self.num_confirmed_cells = 0
        self.num_confirmed_blocked_cells = 0

        self.num_astar_calls = 0
        self.num_bumps = 0
        self.num_early_termination = 0

        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                self.maze[row][col].reset()

        self.num_examinations = 0

    @abstractmethod
    def execution(self, full_maze: np.array, target_pos: tuple = None):
        pass

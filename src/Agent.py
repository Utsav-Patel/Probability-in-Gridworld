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
    # Method to initialize some variables for this class
    def __init__(self):
        self.maze = list()
        # Compute four and eight neighbors of each cell
        for row in range(NUM_ROWS):
            cell_list = list()
            for col in range(NUM_COLS):
                cell_list.append(Cell())
                for ind in range(len(X)):
                    neighbor = (row + X[ind], col + Y[ind])
                    if check(neighbor):
                        cell_list[col].four_neighbors.append(neighbor)
                for ind in range(-1, 2):
                    for ind2 in range(-1, 2):
                        neighbor = (row + ind, col + ind2)
                        if neighbor == (row, col):
                            continue
                        if check(neighbor):
                            cell_list[col].eight_neighbors.append(neighbor)
            self.maze.append(cell_list)

        # Initialize some important variables of this class
        self.current_position = STARTING_POSITION_OF_AGENT
        self.probability_of_containing_target = np.zeros((NUM_ROWS, NUM_COLS)) + (1.0 / (NUM_ROWS * NUM_COLS))
        self.false_negative_rates = np.zeros((NUM_ROWS, NUM_COLS)) + 0.5
        self.final_paths = list()
        self.parents = dict()
        self.children = dict()
        self.current_estimated_goal = list()
        self.num_examinations = 0

        # set list of global threshold
        self.global_threshold = list()
        self.global_threshold.append(math.ceil(math.log(ACCURACY_TO_ACHIEVE) / math.log(FLAT_FALSE_NEGATIVE_RATE)))
        self.global_threshold.append(math.ceil(math.log(ACCURACY_TO_ACHIEVE) / math.log(HILLY_FALSE_NEGATIVE_RATE)))
        self.global_threshold.append(math.ceil(math.log(ACCURACY_TO_ACHIEVE) / math.log(FOREST_FALSE_NEGATIVE_RATE)))

    def pre_planning(self, agent_num=6):
        """
        Method to find current estimated goal
        :param agent_num: agent number for which you want to find estimated goal
        :return: Nothing to return as we are setting attribute
        """
        self.current_estimated_goal = compute_current_estimated_goal(self.maze, self.current_position, agent_num,
                                                                     self.probability_of_containing_target,
                                                                     self.false_negative_rates)

    # General method for planning
    def planning(self, goal_pos):
        """
        Method to find a path from current position to current estimated goal
        :param goal_pos: agent's current estimated goal
        :return: Nothing as we are storing results in agent's parents object
        """
        self.parents, num_explored_nodes = astar_search(self.maze, self.current_position, goal_pos)[:2]
        # self.num_cells_processed_while_planning += num_explored_nodes

    # reset method
    def reset(self):
        """
        Reset method to reset each each variable
        :return: Nothing
        """
        self.current_position = STARTING_POSITION_OF_AGENT
        self.probability_of_containing_target = np.zeros((NUM_ROWS, NUM_COLS)) + (1.0 / (NUM_ROWS * NUM_COLS))
        self.false_negative_rates = np.zeros((NUM_ROWS, NUM_COLS)) + 0.5
        self.final_paths = list()
        self.parents = dict()
        self.children = dict()
        self.current_estimated_goal = list()

        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                self.maze[row][col].reset()

        self.num_examinations = 0

    @abstractmethod
    def execution(self, full_maze: np.array, target_pos: tuple = None):
        pass

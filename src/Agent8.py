import math

import numpy as np

from helpers.agent8 import forward_execution, parent_to_child_dict
from src.Agent import Agent


class Agent8(Agent):
    def __init__(self):
        super().__init__()

    # Override execution method of Agent class
    def execution(self, full_maze: np.array, target_position=None):
        percent_of_cells_to_examine = 30  # what % of cells in path from A* to examine with highest probabilities

        # get path from start(current_position) to goal(current_estimated_goal)
        # print("Parents:", self.parents)
        temp_child = parent_to_child_dict(self.parents, self.current_estimated_goal)
        # print("Child:", temp_child)
        # number of cells to examine from the whole path by A*
        num_cells_to_examine = math.ceil((percent_of_cells_to_examine * len(temp_child)) / 100)

        temp_current_pos = self.current_position

        # list that contains the cells in the path but in the descending order of their probability of containing target
        path_list = list()

        # if len(temp_child) == 1 and temp_child == {(0, 0): (0, 0)}:                  # Condition for start element only
        #     # print("here")
        #     path_list.append([self.maze[temp_current_pos[0]][temp_current_pos[1]].probability_of_containing_target *
        #                       (1 - self.maze[temp_current_pos[0]][temp_current_pos[1]].false_negative_rate),
        #                       temp_current_pos])

        """loop to store the prob of containing target with the indices in list of list from start 
               to end of path returned by A"""
        while temp_current_pos != temp_child[temp_current_pos]:
            # print("here") (1 - self.maze[temp_current_pos[0]][temp_current_pos[1]].false_negative_rate)
            path_list.append([self.maze[temp_current_pos[0]][temp_current_pos[1]].probability_of_containing_target,
                              temp_current_pos])
            temp_current_pos = temp_child[temp_current_pos]

        path_list.append([self.maze[temp_current_pos[0]][temp_current_pos[1]].probability_of_containing_target *
                          (1 - self.maze[temp_current_pos[0]][temp_current_pos[1]].false_negative_rate),
                          temp_current_pos])
        # Sort the cells w.r.t to their prob of containing target in descending order
        path_list.sort(reverse=True)

        # set containing cells to examine in the forward execution
        cells_to_examine = set()
        count = 0  # keeps track of the index in the path_list
        # print("Dict len:", len(temp_child))
        print('Target position', target_position)
        print('Current path', path_list)
        for element in path_list:
            print(self.maze[element[1][0]][element[1][1]].is_blocked, full_maze[element[1][0]][element[1][1]], end=" ")
        print()
        # print("Cells to examine:", num_cells_to_examine)
        print('Children', temp_child)
        # loop to store num_cells_to_examine in set which will be used in forward execution
        while num_cells_to_examine > 0:
            cells_to_examine.add(path_list[count][1])
            count += 1
            num_cells_to_examine -= 1

        self.num_astar_calls += 1
        self.num_bumps += 1

        # make changes in forward_execution()
        current_path, num_backtracks, truth_value, examinations = forward_execution(self.maze, full_maze,
                                                                                    self.current_position,
                                                                                    self.current_estimated_goal,
                                                                                    self.parents, cells_to_examine,
                                                                                    target_position)[:4]

        self.current_position = current_path[-1]
        self.final_paths.append(current_path)
        self.num_examinations += examinations
        self.num_backtracks += num_backtracks

        return truth_value

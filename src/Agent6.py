# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 14:55:46 2021

@author: Gambit
"""

import numpy as np


from helpers.agent6 import forward_execution, examine_and_propogate_probability
from src.Agent import Agent
# Blindfolded agent's class
class Agent6(Agent):
    def __init__(self):
        super().__init__()

    # Override execution method of Agent class
    def execution(self, full_maze: np.array):
        self.num_astar_calls += 1
        self.num_bumps += 1
        current_path, num_backtracks = forward_execution(self.maze, full_maze, self.current_position, 
                                                         self.current_estimated_goal, self.parents,
                                                         want_to_explore_field_of_view=False)[:2]
        self.current_position = current_path[-1]
        self.final_paths.append(current_path)
        self.num_backtracks += num_backtracks

    
    def examine(self, full_maze: np.array, target_pos):
        value = examine_and_propogate_probability(self.maze, full_maze, self.current_position, target_pos, self.current_estimated_goal, self.parents)
        self.num_examinations += 1
        return value
        

import numpy as np

from helpers.agent6 import forward_execution, parent_to_child_dict
from src.Agent import Agent
from helpers.helper import examine_and_propagate_probability


# Blindfolded agent's class
class Agent6(Agent):
    def __init__(self):
        super().__init__()

    # Override execution method of Agent class
    def execution(self, full_maze: np.array, target_pos: tuple = None):
        self.num_astar_calls += 1
        self.num_bumps += 1
        current_path, num_backtracks = forward_execution(self.maze, full_maze, self.current_position,
                                                         self.current_estimated_goal, self.parents)[:2]
        self.current_position = current_path[-1]
        self.final_paths.append(current_path)
        self.num_backtracks += num_backtracks

    def examine(self, full_maze: np.array, target_pos):
        children = parent_to_child_dict(self.parents, self.current_estimated_goal)
        value = examine_and_propagate_probability(self.maze, self.current_position, target_pos,
                                                  self.current_estimated_goal, children[self.current_position])
        self.num_examinations += 1
        return value

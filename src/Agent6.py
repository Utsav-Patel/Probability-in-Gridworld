# Necessary imports
import numpy as np

from helpers.agent6 import forward_execution
from src.Agent import Agent
from helpers.helper import examine_and_propagate_probability, parent_to_child_dict


# Blindfolded agent's class
class Agent6(Agent):
    def __init__(self):
        super().__init__()

    # Override execution method of Agent class
    def execution(self, full_maze: np.array, target_pos: tuple = None):
        """
        Agent 6,7, and 8's execution method
        :param full_maze: origin maze array
        :param target_pos: actual target position
        :return: Nothing
        """

        # Calculate children from parents dictionary
        self.children = parent_to_child_dict(self.parents, self.current_estimated_goal)

        # Agent will move along the planned path to reach current estimated goal
        current_path, num_backtracks = forward_execution(self.maze, self.false_negative_rates, full_maze,
                                                         self.current_position, self.current_estimated_goal,
                                                         self.children)[:2]
        # Pick the last element of the current path to get agent's current position
        self.current_position = current_path[-1]

        # Append current path to final path
        self.final_paths.append(current_path)

    def examine(self, target_pos):
        """
        Method is used to examine the agent's current cell if it has reached to current estimated goal otherwise set
        next cell to block because we can't be able to reach because of that.
        :param target_pos: target's position
        :return: True if agent has found out target otherwise False
        """
        is_target_found = examine_and_propagate_probability(self.maze, self.probability_of_containing_target,
                                                            self.false_negative_rates, self.current_position,
                                                            target_pos, self.current_estimated_goal,
                                                            self.children[self.current_position])
        if self.current_position == self.current_estimated_goal:
            self.num_examinations += 1
        return is_target_found

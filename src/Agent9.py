import numpy as np

from constants import NUM_COLS, NUM_ROWS, ZERO_PROBABILITY, ONE_PROBABILITY
from src.Agent import Agent
from helpers.helper import examine_and_propagate_probability, parent_to_child_dict, update_status, \
    compute_current_estimated_goal


# Agent 9 class
class Agent9(Agent):

    def __init__(self):
        """
        Initialize agent 9's specific attributes
        """
        super().__init__()
        # to store the sensed result of agent 9
        self.is_target_in_neighbors = False
        # to store the prediction for the next time stamp
        self.probability_of_containing_target_next_step = np.zeros((NUM_ROWS, NUM_COLS)) + (1.0 / (NUM_ROWS * NUM_COLS))

    def reset(self):
        """
        To reset all variables of the class
        """
        super(Agent9, self).reset()
        self.is_target_in_neighbors = False
        self.probability_of_containing_target_next_step = np.zeros((NUM_ROWS, NUM_COLS)) + (1.0 / (NUM_ROWS * NUM_COLS))

    def sense(self, target_position: tuple):
        """
        Method is used to sense neighbors of agent and update belief state accordingly
        :param target_position: Actual target position
        :return: None
        """
        # Check neighbors and sense whether target is in eight neighbors or not
        self.is_target_in_neighbors = False
        for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
            if target_position == neighbor:
                self.is_target_in_neighbors = True

        # If agent has sensed that the target is in neighborhood, then it will update its belief state accordingly
        if self.is_target_in_neighbors:
            sensed_neighbor_probability = ZERO_PROBABILITY

            # Take probability sum and scale up probability accordingly
            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                sensed_neighbor_probability += \
                    self.probability_of_containing_target_next_step[neighbor[0]][neighbor[1]]

            # Added assert to check the range of probability
            assert ZERO_PROBABILITY < sensed_neighbor_probability <= ONE_PROBABILITY + 1e-5

            # Assign probability of containing target to zero and then scale up
            self.probability_of_containing_target = 0 * self.probability_of_containing_target_next_step
            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                self.probability_of_containing_target[neighbor[0]][neighbor[1]] = \
                    self.probability_of_containing_target_next_step[neighbor[0]][neighbor[1]] / \
                    sensed_neighbor_probability
            # Set probability to zero for next time stamps
            self.probability_of_containing_target_next_step = 0 * self.probability_of_containing_target

        else:
            # This is the case when agent could not be able to sense target in its neighbors.
            sensed_neighbor_probability = ZERO_PROBABILITY
            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                sensed_neighbor_probability += \
                    self.probability_of_containing_target_next_step[neighbor[0]][neighbor[1]]

            assert ZERO_PROBABILITY <= sensed_neighbor_probability < ONE_PROBABILITY

            self.probability_of_containing_target = self.probability_of_containing_target_next_step /\
                                                    (ONE_PROBABILITY - sensed_neighbor_probability)

            self.probability_of_containing_target_next_step = 0 * self.probability_of_containing_target_next_step

            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                self.probability_of_containing_target[neighbor[0]][neighbor[1]] = ZERO_PROBABILITY

        # Compute prediction for the next time stamp using current belief
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                valid_neighbors_cnt = 0
                for neighbor in self.maze[row][col].four_neighbors:
                    if not self.maze[neighbor[0]][neighbor[1]].is_blocked:
                        valid_neighbors_cnt += 1
                if valid_neighbors_cnt > 0:
                    for neighbor in self.maze[row][col].four_neighbors:
                        if not self.maze[neighbor[0]][neighbor[1]].is_blocked:
                            self.probability_of_containing_target_next_step[neighbor[0]][neighbor[1]] += \
                                self.probability_of_containing_target[row][col] / valid_neighbors_cnt

    def pre_planning(self, agent_num=9):
        """
        Method is used to find estimated goal
        :param agent_num: agent number
        :return: None
        """
        # Find estimated goal only when target is not sensed in neighbors
        if not self.is_target_in_neighbors:
            self.current_estimated_goal = compute_current_estimated_goal(self.maze, self.current_position, agent_num,
                                                                         self.probability_of_containing_target,
                                                                         self.false_negative_rates,
                                                                         self.probability_of_containing_target_next_step)

    def planning(self, goal_pos):
        """
        Method is used to find a path
        :param goal_pos: goal position where agent wants to reach
        :return: None
        """
        # Find path only when target is not sensed in neighbors
        if not self.is_target_in_neighbors:
            super().planning(goal_pos)

    def execution(self, full_maze: np.array, target_pos: tuple = None):
        """
        Method is used to move target if it didn't sense target in its neighbors otherwise it will examine current cell.
        :param full_maze: Full maze object
        :param target_pos: target position at current time stamp
        :return: True if target is found otherwise False
        """

        # Take current position of the agent and set some initial variables
        cur_pos = self.current_position
        current_path = [cur_pos]
        target_found = False
        update_status(self.maze, self.false_negative_rates, full_maze, cur_pos)

        # If target is sensed in last timestamp, then examine the current cell in the current timestamp
        if self.is_target_in_neighbors:
            self.num_examinations += 1
            target_found = examine_and_propagate_probability(self.maze, self.probability_of_containing_target_next_step,
                                                             self.false_negative_rates, cur_pos, target_pos, cur_pos,
                                                             cur_pos)
        else:
            # Move agent one step.
            children = parent_to_child_dict(self.parents, self.current_estimated_goal)

            # If next cell is blocked in agent's path, agent will change its belief state and won't move in this
            # timestamp
            if full_maze[children[cur_pos][0]][children[cur_pos][1]] == 1:
                self.maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
                examine_and_propagate_probability(self.maze, self.probability_of_containing_target_next_step,
                                                  self.false_negative_rates, cur_pos, target_pos,
                                                  self.current_estimated_goal, children[cur_pos])
            else:
                # Otherwise move one step and update status of next cell
                update_status(self.maze, self.false_negative_rates, full_maze, children[cur_pos])
                current_path.append(children[cur_pos])

        self.current_position = current_path[-1]
        self.final_paths.append(current_path)

        return target_found

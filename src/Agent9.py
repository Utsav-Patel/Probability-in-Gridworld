import numpy as np

from constants import STARTING_POSITION_OF_AGENT, NUM_COLS, NUM_ROWS, ZERO_PROBABILITY, ONE_PROBABILITY
from src.Agent import Agent
from helpers.helper import examine_and_propagate_probability, parent_to_child_dict, update_status


class Agent9(Agent):

    def __init__(self):
        super().__init__()
        self.is_target_in_neighbors = False

    def sense(self, target_position: tuple):
        self.is_target_in_neighbors = False
        for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
            if target_position == neighbor:
                self.is_target_in_neighbors = True

        if self.is_target_in_neighbors:
            print('Target found in neighbors')
            sensed_neighbor_probability = ZERO_PROBABILITY
            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                sensed_neighbor_probability += \
                    self.maze[neighbor[0]][neighbor[1]].probability_of_containing_target_next_step

            print('Sensed probability', sensed_neighbor_probability)
            assert ZERO_PROBABILITY < sensed_neighbor_probability <= ONE_PROBABILITY + 1e-5

            for row in range(NUM_ROWS):
                for col in range(NUM_COLS):
                    if (row, col) in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                        self.maze[row][col].probability_of_containing_target = \
                            self.maze[row][col].probability_of_containing_target_next_step / sensed_neighbor_probability
                    else:
                        self.maze[row][col].probability_of_containing_target = ZERO_PROBABILITY
                    self.maze[row][col].probability_of_containing_target_next_step = ZERO_PROBABILITY
        else:
            print('Target not found in neighbors')
            sensed_neighbor_probability = ZERO_PROBABILITY
            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                sensed_neighbor_probability += \
                    self.maze[neighbor[0]][neighbor[1]].probability_of_containing_target_next_step

            print('Sensed probability', sensed_neighbor_probability)
            assert ZERO_PROBABILITY <= sensed_neighbor_probability < ONE_PROBABILITY

            for row in range(NUM_ROWS):
                for col in range(NUM_COLS):
                    self.maze[row][col].probability_of_containing_target = \
                        self.maze[row][col].probability_of_containing_target_next_step / \
                        (ONE_PROBABILITY - sensed_neighbor_probability)
                    self.maze[row][col].probability_of_containing_target_next_step = ZERO_PROBABILITY

            for neighbor in self.maze[self.current_position[0]][self.current_position[1]].eight_neighbors:
                self.maze[neighbor[0]][neighbor[1]].probability_of_containing_target = ZERO_PROBABILITY

            # for row in range(NUM_ROWS):
            #     for col in range(NUM_COLS):
            #         self.maze[row][col].probability_of_containing_target = \
            #             self.maze[row][col].probability_of_containing_target_next_step
            #         self.maze[row][col].probability_of_containing_target_next_step = 0.0

        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                valid_neighbors_cnt = 0
                for neighbor in self.maze[row][col].four_neighbors:
                    if not self.maze[neighbor[0]][neighbor[1]].is_blocked:
                        valid_neighbors_cnt += 1
                if valid_neighbors_cnt > 0:
                    for neighbor in self.maze[row][col].four_neighbors:
                        if not self.maze[neighbor[0]][neighbor[1]].is_blocked:
                            self.maze[neighbor[0]][neighbor[1]].probability_of_containing_target_next_step += \
                                self.maze[row][col].probability_of_containing_target / \
                                valid_neighbors_cnt

    def pre_planning(self, agent_num=9):
        if not self.is_target_in_neighbors:
            super().pre_planning(agent_num)

    def planning(self, goal_pos):
        if not self.is_target_in_neighbors:
            super().planning(goal_pos)

    def execution(self, full_maze: np.array, target_pos: tuple = None):
        cur_pos = self.current_position
        current_path = [cur_pos]
        target_found = False
        update_status(self.maze, full_maze, cur_pos)

        if self.is_target_in_neighbors:
            self.num_examinations += 1
            # no_run = 10
            # while no_run > 0:
            target_found = target_found or examine_and_propagate_probability(self.maze, full_maze, cur_pos,
                                                                             target_pos, cur_pos, cur_pos)
            # no_run -= 1
        else:
            children = parent_to_child_dict(self.parents, self.current_estimated_goal)
            # print(children)
            # Setting current position to starting position so we can start iterating from start_pos

            if full_maze[children[cur_pos][0]][children[cur_pos][1]] == 1:
                self.maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
                examine_and_propagate_probability(self.maze, full_maze, cur_pos, target_pos,
                                                  self.current_estimated_goal, children[cur_pos])
            else:
                update_status(self.maze, full_maze, children[cur_pos])
                current_path.append(children[cur_pos])

        self.current_position = current_path[-1]
        self.final_paths.append(current_path)

        return target_found

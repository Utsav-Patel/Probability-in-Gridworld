import random
import numpy as np
import math

from constants import NUM_ROWS, NUM_COLS, INF, HILLY_FALSE_NEGATIVE_RATE, FLAT_FALSE_NEGATIVE_RATE, \
    FOREST_FALSE_NEGATIVE_RATE, ZERO_PROBABILITY, ONE_PROBABILITY
from helpers.helper import parent_to_child_dict, length_of_path_from_source_to_all_nodes, compare_fractions, \
    examine_and_propagate_probability


def set_random_target():
    x_pos = random.randint(0, NUM_ROWS - 1)
    y_pos = random.randint(0, NUM_COLS - 1)
    return x_pos, y_pos


def compute_current_estimated_goal(maze, current_pos, num_of_cells_processed, agent=6):
    if num_of_cells_processed < 1:
        return current_pos

    max_p = 0.0
    cells_with_max_p = list()
    cells_with_least_d = list()
    least_distance = INF
    distance_array = length_of_path_from_source_to_all_nodes(maze, current_pos)
    # print(distance_array)

    sum_probabilities = 0.0
    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            sum_probabilities += maze[row][col].probability_of_containing_target
    if agent == 6:
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                maze[row][col].probability_of_containing_target /= sum_probabilities
                if compare_fractions(maze[row][col].probability_of_containing_target, max_p) == 1:
                    max_p = maze[row][col].probability_of_containing_target
                    cells_with_max_p = list()
                    cells_with_max_p.append((row, col))
                elif compare_fractions(maze[row][col].probability_of_containing_target, max_p) == 0:
                    cells_with_max_p.append((row, col))

    elif agent == 7:
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                maze[row][col].probability_of_containing_target /= sum_probabilities
                x = (1 - maze[row][col].false_negative_rate) * maze[row][col].probability_of_containing_target
                if compare_fractions(x, max_p) == 1:
                    max_p = x
                    cells_with_max_p = list()
                    cells_with_max_p.append((row, col))
                elif compare_fractions(x, max_p) == 0:
                    cells_with_max_p.append((row, col))

    elif agent == 8:
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                if (row, col) == current_pos:
                    continue
                maze[row][col].probability_of_containing_target /= sum_probabilities
                x = (1 - maze[row][col].false_negative_rate) * maze[row][col].probability_of_containing_target / \
                    distance_array[row][col]
                if compare_fractions(x, max_p) == 1:
                    max_p = x
                    cells_with_max_p = list()
                    cells_with_max_p.append((row, col))
                elif compare_fractions(x, max_p) == 0:
                    cells_with_max_p.append((row, col))

    # print("cells with max p =", cells_with_max_p)
    for item in cells_with_max_p:
        if distance_array[item[0]][item[1]] < least_distance:
            least_distance = distance_array[item[0]][item[1]]
            cells_with_least_d = list()
            cells_with_least_d.append((item[0], item[1]))
        elif distance_array[item[0]][item[1]] == least_distance:
            cells_with_least_d.append((item[0], item[1]))

    if len(cells_with_least_d) > 1:
        random_index = random.randint(0, len(cells_with_least_d) - 1)
        # print(cells_with_least_d[random_index])
        return cells_with_least_d[random_index]
    else:
        # print(cells_with_least_d[0])
        return cells_with_least_d[0]


#
# def examine_and_propagate_probability(maze, full_maze, current_pos, target_pos, current_estimated_goal, node):
#     # check if current position of Agent is current_estimated_goal
#     if current_pos == current_estimated_goal:
#         # check is current_estimates_goal is the target itself or not
#         if current_pos == target_pos:
#             if full_maze[current_pos[0]][current_pos[1]] == 2:  # for flat terrain
#                 x = random.randint(0, 99)  # generate random number between 0 and 99
#                 if x < 20:  # compute probability if random number < 20
#                     compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
#                 else:
#                     # if random number >20 and current_estimated_goal is target then return True as target is found
#                     return True
#             elif full_maze[current_pos[0]][current_pos[1]] == 3:  # for hilly terrain
#                 x = random.randint(0, 99)  # generate random number between 0 and 99
#                 if x < 50:  # compute probability if random number < 50
#                     compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
#                 else:
#                     # if random number >50 and current_estimated_goal is target then return True as target is found
#                     return True
#             elif full_maze[current_pos[0]][current_pos[1]] == 4:  # for forest terrain
#                 x = random.randint(0, 99)  # generate random number between 0 and 99
#                 if x < 80:  # compute probability if random number < 80
#                     compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
#                 else:
#                     # if random number >80 and current_estimated_goal is target then return True as target is found
#                     return True
#
#         # when current_estimated_goal doesn't contains target
#         else:
#             # propagate the probability
#             compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
#             # return false as target is not found
#             return False
#
#     # else block executed when node is blocked.
#     elif maze[node[0]][node[1]].is_blocked:
#         p_of_x_y = maze[node[0]][node[1]].probability_of_containing_target
#         remaining_probability = ONE_PROBABILITY - p_of_x_y
#         print("In probability calculation", node, p_of_x_y, end=" ")
#         for row in range(NUM_ROWS):
#             for column in range(NUM_COLS):
#                 if node == (row, column):
#                     maze[row][column].probability_of_containing_target = ZERO_PROBABILITY
#                 else:
#                     maze[row][column].probability_of_containing_target = \
#                         maze[row][column].probability_of_containing_target / remaining_probability
#
#     else:
#         compute_probability(maze[node[0]][node[1]].false_negative_rate, maze, node)
#     return False


def compute_probability(false_negative_rate, maze, current_pos):
    # propagate the probability accordingly to all cells of the maze
    p_of_x_y = maze[current_pos[0]][current_pos[1]].probability_of_containing_target

    reduced_probability = p_of_x_y * false_negative_rate
    probability_denominator = ONE_PROBABILITY - p_of_x_y + reduced_probability

    for row in range(NUM_ROWS):
        for column in range(NUM_COLS):
            if current_pos == (row, column):
                maze[row][column].probability_of_containing_target = reduced_probability / probability_denominator
            else:
                maze[row][column].probability_of_containing_target = maze[row][column].probability_of_containing_target \
                                                                     / probability_denominator


def forward_execution(maze: list, maze_array: np.array, start_pos: tuple, goal_pos: tuple, parents: dict,
                      cells_to_examine, target_position, global_threshold):
    """
    This is the repeated forward function which can be used with any algorithm (astar or bfs). This function will
    repeatedly call corresponding algorithm function until it reaches goal or finds out there is no path till goal.
    :param global_threshold: list that contains global threshold of examinations for flat, hilly and forest in respected
                            order
    :param goal_pos: The current_estimates_goal position
    :param target_position: The actual target location in the maze
    :param maze: Maze array of agent
    :param maze_array: Original (Full) Maze array
    :param start_pos: starting position of the maze from where agent want to start
    :param parents: parent of each node in the path
    :param cells_to_examine: set that contains the cells that should be examined while traversing on the path
    :return: This function will return final paths on which agent moved to reach goal or empty list if agent can't find
            path to goal. Second is total number of processed nodes while running the algorithm.
    """

    num_backtracks = 0
    children = parent_to_child_dict(parents, goal_pos)
    # print(children)

    # Setting current position to starting position so we can start iterating from start_pos
    cur_pos = start_pos

    current_path = [cur_pos]

    truth_value = False  # indicates whether target has been found while examining a target
    examinations = 0
    # Iterating from start_pos to goal_pos if we won't get any blocks in between otherwise we are terminating the
    # iteration.
    while True:

        if maze_array[cur_pos[0]][cur_pos[1]] == 1:  # if the cell is blocked
            maze[cur_pos[0]][cur_pos[1]].is_blocked = True
            maze[cur_pos[0]][cur_pos[1]].max_threshold_of_examinations = 0
        elif maze_array[cur_pos[0]][cur_pos[1]] == 2:  # if cell has flat terrain type
            maze[cur_pos[0]][cur_pos[1]].false_negative_rate = FLAT_FALSE_NEGATIVE_RATE
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False

            if maze[cur_pos[0]][cur_pos[1]].previous_visits == 0:  # cell has not been visited previously

                # set the cell's max_threshold of examinations from list global_threshold
                maze[cur_pos[0]][cur_pos[1]].max_threshold_of_examinations = global_threshold[0]
        elif maze_array[cur_pos[0]][cur_pos[1]] == 3:  # if cell has hilly terrain type
            maze[cur_pos[0]][cur_pos[1]].false_negative_rate = HILLY_FALSE_NEGATIVE_RATE
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False

            if maze[cur_pos[0]][cur_pos[1]].previous_visits == 0:  # cell has not been visited previously
                # set the cell's max_threshold of examinations from list global_threshold
                maze[cur_pos[0]][cur_pos[1]].max_threshold_of_examinations = global_threshold[1]
        elif maze_array[cur_pos[0]][cur_pos[1]] == 4:  # if cell has forest terrain type
            maze[cur_pos[0]][cur_pos[1]].false_negative_rate = FOREST_FALSE_NEGATIVE_RATE
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False

            if maze[cur_pos[0]][cur_pos[1]].previous_visits == 0:  # cell has not been visited previously
                # set the cell's max_threshold of examinations from list global_threshold
                maze[cur_pos[0]][cur_pos[1]].max_threshold_of_examinations = global_threshold[2]
        else:
            raise Exception("Invalid value in maze_array")

        # examination of curr_pos if it is in cells_to_examine set
        if cur_pos in cells_to_examine:
            current_visit_number = maze[cur_pos[0]][cur_pos[1]].previous_visits + 1
            previous_examinations = maze[cur_pos[0]][cur_pos[1]].previous_examinations
            no_times_to_examine = (maze[cur_pos[0]][cur_pos[1]].max_threshold_of_examinations / global_threshold[0]) * \
                                  current_visit_number - previous_examinations

            no_times_to_examine = min(maze[cur_pos[0]][cur_pos[1]].max_threshold_of_examinations - previous_examinations
                                      , no_times_to_examine)

            maze[cur_pos[0]][cur_pos[1]].previous_examinations += no_times_to_examine

            while no_times_to_examine > 0:
                # truth_value = examine(maze_array, target_position, parents, goal_pos, maze, cur_pos)
                truth_value = examine_and_propagate_probability(maze, cur_pos, target_position, goal_pos,
                                                                cur_pos)
                # assert not truth_value
                examinations += 1
                if truth_value:
                    break
                no_times_to_examine -= 1

            maze[cur_pos[0]][cur_pos[1]].previous_visits += 1

        if truth_value:
            # cur_pos = children[cur_pos]  # update curr_pos to the next cell in path i.e child of curr_pos
            # current_path.append(cur_pos)  # append curr_pos as we traversed on it successfully
            return current_path, num_backtracks, truth_value, examinations

        if cur_pos == children[cur_pos]:  # last cell reached
            return current_path, num_backtracks, truth_value, examinations

        # If we encounter any block in the path, we have to terminate the iteration
        if maze_array[children[cur_pos][0]][children[cur_pos][1]] == 1:
            maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
            # truth_value = examine(maze_array, target_position, parents, goal_pos, maze, cur_pos)
            truth_value = examine_and_propagate_probability(maze, cur_pos, target_position, goal_pos,
                                                            children[cur_pos])
            assert not truth_value
            return current_path, num_backtracks, truth_value, examinations

        cur_pos = children[cur_pos]  # update curr_pos to the next cell in path i.e child of curr_pos
        current_path.append(cur_pos)  # append curr_pos as we traversed on it successfully


# def examine(full_maze: np.array, target_pos, parents, current_estimated_goal, maze, current_position):
#     children = parent_to_child_dict(parents, current_estimated_goal)
#     value = examine_and_propagate_probability(maze, full_maze, current_position, target_pos,
#                                               current_estimated_goal, children[current_position])
#     return value

def calculate_global_threshold(accuracy):
    """
    Calculates the global threshold for examination of the each cell of different terrain type
    :param accuracy: Accuracy to be achieved in the final result
    :return: list of the global threshold contains threshold for flat, hilly and forest type.
    """
    global_threshold = list()
    flat_threshold = math.ceil(math.log(accuracy) / math.log(FLAT_FALSE_NEGATIVE_RATE))
    global_threshold.append(flat_threshold)
    hilly_threshold = math.ceil(math.log(accuracy) / math.log(HILLY_FALSE_NEGATIVE_RATE))
    global_threshold.append(hilly_threshold)
    forest_threshold = math.ceil(math.log(accuracy) / math.log(FOREST_FALSE_NEGATIVE_RATE))
    global_threshold.append(forest_threshold)

    return global_threshold

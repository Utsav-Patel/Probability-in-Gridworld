import random
import numpy as np

from queue import Queue

from constants import NUM_ROWS, NUM_COLS, X, Y, INF, HILLY_FALSE_NEGATIVE_RATE, FLAT_FALSE_NEGATIVE_RATE,\
    FOREST_FALSE_NEGATIVE_RATE
from helpers.helper import check, parent_to_child_dict, compare_fractions, divide_fractions, subtract_fractions,\
    multiply_fractions, add_fractions


def generate_grid_manually():
    """
    This is the function to generate grid manually. This is helpful for the initial testing and problem 1.
    :return: Manually generated numpy array.
    """
    array = np.zeros((NUM_ROWS, NUM_COLS))

    array[0][0] = 3
    array[0][1] = 1
    array[0][2] = 1
    array[1][0] = 2
    array[1][1] = 2
    array[1][2] = 3
    array[2][0] = 3
    array[2][1] = 4
    array[2][2] = 4
    return array


def set_random_target():
    x_pos = random.randint(0, NUM_ROWS - 1)
    y_pos = random.randint(0, NUM_COLS - 1)
    return x_pos, y_pos


def forward_execution(maze: list, maze_array: np.array, start_pos: tuple, goal_pos: tuple, parents: dict):
    """
    This is the repeated forward function which can be used with any algorithm (astar or bfs). This function will
    repeatedly call corresponding algorithm function until it reaches goal or finds out there is no path till goal.
    :param maze: Maze array of agent
    :param maze_array: Original (Full) Maze array
    :param start_pos: starting position of the maze from where agent want to start
    :param parents: parent of each node in the path
    :param want_to_explore_field_of_view: It will explore field of view if this attribute is true otherwise it won't
    :param is_backtrack_strategy_on: If you want to run strategy 2, this attribute should be set to true
    :return: This function will return final paths on which agent moved to reach goal or empty list if agent can't find
            path to goal. Second is total number of processed nodes while running the algorithm.
    """

    num_backtracks = 0
    # GOAL_POSITION_OF_AGENT = goal_pos
    children = parent_to_child_dict(parents, goal_pos)
    # print(children)
    # Setting current position to starting position so we can start iterating from start_pos
    cur_pos = start_pos

    current_path = [cur_pos]
    # if is_backtrack_strategy_on:
    #     last_cell_which_is_not_in_dead_end = cur_pos

    # Iterating from start_pos to goal_pos if we won't get any blocks in between otherwise we are terminating the
    # iteration.
    while True:

        # if is_backtrack_strategy_on:
        #     path_exist_from_the_last_point = 0

        # maze[cur_pos[0]][cur_pos[1]].is_confirmed = True

        if maze_array[cur_pos[0]][cur_pos[1]] == 1:
            maze[cur_pos[0]][cur_pos[1]].is_blocked = True
        elif maze_array[cur_pos[0]][cur_pos[1]] == 2:
            maze[cur_pos[0]][cur_pos[1]].false_negative_rate = FLAT_FALSE_NEGATIVE_RATE
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False
        elif maze_array[cur_pos[0]][cur_pos[1]] == 3:
            maze[cur_pos[0]][cur_pos[1]].false_negative_rate = HILLY_FALSE_NEGATIVE_RATE
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False
        elif maze_array[cur_pos[0]][cur_pos[1]] == 4:
            maze[cur_pos[0]][cur_pos[1]].false_negative_rate = FOREST_FALSE_NEGATIVE_RATE
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False
        else:
            raise Exception("Invalid value in maze_array")

        # Explore the field of view and update the blocked nodes if there's any in the path.
        # if want_to_explore_field_of_view:
        #     for ind in range(len(X)):
        #
        #         neighbour = (cur_pos[0] + X[ind], cur_pos[1] + Y[ind])
        #         if check(neighbour):
        #             # maze[neighbour[0]][neighbour[1]].is_confirmed = True
        #             if maze_array[neighbour[0]][neighbour[1]] == 1:
        #                 maze[neighbour[0]][neighbour[1]].is_blocked = True
        #
        #         # Here, we are finding whether the current node is a part of the dead end or not. If there is a path
        #         # exists other than its child and parent, then this node should not be part of dead end because
        #         # there is another path available which you can explore.
        #         if is_backtrack_strategy_on and (check(neighbour)) and (children[cur_pos] != neighbour) \
        #                 and (parents[cur_pos] != neighbour) and (maze_array[neighbour[0]][neighbour[1]] == 0):
        #             path_exist_from_the_last_point += 1
        #
        # if is_backtrack_strategy_on:
        #
        #     # If we can find such a node which we can explore later using current node, then this node should not be
        #     # part of the dead end path.
        #     if path_exist_from_the_last_point > 0:
        #         last_cell_which_is_not_in_dead_end = cur_pos

        if cur_pos == children[cur_pos]:
            break
        # If we encounter any block in the path, we have to terminate the iteration
        if maze_array[children[cur_pos][0]][children[cur_pos][1]] == 1:
            break
        cur_pos = children[cur_pos]
        current_path.append(cur_pos)

    if cur_pos != goal_pos:

        # Change the start node to last unblocked node and backtrack if it is set to any positive integer.
        maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
        # maze[children[cur_pos][0]][children[cur_pos][1]].is_confirmed = True

        # if is_backtrack_strategy_on:
        #
        #     # We keep backtracking cell until we reached a cell from where we can explore a new path. Also, we are
        #     # manually blocking those dead end nodes because they are not useful anymore.
        #     while cur_pos != last_cell_which_is_not_in_dead_end:
        #         num_backtracks += 1
        #         maze[cur_pos[0]][cur_pos[1]].is_blocked = True
        #         cur_pos = parents[cur_pos]
        #         current_path.append(cur_pos)

    return current_path, num_backtracks


def length_of_path_from_source_to_all_nodes(maze: list, start_pos: tuple):
    """
    This function will return length of path from source to goal if it exists otherwise it will return INF
    :param maze_array: binary Maze Array
    :param start_pos: Starting position of the maze from where you want to start
    :param goal_pos: Goal position of the maze where you want to reach
    :return: Shortest distance from the source to goal on the given maze array
    """

    # Initialize queue to compute distance
    q = Queue()

    # Initialize distance array
    distance_array = np.full((NUM_ROWS, NUM_COLS), INF)

    # Adding starting position to the queue and assigning its distance to zero
    q.put(start_pos)
    distance_array[start_pos[0]][start_pos[1]] = 0

    # Keep popping value from the queue until it gets empty
    while not q.empty():
        current_node = q.get()

        # Iterating over valid neighbours of current node
        for neighbour in maze[current_node[0]][current_node[1]].four_neighbors:
            # neighbour = (current_node[0] + X[ind], current_node[1] + Y[ind])
            if check(neighbour) and \
                    (distance_array[neighbour[0]][neighbour[1]] > distance_array[current_node[0]][current_node[1]] + 1) \
                    and (maze[neighbour[0]][neighbour[1]].is_blocked != 1):
                q.put(neighbour)
                distance_array[neighbour[0]][neighbour[1]] = distance_array[current_node[0]][current_node[1]] + 1

    return distance_array

def length_of_path_from_source_to_goal(maze_array: np.array, start_pos: tuple, goal_pos: tuple):
    """
    This function will return length of path from source to goal if it exists otherwise it will return INF
    :param maze_array: binary Maze Array
    :param start_pos: Starting position of the maze from where you want to start
    :param goal_pos: Goal position of the maze where you want to reach
    :return: Shortest distance from the source to goal on the given maze array
    """

    # Initialize queue to compute distance
    q = Queue()

    # Initialize distance array
    distance_array = np.full((NUM_ROWS, NUM_COLS), INF)

    # Adding starting position to the queue and assigning its distance to zero
    q.put(start_pos)
    distance_array[start_pos[0]][start_pos[1]] = 0

    # Keep popping value from the queue until it gets empty
    while not q.empty():
        current_node = q.get()

        # If goal position is found, we should return its distance
        if current_node == goal_pos:
            return distance_array[goal_pos[0]][goal_pos[1]]

        # Iterating over valid neighbours of current node
        for ind in range(len(X)):
            neighbour = (current_node[0] + X[ind], current_node[1] + Y[ind])
            if check(neighbour) and \
                    (distance_array[neighbour[0]][neighbour[1]] > distance_array[current_node[0]][current_node[1]] + 1) \
                    and (maze_array[neighbour[0]][neighbour[1]] != 1):
                q.put(neighbour)
                distance_array[neighbour[0]][neighbour[1]] = distance_array[current_node[0]][current_node[1]] + 1

    return distance_array[goal_pos[0]][goal_pos[1]]


def compute_current_estimated_goal(maze, current_pos, num_of_cells_processed):
    if num_of_cells_processed < 1:
        return current_pos

    max_p = [0, 1]
    cells_with_max_p = list()
    cells_with_least_d = list()
    least_distance = INF
    distance_array = length_of_path_from_source_to_all_nodes(maze, current_pos)
    # print(distance_array)
    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            if compare_fractions(maze[row][col].probability_of_containing_target, max_p) == 1:
                max_p = maze[row][col].probability_of_containing_target
                cells_with_max_p = list()
                cells_with_max_p.append((row, col))
            elif compare_fractions(maze[row][col].probability_of_containing_target, max_p) == 0:
                cells_with_max_p.append((row, col))

    # print("cells with max p =", cells_with_max_p)
    for item in cells_with_max_p:
        # if (distance_array[item[0]][item[1]] < least_distance) and (distance_array[item[0]][item[1]] != 0):
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


def examine_and_propogate_probability(maze, full_maze, current_pos, target_pos, current_estimated_goal, node):
    if current_pos == current_estimated_goal:
        if current_pos == target_pos:
            if full_maze[current_pos[0]][current_pos[1]] == 2:
                x = random.randint(0, 99)
                if x < 20:
                    compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
                else:
                    return True
            elif full_maze[current_pos[0]][current_pos[1]] == 3:
                x = random.randint(0, 99)
                if x < 50:
                    compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
                else:
                    return True
            elif full_maze[current_pos[0]][current_pos[1]] == 4:
                x = random.randint(0, 99)
                if x < 80:
                    compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)
                else:
                    return True
        else:
            compute_probability(maze[current_pos[0]][current_pos[1]].false_negative_rate, maze, current_pos)

            return False

    else:
        #children = parent_to_child_dict(parents, current_estimated_goal)
        #p_of_x_y = maze[children[current_pos][0]][children[current_pos][1]].probability_of_containing_target
        p_of_x_y = maze[node[0]][node[1]].probability_of_containing_target
        for row in range(NUM_ROWS):
            for column in range(NUM_COLS):
                #if (children[current_pos] == (row, column)):
                if node == (row, column):
                    maze[row][column].probability_of_containing_target = [0, 1]
                else:
                    # maze[row][column].probability_of_containing_target[0] = maze[row][column].probability_of_containing_target[0]/maze[row][column].probability_of_containing_target[1]
                    # maze[row][column].probability_of_containing_target[1] = (1-(p_of_x_y[0]/p_of_x_y[1]))
                    maze[row][column].probability_of_containing_target = divide_fractions(
                        maze[row][column].probability_of_containing_target, subtract_fractions([1, 1], p_of_x_y))
    return False


def compute_probability(false_negative_rate, maze, current_pos):
    p_of_x_y = maze[current_pos[0]][current_pos[1]].probability_of_containing_target
    for row in range(NUM_ROWS):
        for column in range(NUM_COLS):
            if (current_pos == (row, column)):
                # maze[row][column].probability_of_containing_target[0] = (p_of_x_y[0]/p_of_x_y[1])*false_negative_rate
                # maze[row][column].probability_of_containing_target[1] = (1 - (p_of_x_y[0]/p_of_x_y[1]))+((p_of_x_y[0]/p_of_x_y[1])*false_negative_rate)
                maze[row][column].probability_of_containing_target = divide_fractions(
                    multiply_fractions(p_of_x_y, false_negative_rate),
                    add_fractions(subtract_fractions([1, 1], p_of_x_y),
                                  multiply_fractions(p_of_x_y, false_negative_rate)))
            else:
                # maze[row][column].probability_of_containing_target[0] = maze[row][column].probability_of_containing_target[0]/maze[row][column].probability_of_containing_target[1]
                # maze[row][column].probability_of_containing_target[1] = ((1 - (p_of_x_y[0]/p_of_x_y[1]))+((p_of_x_y[0]/p_of_x_y[1])*false_negative_rate))
                maze[row][column].probability_of_containing_target = divide_fractions(
                    maze[row][column].probability_of_containing_target,
                    add_fractions(subtract_fractions([1, 1], p_of_x_y),
                                  multiply_fractions(p_of_x_y, false_negative_rate)))

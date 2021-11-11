import math
import random
import numpy as np
import matplotlib.pyplot as plt

from sortedcontainers import SortedSet
from queue import Queue

from constants import NUM_COLS, NUM_ROWS, X, Y, INF, IMG_PATH, ONE_PROBABILITY, ZERO_PROBABILITY,\
    FLAT_FALSE_NEGATIVE_RATE, HILLY_FALSE_NEGATIVE_RATE, FOREST_FALSE_NEGATIVE_RATE


def check(current_position: tuple):
    """
    Check whether current point is in the grid or not
    :param current_position: current point
    :return: True if the current point is in the grid otherwise False
    """
    if (0 <= current_position[0] < NUM_ROWS) and (0 <= current_position[1] < NUM_COLS):
        return True
    return False


def avg(lst: list):
    """
    This function computes average of the given list. If the length of list is zero, it will return zero.
    :param lst: list for which you want to compute average
    :return: average of the given list
    """
    if len(lst) == 0:
        return 0
    return sum(lst) / len(lst)


def generate_grid_with_probability_p(p):
    """
    This function will generate the uniform random grid of size NUM_ROWS X NUM_COLS.
    :param p: probability of cell being blocked
    :return: Grid of size NUM_ROWS X NUM_COLS with each cell having uniform probability of being blocked is p.
    """
    from constants import NUM_COLS, NUM_ROWS, STARTING_POSITION_OF_AGENT

    while True:
        randomly_generated_array = np.random.uniform(low=0.0, high=1.0, size=NUM_ROWS * NUM_COLS).reshape(NUM_ROWS,
                                                                                                          NUM_COLS)
        # randomly_generated_array[GOAL_POSITION_OF_AGENT[0]][GOAL_POSITION_OF_AGENT[1]] = 0
        randomly_generated_array[randomly_generated_array >= (1 - p)] = 1
        randomly_generated_array[randomly_generated_array < (1 - p) / 3] = 2
        randomly_generated_array[
            ((randomly_generated_array >= (1 - p) / 3) & (randomly_generated_array < (1 - p) * 2 / 3))] = 3
        randomly_generated_array[
            ((randomly_generated_array >= (1 - p) * 2 / 3) & (randomly_generated_array < (1 - p)))] = 4

        if randomly_generated_array[STARTING_POSITION_OF_AGENT[0]][STARTING_POSITION_OF_AGENT[1]] == 1:
            continue

        return randomly_generated_array


def manhattan_distance(pos1: tuple, pos2: tuple):
    """
    Compute Manhattan distance between two points
    :param pos1: Coordinate of first point
    :param pos2: Coordinate of second point
    :return: Manhattan distance between two points
    """
    distance = 0
    for ind in range(len(pos1)):
        distance += abs(pos1[ind] - pos2[ind])
    return distance


def compare_fractions(num_1, num_2):
    # a = num_1[0]
    # b = num_1[1]
    # c = num_2[0]
    # d = num_2[1]

    # val = (a * d - b * c) / (b * d)

    if num_1 - num_2 > 0:
        return 1
    elif num_1 - num_2 < 0:
        return 2
    else:
        return 0


def compute_explored_cells_from_path(paths: list):
    """
    This function will compute the trajectory length from the list of paths returned by any repeated forward algorithm
    :param paths: list of paths
    :return: trajectory length
    """

    trajectory_length = 0
    for path in paths:
        trajectory_length += len(path)
    trajectory_length -= len(paths)
    return trajectory_length


def parent_to_child_dict(parent: dict, starting_position: tuple):
    """
    This function is helpful to generate children dictionary from parents dictionary
    :param parent: parent dictionary
    :param starting_position: starting position of the last function
    :return: generate child dictionary from parent.
    """
    child = dict()

    child[starting_position] = starting_position
    cur_pos = starting_position
    # print(parent)
    # print(parent[cur_pos])
    # Storing child of each node so we can iterate from start_pos to goal_pos
    while cur_pos != parent[cur_pos]:
        child[parent[cur_pos]] = cur_pos
        cur_pos = parent[cur_pos]

    return child


def generate_target_position(full_maze: list):
    while True:
        x = random.randint(0, len(full_maze) - 1)
        y = random.randint(0, len(full_maze) - 1)
        if full_maze[x][y] == 1:
            continue
        return x, y


def length_of_path_from_source_to_all_nodes(maze: list, start_pos: tuple):
    """
    This function will return length of path from source to goal if it exists otherwise it will return INF
    :param maze: Maze object
    :param start_pos: Starting position of the maze from where you want to start
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
    sum_probabilities_next_step = 0.0

    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            sum_probabilities += maze[row][col].probability_of_containing_target
            sum_probabilities_next_step += maze[row][col].probability_of_containing_target_next_step

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

    elif agent == 9:
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                if (row, col) == current_pos:
                    continue
                maze[row][col].probability_of_containing_target_next_step /= sum_probabilities_next_step
                # (1 - maze[row][col].false_negative_rate) *
                x = maze[row][col].probability_of_containing_target_next_step / distance_array[row][col]
                if compare_fractions(x, max_p) == 1:
                    max_p = x
                    cells_with_max_p = list()
                    cells_with_max_p.append((row, col))
                elif compare_fractions(x, max_p) == 0:
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


def astar_search(maze: list, start_pos: tuple, goal_pos: tuple):
    """
    Function to compute A* search
    :param maze: maze is a list of list
    :param start_pos: starting position of the maze from where we want to start A* search
    :param goal_pos: goal position of the maze to where agent wants to reach
    :return: Returning the path from goal_pos to start_pos if it exists
    """

    # Initialize a set for visited nodes
    visited_nodes = set()

    # Initialize a sorted set to pop least value element from the set
    sorted_set = SortedSet()

    # Initialize a dictionary to store a random value assigned to each node. This dictionary would be helpful to know
    # the value of a node when we want to remove a particular node from the sorted set
    node_to_random_number_mapping = dict()

    # Initialize another dictionary to store parent information
    parents = dict()

    # Initialize g and f for the starting position
    maze[start_pos[0]][start_pos[1]].g = 0
    maze[start_pos[0]][start_pos[1]].h = manhattan_distance(start_pos, goal_pos)
    maze[start_pos[0]][start_pos[1]].f = maze[start_pos[0]][start_pos[1]].h

    # Assigning a random number to start position to the starting position and adding to visited nodes
    node_to_random_number_mapping[start_pos] = random.uniform(0, 1)
    visited_nodes.add(start_pos)

    # Add start position node into the sorted set. We are giving priority to f(n), h(n), and g(n) in the decreasing
    # order. Push random number for random selection if there is conflict between two nodes
    # (If f(n), g(n), and h(n) are same for two nodes)
    sorted_set.add(((maze[start_pos[0]][start_pos[1]].f, maze[start_pos[0]][start_pos[1]].h,
                     maze[start_pos[0]][start_pos[1]].g, node_to_random_number_mapping[start_pos]), start_pos))

    parents[start_pos] = start_pos

    num_explored_nodes = 0

    # Running the loop until we reach our goal state or the sorted set is empty
    while sorted_set.__len__() != 0:
        # Popping first (shortest) element from the sorted set
        current_node = sorted_set.pop(index=0)

        # Increase the number of explored nodes
        num_explored_nodes += 1

        # If we have found the goal position, we can return parents and total explored nodes
        if current_node[1] == goal_pos:
            return parents, num_explored_nodes

        # Otherwise, we need to iterate through each child of the current node
        for val in range(len(X)):
            neighbour = (current_node[1][0] + X[val], current_node[1][1] + Y[val])

            # Neighbour should not go outside our maze and it should not be blocked if we want to visit that particular
            # neighbour
            if check(neighbour) and (not maze[neighbour[0]][neighbour[1]].is_blocked):

                # If neighbour is being visited first time, we should change its g(n) and f(n) accordingly. Also, we
                # need to assign a random value to it for the time of conflict. In the end, we will add all those things
                # into the sorted set and update its parent
                if neighbour not in visited_nodes:
                    maze[neighbour[0]][neighbour[1]].g = maze[current_node[1][0]][current_node[1][1]].g + 1
                    maze[neighbour[0]][neighbour[1]].h = manhattan_distance(neighbour, goal_pos)
                    maze[neighbour[0]][neighbour[1]].f = maze[neighbour[0]][neighbour[1]].g + \
                                                         maze[neighbour[0]][neighbour[1]].h
                    node_to_random_number_mapping[neighbour] = random.uniform(0, 1)
                    visited_nodes.add(neighbour)
                    sorted_set.add(((maze[neighbour[0]][neighbour[1]].f,
                                     maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                                     node_to_random_number_mapping[neighbour]), neighbour))
                    parents[neighbour] = current_node[1]

                # If a particular neighbour is already visited, we should compare its f(n) value to its previous f(n)
                # value. If current computed f(n) value is less than the previously computed value, we should remove
                # previously computed value and add new value to the sorted set
                else:
                    neighbour_g = maze[current_node[1][0]][current_node[1][1]].g + 1
                    neighbour_f = maze[neighbour[0]][neighbour[1]].h + neighbour_g
                    if neighbour_f < maze[neighbour[0]][neighbour[1]].f:

                        # The following if condition is needed only when the heuristic is inadmissible otherwise a
                        # neighbour has to be in the sorted set if we are able to find out less value of f(n) for that
                        # particular neighbour
                        if ((maze[neighbour[0]][neighbour[1]].f,
                             maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                             node_to_random_number_mapping[neighbour]), neighbour) \
                                in sorted_set:
                            sorted_set.remove(
                                ((maze[neighbour[0]][neighbour[1]].f,
                                  maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                                  node_to_random_number_mapping[neighbour]), neighbour))
                        maze[neighbour[0]][neighbour[1]].g = neighbour_g
                        maze[neighbour[0]][neighbour[1]].f = neighbour_f
                        node_to_random_number_mapping[neighbour] = random.uniform(0, 1)
                        sorted_set.add(
                            ((maze[neighbour[0]][neighbour[1]].f,
                              maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                              node_to_random_number_mapping[neighbour]), neighbour))
                        parents[neighbour] = current_node[1]

    return parents, num_explored_nodes


def compute_probability(maze, current_pos):
    p_of_x_y = maze[current_pos[0]][current_pos[1]].probability_of_containing_target

    reduced_probability = p_of_x_y * maze[current_pos[0]][current_pos[1]].false_negative_rate
    probability_denominator = ONE_PROBABILITY - p_of_x_y + reduced_probability

    for row in range(NUM_ROWS):
        for column in range(NUM_COLS):
            if current_pos == (row, column):
                maze[row][column].probability_of_containing_target = reduced_probability / probability_denominator
            else:
                maze[row][column].probability_of_containing_target = maze[row][column].probability_of_containing_target\
                                                                     / probability_denominator


def check_and_propagate_probability(maze, current_pos, target_pos):
    if current_pos == target_pos:
        if maze[current_pos[0]][current_pos[1]].false_negative_rate == 0.2:
            x = random.randint(0, 99)
            if x < 20:
                compute_probability(maze, current_pos)
            else:
                return True
        elif maze[current_pos[0]][current_pos[1]].false_negative_rate == 0.5:
            x = random.randint(0, 99)
            if x < 50:
                compute_probability(maze, current_pos)
            else:
                return True
        elif maze[current_pos[0]][current_pos[1]].false_negative_rate == 0.8:
            x = random.randint(0, 99)
            if x < 80:
                compute_probability(maze, current_pos)
            else:
                return True
    else:
        compute_probability(maze, current_pos)

    return False


def examine_and_propagate_probability(maze, current_pos, target_pos, current_estimated_goal, node):
    if current_pos == current_estimated_goal:
        return check_and_propagate_probability(maze, current_pos, target_pos)

    elif maze[node[0]][node[1]].is_blocked:
        p_of_x_y = maze[node[0]][node[1]].probability_of_containing_target
        remaining_probability = ONE_PROBABILITY - p_of_x_y

        for row in range(NUM_ROWS):
            for column in range(NUM_COLS):
                if node == (row, column):
                    maze[row][column].probability_of_containing_target = ZERO_PROBABILITY
                else:
                    maze[row][column].probability_of_containing_target = \
                        maze[row][column].probability_of_containing_target / remaining_probability
        return False
    else:
        return check_and_propagate_probability(maze, node, target_pos)


def update_status(maze: list, maze_array: np.array, cur_pos: tuple):
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


def plot_boxplot(data: list, title: str, legend: list, filename: str):
    fig, ax = plt.subplots()
    bp = ax.boxplot(data, patch_artist=True, notch='True', vert=0)
    ax.set_yticklabels(legend)
    plt.title(title)
    plt.savefig(IMG_PATH + filename)
    plt.show()

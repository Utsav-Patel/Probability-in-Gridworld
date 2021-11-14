import random
import numpy as np

from sortedcontainers import SortedSet
from queue import Queue

from constants import NUM_COLS, NUM_ROWS, X, Y, INF, ONE_PROBABILITY, ZERO_PROBABILITY, STARTING_POSITION_OF_AGENT, \
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

    while True:
        randomly_generated_array = np.random.uniform(low=0.0, high=1.0, size=NUM_ROWS * NUM_COLS).reshape(NUM_ROWS,
                                                                                                          NUM_COLS)
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


def compare_fractions(num_1: float, num_2: float):
    """
    Compare function for two float variable
    :param num_1: first variable
    :param num_2: second variable
    :return: 1 if num1 > num2, 2 if num1 < num2, 0 if num1 == num2
    """
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
    """
    Generate target position
    :param full_maze: Full/Origin maze
    :return: x, y coordinate of generated target position
    """
    while True:
        x = random.randint(0, NUM_ROWS - 1)
        y = random.randint(0, NUM_COLS - 1)
        if full_maze[x][y] != 4:
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


def compute_current_estimated_goal(maze: list, current_pos: tuple, agent: int,
                                   probability_of_containing_target: np.ndarray,
                                   false_negative_rates: np.ndarray, probability_of_containing_target_next_step=None):
    """
    Function is used to find a current estimated goal
    :param maze: Maze object
    :param current_pos: current position
    :param agent: agent number
    :param probability_of_containing_target: numpy array to store probabilities (current belief)
    :param false_negative_rates: numpy array to store false negative rates
    :param probability_of_containing_target_next_step: prediction using current belief for agent 9
    :return: current estimated goal
    """
    # Prepare a list to store indexes of maximum probabilities
    indexes_of_max_probability = list()

    # distance array to find distance from source to all nodes
    distance_array = length_of_path_from_source_to_all_nodes(maze, current_pos)

    # Assign current cell's distance to zero to avoid divide by zero error.
    distance_array[current_pos[0]][current_pos[1]] = INF

    # Normalize the probability
    probability_of_containing_target /= np.sum(probability_of_containing_target)
    if probability_of_containing_target_next_step is not None:
        probability_of_containing_target_next_step /= np.sum(probability_of_containing_target_next_step)

    # Find indexes of maximum probability for agent 6
    if agent == 6:
        indexes_of_max_probability = np.where(probability_of_containing_target ==
                                              np.amax(probability_of_containing_target))

    # Find indexes of maximum probability for agent 7
    elif agent == 7:
        probability_of_finding_target = np.multiply(probability_of_containing_target,
                                                    ONE_PROBABILITY - false_negative_rates)
        indexes_of_max_probability = np.where(probability_of_finding_target == np.amax(probability_of_finding_target))

    # Find indexes of maximum probability for agent 8
    elif agent == 8:
        probability_of_finding_target = np.multiply(probability_of_containing_target,
                                                    ONE_PROBABILITY - false_negative_rates)
        utility_function = np.divide(probability_of_finding_target, distance_array)
        indexes_of_max_probability = np.where(utility_function == np.amax(utility_function))

    # Find indexes of maximum probability for agent 9
    elif agent == 9:
        probability_of_finding_target = np.multiply(probability_of_containing_target_next_step,
                                                    ONE_PROBABILITY - false_negative_rates)
        utility_function = np.divide(probability_of_finding_target, distance_array)
        indexes_of_max_probability = np.where(utility_function == np.amax(utility_function))

    # Find indexes from maximum probability with least distance
    indexes_with_max_probability_and_min_distance = np.where(distance_array[indexes_of_max_probability] ==
                                                             np.amin(distance_array[indexes_of_max_probability]))

    # Choose randomly if there's a tie
    random_num = random.randint(0, indexes_with_max_probability_and_min_distance[0].shape[0] - 1)

    # Return position of current estimated target
    return indexes_of_max_probability[0][indexes_with_max_probability_and_min_distance[0][random_num]],\
           indexes_of_max_probability[1][indexes_with_max_probability_and_min_distance[0][random_num]]


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


def compute_probability_when_agent_fails_to_find_target(probability_of_containing_target: np.array,
                                                        false_negative_rates: np.array, current_pos: tuple):
    """
    Compute probability when agent fails to find target
    :param probability_of_containing_target: update probability directly in this array
    :param false_negative_rates: numpy array
    :param current_pos: agent's current position
    :return: None
    """
    p_of_x_y = probability_of_containing_target[current_pos[0]][current_pos[1]]

    reduced_probability = p_of_x_y * false_negative_rates[current_pos[0]][current_pos[1]]
    probability_denominator = np.sum(probability_of_containing_target) - p_of_x_y + reduced_probability

    probability_of_containing_target /= probability_denominator
    probability_of_containing_target[current_pos[0]][current_pos[1]] = reduced_probability/probability_denominator


def check_and_propagate_probability(probability_of_containing_target: np.array, false_negative_rates: np.array,
                                    current_pos: tuple, target_pos: tuple):
    """
    Function is used to check and propagate probabilities
    :param probability_of_containing_target: numpy array to store probabilities
    :param false_negative_rates: numpy array to store false negative rates
    :param current_pos: current position
    :param target_pos: target position
    :return: True if agent is able to find out target otherwise False
    """

    # If agent is at target's position, then use false negative rate to find target in it. If agent has found out,
    # return True otherwise change belief accordingly
    if current_pos == target_pos:

        # If current cell is flat
        if false_negative_rates[current_pos[0]][current_pos[1]] == 0.2:
            x = random.randint(0, 99)
            if x < 20:
                compute_probability_when_agent_fails_to_find_target(probability_of_containing_target,
                                                                    false_negative_rates, current_pos)
            else:
                return True

        # If current cell is hilly
        elif false_negative_rates[current_pos[0]][current_pos[1]] == 0.5:
            x = random.randint(0, 99)
            if x < 50:
                compute_probability_when_agent_fails_to_find_target(probability_of_containing_target,
                                                                    false_negative_rates, current_pos)
            else:
                return True

        # If current cell is forest
        elif false_negative_rates[current_pos[0]][current_pos[1]] == 0.8:
            x = random.randint(0, 99)
            if x < 80:
                compute_probability_when_agent_fails_to_find_target(probability_of_containing_target,
                                                                    false_negative_rates, current_pos)
            else:
                return True
    else:
        compute_probability_when_agent_fails_to_find_target(probability_of_containing_target,
                                                            false_negative_rates, current_pos)

    return False


def examine_and_propagate_probability(maze, probability_of_containing_target, false_negative_rates, current_pos,
                                      target_pos, current_estimated_goal, node):
    """
    Examine and change probability according to different conditions
    :param maze: Maze object
    :param probability_of_containing_target: numpy array of probabilities
    :param false_negative_rates: numpy array of false negative rates
    :param current_pos: current position
    :param target_pos: target position
    :param current_estimated_goal: current estimated goal
    :param node: to change examine
    :return:
    """

    # check and propagate probability if current position of agent is at estimated target
    if current_pos == current_estimated_goal:
        return check_and_propagate_probability(probability_of_containing_target, false_negative_rates, current_pos,
                                               target_pos)

    # If maze's node is blocked, change probability accordingly
    elif maze[node[0]][node[1]].is_blocked:
        p_of_x_y = probability_of_containing_target[node[0]][node[1]]
        remaining_probability = np.sum(probability_of_containing_target) - p_of_x_y

        probability_of_containing_target /= remaining_probability
        probability_of_containing_target[node[0]][node[1]] = ZERO_PROBABILITY

        return False
    else:
        # Else examine node
        return check_and_propagate_probability(probability_of_containing_target, false_negative_rates, node, target_pos)


def update_status(maze: list, false_negative_rates: np.ndarray, maze_array: np.array, cur_pos: tuple):
    """
    Function is used to update status of current cell of agent
    :param maze: agent's maze object
    :param false_negative_rates: numpy array of false negative rates
    :param maze_array: numpy array of full maze
    :param cur_pos: agent's current position
    :return:
    """
    # Change in agent's maze according to full maze
    if maze_array[cur_pos[0]][cur_pos[1]] == 1:
        maze[cur_pos[0]][cur_pos[1]].is_blocked = True
    elif maze_array[cur_pos[0]][cur_pos[1]] == 2:
        false_negative_rates[cur_pos[0]][cur_pos[1]] = FLAT_FALSE_NEGATIVE_RATE
        maze[cur_pos[0]][cur_pos[1]].is_blocked = False
    elif maze_array[cur_pos[0]][cur_pos[1]] == 3:
        false_negative_rates[cur_pos[0]][cur_pos[1]] = HILLY_FALSE_NEGATIVE_RATE
        maze[cur_pos[0]][cur_pos[1]].is_blocked = False
    elif maze_array[cur_pos[0]][cur_pos[1]] == 4:
        false_negative_rates[cur_pos[0]][cur_pos[1]] = FOREST_FALSE_NEGATIVE_RATE
        maze[cur_pos[0]][cur_pos[1]].is_blocked = False
    else:
        raise Exception("Invalid value in maze_array")

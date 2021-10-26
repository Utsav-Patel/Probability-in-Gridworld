"""
Necessary Imports
"""
import random
from datetime import datetime
from queue import Queue

import matplotlib.pyplot as plt
import numpy as np
from sortedcontainers import SortedSet

from constants import NUM_ROWS, NUM_COLS, X, Y, GOAL_POSITION_OF_AGENT, STARTING_POSITION_OF_AGENT, INF, \
    RELATIVE_POSITION_OF_TWO_MANDATORY_NEIGHBORS, RELATIVE_POSITION_OF_TWO_SENSED_NEIGHBORS, \
    RELATIVE_POSITION_OF_NEIGHBORS_TO_CHECK, RELATIVE_POSITION_OF_NEIGHBORS_TO_UPDATE


def avg(lst: list):
    """
    This function computes average of the given list. If the length of list is zero, it will return zero.
    :param lst: list for which you want to compute average
    :return: average of the given list
    """
    if len(lst) == 0:
        return 0
    return sum(lst) / len(lst)


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


def compute_heuristics(maze: list, h_func):
    """
    Compute Heuristic for the current maze
    :param maze: maze of type list
    :param h_func: Heuristic function we want to use
    :return: None as we are updating in the same maze object
    """

    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            if not maze[row][col].is_blocked:
                maze[row][col].h = h_func((row, col), GOAL_POSITION_OF_AGENT)


def generate_grid_manually():
    """
    This is the function to generate grid manually. This is helpful for the initial testing and problem 1.
    :return: Manually generated numpy array.
    """
    array = np.zeros((NUM_ROWS, NUM_COLS))

    array[1][0] = 1
    array[1][2] = 1
    array[1][3] = 1
    array[2][3] = 1
    array[3][3] = 1
    array[3][4] = 1
    return array


def generate_grid_with_probability_p(p):
    """
    This function will generate the uniform random grid of size NUM_ROWS X NUM_COLS.
    :param p: probability of cell being blocked
    :return: Grid of size NUM_ROWS X NUM_COLS with each cell having uniform probability of being blocked is p.
    """
    from constants import NUM_COLS, NUM_ROWS
    randomly_generated_array = np.random.uniform(low=0.0, high=1.0, size=NUM_ROWS * NUM_COLS).reshape(NUM_ROWS,
                                                                                                      NUM_COLS)
    randomly_generated_array[STARTING_POSITION_OF_AGENT[0]][STARTING_POSITION_OF_AGENT[1]] = 0
    randomly_generated_array[GOAL_POSITION_OF_AGENT[0]][GOAL_POSITION_OF_AGENT[1]] = 0
    randomly_generated_array[randomly_generated_array < (1 - p)] = 0
    randomly_generated_array[randomly_generated_array >= (1 - p)] = 1
    return randomly_generated_array


def check(current_position: tuple):
    """
    Check whether current point is in the grid or not
    :param current_position: current point
    :return: True if the current point is in the grid otherwise False
    """
    if (0 <= current_position[0] < NUM_ROWS) and (0 <= current_position[1] < NUM_COLS):
        return True
    return False


def create_maze_array_from_discovered_grid(maze: list):
    """
    This function will generate numpy array from maze list for the discovered grid
    :param maze: Maze object
    :return: Numpy array
    """
    maze_array = np.ones((NUM_ROWS, NUM_COLS))

    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            if maze[row][col].is_confirmed and (not maze[row][col].is_blocked):
                maze_array[row][col] = 0

    return maze_array


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
                    and (maze_array[neighbour[0]][neighbour[1]] == 0):
                q.put(neighbour)
                distance_array[neighbour[0]][neighbour[1]] = distance_array[current_node[0]][current_node[1]] + 1

    return distance_array[goal_pos[0]][goal_pos[1]]


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

    # Storing child of each node so we can iterate from start_pos to goal_pos
    while cur_pos != parent[cur_pos]:
        child[parent[cur_pos]] = cur_pos
        cur_pos = parent[cur_pos]

    return child


def compute_num_confirmed_cells(maze: list):
    """
    This function is used to compute number of confirmed cells for our maze
    :param maze: Maze object
    :return: number of confirmed in the maze
    """
    num_confirmed_cells = 0
    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            if maze[row][col].is_confirmed:
                num_confirmed_cells += 1
    return num_confirmed_cells


def compute_num_confirmed_blocked_cells(maze: list):
    """
    This function is used to compute number of confirmed blocks for our maze
    :param maze: Maze object
    :return: number of confirmed blocks in the maze
    """
    num_confirmed_blocked_cells = 0
    for row in range(NUM_ROWS):
        for col in range(NUM_COLS):
            if maze[row][col].is_confirmed and maze[row][col].is_blocked:
                num_confirmed_blocked_cells += 1
    return num_confirmed_blocked_cells


def astar_search(maze: list, start_pos: tuple):
    """
    Function to compute A* search
    :param maze: maze is a list of list
    :param start_pos: starting position of the maze from where we want to start A* search
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
    maze[start_pos[0]][start_pos[1]].f = maze[start_pos[0]][start_pos[1]].h

    # Assigning a random number to start position to the starting position and adding to visited nodes
    node_to_random_number_mapping[start_pos] = random.uniform(0, 1)
    visited_nodes.add(start_pos)

    # Add start position node into the sorted set. We are giving priority to f(n), h(n), and g(n) in the decreasing
    # order. Push random number for random selection if there is conflict between two nodes
    # (If f(n), g(n), and h(n) are same for two nodes)
    sorted_set.add(((maze[start_pos[0]][start_pos[1]].f, maze[start_pos[0]][start_pos[1]].probability_of_being_blocked,
                     maze[start_pos[0]][start_pos[1]].h, maze[start_pos[0]][start_pos[1]].g,
                     node_to_random_number_mapping[start_pos]), start_pos))

    parents[start_pos] = start_pos

    num_explored_nodes = 0

    # Running the loop until we reach our goal state or the sorted set is empty
    while sorted_set.__len__() != 0:
        # Popping first (shortest) element from the sorted set
        current_node = sorted_set.pop(index=0)

        # Increase the number of explored nodes
        num_explored_nodes += 1

        # If we have found the goal position, we can return parents and total explored nodes
        if current_node[1] == GOAL_POSITION_OF_AGENT:
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
                    maze[neighbour[0]][neighbour[1]].f = maze[neighbour[0]][neighbour[1]].g + \
                                                         maze[neighbour[0]][neighbour[1]].h
                    node_to_random_number_mapping[neighbour] = random.uniform(0, 1)
                    visited_nodes.add(neighbour)
                    sorted_set.add(((maze[neighbour[0]][neighbour[1]].f,
                                     maze[neighbour[0]][neighbour[1]].probability_of_being_blocked,
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
                             maze[neighbour[0]][neighbour[1]].probability_of_being_blocked,
                             maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                             node_to_random_number_mapping[neighbour]), neighbour) \
                                in sorted_set:
                            sorted_set.remove(
                                ((maze[neighbour[0]][neighbour[1]].f,
                                  maze[neighbour[0]][neighbour[1]].probability_of_being_blocked,
                                  maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                                  node_to_random_number_mapping[neighbour]), neighbour))
                        maze[neighbour[0]][neighbour[1]].g = neighbour_g
                        maze[neighbour[0]][neighbour[1]].f = neighbour_f
                        node_to_random_number_mapping[neighbour] = random.uniform(0, 1)
                        sorted_set.add(
                            ((maze[neighbour[0]][neighbour[1]].f,
                              maze[neighbour[0]][neighbour[1]].probability_of_being_blocked,
                              maze[neighbour[0]][neighbour[1]].h, maze[neighbour[0]][neighbour[1]].g,
                              node_to_random_number_mapping[neighbour]), neighbour))
                        parents[neighbour] = current_node[1]

    return parents, num_explored_nodes


def forward_execution(maze: list, maze_array: np.array, start_pos: tuple, parents: dict,
                      want_to_explore_field_of_view: bool, is_backtrack_strategy_on: bool = False):
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

    children = parent_to_child_dict(parents, GOAL_POSITION_OF_AGENT)

    # Setting current position to starting position so we can start iterating from start_pos
    cur_pos = start_pos

    current_path = [cur_pos]
    if is_backtrack_strategy_on:
        last_cell_which_is_not_in_dead_end = cur_pos

    # Iterating from start_pos to goal_pos if we won't get any blocks in between otherwise we are terminating the
    # iteration.
    while True:

        if is_backtrack_strategy_on:
            path_exist_from_the_last_point = 0

        maze[cur_pos[0]][cur_pos[1]].is_confirmed = True

        if maze_array[cur_pos[0]][cur_pos[1]] == 1:
            maze[cur_pos[0]][cur_pos[1]].is_blocked = True
        else:
            maze[cur_pos[0]][cur_pos[1]].is_blocked = False

        # Explore the field of view and update the blocked nodes if there's any in the path.
        if want_to_explore_field_of_view:
            for ind in range(len(X)):

                neighbour = (cur_pos[0] + X[ind], cur_pos[1] + Y[ind])
                if check(neighbour):
                    maze[neighbour[0]][neighbour[1]].is_confirmed = True
                    if maze_array[neighbour[0]][neighbour[1]] == 1:
                        maze[neighbour[0]][neighbour[1]].is_blocked = True

                # Here, we are finding whether the current node is a part of the dead end or not. If there is a path
                # exists other than its child and parent, then this node should not be part of dead end because
                # there is another path available which you can explore.
                if is_backtrack_strategy_on and (check(neighbour)) and (children[cur_pos] != neighbour) \
                        and (parents[cur_pos] != neighbour) and (maze_array[neighbour[0]][neighbour[1]] == 0):
                    path_exist_from_the_last_point += 1

        if is_backtrack_strategy_on:

            # If we can find such a node which we can explore later using current node, then this node should not be
            # part of the dead end path.
            if path_exist_from_the_last_point > 0:
                last_cell_which_is_not_in_dead_end = cur_pos

        if cur_pos == children[cur_pos]:
            break
        # If we encounter any block in the path, we have to terminate the iteration
        if maze_array[children[cur_pos][0]][children[cur_pos][1]] == 1:
            break
        cur_pos = children[cur_pos]
        current_path.append(cur_pos)

    if cur_pos != GOAL_POSITION_OF_AGENT:

        # Change the start node to last unblocked node and backtrack if it is set to any positive integer.
        maze[children[cur_pos][0]][children[cur_pos][1]].is_blocked = True
        maze[children[cur_pos][0]][children[cur_pos][1]].is_confirmed = True

        if is_backtrack_strategy_on:

            # We keep backtracking cell until we reached a cell from where we can explore a new path. Also, we are
            # manually blocking those dead end nodes because they are not useful anymore.
            while cur_pos != last_cell_which_is_not_in_dead_end:
                num_backtracks += 1
                maze[cur_pos[0]][cur_pos[1]].is_blocked = True
                cur_pos = parents[cur_pos]
                current_path.append(cur_pos)

    return current_path, num_backtracks


def single_plot(x, y, title, xlabel, ylabel, savefig_name, fontsize: int = 10):
    """
    This function is used to plot a single plot
    :param x: X axis list
    :param y: Y axis list
    :param title: title of the plot
    :param xlabel: x-label of the plot
    :param ylabel: y-label of the plot
    :param savefig_name: name of the figure which you want to use save
    :param fontsize: change size of the all font (title, x-label, and y-label)
    :return:
    """
    fig, axs = plt.subplots()
    axs.plot(x, y, marker='.', ms=10.0, c='blue', mfc='red')
    axs.set_title(title, fontsize=fontsize)
    axs.set_xlabel(xlabel, fontsize=fontsize)
    axs.set_ylabel(ylabel, fontsize=fontsize)
    plt.savefig(savefig_name)
    plt.show()


def multiple_plot(x, y, title, xlabel, ylabel, savefig_name, legends, fontsize: int = 10):
    """
    This function is used to add multiple plots on y axis
    :param x: X axis list
    :param y: Y axis list of list
    :param title: title of the plot
    :param xlabel: x-label of the plot
    :param ylabel: y-label of the plot
    :param savefig_name: name of the figure which you want to use save
    :param legends: add legends to this multiple plots
    :param fontsize: change size of the all font (title, x-label, and y-label)
    :return:
    """
    fig, axs = plt.subplots()
    for array in y:
        axs.plot(x, array, marker='.', ms=10.0, mfc='red')
    axs.legend(legends)
    axs.set_title(title, fontsize=fontsize)
    axs.set_xlabel(xlabel, fontsize=fontsize)
    axs.set_ylabel(ylabel, fontsize=fontsize)
    plt.savefig(savefig_name)
    plt.show()


def reset_probability_of_being_block_for_each_cell_after_execution(maze: list):
    """
    This function is helpful for the problem 5. This function resets the previous probabilities.
    :param maze: Maze object
    :return: Nothing as we are directly changing into maze object
    """

    # Initialize the current position so that we can use DFS and update probability of neighbor of each visited cell.
    current_position = STARTING_POSITION_OF_AGENT

    # Use Queue for DFS and set to keep track visited cells
    visited_cells = set()
    queue = Queue()
    queue.put(current_position)
    visited_cells.add(current_position)

    # This loop will run until all visited cells are visited again
    while not queue.empty():
        current_position = queue.get()

        # Resetting the probability of neighbor of current_position
        for neighbor in maze[current_position[0]][current_position[1]].eight_neighbors:
            if not maze[neighbor[0]][neighbor[1]].is_confirmed:
                # print(maze[neighbor[0]][neighbor[1]].num_visited_neighbors)
                maze[neighbor[0]][neighbor[1]].probability_of_being_blocked = 0
                maze[neighbor[0]][neighbor[1]].min_hidden_cell_neighbor = INF

        # Adding visited neighbor to the queue so that we can iterate it again
        for ind in range(len(X)):
            neighbor = (current_position[0] + X[ind], current_position[1] + Y[ind])
            if check(neighbor) and (neighbor not in visited_cells) and maze[neighbor[0]][neighbor[1]].is_visited:
                queue.put(neighbor)
                visited_cells.add(neighbor)


def compute_probability_of_being_block_for_each_cell_after_execution(maze: list):
    """
    This function will compute probability of each valid cell and directly update in the maze.
    :param maze: Maze object
    :return: Nothing as we are directly updating in the maze object
    """

    # Reset previous probability before using it
    reset_probability_of_being_block_for_each_cell_after_execution(maze)

    # Set current position from where we can use DFS
    current_position = STARTING_POSITION_OF_AGENT

    # Initialize one set and queue for the DFS
    visited_cells = set()
    queue = Queue()
    queue.put(current_position)

    # This loop will run until there is no cell left to be visited
    while not queue.empty():
        current_position = queue.get()

        # Compute number of hidden cells and hidden block
        num_hidden_cells = maze[current_position[0]][current_position[1]].num_neighbor - \
                           (maze[current_position[0]][current_position[1]].num_confirmed_blocked +
                            maze[current_position[0]][current_position[1]].num_confirmed_unblocked)
        num_hidden_block = maze[current_position[0]][current_position[1]].num_sensed_blocked - \
                           maze[current_position[0]][current_position[1]].num_confirmed_blocked

        assert maze[current_position[0]][current_position[1]].num_neighbor == \
               (maze[current_position[0]][current_position[1]].num_sensed_blocked +
                maze[current_position[0]][current_position[1]].num_sensed_unblocked)

        # Add assert for the particular case so that we can make sure our code is working perfectly
        if num_hidden_cells > 0:
            assert num_hidden_block < num_hidden_cells
            assert num_hidden_block > 0

        num_unconfirmed_cells = 0
        # Visit each unconfirmed neighbor of current cell and update it's probability if it's valid to update.
        for neighbor in maze[current_position[0]][current_position[1]].eight_neighbors:
            if (not maze[neighbor[0]][neighbor[1]].is_confirmed) and \
                    maze[neighbor[0]][neighbor[1]].min_hidden_cell_neighbor > num_hidden_cells:
                num_unconfirmed_cells += 1
                maze[neighbor[0]][neighbor[1]].probability_of_being_blocked = num_hidden_block / num_hidden_cells
                maze[neighbor[0]][neighbor[1]].min_hidden_cell_neighbor = num_hidden_cells

        assert num_unconfirmed_cells <= num_hidden_cells
        # Add visited neighbor to queue to we can process again.
        for ind in range(len(X)):
            neighbor = (current_position[0] + X[ind], current_position[1] + Y[ind])
            if check(neighbor) and (neighbor not in visited_cells) and maze[neighbor[0]][neighbor[1]].is_visited:
                queue.put(neighbor)
                visited_cells.add(neighbor)

    # queue.queue.clear()
    # visited_cells.clear()
    # current_position = STARTING_POSITION_OF_AGENT
    #
    # queue.put(current_position)
    #
    # while not queue.empty():
    #     current_position = queue.get()
    #     for neighbor in maze[current_position[0]][current_position[1]].eight_neighbors:
    #         if not maze[neighbor[0]][neighbor[1]].is_confirmed:
    #             maze[neighbor[0]][neighbor[1]].probability_of_being_blocked = \
    #                 maze[neighbor[0]][neighbor[1]].probability_of_being_blocked / \
    #                 maze[neighbor[0]][neighbor[1]].num_visited_neighbors
    #     for ind in range(len(X)):
    #         neighbor = (current_position[0] + X[ind], current_position[1] + Y[ind])
    #         if check(neighbor) and (neighbor not in visited_cells) and maze[neighbor[0]][neighbor[1]].is_visited:
    #             queue.put(neighbor)
    #             visited_cells.add(neighbor)


def add_constraint_to_knowledge_base(knowledge_base: list, variable_to_constraint_dict: dict, unconfirmed_cells: set,
                                     num_blocked_cells_in_unconfirmed_cells: int):
    """
    This function adds the given constraint to our knowledge base
    :param knowledge_base: knowledge base where we add our information
    :param variable_to_constraint_dict: dictionary where we maintain constraint index of knowledge base for each variable
    :param unconfirmed_cells: set which we want to add in the knowledge base
    :param num_blocked_cells_in_unconfirmed_cells: number of sensed block cells which are in the unconfirmed cell
    :return: Nothing as we are directly updating in the objects
    """

    # If there is no element in the set, there is no point to add it in the knowledge base
    if len(unconfirmed_cells) == 0:
        return

    # visit each cell of unconfirmed_cells set update in variable to constraint dictionary
    for cell in unconfirmed_cells:
        if cell not in variable_to_constraint_dict:
            variable_to_constraint_dict[cell] = set()
        variable_to_constraint_dict[cell].add(len(knowledge_base))

    # Creating list instead of tuple so that we can update the value of list for each constraint
    knowledge_base.append([unconfirmed_cells, num_blocked_cells_in_unconfirmed_cells])


def sense_current_node(maze, current_position: tuple, full_maze: np.array, knowledge_base=None,
                       variable_to_constraint_dict=None):
    """
    This function is used to sense the current node and update details in the knowledge base
    :param maze: Maze object
    :param current_position: position of the maze for which you want to sense
    :param full_maze: full maze
    :param knowledge_base: list which contains the knowledge base
    :param variable_to_constraint_dict: variable to constraint dictionary
    :return: Nothing as we are directly updating into maze object
    """

    # If the default argument is None then set empty list and set for variable
    if knowledge_base is None:
        knowledge_base = list()

    if variable_to_constraint_dict is None:
        variable_to_constraint_dict = dict()

    # Create a set to store all variable in a constraint
    unconfirmed_cells = set()
    num_blocked_cells_in_unconfirmed_cells = 0

    # Visit all eight neighbors and sense them
    for neighbor in maze[current_position[0]][current_position[1]].eight_neighbors:

        # Update the status for confirm and non-confirm conditions
        if maze[neighbor[0]][neighbor[1]].is_confirmed:
            if maze[neighbor[0]][neighbor[1]].is_blocked:
                maze[current_position[0]][current_position[1]].num_confirmed_blocked += 1
                maze[current_position[0]][current_position[1]].num_sensed_blocked += 1
            else:
                maze[current_position[0]][current_position[1]].num_confirmed_unblocked += 1
                maze[current_position[0]][current_position[1]].num_sensed_unblocked += 1
        else:

            # Adding this to set so that we can add it to our knowledge base
            unconfirmed_cells.add(neighbor)

            if full_maze[neighbor[0]][neighbor[1]] == 1:
                num_blocked_cells_in_unconfirmed_cells += 1
                maze[current_position[0]][current_position[1]].num_sensed_blocked += 1
            else:
                maze[current_position[0]][current_position[1]].num_sensed_unblocked += 1

    # Add this constraint to our knowledge base
    add_constraint_to_knowledge_base(knowledge_base, variable_to_constraint_dict, unconfirmed_cells,
                                     num_blocked_cells_in_unconfirmed_cells)


def check_constraint(variables_set: set, value: int):
    """
    This function is used to check whether the given constrain is valid of not
    :param variables_set: variables of the constraint
    :param value: value of the given constraint
    :return: True if the constraint can be satisfied otherwise False
    """
    if (len(variables_set) < value) or (value < 0):
        return False
    return True


def backtracking_search(current_variable_index: int, total_num_variables: int, variables: list,
                        assigned_values_each_variable: dict, frequency_of_each_output_for_each_variable: dict,
                        knowledge_base: list, variable_to_constraint_dict: dict, most_constraint_variable: tuple):
    """
    This function is used for backtracking search. It will recursively take the value of each variable from the domain
    and try to find possible answers.
    :param current_variable_index: index of the current variable index
    :param total_num_variables: total number of indexes for the backtrack
    :param variables: list of variable in the given constraints
    :param assigned_values_each_variable: to keep track of assigned value for each variable
    :param frequency_of_each_output_for_each_variable: to keep track the frequency of valid outputs
    :param knowledge_base: our knowledge base
    :param variable_to_constraint_dict: dictionary which can give you list of index of constrain where the given
            variable is in the knowledge base
    :param most_constraint_variable: one variable which has most constraint in the given knowledge base
    :return: Nothing as we are directly updating in the objects
    """
    # Condition when all the constraints are satisfied with the given set of values
    if current_variable_index == total_num_variables:

        # Increase frequency by one for each valid set
        for variable in assigned_values_each_variable:
            if variable not in frequency_of_each_output_for_each_variable:
                frequency_of_each_output_for_each_variable[variable] = [0, 0]
            frequency_of_each_output_for_each_variable[variable][assigned_values_each_variable[variable]] += 1
        return

    # Loop over the all values which are in the domain.
    for value in [0, 1]:

        # Assign the value to that particular variable
        assigned_values_each_variable[variables[current_variable_index]] = value

        is_current_value_satisfied_all_constraints = True
        list_of_constraint_index_for_variable = list()

        # Iterate over all the constraints from the knowledge base and make it solved.
        for constraint_index in variable_to_constraint_dict[most_constraint_variable]:
            if variables[current_variable_index] in knowledge_base[constraint_index][0]:
                list_of_constraint_index_for_variable.append(constraint_index)
                knowledge_base[constraint_index][0].remove(variables[current_variable_index])
                knowledge_base[constraint_index][1] -= value
                if not check_constraint(knowledge_base[constraint_index][0], knowledge_base[constraint_index][1]):
                    is_current_value_satisfied_all_constraints = False

        # If all constrains are satisfied after giving value to the variable, run backtracking search
        if is_current_value_satisfied_all_constraints:
            backtracking_search(current_variable_index + 1, total_num_variables, variables,
                                assigned_values_each_variable, frequency_of_each_output_for_each_variable,
                                knowledge_base, variable_to_constraint_dict, most_constraint_variable)

        # reverse the changes which we made earlier
        for constraint_index in list_of_constraint_index_for_variable:
            knowledge_base[constraint_index][0].add(variables[current_variable_index])
            knowledge_base[constraint_index][1] += value


def most_constraint_variable_list(variable_to_constraint_dict: dict, want_to_use_probability_approach: bool,
                                  maze: list):
    """
    Find the list of most constraint variables
    :param variable_to_constraint_dict: dictionary containing key value pair of variable to index of constraint in
            knowledge base
    :param want_to_use_probability_approach: if you want to get list based on probability then set it to True otherwise
            it should be False
    :param maze: Maze object
    :return: list containing most constraint variables
    """

    # If you want to get most constraint variable based on probability
    if want_to_use_probability_approach:
        max_probability = 0
        cells_with_same_highest_probability = list()

        # Iterate over variable, find probability and update correspondingly
        for variable in variable_to_constraint_dict:
            probability = len(variable_to_constraint_dict[variable]) / maze[variable[0]][variable[1]].num_neighbor
            if max_probability < probability:
                max_probability = probability
                cells_with_same_highest_probability.clear()
                cells_with_same_highest_probability.append(variable)
            elif max_probability == probability:
                cells_with_same_highest_probability.append(variable)

        return cells_with_same_highest_probability

    # Otherwise else would look at the absolute value of most constraint variables choose those variables which are in
    # most number of constraints
    else:
        cells_which_are_most_constraint_variable = list()
        max_num_constraint = 0
        for variable in variable_to_constraint_dict:
            if max_num_constraint < len(variable_to_constraint_dict[variable]):
                max_num_constraint = len(variable_to_constraint_dict[variable])
                cells_which_are_most_constraint_variable.clear()
                cells_which_are_most_constraint_variable.append(variable)
            elif max_num_constraint == len(variable_to_constraint_dict[variable]):
                cells_which_are_most_constraint_variable.append(variable)

        return cells_which_are_most_constraint_variable


def make_inference_from_most_constraint_variable(knowledge_base: list, variable_to_constraint_dict: dict,
                                                 most_constraint_variable: tuple):
    """
    This function is used to make inferences from the most constraint variable
    :param knowledge_base: Agent's knowledge base
    :param variable_to_constraint_dict: variable to list of index in knowledge base constraint dictionary
    :param most_constraint_variable: most constraint variable
    :return: Queue which contains the nodes which are inferred
    """

    # Initialise few things
    constraints_containing_most_constraint_variable_set = set()
    constraints_containing_most_constraint_variable_list = list()
    current_constraint_list = list()

    # Iterate over each constraint to find the most constraint variable
    for constraint_index in variable_to_constraint_dict[most_constraint_variable]:
        current_constraint_list.append(knowledge_base[constraint_index])
        for variable in knowledge_base[constraint_index][0]:
            if variable not in constraints_containing_most_constraint_variable_set:
                constraints_containing_most_constraint_variable_set.add(variable)
                constraints_containing_most_constraint_variable_list.append(variable)

    # Check the statement
    assert len(constraints_containing_most_constraint_variable_list) <= 20

    num_variables = len(constraints_containing_most_constraint_variable_list)
    frequency_of_each_output_for_each_variable = dict()
    assigned_values_each_variable = dict()

    # Call backtracking search
    backtracking_search(0, num_variables, constraints_containing_most_constraint_variable_list,
                        assigned_values_each_variable, frequency_of_each_output_for_each_variable, knowledge_base,
                        variable_to_constraint_dict, most_constraint_variable)

    # Check and add inferred items to the queue
    queue = Queue()
    for variable in frequency_of_each_output_for_each_variable:
        if (frequency_of_each_output_for_each_variable[variable][0] == 0 and
            frequency_of_each_output_for_each_variable[variable][1] != 0) or \
                (frequency_of_each_output_for_each_variable[variable][1] == 0 and
                 frequency_of_each_output_for_each_variable[variable][0] != 0):
            queue.put(variable)

    return queue


def make_inference_from_most_constraint_variable_list(knowledge_base: list, variable_to_constraint_dict: dict,
                                                      want_to_use_probability_approach: bool = False,
                                                      maze: list = None, list_of_variables_in_the_path=None):
    """
    This function would make inferences from the list of variables and then most constrained variables.
    :param knowledge_base: Agent's knowledge base
    :param variable_to_constraint_dict: variable to constraint dictionary
    :param want_to_use_probability_approach: how you want to choose most constrained variables
    :param maze: Maze object
    :param list_of_variables_in_the_path: variables which are in my current path
    :return:
    """

    # If we haven't passed anything, we should make list_of_variables_in_the_path to empty list
    if list_of_variables_in_the_path is None:
        list_of_variables_in_the_path = list()

    # Extract most constrained variables
    most_constraint_variables = most_constraint_variable_list(variable_to_constraint_dict,
                                                              want_to_use_probability_approach, maze)

    # If there is no constrains in the knowledge base, we are returning empty queue
    if len(most_constraint_variables) == 0:
        return Queue()

    # to keep track of visited varialbes
    visited_variables = set()
    for variable in list_of_variables_in_the_path:
        most_constraint_variables.append(variable)

    # Creating an empty queue which would be useful to return inferred cells
    queue = Queue()

    # Iterate over each most constrained variable
    for most_constraint_variable in most_constraint_variables:

        if most_constraint_variable in visited_variables:
            continue
        visited_variables.add(most_constraint_variable)

        # Get another queue which contains the list of variables which are inferred
        queue2 = make_inference_from_most_constraint_variable(knowledge_base, variable_to_constraint_dict,
                                                              most_constraint_variable)

        # Popping element and storing into final queue
        while not queue2.empty():
            queue.put(queue2.get())

    # Here, we are iterating over most constrained variables in decreasing order for 0.5 seconds
    start_time = datetime.now()
    for variable in variable_to_constraint_dict:

        # Don't need to visit again if we have already visited
        if variable in visited_variables:
            continue
        visited_variables.add(variable)

        # Try to make inference if it's possible
        queue2 = make_inference_from_most_constraint_variable(knowledge_base, variable_to_constraint_dict, variable)
        while not queue2.empty():
            queue.put(queue2.get())
        end_time = datetime.now()

        # If the threshold is exhausted, then break this loop
        if (end_time - start_time).total_seconds() > 0.5:
            break

    return queue


def remove_variable_from_knowledge_base(knowledge_base: list, variable_to_constraint_dict: dict, current_node: tuple,
                                        current_node_val: int):
    """
    This function is used to remove element from our knowledge base
    :param knowledge_base: Knowledge base
    :param variable_to_constraint_dict: variable to constraint dictionary
    :param current_node: remove this variable from all constraint list
    :param current_node_val: value associated with the current cell
    :return:
    """

    # Iterate through each constraint and remove it
    if current_node in variable_to_constraint_dict:
        for constraint_index in variable_to_constraint_dict[current_node]:
            knowledge_base[constraint_index][0].remove(current_node)
            knowledge_base[constraint_index][1] -= current_node_val
        variable_to_constraint_dict.pop(current_node)


def can_infer(num_sensed_blocked: int, num_confirmed_blocked: int, num_sensed_unblocked: int,
              num_confirmed_unblocked: int):
    """
    check whether we can infer or not from the current set of variables
    :param num_sensed_blocked: number of sensed blocks
    :param num_confirmed_blocked: number of confirmed blocks
    :param num_sensed_unblocked: number of sensed unblocks
    :param num_confirmed_unblocked: number confirmed unblocks
    :return: True if we can infer anything otherwise False
    """

    # Check precondition
    assert (num_sensed_blocked >= num_confirmed_blocked) and (num_sensed_unblocked >= num_confirmed_unblocked)

    # Condition whether we can infer or not
    if ((num_sensed_blocked == num_confirmed_blocked) and (num_sensed_unblocked > num_confirmed_unblocked)) or \
            ((num_sensed_unblocked == num_confirmed_unblocked) and (num_sensed_blocked > num_confirmed_blocked)):
        return True
    return False


def is_ambiguous(num_sensed_blocked: int, num_confirmed_blocked: int, num_sensed_unblocked: int,
                 num_confirmed_unblocked: int):
    """
    Check whether the conditions are ambiguous or not
    :param num_sensed_blocked: number of sensed blocks
    :param num_confirmed_blocked: number of confirmed blocks
    :param num_sensed_unblocked: number of sensed unblocks
    :param num_confirmed_unblocked: number of confirmed unblocks
    :return: True if the given set of variables are ambiguous otherwise False
    """
    assert (num_sensed_blocked >= num_confirmed_blocked) and (num_sensed_unblocked >= num_confirmed_unblocked)
    if (num_sensed_blocked > num_confirmed_blocked) and (num_sensed_unblocked > num_confirmed_unblocked):
        return True
    return False


def find_block_while_inference(maze: list, current_position: tuple, full_maze: np.array, entire_trajectory_nodes=None,
                               want_to_use_one_node_inference_strategy: bool = True,
                               want_to_use_two_node_inference_strategy: bool = False,
                               want_to_use_three_node_inference_strategy: bool = False,
                               want_to_use_most_constraint_variable_for_backtracking_search: bool = False,
                               want_to_use_probability_approach=True,
                               knowledge_base=None, variable_to_constraint_dict=None,
                               list_of_variables_in_the_path=None):
    """
    This function helps us find blocks in the path if they exist while also performing inference.
    :param maze: This agents copy of the maze.
    :param current_position: This is the current position of the agent.
    :param full_maze: This is the full maze.
    :param entire_trajectory_nodes: This is the set of nodes that are in the agents current trajectory.
    :param want_to_use_one_node_inference_strategy: This parameter lets us set if we want use Agent 3's inference
    strategy.
    :param want_to_use_two_node_inference_strategy: This parameter lets us set if we want use Agent 4's 2 cell inference
     strategy.
    :param want_to_use_three_node_inference_strategy: This parameter lets us set if we want use Agent 4's 3 cell
    inference strategy.
    :param want_to_use_most_constraint_variable_for_backtracking_search: This parameter lets us set if we want use Agent
     4's Advanced inference strategy.
    :param want_to_use_probability_approach: This parameter lets us decide how we want to get the most constrained
    variable.
    :param knowledge_base: This is the knowledge base of the Agent
    :param variable_to_constraint_dict: This dictionary maps the variables to constraints.
    :param list_of_variables_in_the_path: This list stores the variables that are currently in our path.
    :return: True if there is a block in path else false.
    """
    # We initialize parameters if we get them as None.
    if knowledge_base is None:
        knowledge_base = list()

    if entire_trajectory_nodes is None:
        entire_trajectory_nodes = set()

    if variable_to_constraint_dict is None:
        variable_to_constraint_dict = dict()

    if list_of_variables_in_the_path is None:
        list_of_variables_in_the_path = list()

    # We create this queue to help us store all the variables that are going to be confirmed and subsequently store
    # those cells in a set so that we can access them in constant time.
    inference_items = Queue()
    items_in_the_queue = set()
    is_block_node_in_current_path = False

    # We put the current position of the agent into the Queue and set.
    inference_items.put(current_position)
    items_in_the_queue.add(current_position)

    # We run this while loop until the queue is empty and there is nothing left to be inferred.
    while not inference_items.empty():
        while not inference_items.empty():
            current_node = inference_items.get()
            items_in_the_queue.remove(current_node)

            # If the current node is not set as confirmed in the maze then we set it to confirmed and made updates
            # accordingly
            if not maze[current_node[0]][current_node[1]].is_confirmed:

                maze[current_node[0]][current_node[1]].is_confirmed = True
                current_node_val = 0

                if full_maze[current_node[0]][current_node[1]] == 1:
                    maze[current_node[0]][current_node[1]].is_blocked = True
                    current_node_val = 1

                    # If current node is in the trajectory then we should return True for this function
                    if current_node in entire_trajectory_nodes:
                        is_block_node_in_current_path = True
                else:
                    maze[current_node[0]][current_node[1]].is_blocked = False

                # Remove this confirmed cell from the knowledge base
                remove_variable_from_knowledge_base(knowledge_base, variable_to_constraint_dict, current_node,
                                                    current_node_val)

                # Iterate over eight neighbors, update their status and add it into the queue
                for neighbor in maze[current_node[0]][current_node[1]].eight_neighbors:
                    if not maze[neighbor[0]][neighbor[1]].is_visited:
                        continue
                    if maze[current_node[0]][current_node[1]].is_blocked:
                        maze[neighbor[0]][neighbor[1]].num_confirmed_blocked += 1
                    else:
                        maze[neighbor[0]][neighbor[1]].num_confirmed_unblocked += 1

                    if not (neighbor in items_in_the_queue):
                        items_in_the_queue.add(neighbor)
                        inference_items.put(neighbor)

            # If the current cell is visited, we can inference rules to them
            if maze[current_node[0]][current_node[1]].is_visited:

                # This rule applies to the 3rd as well as the 4th agent where we are trying to infer using only one
                # cell's inference
                if want_to_use_one_node_inference_strategy:
                    if can_infer(maze[current_node[0]][current_node[1]].num_sensed_blocked,
                                 maze[current_node[0]][current_node[1]].num_confirmed_blocked,
                                 maze[current_node[0]][current_node[1]].num_sensed_unblocked,
                                 maze[current_node[0]][current_node[1]].num_confirmed_unblocked):

                        for neighbor in maze[current_node[0]][current_node[1]].eight_neighbors:
                            if (neighbor not in items_in_the_queue) and \
                                    (not maze[neighbor[0]][neighbor[1]].is_confirmed):
                                items_in_the_queue.add(neighbor)
                                inference_items.put(neighbor)

                # Three cells inference strategy
                if want_to_use_three_node_inference_strategy:

                    # Iterating over all possible mandatory neighbors
                    for index in range(len(RELATIVE_POSITION_OF_TWO_MANDATORY_NEIGHBORS)):

                        # Fetching relative position of mandatory neighbors
                        two_mandatory_neighbors = RELATIVE_POSITION_OF_TWO_MANDATORY_NEIGHBORS[index]
                        num_of_neighbors = 0

                        # Check precondition of mandatory neighbors
                        for relative_position in two_mandatory_neighbors:
                            neighbor = (current_node[0] + relative_position[0], current_node[1] + relative_position[1])
                            if check(neighbor) and maze[neighbor[0]][neighbor[1]].is_visited:
                                num_of_neighbors += 1

                        # Check whether you found two neighbors or not
                        if num_of_neighbors == 2:
                            num_sensed_blocked = maze[current_node[0]][current_node[1]].num_sensed_blocked
                            num_confirmed_blocked = maze[current_node[0]][current_node[1]].num_confirmed_blocked
                            num_sensed_unblocked = maze[current_node[0]][current_node[1]].num_sensed_unblocked
                            num_confirmed_unblocked = maze[current_node[0]][current_node[1]].num_confirmed_unblocked

                            # Current node should not be blocked
                            if maze[current_node[0]][current_node[1]].is_blocked:
                                num_confirmed_blocked += 1
                                num_sensed_blocked += 1
                                assert False
                            else:
                                num_confirmed_unblocked += 1
                                num_sensed_unblocked += 1

                            # Iterate over two mandatory neighbors and compute necessary things
                            for ind in range(len(two_mandatory_neighbors)):
                                relative_position = two_mandatory_neighbors[ind]
                                neighbor = (
                                    current_node[0] + relative_position[0], current_node[1] + relative_position[1])

                                # Factor is used to subtract the middle element
                                if ind == 0:
                                    factor = -1
                                else:
                                    factor = 1
                                num_sensed_blocked += factor * maze[neighbor[0]][neighbor[1]].num_sensed_blocked
                                num_confirmed_blocked += factor * maze[neighbor[0]][neighbor[1]].num_confirmed_blocked
                                num_sensed_unblocked += factor * maze[neighbor[0]][neighbor[1]].num_sensed_unblocked
                                num_confirmed_unblocked += factor * \
                                                           maze[neighbor[0]][neighbor[1]].num_confirmed_unblocked

                                if maze[neighbor[0]][neighbor[1]].is_blocked:
                                    num_confirmed_blocked += factor
                                    num_sensed_blocked += factor
                                    assert False
                                else:
                                    num_confirmed_unblocked += factor
                                    num_sensed_unblocked += factor

                            # Check whether we can infer from the current set of knowledge
                            if can_infer(num_sensed_blocked, num_confirmed_blocked, num_sensed_unblocked,
                                         num_confirmed_unblocked):

                                for relative_position in RELATIVE_POSITION_OF_TWO_SENSED_NEIGHBORS[index]:
                                    neighbor = (
                                        current_node[0] + relative_position[0], current_node[1] + relative_position[1])

                                    if check(neighbor) and (neighbor not in items_in_the_queue) and \
                                            (not maze[neighbor[0]][neighbor[1]].is_confirmed):
                                        items_in_the_queue.add(neighbor)
                                        inference_items.put(neighbor)

                            # Otherwise we will check the ambiguity and add it in our knowledge base
                            elif is_ambiguous(num_sensed_blocked, num_confirmed_blocked, num_sensed_unblocked,
                                              num_confirmed_unblocked):

                                # Initialise some variables
                                unconfirmed_cells = set()
                                num_blocked_cells_in_unconfirmed_cells = 0

                                # Compute relative position and create one set for that
                                for relative_position in RELATIVE_POSITION_OF_TWO_SENSED_NEIGHBORS[index]:
                                    neighbor = (
                                        current_node[0] + relative_position[0], current_node[1] + relative_position[1])
                                    if check(neighbor) and (not maze[neighbor[0]][neighbor[1]].is_confirmed):
                                        unconfirmed_cells.add(neighbor)
                                        num_blocked_cells_in_unconfirmed_cells += full_maze[neighbor[0]][neighbor[1]]

                                # write assert to check some conditions
                                assert num_blocked_cells_in_unconfirmed_cells == num_sensed_blocked - num_confirmed_blocked
                                assert len(unconfirmed_cells) >= num_sensed_blocked - num_confirmed_blocked

                                # Add constraint to the knowledge
                                add_constraint_to_knowledge_base(knowledge_base, variable_to_constraint_dict,
                                                                 unconfirmed_cells,
                                                                 num_blocked_cells_in_unconfirmed_cells)

                # If you want to use two cell inferences
                if want_to_use_two_node_inference_strategy:

                    # Iterate over four neighbors which are iterable
                    for index in range(len(X)):
                        neighbor = (current_node[0] + X[index], current_node[1] + Y[index])

                        # Check for valid and visited neighbors
                        if check(neighbor) and maze[neighbor[0]][neighbor[1]].is_visited:
                            num_not_confirmed_cells = 0

                            num_sensed_blocked = 0
                            num_confirmed_blocked = 0
                            num_sensed_unblocked = 0
                            num_confirmed_unblocked = 0

                            # Can comment the below two if and else
                            if maze[current_node[0]][current_node[1]].is_blocked:
                                num_confirmed_blocked += 1
                                num_sensed_blocked += 1
                                assert False
                            else:
                                num_confirmed_unblocked += 1
                                num_sensed_unblocked += 1

                            if maze[neighbor[0]][neighbor[1]].is_blocked:
                                num_confirmed_blocked -= 1
                                num_sensed_blocked -= 1
                                assert False
                            else:
                                num_confirmed_unblocked -= 1
                                num_sensed_unblocked -= 1

                            assert (num_confirmed_blocked == 0) and (num_confirmed_unblocked == 0)

                            # check the status of neighbors which has to be confirmed
                            for relative_position in RELATIVE_POSITION_OF_NEIGHBORS_TO_CHECK[index]:
                                cell = (current_node[0] + relative_position[0], current_node[1] + relative_position[1])
                                if check(cell):
                                    if not maze[cell[0]][cell[1]].is_confirmed:
                                        num_not_confirmed_cells += 1
                                    elif maze[cell[0]][cell[1]].is_blocked:
                                        num_confirmed_blocked += 1
                                        num_sensed_blocked += 1
                                    else:
                                        num_confirmed_unblocked += 1
                                        num_sensed_unblocked += 1

                            # If number of not confirmed cells is zero, only then we should move ahead
                            if num_not_confirmed_cells == 0:

                                # Do X2 - X1 for each attribute
                                num_sensed_blocked += maze[current_node[0]][current_node[1]].num_sensed_blocked - \
                                                      maze[neighbor[0]][neighbor[1]].num_sensed_blocked
                                num_confirmed_blocked += maze[current_node[0]][current_node[1]].num_confirmed_blocked - \
                                                         maze[neighbor[0]][neighbor[1]].num_confirmed_blocked
                                num_sensed_unblocked += maze[current_node[0]][current_node[1]].num_sensed_unblocked - \
                                                        maze[neighbor[0]][neighbor[1]].num_sensed_unblocked
                                num_confirmed_unblocked += maze[current_node[0]][
                                                               current_node[1]].num_confirmed_unblocked - \
                                                           maze[neighbor[0]][neighbor[1]].num_confirmed_unblocked

                                # Check the hypothesis
                                assert (num_sensed_blocked >= num_confirmed_blocked) and \
                                       (num_sensed_unblocked >= num_confirmed_unblocked)

                                # Check whether we can infer anything or not
                                if can_infer(num_sensed_blocked, num_confirmed_blocked, num_sensed_unblocked,
                                             num_confirmed_unblocked):

                                    for relative_position in RELATIVE_POSITION_OF_NEIGHBORS_TO_UPDATE[index]:
                                        cell = (
                                            current_node[0] + relative_position[0],
                                            current_node[1] + relative_position[1])

                                        # Update all unconfirmed cells into queue
                                        if check(cell) and (cell not in items_in_the_queue) and \
                                                (not maze[cell[0]][cell[1]].is_confirmed):
                                            items_in_the_queue.add(cell)
                                            inference_items.put(cell)

                                # Check this to make sure we will add valid things into our knowledge base
                                elif is_ambiguous(num_sensed_blocked, num_confirmed_blocked, num_sensed_unblocked,
                                                  num_confirmed_unblocked):

                                    # Initialise attributes to store it in our knowledge base
                                    unconfirmed_cells = set()
                                    num_blocked_cells_in_unconfirmed_cells = 0
                                    for relative_position in RELATIVE_POSITION_OF_NEIGHBORS_TO_UPDATE[index]:
                                        cell = (
                                            current_node[0] + relative_position[0],
                                            current_node[1] + relative_position[1])
                                        if check(cell) and (not maze[cell[0]][cell[1]].is_confirmed):
                                            unconfirmed_cells.add(cell)
                                            num_blocked_cells_in_unconfirmed_cells += full_maze[cell[0]][cell[1]]

                                    # Check some precondition
                                    assert num_blocked_cells_in_unconfirmed_cells == num_sensed_blocked - num_confirmed_blocked
                                    assert (len(unconfirmed_cells) >= (num_sensed_blocked - num_confirmed_blocked))

                                    # Add this constraint to our knowledge base
                                    add_constraint_to_knowledge_base(knowledge_base, variable_to_constraint_dict,
                                                                     unconfirmed_cells,
                                                                     num_blocked_cells_in_unconfirmed_cells)

        # This is the advance strategy using backtracking search
        if want_to_use_most_constraint_variable_for_backtracking_search:

            # Getting queue of cells which are inferred
            queue = make_inference_from_most_constraint_variable_list(knowledge_base, variable_to_constraint_dict,
                                                                      want_to_use_probability_approach=
                                                                      want_to_use_probability_approach, maze=maze,
                                                                      list_of_variables_in_the_path=
                                                                      list_of_variables_in_the_path)

            # Adding into queue until it gets empty
            while not queue.empty():
                current_node = queue.get()
                if current_node not in items_in_the_queue:
                    items_in_the_queue.add(current_node)
                    inference_items.put(current_node)

    return is_block_node_in_current_path

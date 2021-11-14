import numpy as np
import matplotlib.pyplot as plt

from helpers.helper import update_status


def generate_grid_manually():
    """
    This is the function to generate grid manually. This is helpful for the initial testing and problem 1.
    :return: Manually generated numpy array.
    """

    array = [[4, 1, 2, 2, 2], [1, 1, 2, 2, 2], [4, 1, 2, 2, 2], [4, 1, 2, 2, 2], [4, 1, 2, 2, 2]]
    return array


def forward_execution(maze: list, false_negative_rates: np.ndarray, maze_array: np.array, start_pos: tuple,
                      goal_pos: tuple, children: dict):
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

        update_status(maze, false_negative_rates, maze_array, cur_pos)

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


def plot_boxplot(data: list, title: str, titles_for_each_plot: list):
    fig, ax = plt.subplots(3)
    for ind in range(len(data)):
        ax[ind].boxplot(data[ind], patch_artist=True, notch='True', vert=0)
        # ax[ind].set_yticklabels(legends[ind])
        ax[ind].set_title(titles_for_each_plot[ind])
    fig.suptitle(title)
    # fig.text(0.5, 0.04, 'Values', ha='center')
    fig.tight_layout()
    # plt.savefig('tmp.png')
    plt.show()


def plot_histogram(data: list, labels: list, title: str):
    plt.style.use('seaborn-deep')

    # x = np.random.normal(1, 2, 5000)
    # y = np.random.normal(-1, 3, 2000)
    # bins = np.linspace(-10, 10, 30)

    plt.hist(data, bins=20, label=labels)
    plt.title(title)
    plt.legend(loc='upper right')
    plt.show()

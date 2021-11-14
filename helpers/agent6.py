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
    :param false_negative_rates: numpy array which shows false negative rates of each cell
    :param maze_array: Original (Full) Maze array
    :param goal_pos: Goal position to reach
    :param children: children dictionary to move along the path
    :param start_pos: starting position of the maze from where agent want to start
    :return: This function will return final paths on which agent moved to reach goal or empty list if agent can't find
            path to goal. Second is total number of processed nodes while running the algorithm.
    """

    # Setting current position to starting position so we can start iterating from start_pos
    cur_pos = start_pos

    current_path = [cur_pos]

    # Iterating from start_pos to goal_pos if we won't get any blocks in between otherwise we are terminating the
    # iteration.
    while True:

        # Update the status of the current cell
        update_status(maze, false_negative_rates, maze_array, cur_pos)
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

    return current_path


def plot_boxplot(data: list, title: str, titles_for_each_plot: list):
    """
    Function is used to plot boxplot for agent 6,7 and 8
    :param data: list of list containing data
    :param title: title for the box plot
    :param titles_for_each_plot: title for each subplot
    :return: None
    """
    fig, ax = plt.subplots(3)
    for ind in range(len(data)):
        ax[ind].boxplot(data[ind], patch_artist=True, notch='True', vert=0)
        ax[ind].set_title(titles_for_each_plot[ind])
    fig.suptitle(title)
    fig.tight_layout()
    plt.show()


def plot_histogram(data: list, labels: list, title: str):
    """
    Function is used to plot histogram for agent 6,7 and 8
    :param data: list of list containing data
    :param labels: labels of histogram
    :param title: main title of the histogram
    :return: None
    """
    plt.style.use('seaborn-deep')
    plt.hist(data, bins=20, label=labels)
    plt.title(title)
    plt.legend(loc='upper right')
    plt.show()

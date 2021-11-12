import random
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from constants import X, Y
from helpers.helper import check


def move_target(current_position_of_target: tuple, full_maze: np.array):

    possible_positions_to_move = list()
    for ind in range(len(X)):
        neighbor = (current_position_of_target[0] + X[ind], current_position_of_target[1] + Y[ind])
        if check(neighbor) and full_maze[neighbor[0]][neighbor[1]] != 1:
            possible_positions_to_move.append(neighbor)

    # assert len(possible_positions_to_move) >= 1
    return possible_positions_to_move[random.randint(0, len(possible_positions_to_move) - 1)]


def plot_boxplot(data: list, title: str, titles_for_each_plot: list, legends: list):
    fig, ax = plt.subplots(3)
    for ind in range(len(legends)):
        ax[ind].boxplot(data[legends[ind]], patch_artist=True, notch='True', vert=0)
        # ax[ind].set_yticklabels(legends[ind])
        ax[ind].set_title(titles_for_each_plot[ind])
    fig.suptitle(title)
    # fig.text(0.5, 0.04, 'Values', ha='center')
    fig.tight_layout()
    # plt.savefig(IMG_PATH + filename)
    plt.show()


def plot_histogram(data: dict):
    fig = plt.figure(figsize=(10, 5))
    gs = GridSpec(nrows=2, ncols=2)

    ax0 = fig.add_subplot(gs[0, 0])
    ax0.hist(data['total_movements'], bins=100)
    ax0.set_title('Total Movements')

    ax1 = fig.add_subplot(gs[0, 1])
    ax1.hist(data['total_examinations'], bins=100)
    ax1.set_title('Total Examinations')

    ax2 = fig.add_subplot(gs[1, :])
    ax2.hist(data['total_cost'], bins=100)
    ax2.set_title('Total Actions')

    fig.tight_layout()
    fig.suptitle('Histograms')
    # plt.savefig(IMG_PATH + filename)
    # plt.legend(loc='upper right')
    plt.show()
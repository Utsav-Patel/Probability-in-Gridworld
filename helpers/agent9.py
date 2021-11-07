import random
import numpy as np
from constants import X, Y
from helpers.helper import check


def move_target(current_position_of_target: tuple, full_maze: np.array):

    possible_positions_to_move = list()
    for ind in range(len(X)):
        neighbor = (current_position_of_target[0] + X[ind], current_position_of_target[1] + Y[ind])
        if check(neighbor) and full_maze[neighbor[0]][neighbor[1]] != 1:
            possible_positions_to_move.append(neighbor)

    assert len(possible_positions_to_move) >= 1
    return possible_positions_to_move[random.randint(0, len(possible_positions_to_move) - 1)]

import random
import numpy as np
from constants import X, Y, NUM_ROWS, NUM_COLS


def check(current_position: tuple):
    """
    Check whether current point is in the grid or not
    :param current_position: current point
    :return: True if the current point is in the grid otherwise False
    """
    if (0 <= current_position[0] < NUM_ROWS) and (0 <= current_position[1] < NUM_COLS):
        return True
    return False


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


def generate_target_position(full_maze: list):
    while True:
        x = random.randint(0, len(full_maze) - 1)
        y = random.randint(0, len(full_maze) - 1)
        if full_maze[x][y] == 1:
            continue
        return x, y


def move_target(current_position_of_target: tuple, full_maze: np.array):

    possible_positions_to_move = list()
    for ind in range(len(X)):
        neighbor = (current_position_of_target[0] + X[ind], current_position_of_target[1] + Y[ind])
        if check(neighbor) and full_maze[neighbor[0]][neighbor[1]] != 1:
            possible_positions_to_move.append(neighbor)

    assert len(possible_positions_to_move) >= 1
    return possible_positions_to_move[random.randint(0, len(possible_positions_to_move) - 1)]

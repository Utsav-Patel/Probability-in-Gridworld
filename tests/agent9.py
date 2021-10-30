from helpers.agent9 import generate_target_position, generate_grid_with_probability_p
from constants import PROBABILITY_OF_GRID


def find_moving_target():
    full_maze = generate_grid_with_probability_p(PROBABILITY_OF_GRID)
    target_position = generate_target_position(full_maze)
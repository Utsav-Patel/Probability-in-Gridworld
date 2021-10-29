# This file contains the constants which is used in the project

import numpy as np

NUM_ROWS = 3
NUM_COLS = 3
INF = 1e9

X = [1, 0, -1, 0]
Y = [0, 1, 0, -1]

# Change this path to other path where you want to store your images
IMG_PATH = "C:\\Users\\Gambit\\Documents\\Rutgers\\Rutgers 520\\Assignment 3\\Graphs"

STARTING_POSITION_OF_AGENT = (0, 0)

GOAL_POSITION_OF_AGENT = (NUM_ROWS-1, NUM_COLS-1)

NUM_ITERATION_FOR_EACH_PROBABILITY = 50
#NUM_UNIFORM_SAMPLE_FOR_EACH_PROBABILITY = 34
#START_PROBABILITY = 0.0
#END_PROBABILITY = 0.33
PROBABILITY_OF_GRID = 0.3
#LIST_OF_PROBABILITIES = np.linspace(START_PROBABILITY, END_PROBABILITY, NUM_UNIFORM_SAMPLE_FOR_EACH_PROBABILITY)

# Constants for inference of three neighbors

RELATIVE_POSITION_OF_TWO_MANDATORY_NEIGHBORS = list()
RELATIVE_POSITION_OF_TWO_SENSED_NEIGHBORS = list()

for ind in range(len(X)):
    two_neighbors = list()
    two_neighbors.append((X[ind], Y[ind]))
    two_neighbors.append((X[ind] * 2, Y[ind] * 2))
    RELATIVE_POSITION_OF_TWO_MANDATORY_NEIGHBORS.append(two_neighbors)

    neighbors_to_check = list()
    for position in np.arange(-1, 4, 2):
        for factor in np.arange(-1, 2, 1):
            neighbors_to_check.append((X[ind] * position + abs(Y[ind]) * factor, Y[ind] * position + abs(X[ind]) * factor))

    RELATIVE_POSITION_OF_TWO_SENSED_NEIGHBORS.append(neighbors_to_check)

# Constants for inference of two neighbors

RELATIVE_POSITION_OF_NEIGHBORS_TO_CHECK = list()
RELATIVE_POSITION_OF_NEIGHBORS_TO_UPDATE = list()

for ind in range(len(X)):
    neighbors_to_update = list()
    neighbors_to_check = list()
    for val in np.arange(-1, 2, 1):
        neighbors_to_update.append((-1 * X[ind] + abs(Y[ind]) * val, -1 * Y[ind] + abs(X[ind]) * val))
        neighbors_to_check.append((2 * X[ind] + abs(Y[ind]) * val, 2 * Y[ind] + abs(X[ind]) * val))

    RELATIVE_POSITION_OF_NEIGHBORS_TO_CHECK.append(neighbors_to_check)
    RELATIVE_POSITION_OF_NEIGHBORS_TO_UPDATE.append(neighbors_to_update)


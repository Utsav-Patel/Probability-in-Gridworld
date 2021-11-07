# import necessary requirements

import time
import numpy as np

from constants import STARTING_POSITION_OF_AGENT, INF, PROBABILITY_OF_GRID, NUM_ROWS, NUM_COLS, NUM_ITERATIONS
from helpers.agent8 import set_random_target, examine_and_propagate_probability
from helpers.helper import generate_grid_with_probability_p, compute_explored_cells_from_path, add_fractions, \
    length_of_path_from_source_to_goal

from src.Agent8 import Agent8                                                       # make changes in Agent8 src

agent = Agent8()                                                                    # Initialize Agent8


def find_the_target():
    """
    Function to find the target
    :return: results containing final sum of probabilities of each cell(containing), examinations, movements
    """
    global p                                                                # contains sum of probabilities of each cell
    agent.reset()                                                           # reset all the attributes of Agent8
    target_found = False                                                    # used to check if target if found or not
    agent_num = 7                                                           # indicates which type of prob to use
    result = list()                                                         # list that contains final results

    # loop for find a "reachable" target
    while True:
        random_maze = generate_grid_with_probability_p(PROBABILITY_OF_GRID)  # generate gridworld with full information
        target_pos = set_random_target()                                     # setting the target randomly

        # loop till target isn't blocked
        while random_maze[target_pos[0]][target_pos[1]] != 4:
            target_pos = set_random_target()  # setting the target randomly
        # check if target is reachable or by finding path length to it from start
        if length_of_path_from_source_to_goal(random_maze, STARTING_POSITION_OF_AGENT, target_pos) != INF:
            break

    # reachable target is set, so now reset all the variables of the Agent8
    agent.reset()
    # print("Main Here")

    # loop target_found is FALSE

    while not target_found:
        agent.pre_planning(agent_num)                                   # set the agent.current_estimated_goal attribute

        # find path from agent.current_position to agent.current_estimates_goal using A* and set parents dict()
        agent.planning(agent.current_estimated_goal)

        # loop till the path given by Astar contains agent.current_estimated_goal
        while agent.current_estimated_goal not in agent.parents:
            # not in path so make it blocked and examine it and propagate the probability to all the cells accordingly.
            agent.maze[agent.current_estimated_goal[0]][agent.current_estimated_goal[1]].is_blocked = True
            examine_and_propagate_probability(agent.maze, random_maze, agent.current_position, target_pos,
                                              agent.current_estimated_goal, agent.current_estimated_goal)

            agent.pre_planning(agent_num)                                       # set new agent.current_estimated_target

            # find path from agent.current_position to new agent.current_estimates_goal using A*
            agent.planning(agent.current_estimated_goal)

        # out of loop means a "reachable" agent.current_estimated_goal is found
        # execution and examine k% cells with top probabilities.
        target_found = agent.execution(random_maze, target_pos)

        p = 0.0

        # compute final sum of probabilities
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                p += agent.maze[row][col].probability_of_containing_target
        print("Total Probability:", p)

    movements = compute_explored_cells_from_path(agent.final_paths)
    result.append(p)
    result.append(agent.num_examinations)
    result.append(movements)
    return result


if __name__ == "__main__":
    results = find_the_target()

    print("Sum of Probability:", results[0])
    print("Total examinations:", results[1])
    print("Total movements:", results[2])
    print("Total cost:", (results[1]+results[2]))

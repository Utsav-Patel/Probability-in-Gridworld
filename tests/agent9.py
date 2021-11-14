"""
This is the test file for agent 9.
"""

# Necessary imports
from datetime import datetime
import multiprocessing
import numpy as np
import pickle

from helpers.helper import generate_grid_with_probability_p, length_of_path_from_source_to_goal,\
    generate_target_position, examine_and_propagate_probability, compute_explored_cells_from_path
from helpers.agent9 import move_target
from constants import PROBABILITY_OF_GRID, STARTING_POSITION_OF_AGENT, INF, NUM_ITERATIONS
from src.Agent9 import Agent9


def find_moving_target(num: int):
    """
    This function is used to test agent 9
    :param num: iteration number
    :return: (movements, examinations, actions)
    """

    # Start running agent 9
    print('Running for', num)
    agent = Agent9()
    agent_num = 9

    # Keep generating grid and target position until we will get valid pairs.
    while True:
        full_maze = generate_grid_with_probability_p(PROBABILITY_OF_GRID)
        target_position = generate_target_position(full_maze)
        if length_of_path_from_source_to_goal(full_maze, STARTING_POSITION_OF_AGENT, target_position) != INF:
            break

    target_found = False

    # Run the following while loop until we will get the target
    while not target_found:

        # First, agent will sense its neighbor and update its belief and prediction for the next step
        agent.sense(target_position)

        # Second, agent will find an estimated goal using the prediction probability
        agent.pre_planning(agent_num)

        # Third, agent will plan a path to reach an estimated goal
        agent.planning(agent.current_estimated_goal)

        # If the current estimated goal is not reachable, agent will assign the probability to zero and redo
        # pre-planning and planning until it will get a reachable target
        while (not agent.is_target_in_neighbors) and (agent.current_estimated_goal not in agent.parents):
            agent.maze[agent.current_estimated_goal[0]][agent.current_estimated_goal[1]].is_blocked = True
            examine_and_propagate_probability(agent.maze, agent.probability_of_containing_target_next_step,
                                              agent.false_negative_rates, agent.current_position, target_position,
                                              agent.current_estimated_goal, agent.current_estimated_goal)
            agent.pre_planning(agent_num)
            agent.planning(agent.current_estimated_goal)

        # Then, next time step will come and target will move one step
        target_position = move_target(target_position, full_maze)

        # Agent will start its execution and examination
        target_found = agent.execution(full_maze, target_position)

    # Compute number of movements
    movements = compute_explored_cells_from_path(agent.final_paths)

    return agent.num_examinations, movements, agent.num_examinations + movements


if __name__ == "__main__":

    # Initialize empty list
    total_examinations = list()
    total_movements = list()
    total_cost = list()

    # start time
    start_time = datetime.now()

    # Find total number of cpu
    n_cores = int(multiprocessing.cpu_count())
    print('Number of cores', n_cores)
    p = multiprocessing.Pool(processes=n_cores)

    # Run total NUM_ITERATIONS times
    results = p.imap_unordered(find_moving_target, range(NUM_ITERATIONS))

    # Store results in final results
    for result in results:
        total_examinations.append(result[0])
        total_movements.append(result[1])
        total_cost.append(result[2])

    # Dump result in pickle file
    with open('../data/agent9_data.pkl', 'wb') as f:
        pickle.dump({'total_cost': total_cost, 'total_examinations': total_examinations, 'total_movements':
            total_movements}, f)

    # Print final results
    end_time = datetime.now()
    print("Average Number of movements of agent 9 = ", np.average(total_movements))
    print("Average Number of total examinations of agent 9= ", np.average(total_examinations))
    print("Average cost of agent 9 = ", np.average(total_cost))
    print(f"Runtime = {end_time - start_time}")

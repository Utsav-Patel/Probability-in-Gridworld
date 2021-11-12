import time
import numpy as np
import multiprocessing
from datetime import datetime

from constants import STARTING_POSITION_OF_AGENT, INF, PROBABILITY_OF_GRID, NUM_ROWS, NUM_COLS, NUM_ITERATIONS
from helpers.helper import generate_grid_with_probability_p, compute_explored_cells_from_path, \
    length_of_path_from_source_to_goal, examine_and_propagate_probability, plot_boxplot, generate_target_position
from src.Agent6 import Agent6

agent = Agent6()
legends = ['Agent6', 'Agent7', 'Agent8']


def find_the_target(num: int):
    print('Running for:', num)
    agent.reset()
    agents = [6, 7, 8]
    x = list()
    # cnt = 0
    while True:
        random_maze = generate_grid_with_probability_p(PROBABILITY_OF_GRID)
        target_pos = generate_target_position(random_maze)
        # random_maze = generate_grid_manually()
        # target_pos = (0, 0)
        if length_of_path_from_source_to_goal(random_maze, STARTING_POSITION_OF_AGENT, target_pos) != INF:
            break

    for agent_num in agents:
        print('Starting agent', agent_num)
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        print("date and time =", dt_string)

        agent.reset()
        target_found = False
        while not target_found:

            agent.pre_planning(agent_num)

            agent.planning(agent.current_estimated_goal)
            while agent.current_estimated_goal not in agent.parents:
                agent.maze[agent.current_estimated_goal[0]][agent.current_estimated_goal[1]].is_blocked = True
                examine_and_propagate_probability(agent.maze, agent.current_position, target_pos,
                                                  agent.current_estimated_goal, agent.current_estimated_goal)
                agent.pre_planning(agent_num)
                agent.planning(agent.current_estimated_goal)
                # print('Full Maze')
                # print(random_maze)
                #
                # for row in range(NUM_ROWS):
                #     for col in range(NUM_COLS):
                #         print(agent.maze[row][col].probability_of_containing_target, end=" ")
                #     print()
                #
                # print("Current estimated goal:", agent.current_estimated_goal)
                # print("Current Position:", agent.current_position)
                # print("Parents:", agent.parents)

            agent.execution(random_maze)

            target_found = agent.examine(random_maze, target_pos)

            p = 0.0
            for row in range(NUM_ROWS):
                for col in range(NUM_COLS):
                    p += agent.maze[row][col].probability_of_containing_target

        # print('Total counts', cnt)
        movements = compute_explored_cells_from_path(agent.final_paths)
        x.append([p, agent.num_examinations, movements])
        # return [agent_num,p, agent.num_examinations, movements]

        print('ending agent', agent_num)
        now = datetime.now()
        dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
        print("date and time =", dt_string)
    return x


if __name__ == "__main__":

    total_p_6 = list()
    total_examinations_6 = list()
    total_movements_6 = list()
    total_cost_6 = list()

    total_p_7 = list()
    total_examinations_7 = list()
    total_movements_7 = list()
    total_cost_7 = list()

    total_p_8 = list()
    total_examinations_8 = list()
    total_movements_8 = list()
    total_cost_8 = list()

    start_time = time.time()

    n_cores = int(multiprocessing.cpu_count())

    # print('Number of cores', n_cores)
    p = multiprocessing.Pool(processes=n_cores)

    results = p.imap_unordered(find_the_target, range(NUM_ITERATIONS))

    for result in results:
        # print(result)
        total_p_6.append(result[0][0])
        total_examinations_6.append(result[0][1])
        total_movements_6.append(result[0][2])
        total_cost_6.append(total_examinations_6[-1] + total_movements_6[-1])

        total_p_7.append(result[1][0])
        total_examinations_7.append(result[1][1])
        total_movements_7.append(result[1][2])
        total_cost_7.append(total_examinations_7[-1] + total_movements_7[-1])

        total_p_8.append(result[2][0])
        total_examinations_8.append(result[2][1])
        total_movements_8.append(result[2][2])
        total_cost_8.append(total_examinations_8[-1] + total_movements_8[-1])

    # plot_boxplot([total_cost_6, total_cost_7, total_cost_8], 'boxplot for total cost', legends, 'total_cost.png')

    end_time = time.time()
    print("Average Number of movements of agent 6 = ", np.average(total_movements_6))
    print("Average total probability of agent 6 = ", np.average(total_p_6))
    print("Average Number of total examinations of agent 6 = ", np.average(total_examinations_6))
    print("Total average cost of agent 6 = ", np.average(total_movements_6) + np.average(total_examinations_6))
    print("Average Number of movements of agent 7 = ", np.average(total_movements_7))
    print("Average total probability of agent 7 = ", np.average(total_p_7))
    print("Average Number of total examinations = 7", np.average(total_examinations_7))
    print("Total average cost of agent 7 = ", np.average(total_movements_7) + np.average(total_examinations_7))
    print("Average Number of movements of agent 8 = ", np.average(total_movements_8))
    print("Average total probability of agent 8 = ", np.average(total_p_8))
    print("Average Number of total examinations of agent 8 = ", np.average(total_examinations_8))
    print("Total average cost of agent 8 = ", np.average(total_movements_8) + np.average(total_examinations_8))
    print(f"Runtime = {end_time - start_time}")

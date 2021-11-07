from datetime import datetime
from helpers.helper import generate_grid_with_probability_p, length_of_path_from_source_to_goal,\
    generate_target_position, examine_and_propagate_probability, compute_explored_cells_from_path
from helpers.agent9 import move_target
from constants import PROBABILITY_OF_GRID, STARTING_POSITION_OF_AGENT, INF, NUM_ROWS, NUM_COLS
from src.Agent9 import Agent9

agent = Agent9()
agent_num = 9


def find_moving_target():
    agent.reset()
    while True:
        full_maze = generate_grid_with_probability_p(PROBABILITY_OF_GRID)
        target_position = generate_target_position(full_maze)
        if length_of_path_from_source_to_goal(full_maze, STARTING_POSITION_OF_AGENT, target_position) != INF:
            break

    print('Starting agent', agent_num)
    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    print("date and time =", dt_string)

    agent.reset()
    target_found = False

    print('Original Maze')
    print(full_maze)

    # for row in range(NUM_ROWS):
    #     for col in range(NUM_COLS):
    #         print(agent.maze[row][col].four_neighbors, end=" ")
    #     print()

    while not target_found:

        print('Agent current position', agent.current_position)
        print('Target current position', target_position)

        agent.sense(target_position)
        agent.pre_planning(agent_num)

        agent.planning(agent.current_estimated_goal)
        while agent.current_estimated_goal not in agent.parents:
            agent.maze[agent.current_estimated_goal[0]][agent.current_estimated_goal[1]].is_blocked = True
            examine_and_propagate_probability(agent.maze, full_maze, agent.current_position, target_position,
                                              agent.current_estimated_goal, agent.current_estimated_goal)
            agent.pre_planning(agent_num)
            agent.planning(agent.current_estimated_goal)

        target_position = move_target(target_position, full_maze)
        target_found = agent.execution(full_maze, target_position)

        # if agent.current_position == target_position:
        #     print(agent.is_target_in_neighbors)
        #     print(target_found)
        #     input()
        # target_found = agent.examine(random_maze, target_pos)

        p = 0.0
        p_next = 0.0
        # print('Probability of containing target')
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                p += agent.maze[row][col].probability_of_containing_target
                p_next += agent.maze[row][col].probability_of_containing_target_next_step
                # print(format(agent.maze[row][col].probability_of_containing_target, ".5f"), end=" ")
            # print()

        # print('Probability of containing target for next moment')
        # for row in range(NUM_ROWS):
        #     for col in range(NUM_COLS):
        #         print(format(agent.maze[row][col].probability_of_containing_target_next_step, ".5f"), end=" ")
        #     print()

        print('Total probability', p)
        print('Total probability next', p_next)

        # input()

    # print('Total counts', cnt)
    movements = compute_explored_cells_from_path(agent.final_paths)
    # x.append([p, agent.num_examinations, movements])
    # return [agent_num,p, agent.num_examinations, movements]

    print('Number of examinations', agent.num_examinations)
    print('Number of movements', movements)

    print('Total costs', agent.num_examinations + movements)

    print('ending agent', agent_num)
    now = datetime.now()
    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
    print("date and time =", dt_string)

find_moving_target()
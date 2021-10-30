from constants import NUM_COLS, NUM_ROWS, STARTING_POSITION_OF_AGENT, INF
from helpers.agent6 import generate_grid_with_probability_p, set_random_target, generate_grid_manually, add_fractions, length_of_path_from_source_to_goal,\
examine_and_propogate_probability, compute_explored_cells_from_path
from src.Agent6 import Agent6
import time

import numpy as np

def check_random_maze():
    random_maze = generate_grid_with_probability_p(0.3)
    print("Number of Starting positions = ", random_maze[random_maze == 0].shape[0])
    print("Number of 1 = ", random_maze[random_maze == 1].shape[0])
    print("Number of 2 = ", random_maze[random_maze == 2].shape[0])
    print("Number of 3 = ", random_maze[random_maze == 3].shape[0])
    print("Number of 4 = ", random_maze[random_maze == 4].shape[0])


# =============================================================================
# def generate_maze_from_array(maze_array):
#     
#     maze = Maze(NUM_COLS, NUM_ROWS)
#     
#     for row in range(NUM_ROWS):
#             for col in range(NUM_COLS):
#                 if maze_array[row][col] == 1:
#                     maze.maze[row][col].is_blocked = True
#                 elif maze_array[row][col] == 2:
#                     maze.maze[row][col].is_flat = True
#                 elif maze_array[row][col] == 3:
#                     maze.maze[row][col].is_forest = True
#                 elif maze_array[row][col] == 4:
#                     maze.maze[row][col].is_hilly = True
#                 else:
#                     maze.maze[row][col].is_blocked = False
#                     
#     GOAL_POSITION_OF_AGENT = random_target()
#     while (maze_array[GOAL_POSITION_OF_AGENT[0]][GOAL_POSITION_OF_AGENT[1]] == 1):
#            GOAL_POSITION_OF_AGENT = random_target()
#     maze.maze[GOAL_POSITION_OF_AGENT[0]][GOAL_POSITION_OF_AGENT[1]].is_goal = True
#     return maze
#     
# random_maze = generate_grid_with_probability_p(0.3)
# final_maze = generate_maze_from_array(random_maze)
# =============================================================================

agent = Agent6()


def find_the_target():
    agent.reset()
    target_found = False
    
    
    while(True):
        #random_maze = generate_grid_manually()
        random_maze = generate_grid_with_probability_p(0.3)
        while random_maze[STARTING_POSITION_OF_AGENT[0]][STARTING_POSITION_OF_AGENT[1]] == 1:
            random_maze = generate_grid_with_probability_p(0.3)
        target_pos = set_random_target()
        while random_maze[target_pos[0]][target_pos[1]] == 1:
            target_pos = set_random_target()
        #print("Target pos =", target_pos)
        if(length_of_path_from_source_to_goal(random_maze, STARTING_POSITION_OF_AGENT, target_pos) != INF):
            break
        
    print("done setting maze and target")
    while not target_found:
        agent.pre_planning()
        while length_of_path_from_source_to_goal(random_maze, STARTING_POSITION_OF_AGENT, agent.current_estimated_goal) == INF:
            agent.maze[agent.current_estimated_goal[0]][agent.current_estimated_goal[1]].is_blocked = True
            examine_and_propogate_probability(agent.maze, random_maze, agent.current_position, target_pos, agent.current_estimated_goal, agent.current_estimated_goal)
            agent.pre_planning()
        #print("Current Estimated Goal =", agent.current_estimated_goal)
        agent.planning(agent.current_estimated_goal)
        #print("Current pos before execution =", agent.current_position)
        #print("Current Estimated Goal =", agent.current_estimated_goal)
        # print(agent.parents)
        # print(agent.num_cells_processed_while_planning)
  
        agent.execution(random_maze)
        #print("Target pos =", target_pos)
        #print("Current pos =", agent.current_position)
        
        # print("Final path = ",agent.final_paths)
  
        target_found = agent.examine(random_maze, target_pos)
    
        #print("Target pos =", target_pos)
        #print("Current pos =", agent.current_position)
        #print("Was target found ?", target_found)
        p = [0, 1]
        for row in range(NUM_ROWS):
            for col in range(NUM_COLS):
                p = add_fractions(p, agent.maze[row][col].probability_of_containing_target)
                    # print(agent.maze[row][col].probability_of_containing_target)
      
        #print("Total Probability =", p[0] / p[1])
        #if agent.num_cells_processed_while_planning > 200:
         #   break
    movements = compute_explored_cells_from_path(agent.final_paths)
    #print("Number of total examinations = ",agent.num_examinations)
    #print("Number of movements = ", movements)
    return [p, agent.num_examinations, movements, agent.num_astar_calls]

#find_the_target()
n_sim = 2
total_p = []
total_examinations = []
total_movements = []
total_astar = []
start_time = time.time()
for i in range(1,n_sim):
    total = find_the_target()
    total_p.append(total[0])
    total_examinations.append(total[1])
    total_movements.append(total[2])
    total_astar.append(total[3])
    
end_time = time.time()
print("Average Number of movements = ", np.average(total_movements))    
print("Average total probability = ", np.average(total_p)) 
print("Average Number of total examinations = ", np.average(total_examinations)) 
print("Average Number of Astar calls = ", np.average(total_astar)) 
print("total examinations divided by astar calls =",np.average(total_examinations)/np.average(total_astar))
print(f"Runtime = {end_time - start_time}")
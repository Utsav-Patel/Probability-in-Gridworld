from constants import INF, STARTING_POSITION_OF_AGENT, IMG_PATH, \
    NUM_ITERATION_FOR_EACH_PROBABILITY, NUM_COLS, NUM_ROWS
from helpers.agent6 import generate_grid_with_probability_p, random_target, compute_current_estimated_goal
from src.Agent import Agent
from src.Agent6 import Agent6



def check_random_maze():
    
    random_maze = generate_grid_with_probability_p(0.3)
    print("Number of Starting positions = ", random_maze[random_maze==0].shape[0])
    print("Number of 1 = ", random_maze[random_maze==1].shape[0])
    print("Number of 2 = ", random_maze[random_maze==2].shape[0])
    print("Number of 3 = ", random_maze[random_maze==3].shape[0])
    print("Number of 4 = ", random_maze[random_maze==4].shape[0])


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
agent.pre_planning()
agent.current_estimated_goal

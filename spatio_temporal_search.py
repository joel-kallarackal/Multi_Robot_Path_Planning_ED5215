import random
from queue import PriorityQueue
import copy
from GridWorld import GridWorld
from GridWorld3D import GridWorld3D
import numpy as np


def generate_unique_positions(total_count, grid_width, grid_height, existing_positions=set()):
    """Generate `total_count` unique positions avoiding `existing_positions`."""
    positions = set(existing_positions)
    while len(positions) < len(existing_positions) + total_count:
        pos = (random.randint(0, grid_width-1), random.randint(0, grid_height-1))
        if pos not in positions:  
            positions.add(pos)
    return list(positions - existing_positions) 

def generate_problem(grid_width, grid_height,n_bots=3,n_obstacles=0):
    # starts = [(random.randint(0, grid_width-1),random.randint(0, grid_height-1)) for i in range(n_bots)]
    # goals = [(random.randint(0, grid_width-1),random.randint(0, grid_height-1)) for i in range(n_bots)]
    
    # obstacles = [(random.randint(0, grid_width-1),random.randint(0, grid_height-1)) for i in range(n_obstacles)]
    
    starts = generate_unique_positions(n_bots, grid_width, grid_height)
    goals = generate_unique_positions(n_bots, grid_width, grid_height, existing_positions=set(starts))
    obstacles = generate_unique_positions(n_obstacles, grid_width, grid_height, existing_positions=set(starts) | set(goals))


    
    return starts, goals, obstacles

def initialise_grid(grid_width, grid_height,n_bots=3,n_obstacles=0):
    starts, goals, obstacles = generate_problem(grid_width, grid_height, n_bots, n_obstacles)
    grid = GridWorld(grid_width, grid_height, obstacles)
    return grid, starts, goals
    
def a_star(grid, start, goal, costs, heuristic):
    path = [] #this should contai list of nodes [start, (20,30), (21,30), ...., goal] as a path from start to goal
    visited = []
    parent = {start:None}
    curr_costs = {start:0}
    
    fringe = PriorityQueue()
    fringe.put((0+get_heuristic_cost(goal, start, heuristic), 0, get_heuristic_cost(goal, start, heuristic), start))
    
    while fringe:
        total, g, h, curr_node = fringe.get()
        
        if curr_node == goal:
            node = curr_node
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1]
        
        if curr_node in visited:
            continue
        
        visited.append(curr_node)
        
        neighbors = grid.get_neighbors(curr_node[0], curr_node[1])
        for neighbor in neighbors:
            new_cost = g+1 #get_edge_cost(costs, curr_node, neighbor)
            new_cost_total = new_cost+get_heuristic_cost(goal, neighbor, heuristic)
            if neighbor not in curr_costs or (curr_costs[neighbor])>new_cost:
                fringe.put((new_cost_total, new_cost, get_heuristic_cost(goal, neighbor, heuristic), neighbor))
                curr_costs[neighbor] = new_cost
                parent[neighbor] = curr_node
    
    
    return path

def get_heuristic_cost(goal, node, heuristic):
    if heuristic==0:
        return manhattan_heuristice(goal, node)
    if heuristic==1:
        return euclidean_heuristic(goal, node)
    if heuristic>=2:
        return heuristic*manhattan_heuristice(goal, node)
    
def euclidean_heuristic(goal, node):
    distance = ((goal[0]-node[0])**2 + (goal[1]-node[1])**2)**0.5 #update this variable and return the calcuated/updated value
    return distance

def manhattan_heuristice(goal, node):
    distance = abs(goal[0]-node[0])+abs(goal[1]-node[1]) #update this variable and return the calcuated/updated value
    return distance

def a_star_3D(grid, start, goal, costs, heuristic):
    path = []
    visited = []
    parent = {start:None}
    curr_costs = {start:0}
    
    fringe = PriorityQueue()
    fringe.put((0+get_heuristic_cost(goal, start, heuristic), 0, get_heuristic_cost(goal, start, heuristic), start))
    count = 0
    while fringe:
        total, g, h, curr_node = fringe.get()
        count+=1
        if (curr_node[0],curr_node[1]) == goal:
            node = curr_node
            while node is not None:
                path.append(node)
                node = parent[node]
            
            return path[::-1]
        
        if curr_node in visited:
            continue
        
        visited.append(curr_node)
        
        neighbors = grid.get_neighbors(curr_node[0], curr_node[1], curr_node[2])
        for neighbor in neighbors:
            new_cost = g+1
            new_cost_total = new_cost+get_heuristic_cost(goal, neighbor, heuristic)
            if neighbor not in curr_costs or (curr_costs[neighbor])>new_cost:
                fringe.put((new_cost_total, new_cost, get_heuristic_cost(goal, neighbor, heuristic), neighbor))
                curr_costs[neighbor] = new_cost
                parent[neighbor] = curr_node
    return path

def find_paths(grid,starts,goals):
    paths = [a_star(grid,starts[i],goals[i],"",0) for i in range(len(starts))]
    longest_path_index = 0
    longest_path_len=-1
    for i in range(len(paths)):
        if len(paths[i])>longest_path_len:
            longest_path_len = len(paths[i])
            longest_path_index = copy.copy(i)
        # print(f"Start: {starts[i]}, Goal: {goals[i]}")
        # print(f"Path {i+1}:", paths[i])
        
    # print((paths[longest_path_index][i][0],paths[longest_path_index][i][1]))
    spatio_temporal_grid = []
    for i in range(longest_path_len):
        obstacles = copy.copy(grid.obstacles)
        obstacles.append((paths[longest_path_index][i][0],paths[longest_path_index][i][1]))
        spatio_temporal_grid.append(GridWorld(grid.width, grid.height, obstacles).grid)
        grid3d = GridWorld3D(spatio_temporal_grid)
        
    final_paths = []
    for j in range(len(starts)):
        path = None
        if j!=longest_path_index:
            path = a_star_3D(grid3d,(starts[j][0],starts[j][1],0),goals[j],"",0)
            for i in range(len(path)-1,longest_path_len):
                grid3d.grid[i][goals[j][1],goals[j][0]] = 1
            for m in range(len(path)):
                grid3d.grid[m][path[m][1], path[m][0]] = 1
                if m<(len(path)-2): # Might work to remove moving into each other, might be wrong
                    grid3d.grid[m][path[m+1][1], path[m+1][0]] = 1
            final_paths.append(path)
            print(f"Start: {starts[j]}, Goal: {goals[j]}")
            print(f"Spatio Temporal Path {j+1}:", path)
        # else: 
        #     path = paths[j]
        #     for k in range(len(path)):
        #         path[k] = (path[k][0],path[k][1],k)
        #     final_paths.append(paths[j])
        #     print(f"Start: {starts[j]}, Goal: {goals[j]}")
        #     print(f"Spatio Temporal Path {j+1}:", path)
        
        # Re planning the longest path initially computed
        for a in range(longest_path_len):
            grid3d.grid[a][paths[longest_path_index][a][1],paths[longest_path_index][a][0]]=0
        for t in range(100):
            grid3d.grid = np.vstack([grid3d.grid,grid3d.grid[-1][np.newaxis, :, :]])
        path = a_star_3D(grid3d,(starts[longest_path_index][0],starts[longest_path_index][1],0),goals[longest_path_index],"",0)
        final_paths.append(path)
            
    return final_paths
    
grid, starts, goals = initialise_grid(5,5,10,5)
paths = find_paths(grid, starts, goals)
grid.render_animation(paths,1000)
'''
TODO:
    Fix the path for the longest path found initially, it doesn't account for any of the other paths.
'''
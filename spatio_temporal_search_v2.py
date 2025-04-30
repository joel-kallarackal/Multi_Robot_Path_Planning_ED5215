import random
from queue import PriorityQueue
import copy
from GridWorld import GridWorld
from GridWorld3D import GridWorld3D
import numpy as np
import time
from collections import defaultdict

def build_wait_for_graph(paths):
    wfg = defaultdict(int)  # directed graph: robot A → robot B

    for i in range(len(paths)):
        for j in range(len(paths)):
            if i==j:
                continue
            for t in range(min(len(paths[i]), len(paths[j]))):
                if paths[i][t] == paths[j][t]:   
                    wfg[i] = j             
                
    return wfg

from collections import defaultdict

def build_graph(edge_dict):
    graph = defaultdict(list)
    for u, v in edge_dict.items():
        graph[u].append(v)
    return graph

def find_all_cycles(graph):
    all_cycles = set()
    path = []

    def dfs(current, visited, stack):
        visited.add(current)
        stack.add(current)
        path.append(current)

        for neighbor in graph[current]:
            if neighbor in stack:
                # Found a cycle — extract and normalize it
                cycle_start_index = path.index(neighbor)
                cycle = path[cycle_start_index:]
                # Normalize by rotating so the smallest element is first
                min_index = cycle.index(min(cycle))
                normalized_cycle = tuple(cycle[min_index:] + cycle[:min_index])
                all_cycles.add(normalized_cycle)
            elif neighbor not in visited:
                dfs(neighbor, visited, stack)

        stack.remove(current)
        path.pop()

    for node in graph:
        dfs(node, set(), set())

    return [list(cycle) for cycle in all_cycles]



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

def initialise_grid_narrow_passage1():
    starts = [(1,2),(3,2)]
    goals = [(3,2),(1,2)]
    obstacles = []
    for i in range(5):
        for j in range(5):
            if i!=2:
                obstacles.append((j,i))
            
    grid = GridWorld(5, 5, obstacles)
    return grid, starts, goals

def initialise_grid_narrow_passage2():
    starts = [(1,2),(3,2)]
    goals = [(3,2),(1,2)]
    obstacles = []
    for i in range(5):
        for j in range(5):
            if i!=2 and j not in [0,4]:
                obstacles.append((j,i))
            
    grid = GridWorld(5, 5, obstacles)
    return grid, starts, goals

def initialise_grid_narrow_passage3():
    starts = [(1,2),(3,2)]
    goals = [(3,2),(1,2)]
    obstacles = []
    for i in range(5):
        for j in range(5):
            if i not in [0,2,4] and j not in [0,4]:
                obstacles.append((j,i))
            
    grid = GridWorld(5, 5, obstacles)
    return grid, starts, goals

def initialise_grid_junction_collision1():
    starts = [(1,2),(3,2),(2,1),(2,3)]
    goals = [(3,2),(1,2),(2,3),(2,1)]
    # starts = [(1,2),(3,2),(2,3)]
    # goals = [(3,2),(1,2),(2,1)]
    # starts = [(1,2),(3,2)]
    # goals = [(3,2),(1,2)]
    obstacles = [(1,1),(3,3),(1,3),(3,1)]
            
    grid = GridWorld(5, 5, obstacles)
    return grid, starts, goals

def initialise_grid_junction_collision2():
    starts = [(2,3),(4,3),(3,2),(3,4)]
    goals = [(4,3),(2,3),(3,4),(3,2)]
    # starts = [(2,3),(4,3),(3,2)]
    # goals = [(4,3),(2,3),(3,4)]
    obstacles = [(2,2),(4,4),(2,4),(4,2)]
            
    grid = GridWorld(7, 7, obstacles)
    return grid, starts, goals

def initialise_grid_door():
    starts = [(0,0),(0,1),(0,2),(0,3),(0,4)]
    goals = [(4,0),(4,1),(4,2),(4,3),(4,4)]
    obstacles = [(2,0), (2,1), (2,3),(2,4)]
    
            
    grid = GridWorld(5, 5, obstacles)
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
    initial_grid = copy.copy(grid)
    rep_node = None
    t1 = time.time()
    while fringe:
        t2=time.time()
        if (t2-t1)>10:
            return path, grid, False
        add_layer = False
        try:
            total, g, h, curr_node = fringe.get(timeout=0.25)
            add_layer=False
        except:
            add_layer=True
            # print(fringe.qsize())
        
        distance = int(get_heuristic_cost(goal, (curr_node[0], curr_node[1]), heuristic))
        if add_layer:
            print("Replanning...")
            for i in range(distance):
                initial_grid.copy_and_append_last_layer()
            grid = copy.copy(initial_grid)
            
            path = []
            visited = []
            parent = {start:None}
            curr_costs = {start:0}
            
            fringe = PriorityQueue()
            fringe.put((0+get_heuristic_cost(goal, start, heuristic), 0, get_heuristic_cost(goal, start, heuristic), start))
            count=0
            continue
        
        
        if (curr_node[0],curr_node[1]) == goal:
            node = curr_node
            while node is not None:
                path.append(node)
                node = parent[node]
            return path[::-1], grid, True
        
        if curr_node in visited:
            continue
         
        visited.append(curr_node)
        # print("Total:",grid.depth*25,",Obstacles:",np.sum(grid.grid))
        
        neighbors = grid.get_neighbors(curr_node[0], curr_node[1], curr_node[2])
        for neighbor in neighbors:
            new_cost = g+1
            new_cost_total = new_cost+get_heuristic_cost(goal, neighbor, heuristic)
            if neighbor not in curr_costs or (curr_costs[neighbor])>new_cost:
                fringe.put((new_cost_total, new_cost, get_heuristic_cost(goal, neighbor, heuristic), neighbor))
                curr_costs[neighbor] = new_cost
                parent[neighbor] = curr_node
                
    return path, grid, False

def find_paths(grid,starts,goals):
    paths = [a_star(grid,starts[i],goals[i],"",0) for i in range(len(starts))]
    
    wfg = build_wait_for_graph(paths)
    graph = build_graph(wfg)

    cycles = find_all_cycles(graph)

    print("All cycles:")
    for cycle in cycles:
        print(cycle)


        

    
    path_0 = [(paths[0][i][0],paths[0][i][1],i) for i in range(len(paths[0]))]
    
    # Currently searches through every possible starting path, in case one combination does not have a solution
    # TODO:
    #       Search through every permutation, not just different starts.
    path_found=False
    for h in range(len(paths)):
        spatio_temporal_grid = []
        for i in range(len(paths[h])):
            obstacles = copy.copy(grid.obstacles)
            obstacles.append((paths[h][i][0],paths[h][i][1]))
            spatio_temporal_grid.append(GridWorld(grid.width, grid.height, obstacles).grid)
            grid3d = GridWorld3D(spatio_temporal_grid)
        
        final_paths = []
        final_paths.append(path_0)
        for j in range(0,len(starts)):
            path = None
            if j!=h:
                path, grid3d, path_found = a_star_3D(grid3d,(starts[j][0],starts[j][1],0),goals[j],"",0)
                if not path_found:
                    break    
                if len(path)<grid3d.depth:
                    for i in range(len(path),grid3d.depth):
                        grid3d.grid[i][goals[j][1],goals[j][0]] = 1
                for m in range(len(path)):
                    grid3d.grid[m][path[m][1], path[m][0]] = 1
                    
                final_paths.append(path)
                print(f"Start: {starts[j]}, Goal: {goals[j]}")
                print(f"Spatio Temporal Path {j+1}:", path)
        
        if path_found:
            return final_paths        
    
      
    return None
    
# Random Test
# grid, starts, goals = initialise_grid(5,5,4,6)
# paths = find_paths(grid, starts, goals)
# grid.render_animation(paths,1000)

'''
NARROW PASSAGE
Narrow passage 1 does not have a solution.
Narrow passage 2 and Narrow passage 3 has a solution.
Current Implementation can find solution for Narrow passage 3
Current implementation does not provide solution for Narrow passage 2, Narrow passage 2 is like cornering deadlock.
'''
# grid, starts, goals = initialise_grid_narrow_passage1()
# paths = find_paths(grid, starts, goals)
# grid.render_animation(paths,1000)

'''
JUNCTION COLLISION
'''
# grid, starts, goals = initialise_grid_junction_collision1()
# paths = find_paths(grid, starts, goals)
# grid.render_animation(paths,1000)
'''
TODO:
    1. Narrow Passage 2
        - How? Will changing A* Heurisitc help?
    2. Junction Collision - DONE
    3. Edge collision needs to be solved - DONE
'''

'''
DOOR
'''
grid, starts, goals = initialise_grid_door()
paths = find_paths(grid, starts, goals)
grid.render_animation(paths,1000)
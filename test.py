import heapq
import numpy as np
import matplotlib.pyplot as plt

class SpaceTimeAStar:
    def __init__(self, grid, robots):
        self.grid = grid  # 2D grid world
        self.robots = robots  # Dictionary of robot start and goal positions
        self.time_map = {}  # Stores occupied cells at different time steps

    def heuristic(self, pos, goal):
        return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])  # Manhattan distance

    def get_neighbors(self, pos, time):
        moves = [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)]  # Right, Down, Left, Up, Wait
        neighbors = []
        for dx, dy in moves:
            new_x, new_y = pos[0] + dx, pos[1] + dy
            if 0 <= new_x < self.grid.shape[0] and 0 <= new_y < self.grid.shape[1]:
                if self.grid[new_x, new_y] == 0:  # Check if it's not an obstacle
                    if (new_x, new_y, time + 1) not in self.time_map:  # Check time conflicts
                        neighbors.append(((new_x, new_y), time + 1))
        return neighbors

    def space_time_astar(self, start, goal):
        open_list = []  # Priority queue
        heapq.heappush(open_list, (0 + self.heuristic(start, goal), 0, start, []))
        visited = set()
        
        while open_list:
            _, cost, current, path = heapq.heappop(open_list)
            if current == goal:
                return path + [goal]
            
            if (current, cost) in visited:
                continue
            visited.add((current, cost))
            
            for neighbor, new_time in self.get_neighbors(current, cost):
                new_cost = cost + 1
                heapq.heappush(open_list, (new_cost + self.heuristic(neighbor, goal), new_cost, neighbor, path + [current]))
                self.time_map[(neighbor[0], neighbor[1], new_time)] = True  # Mark space-time occupied
        return []  # No path found

    def plan_paths(self):
        planned_paths = {}
        robots_sorted = sorted(self.robots.items(), key=lambda x: self.heuristic(x[1]['start'], x[1]['goal']), reverse=True)
        
        for robot, positions in robots_sorted:
            start, goal = positions['start'], positions['goal']
            path = self.space_time_astar(start, goal)
            planned_paths[robot] = path if path else [(start)]  # Ensure non-empty path
        
        return planned_paths

# Example Grid
grid_size = (10, 10)
grid = np.zeros(grid_size)
obstacles = [(3, 3), (3, 4), (3, 5), (6, 6), (7, 6), (8, 6)]
for obs in obstacles:
    grid[obs] = 1

robots = {
    "R1": {"start": (0, 0), "goal": (9, 9)},
    "R2": {"start": (0, 9), "goal": (9, 0)},
    "R3": {"start": (5, 2), "goal": (2, 8)}
}

planner = SpaceTimeAStar(grid, robots)
paths = planner.plan_paths()

# Visualize Paths
fig, ax = plt.subplots(figsize=(6, 6))
ax.set_xticks(np.arange(grid_size[1] + 1) - 0.5, minor=True)
ax.set_yticks(np.arange(grid_size[0] + 1) - 0.5, minor=True)
ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.5)
ax.tick_params(which="both", bottom=False, left=False, labelbottom=False, labelleft=False)

for (x, y) in obstacles:
    ax.add_patch(plt.Rectangle((y, x), 1, 1, color="black"))

colors = ["blue", "red", "green"]
for i, (robot, path) in enumerate(paths.items()):
    if len(path) > 1:
        x_vals, y_vals = zip(*path)
        ax.plot(y_vals, x_vals, marker="o", linestyle="-", color=colors[i], label=robot)
        ax.add_patch(plt.Circle((path[0][1] + 0.5, path[0][0] + 0.5), 0.3, color=colors[i]))
        ax.add_patch(plt.Circle((path[-1][1] + 0.5, path[-1][0] + 0.5), 0.3, color=colors[i], alpha=0.5))

plt.legend()
plt.title("Space-Time A* Multi-Robot Path Planning")
plt.show()
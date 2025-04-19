import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import matplotlib.animation as animation


class GridWorld:
    def __init__(self, width, height, obstacles=[]):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=int)  # 0: free, 1: obstacle
        self.obstacles = obstacles
        print(obstacles)
        if len(obstacles)>0:
            for x,y in obstacles:
                self.grid[y, x] = 1
        self.agent_pos = None
    
    def set_agent(self, x, y):
        if self.grid[y, x] == 1:
            raise ValueError("Cannot place agent on an obstacle.")
        self.agent_pos = (x, y)
    
    def move(self, direction):
        if self.agent_pos is None:
            raise ValueError("Agent position not set.")
        x, y = self.agent_pos
        
        if direction == 'up' and y > 0 and self.grid[y - 1, x] == 0:
            self.agent_pos = (x, y - 1)
        elif direction == 'down' and y < self.height - 1 and self.grid[y + 1, x] == 0:
            self.agent_pos = (x, y + 1)
        elif direction == 'left' and x > 0 and self.grid[y, x - 1] == 0:
            self.agent_pos = (x - 1, y)
        elif direction == 'right' and x < self.width - 1 and self.grid[y, x + 1] == 0:
            self.agent_pos = (x + 1, y)
    
    def render(self):
        cmap = mcolors.ListedColormap(['white', 'black', 'blue'])
        grid_display = self.grid.copy()
        if self.agent_pos:
            x, y = self.agent_pos
            grid_display[y, x] = 2  # Mark agent position
        
        fig, ax = plt.subplots(figsize=(self.width, self.height))
        ax.imshow(grid_display, cmap=cmap, origin='upper')
        ax.set_xticks(np.arange(-0.5, self.width, 1))
        ax.set_yticks(np.arange(-0.5, self.height, 1))
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.grid(True, which='both', color='gray', linewidth=0.5)
        plt.show()
    
    def get_neighbors(self, x, y):
        neighbors = []
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height and self.grid[ny, nx] == 0:
                neighbors.append((nx, ny))
        return neighbors
    
    def is_goal_reached(self, goal):
        return self.agent_pos == goal
    
    def render_animation(self, paths, interval=500):
        """
        Render an animation of multiple robots moving along their planned paths.

        Parameters:
        - paths: List of lists, each containing tuples (x, y, t) for a robot.
        - interval: Time interval between frames in milliseconds.
        """
        fig, ax = plt.subplots(figsize=(6, 6))

        # Display the grid using imshow with a white background
        grid_display = np.ones_like(self.grid, dtype=float)  # White background
        grid_display[self.grid == 1] = 0  # Obstacles in black
        ax.imshow(grid_display, cmap="gray_r", origin="upper", extent=[0, self.width, 0, self.height])

        # Get the number of robots
        num_robots = len(paths)
        colors = plt.cm.get_cmap("tab10", num_robots)  # Assign colors to each robot

        # Initialize robot markers
        robots = [ax.plot([], [], 'o', color=colors(i), markersize=12, label=f'Robot {i+1}')[0] for i in range(num_robots)]

        # Plot start and goal positions at the **center** of cells
        for i, path in enumerate(paths):
            start_x, start_y, _ = path[0]
            goal_x, goal_y, _ = path[-1]
            ax.scatter(start_x + 0.5, self.height - start_y - 0.5, marker='s', color='blue', s=200, label=f'Start {i+1}' if i == 0 else None)
            ax.scatter(goal_x + 0.5, self.height - goal_y - 0.5, marker='*', color='green', s=200, label=f'Goal {i+1}' if i == 0 else None)

        # Compute max time step
        max_time = max(max(path, key=lambda p: p[2])[2] for path in paths)

        def update(frame):
            """ Update function for animation """
            for i, path in enumerate(paths):
                # Find the latest position at the current frame
                position = next(((x + 0.5, self.height - y - 0.5) for x, y, t in path if t == frame), None)
                if position:
                    robots[i].set_data(*position)

        ani = animation.FuncAnimation(fig, update, frames=max_time + 1, interval=interval)
        plt.xticks(np.arange(0, self.width, 1))
        plt.yticks(np.arange(0, self.height, 1))
        plt.grid(visible=True, linestyle="--", color="gray", linewidth=0.5)  # Show light gray grid lines
        plt.show()






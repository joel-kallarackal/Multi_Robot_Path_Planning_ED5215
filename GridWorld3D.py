import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from mpl_toolkits.mplot3d import Axes3D

class GridWorld3D:
    def __init__(self, grid_levels):
        self.depth = len(grid_levels)  # Number of 2D grids (layers)
        self.height = len(grid_levels[0])
        self.width = len(grid_levels[0][0])
        self.grid = np.array(grid_levels)
        self.agent_pos = None  # (x, y, z)
    
    def set_agent(self, x, y, z):
        if self.grid[z, y, x] == 1:
            raise ValueError("Cannot place agent on an obstacle.")
        self.agent_pos = (x, y, z)
    
    def move(self, direction):
        if self.agent_pos is None:
            raise ValueError("Agent position not set.")
        x, y, z = self.agent_pos
        
        moves = {
            'up': (x, y, z + 1),
            'up_straight': (x, y - 1, z + 1),
            'up_back': (x, y + 1, z + 1),
            'up_left': (x - 1, y, z + 1),
            'up_right': (x + 1, y, z + 1)
        }
        
        if direction in moves:
            nx, ny, nz = moves[direction]
            # 0 <= nz+1 < self.depth and self.grid[nz+1, ny, nx] == 0 is added to check if the 
            # next move is valid at time t and if it is valid at time t+1, i.e once the robot move there.
            if 0 <= nx < self.width and 0 <= ny < self.height and 0 <= nz+1 < self.depth and self.grid[nz, ny, nx] == 0 and self.grid[nz+1, ny, nx] == 0:
                self.agent_pos = (nx, ny, nz)
    
    def render(self):
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection='3d')
        
        filled_cells = np.argwhere(self.grid == 1)
        agent_x, agent_y, agent_z = self.agent_pos if self.agent_pos else (None, None, None)
        
        ax.scatter(filled_cells[:, 0], filled_cells[:, 1], filled_cells[:, 2], c='black', marker='s', label='Obstacle')
        if self.agent_pos:
            ax.scatter(agent_x, agent_y, agent_z, c='blue', marker='o', s=100, label='Agent')
        
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xticks(range(self.width))
        ax.set_yticks(range(self.height))
        ax.set_zticks(range(self.depth))
        ax.legend()
        plt.show()
    
    def get_neighbors(self, x, y, z):
        # if len(self.grid)-1==z:
        #     self.copy_and_append_last_layer()
            
        neighbors = []
        directions = [
            (0, 0, 1), (0, -1, 1), (0, 1, 1), (-1, 0, 1), (1, 0, 1)
        ]
        
        for dx, dy, dz in directions:
            nx, ny, nz = x + dx, y + dy, z + dz
            if 0 <= nx < self.width and 0 <= ny < self.height and 0 <= nz < self.depth and self.grid[nz, ny, nx] == 0:
                neighbors.append((nx, ny, nz))
        
        return neighbors
    
    def is_goal_reached(self, goal):
        return self.agent_pos == goal
    
    def copy_and_append_last_layer(self):
        # Copies the last layer in the 3D grid and appends it to the same.
        last_layer = self.grid[-1]
        self.grid = np.append(self.grid,[last_layer], axis=0)
        self.depth+=1

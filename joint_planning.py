import heapq
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ---------- A* JOINT PLANNER ----------
def is_valid(pos, grid, other_pos, old_pos):
    rows, cols = len(grid), len(grid[0])
    x, y = pos
    return (0 <= x < rows and 0 <= y < cols and 
            grid[x][y] == 0 and pos != other_pos)

def heuristic(pos1, pos2, goal1, goal2):
    return abs(pos1[0]-goal1[0]) + abs(pos1[1]-goal1[1]) + \
           abs(pos2[0]-goal2[0]) + abs(pos2[1]-goal2[1])

def joint_astar(grid, start1, goal1, start2, goal2):
    visited = set()
    heap = []
    heapq.heappush(heap, (0 + heuristic(start1, start2, goal1, goal2), 0, start1, start2, []))
    directions = [(-1,0), (1,0), (0,-1), (0,1), (0,0)]

    while heap:
        f, g, pos1, pos2, path = heapq.heappop(heap)
        if (pos1, pos2) in visited:
            continue
        visited.add((pos1, pos2))

        if pos1 == goal1 and pos2 == goal2:
            return path + [(pos1, pos2)]

        for d1 in directions:
            pos_old1 = (pos1[0], pos1[1])
            new_pos1 = (pos1[0] + d1[0], pos1[1] + d1[1])
            if not is_valid(new_pos1, grid, pos2, pos_old1): continue

            for d2 in directions:
                pos_old2 = (pos2[0], pos2[1])
                new_pos2 = (pos2[0] + d2[0], pos2[1] + d2[1])
                if not is_valid(new_pos2, grid, new_pos1, pos_old2): continue

                # avoid cross paths
                if new_pos1 == pos2 and new_pos2 == pos1:
                    continue

                new_path = path + [(pos1, pos2)]
                h = heuristic(new_pos1, new_pos2, goal1, goal2)
                heapq.heappush(heap, (g+1+h, g+1, new_pos1, new_pos2, new_path))

    return None

# ---------- PLOTTER ----------
def plot_grid_path_with_lines(grid, path):
    fig, ax = plt.subplots()
    rows, cols = len(grid), len(grid[0])
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols+1))
    ax.set_yticks(range(rows+1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(True)

    # Draw obstacles
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                rect = patches.Rectangle((j, rows-1-i), 1, 1, color='black')
                ax.add_patch(rect)

    # Draw robot positions and lines
    r1_path = []
    r2_path = []
    for t, (r1, r2) in enumerate(path):
        y1, x1 = rows-1-r1[0], r1[1]
        y2, x2 = rows-1-r2[0], r2[1]
        r1_path.append((x1 + 0.5, y1 + 0.5))
        r2_path.append((x2 + 0.5, y2 + 0.5))
        ax.text(x1+0.05, y1+0.7, f'{t}', color='red', fontsize=8)
        ax.text(x2+0.05, y2+0.1, f'{t}', color='blue', fontsize=8)

    # Connect the robot paths with lines
    r1_xs, r1_ys = zip(*r1_path)
    r2_xs, r2_ys = zip(*r2_path)
    ax.plot(r1_xs, r1_ys, color='red', linewidth=2, label='Robot 1')
    ax.plot(r2_xs, r2_ys, color='blue', linewidth=2, label='Robot 2')

    ax.legend()
    ax.set_aspect('equal')
    plt.title("Joint Path of Two Robots with Lines")
    plt.show()
    
import matplotlib.animation as animation

# Animate two robots moving as circles
def animate_robot_paths(grid, path):
    fig, ax = plt.subplots()
    rows, cols = len(grid), len(grid[0])
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks(range(cols+1))
    ax.set_yticks(range(rows+1))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.grid(True)

    # Draw obstacles
    for i in range(rows):
        for j in range(cols):
            if grid[i][j] == 1:
                rect = patches.Rectangle((j, rows-1-i), 1, 1, color='black')
                ax.add_patch(rect)

    # Create robot markers
    robot1 = plt.Circle((0.5, 0.5), 0.1, color='red', label='Robot 1')
    robot2 = plt.Circle((0.5, 0.5), 0.1, color='blue', label='Robot 2')
    goal1 = plt.Circle((0.5, 0.5), 0.1, color='green', label='goal 1')
    goal2 = plt.Circle((0.5, 0.5), 0.1, color='green', label='goal 2')
    ax.add_patch(robot1)
    ax.add_patch(robot2)
    ax.add_patch(goal1)
    ax.add_patch(goal2)
    ax.legend()
    
    goal1.center = path[-1][0]
    goal2.center = path[-1][1]

    def update(frame):
        if frame >= len(path):
            return robot1, robot2
        (r1, r2) = path[frame]
        (rg1, rg2) = path[-1]
        y1, x1 = rows-1-r1[0], r1[1]
        y2, x2 = rows-1-r2[0], r2[1]
        gy1, gx1 = rows-1-rg1[0], rg1[1]
        gy2, gx2 = rows-1-rg2[0], rg2[1]
        goal1.center = (gx1 + 0.5, gy1 + 0.5)
        goal2.center = (gx2 + 0.5, gy2 + 0.5)
        robot1.center = (x1 + 0.5, y1 + 0.5)
        robot2.center = (x2 + 0.5, y2 + 0.5)
        return goal1, goal2, robot1, robot2

    ani = animation.FuncAnimation(fig, update, frames=len(path), interval=1000, blit=True, repeat=False)
    plt.title("Animated Robot Paths")
    plt.show()

# ---------- EXAMPLE USAGE ----------
if __name__ == "__main__":
    # grid = [
    #     [1, 1, 1, 0, 0],
    #     [1, 0, 1, 0, 0],
    #     [1, 0, 0, 0, 0],
    #     [1, 1, 1, 0, 0]
    # ]
    grid = [
        [0, 1, 1, 1, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0]
    ]

    start1 = (2, 1)
    goal1 = (2, 3)
    start2 = (2, 3)
    goal2 = (2, 1)

    path = joint_astar(grid, start1, goal1, start2, goal2)
    # if path:
    #     plot_grid_path_with_lines(grid, path)
    # else:
    #     print("No path found!")
    
    animate_robot_paths(grid, path)


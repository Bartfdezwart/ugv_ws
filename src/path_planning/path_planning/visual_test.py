from path_planning.astar import Astar
from path_planning.visualizer import PathfindingVisualizer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time


grid = np.zeros((300, 450))
num_blocks = 7
block_size = (50, 50)  # height, width

for _ in range(num_blocks):
    r = np.random.randint(0, grid.shape[0] - block_size[0])
    c = np.random.randint(0, grid.shape[1] - block_size[1])
    grid[r:r+block_size[0], c:c+block_size[1]] = 1

start = (100, 100)
goal = (275, 240)

t_start = time.time()
astar = Astar(grid, start, goal)
path = astar.find_path()
print("Found path:", path is not None)
print(f"{(time.time() - t_start)} seconds")
if path is None:
    quit()


# ----------------- Initialize -----------------
fig, ax = plt.subplots(figsize=(6, 6))
finder = Astar(grid, start, goal)
visualizer = PathfindingVisualizer(grid, start, goal, ax=ax)

finished = False


# ----------------- Animation -----------------
def animate(frame):
    global finished

    if finished:
        return []

    done = finder.step()
    # show path if finished
    if done:
        print("Path found")
        finder.path = finder.trace_path(done)
        finished = True
        anim.event_source.stop()

    frontier_positions = [cell.position for cell in finder.frontier] if finder.frontier else []
    return visualizer.update(
        finder.visited, frontier_positions, finder.path if finished else None
    )


anim = FuncAnimation(fig, animate, blit=True, interval=1, cache_frame_data=False)
plt.show()

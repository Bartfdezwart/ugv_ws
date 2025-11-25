import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

class PathfindingVisualizer:
    def __init__(self, grid, start, goal, ax=None, title="A*"):
        self.grid = np.array(grid)
        self.start = start
        self.goal = goal

        self.cmap = ListedColormap([
            'black',     # 0 = wall
            'lightgray', # 1 = empty
            'blue',      # 2 = visited
            'yellow',    # 3 = frontier
            'green',     # 4 = path
            'red',       # 5 = start
            'magenta'    # 6 = goal
        ])

        if ax is None:
            self.fig, self.ax = plt.subplots()
        else:
            self.ax = ax
            self.fig = ax.figure

        # initial image, animated=True for blitting
        self.im = self.ax.imshow(self.grid, cmap=self.cmap, vmin=0, vmax=6, animated=True)
        self.ax.set_title(title)

    def update(self, visited=None, frontier=None, path=None):
        display_grid = self.grid.copy()
        if visited:
            for r, c in visited:
                display_grid[r, c] = 2
        if frontier:
            for r, c in frontier:
                display_grid[r, c] = 3
        if path:
            for r, c in path:
                display_grid[r, c] = 4

        display_grid[self.start] = 5
        display_grid[self.goal] = 6

        self.im.set_array(display_grid)
        return [self.im]
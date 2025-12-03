from collections import defaultdict
import math
from abc import ABC, abstractmethod


class BasePathFinder(ABC):
    def __init__(self, grid, start, goal):
        """
        grid : 2D list (0 = free, 1 = obstacle)
        start, goal : (row, col)
        """
        self.grid = grid
        self.start = start
        self.goal = goal

        self.visited = defaultdict(bool)
        self.frontier = []
        self.path = []

    def inside_grid(self, row, col) -> bool:
        max_rows, max_cols = self.grid.shape
        return (row >= 0) and (row < max_rows) and (col >= 0) and (col < max_cols)

    def is_unblocked(self, row, col):
        return self.grid[row][col] == 0

    def is_destination(self, row, col):
        return row == self.goal[0] and col == self.goal[1]

    @abstractmethod
    def step(self):
        """
        Perform ONE iteration of the algorithm.
        Should update:
          - self.visited
          - self.frontier
          - self.path (when finished)
        Return True if finished, False otherwise.
        """
        pass

    @abstractmethod
    def find_path(self):
        """
        Fully execute the algorithm until a path is found or impossible.
        Must return the final path (list of (r,c)).
        """
        pass


def get_movements_8n():
    """
    Get all possible 8-connectivity movements. Equivalent to get_movements_in_radius(1).
    :return: list of movements with cost [(dx, dy, movement_cost)]
    """
    s2 = math.sqrt(2)
    return [(1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, s2),
            (-1, 1, s2),
            (-1, -1, s2),
            (1, -1, s2)]

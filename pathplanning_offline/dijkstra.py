import heapq
import math


def get_movements_8n():
    return [
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1, 1, math.sqrt(2)),
        (1, -1, math.sqrt(2)),
        (1, 1, math.sqrt(2)),
    ]


class Cell:
    def __init__(self, position):
        self.position = position
        self.parent = None
        self.g = float("inf")

    def __lt__(self, other):
        return self.g < other.g


class Dijkstra:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.rows, self.cols = grid.shape
        self.frontier = []
        self.visited = set()
        self.cells = {}

        start_cell = Cell(start)
        start_cell.g = 0.0
        self.cells[start] = start_cell
        heapq.heappush(self.frontier, start_cell)

    def inside(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def free(self, r, c):
        return self.grid[r, c] == 0

    def find_path(self):
        while self.frontier:
            current = heapq.heappop(self.frontier)

            if current.position in self.visited:
                continue

            if current.position == self.goal:
                return self.reconstruct(current)

            self.visited.add(current.position)
            r, c = current.position

            for dr, dc, cost in get_movements_8n():
                nr, nc = r + dr, c + dc

                if not self.inside(nr, nc) or not self.free(nr, nc):
                    continue

                if dr != 0 and dc != 0:
                    if not (self.free(r + dr, c) and self.free(r, c + dc)):
                        continue

                new_g = current.g + cost

                neighbor = self.cells.get((nr, nc))
                if neighbor is None:
                    neighbor = Cell((nr, nc))
                    self.cells[(nr, nc)] = neighbor

                if new_g < neighbor.g:
                    neighbor.g = new_g
                    neighbor.parent = current
                    heapq.heappush(self.frontier, neighbor)

        return None

    def reconstruct(self, cell):
        path = []
        while cell is not None:
            path.append(cell.position)
            cell = cell.parent
        path.reverse()
        return path
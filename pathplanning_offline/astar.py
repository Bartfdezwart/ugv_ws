import heapq
import math


def get_movements_8n():
    return [
        (-1,  0, 1.0),
        ( 1,  0, 1.0),
        ( 0, -1, 1.0),
        ( 0,  1, 1.0),
        (-1, -1, math.sqrt(2)),
        (-1,  1, math.sqrt(2)),
        ( 1, -1, math.sqrt(2)),
        ( 1,  1, math.sqrt(2)),
    ]


class Cell:
    def __init__(self, position, parent=None, g=float("inf"), h=0.0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h

    def f(self):
        return self.g + self.h

    def __lt__(self, other):
        return self.f() < other.f()


class Astar:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal

        self.rows, self.cols = grid.shape
        self.frontier = []
        self.visited = set()
        self.cells = {}

        start_cell = Cell(
            position=start,
            g=0.0,
            h=self.heuristic(start),
        )

        heapq.heappush(self.frontier, start_cell)
        self.cells[start] = start_cell

    def heuristic(self, pos):
        dx = pos[0] - self.goal[0]
        dy = pos[1] - self.goal[1]
        return math.sqrt(dx * dx + dy * dy)

    def inside(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def free(self, r, c):
        return self.grid[r, c] == 0

    def step(self):
        if not self.frontier:
            return None

        current = heapq.heappop(self.frontier)

        if current.position in self.visited:
            return None

        self.visited.add(current.position)

        if current.position == self.goal:
            return current

        r, c = current.position

        for dr, dc, cost in get_movements_8n():
            nr, nc = r + dr, c + dc

            if not self.inside(nr, nc) or not self.free(nr, nc):
                continue

            if dr != 0 and dc != 0:
                if not (self.free(r + dr, c) and self.free(r, c + dc)):
                    continue

            new_g = current.g + cost
            existing = self.cells.get((nr, nc))

            if existing is None or new_g < existing.g:
                next_cell = Cell(
                    position=(nr, nc),
                    parent=current,
                    g=new_g,
                    h=self.heuristic((nr, nc)),
                )
                self.cells[(nr, nc)] = next_cell
                heapq.heappush(self.frontier, next_cell)

        return None

    def find_path(self):
        while self.frontier:
            result = self.step()
            if result:
                return self.reconstruct(result)
        return None

    def reconstruct(self, cell):
        path = []
        while cell.parent is not None:
            path.append(cell.position)
            cell = cell.parent
        path.reverse()
        return path
from __future__ import annotations

import heapq

from path_planning import BasePathFinder, get_movements_8n


class Cell:
    def __init__(
        self, position, parent: Cell | None = None, g_cost=float("inf"), h_cost=0
    ):
        self.position = position
        self.parent = parent
        self.g_cost = g_cost
        self.h_cost = h_cost

    def f_cost(self):
        return self.g_cost + self.h_cost

    def __lt__(self, other):
        if isinstance(other, Cell):
            return self.f_cost() < other.f_cost()
        return NotImplemented


class Astar(BasePathFinder):
    def __init__(self, grid, start, goal):
        super().__init__(grid, start, goal)

        self.cells = {}

        start_cell = Cell(start, g_cost=0, h_cost=self.calculate_h_value(start))
        heapq.heappush(self.frontier, start_cell)
        self.cells[start] = start_cell

    def step(self) -> None | Cell:
        current_cell: Cell = heapq.heappop(self.frontier)
        current_position = current_cell.position

        if self.cells.get(current_cell.position) is not current_cell:
            return None

        self.visited[current_position] = True

        if self.is_destination(*current_cell.position):
            return current_cell

        directions = get_movements_8n()
        for direction in directions:
            next_position = (
                current_position[0] + direction[0],
                current_position[1] + direction[1],
            )

            # Prevent moving through diagonal walls
            if direction[0] != 0 and direction[1] != 0:
                orth1 = (current_position[0] + direction[0], current_position[1])
                orth2 = (current_position[0], current_position[1] + direction[1])
                if (
                    not self.inside_grid(*orth1)
                    or not self.inside_grid(*orth2)
                    or not self.is_unblocked(*orth1)
                    or not self.is_unblocked(*orth2)
                ):
                    continue

            if (
                self.inside_grid(*next_position)
                and self.is_unblocked(*next_position)
                and not self.visited[next_position]
            ):
                next_cell = Cell(position=next_position)
                next_cell.parent = current_cell

                next_cell.g_cost = current_cell.g_cost + direction[2]
                next_cell.h_cost = self.calculate_h_value(next_cell.position)

                existing_cell = self.cells.get(next_cell.position)
                if existing_cell is None or existing_cell.f_cost() > next_cell.f_cost():
                    self.cells[next_cell.position] = next_cell
                    heapq.heappush(self.frontier, next_cell)

        return None

    def find_path(self) -> None | list:
        while self.frontier:
            reached_last_cell = self.step()
            if reached_last_cell:
                return self.trace_path(reached_last_cell)

        return None

    def trace_path(self, last_cell: Cell) -> list:
        path = []
        current_cell = last_cell
        while current_cell.parent is not None:
            path.append(current_cell.position)
            current_cell = current_cell.parent

        path.reverse()
        return path

    def calculate_h_value(self, position):
        row, col = position
        dest_row, dest_col = self.goal

        dx = row - dest_row
        dy = col - dest_col

        return (dx**2 + dy**2)**0.5


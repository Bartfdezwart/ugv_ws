import numpy as np
import open3d as o3d
from collections import defaultdict, deque
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import cv2
import os

from astar import Astar
from dijkstra import Dijkstra


def clip_to_grid(pos, shape):
    r = int(np.clip(pos[0], 0, shape[0] - 1))
    c = int(np.clip(pos[1], 0, shape[1] - 1))
    return (r, c)


def nearest_free_cell(grid, pos):
    if grid[pos] == 0:
        return pos

    H, W = grid.shape
    q = deque([pos])
    seen = {pos}

    moves = [(-1,0),(1,0),(0,-1),(0,1),
             (-1,-1),(-1,1),(1,-1),(1,1)]

    while q:
        r, c = q.popleft()
        for dr, dc in moves:
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and (nr, nc) not in seen:
                if grid[nr, nc] == 0:
                    return (nr, nc)
                seen.add((nr, nc))
                q.append((nr, nc))

    raise RuntimeError("Grid fully blocked")


def inflate_grid(grid, radius):
    H, W = grid.shape
    inflated = grid.copy()

    wall_cells = np.argwhere(grid == 1)

    for r, c in wall_cells:
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < H and 0 <= nc < W:
                    inflated[nr, nc] = 1

    return inflated

maze_pcd = o3d.io.read_point_cloud("filtered_cloud.ply")
maze_points = np.asarray(maze_pcd.points)

lidar_raw = o3d.io.read_point_cloud("lidar.ply")
lidar_points = np.asarray(lidar_raw.points)

maze_kd = o3d.geometry.KDTreeFlann(maze_pcd)

keep_idx = []
for i, p in enumerate(lidar_points):
    _, _, dist = maze_kd.search_knn_vector_3d(p, 1)
    if dist[0] <= 0.3 ** 2:
        keep_idx.append(i)

lidar_points = lidar_points[keep_idx]

lidar_filtered = o3d.geometry.PointCloud()
lidar_filtered.points = o3d.utility.Vector3dVector(lidar_points)
o3d.io.write_point_cloud("lidar_filtered.ply", lidar_filtered)

grid_res = 0.02
min_points_per_cell = 2
max_gap = 8
min_run_len = 2

xy = lidar_points[:, :2]
grid_idx = np.floor(xy / grid_res).astype(int)

cell_counts = defaultdict(int)
for cell in map(tuple, grid_idx):
    cell_counts[cell] += 1

occupied_cells = {c for c, n in cell_counts.items()
                  if n >= min_points_per_cell}

rows, cols = defaultdict(list), defaultdict(list)
for x, y in occupied_cells:
    rows[y].append(x)
    cols[x].append(y)


def find_runs(vals):
    vals = sorted(vals)
    if not vals:
        return []

    runs = []
    s = p = vals[0]

    for v in vals[1:]:
        if v - p <= max_gap + 1:
            p = v
        else:
            if p - s + 1 >= min_run_len:
                runs.append((s, p))
            s = p = v

    if p - s + 1 >= min_run_len:
        runs.append((s, p))

    return runs


segments = []

for y, xs in rows.items():
    for a, b in find_runs(xs):
        segments.append(("h", (a, y, b, y)))

for x, ys in cols.items():
    for a, b in find_runs(ys):
        segments.append(("v", (x, a, x, b)))


wall_cells = set()
for ori, (x1, y1, x2, y2) in segments:
    if ori == "h":
        for x in range(x1, x2 + 1):
            wall_cells.add((y1, x))
    else:
        for y in range(y1, y2 + 1):
            wall_cells.add((y, x1))


ys, xs = zip(*wall_cells)
min_y, max_y = min(ys), max(ys)
min_x, max_x = min(xs), max(xs)

grid = np.zeros((max_y - min_y + 1, max_x - min_x + 1),
                dtype=np.uint8)

for y, x in wall_cells:
    grid[y - min_y, x - min_x] = 1

print("Grid shape:", grid.shape)


start_requested = np.array([50, 50]) * (0.01 / grid_res)
goal_requested  = np.array([70, 370]) * (0.01 / grid_res)

start = clip_to_grid(start_requested, grid.shape)
goal  = clip_to_grid(goal_requested, grid.shape)

start = nearest_free_cell(grid, start)
goal  = nearest_free_cell(grid, goal)

inflation_radius = int(8 * (0.01 / grid_res))
grid_for_planning = inflate_grid(grid, inflation_radius)

planner = Astar(grid_for_planning, start, goal)
# planner = Dijkstra(grid_for_planning, start, goal)
path = planner.find_path()


vis = np.zeros((*grid.shape, 3), dtype=np.uint8)
vis[grid == 0] = [255, 255, 255]
vis[grid == 1] = [0, 0, 0]
vis[grid_for_planning - grid == 1] = [255, 255, 0]

if path is not None:
    for r, c in path:
        vis[r, c] = [255, 0, 0]

sr, sc = start
gr, gc = goal
vis[sr, sc] = [0, 0, 255]
vis[gr, gc] = [0, 255, 0]

fig, ax = plt.subplots(figsize=(10, 10))
ax.axis("off")
ax.imshow(vis, origin="lower")
plt.show()


nav2_dir = "nav2_map"
os.makedirs(nav2_dir, exist_ok=True)

nav2_map = np.zeros_like(grid, dtype=np.uint8)
nav2_map[grid == 0] = 255
nav2_map[grid == 1] = 0

nav2_map = np.flipud(nav2_map)

map_png = os.path.join(nav2_dir, "map.png")
cv2.imwrite(map_png, nav2_map)

origin_x = min_x * grid_res
origin_y = min_y * grid_res

map_yaml = os.path.join(nav2_dir, "map.yaml")
with open(map_yaml, "w") as f:
    f.write(f"""image: map.png
resolution: {grid_res}
origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]
occupied_thresh: 0.65
free_thresh: 0.25
negate: 0
""")

print(f"[Nav2] Map exported to {nav2_dir}/")
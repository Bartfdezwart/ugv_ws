import numpy as np
import matplotlib.pyplot as plt
import cv2

costmap = cv2.imread("nav2_costmap.png", cv2.IMREAD_GRAYSCALE)

H, W = costmap.shape


cmin = costmap.min()
cmax = costmap.max()

saturated = np.interp(
    costmap.astype(np.float32),
    (cmin, cmax),
    (0, 240)
).astype(np.uint8)

cv2.imwrite("saturated_costmap.png", saturated)

path = np.loadtxt("nav2_path.csv", delimiter=",", skiprows=1)

xs = path[:, 0]
ys = path[:, 1]


resolution = 0.02
origin_x = 0.8
origin_y = -0.6

cols = (xs - origin_x) / resolution
rows = (ys - origin_y) / resolution


plt.figure(figsize=(8, 10))

plt.imshow(saturated, cmap="gray", origin="lower")

plt.plot(cols, rows, 'r-', linewidth=2, label="Nav2 Path")
plt.scatter(cols[0], rows[0], c='blue', s=80, label="Start")
plt.scatter(cols[-1], rows[-1], c='green', s=80, label="Goal")

plt.axis("off")

plt.legend(
    fontsize=16,
    markerscale=2,
    handlelength=3,
    frameon=True,
    title="Legend\n(light = free, dark = occupied)"
)

plt.tight_layout()

plt.savefig(
    "saturated_costmap_with_path.png",
    dpi=300,
    bbox_inches="tight"
)

plt.show()
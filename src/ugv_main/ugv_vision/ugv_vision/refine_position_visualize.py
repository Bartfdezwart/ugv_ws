import numpy as np

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt


class Trilateration:
    def __init__(self):
        # Tag world coordinates
        self.tag_world_positions = {
            3: np.array([3.0, -1.85]),
            4: np.array([-3.0, 0.0]),
            5: np.array([3.0, 0.0]),
            6: np.array([-3.2, 1.85]),
            7: np.array([3.66, 1.85]),
            8: np.array([-2.84, 4.5]),
            9: np.array([0.0, 4.5]),
            10: np.array([2.84, 4.5]),
        }

    def trilateration(
        self,
        visible_ids,
        true_position=np.array((0, 0)),
        tag_position_noise=None,
        refine_position=False,
    ):
        distances = []
        tag_positions = []
        for visible_id in visible_ids:
            tag_position = self.tag_world_positions[visible_id]
            tag_positions.append(tag_position)
            distance = np.linalg.norm(tag_position - true_position)
            distances.append(distance)

        distances = np.array(distances)

        if tag_position_noise is not None:
            assert (
                len(distances) == len(tag_position_noise)
                or len(tag_position_noise) == 1
            )
            distances += tag_position_noise

        rover_xy = self.svd_position(visible_ids, distances, refine_position)

        return rover_xy, tag_positions, distances

    def svd_position(self, tag_ids, distances, refine_position=False):
        Ps, Ds = [], []
        for tid, d in zip(tag_ids, distances):
            if tid in self.tag_world_positions:
                Ps.append(self.tag_world_positions[tid])
                Ds.append(d)

        Ps = np.array(Ps, float)
        Ds = np.array(Ds, float)
        n = len(Ps)

        # Not enough valid tags
        if n < 2:
            return None

        # 2 tags
        if n == 2:
            P1, P2 = Ps
            d1, d2 = Ds
            D = np.linalg.norm(P2 - P1)

            if D > d1 + d2 or D < abs(d1 - d2):
                return None

            a = (d1**2 - d2**2 + D**2) / (2 * D)
            h2 = d1**2 - a**2
            if h2 < 0:
                return None

            P3 = P1 + a * (P2 - P1) / D
            perp = np.array([-(P2[1] - P1[1]) / D, (P2[0] - P1[0]) / D]) * np.sqrt(h2)

            sol1, sol2 = P3 + perp, P3 - perp
            inside = lambda p: -3.0 <= p[0] <= 3.0 and -4.5 <= p[1] <= 4.5
            position = sol1 if inside(sol1) else sol2

            if refine_position:
                position = self.refine_position(position, Ps, Ds)

            return position

        # 3+ tags
        A = np.column_stack((2 * Ps[:, 0], 2 * Ps[:, 1], -np.ones(n)))
        b = (Ps[:, 0] ** 2 + Ps[:, 1] ** 2 - Ds**2).reshape(-1, 1)

        U, S, Vt = np.linalg.svd(A, full_matrices=False)
        x = Vt.T @ (np.linalg.inv(np.diag(S)) @ (U.T @ b))

        position = x[:2, 0]

        if refine_position:
            position = self.refine_position(position, Ps, Ds)

        # print(f"Initial estimated position: {position}")
        # print(f"Refined position: {refined_position}")
        # print(f"Distance errors: {Ds - np.linalg.norm(Ps[:, :2] - position, axis=1)}")
        # print(f"Distance errors refined: {Ds - np.linalg.norm(Ps[:, :2] - refined_position, axis=1)}")
        # print()

        return position

    def refine_position(
        self,
        initial_position,
        beacon_positions,
        distances,
        tolerance=1e-6,
        max_iter=20,
    ):
        def iterative_trilateration(position):
            distance_errors = np.abs(
                (distances - np.linalg.norm(beacon_positions[:, :2] - position, axis=1))
                / distances
            )
            return np.mean(distance_errors)

        minimization_result = minimize(
            iterative_trilateration,
            initial_position.flatten(),
            method="L-BFGS-B",
            tol=tolerance,
            bounds=((-3, 3), (-4.5, 4.5)),
            options={"maxiter": max_iter},
        )

        refined_position = minimization_result.x
        return refined_position


def main():
    np.random.seed(10)

    n_iters = 200

    trilateration = Trilateration()
    true_position = np.array((1, 2))
    visible_ids = [4, 6, 8]
    noise_std = [0.5, 0.5, 0.5]

    estimated_positions = []
    refined_positions = []
    for _ in range(n_iters):
        tag_position_noise = np.random.normal(0, noise_std)
        rover_xy, tag_positions, distances = trilateration.trilateration(
            visible_ids, true_position, tag_position_noise, refine_position=False
        )
        estimated_positions.append(rover_xy)

        refined_xy, _, _ = trilateration.trilateration(
            visible_ids, true_position, tag_position_noise, refine_position=True
        )
        refined_positions.append(refined_xy)

    estimated_positions = np.array(estimated_positions)
    refined_positions = np.array(refined_positions)
    tag_positions = np.array(tag_positions)

    fig, axes = plt.subplots(ncols=2, nrows=1, figsize=(12, 8))
    axes[0].scatter(estimated_positions[:, 0], estimated_positions[:, 1], s=20, color="red", label="predicted")
    axes[0].title.set_text('Estimated position')

    axes[1].scatter(refined_positions[:, 0], refined_positions[:, 1], s=20, color="red", label="predicted")
    axes[1].title.set_text('Refined position')

    for ax in axes:
        ax.scatter(true_position[0], true_position[1], s=30, color="lime", label="GT", zorder=99)
        ax.scatter(tag_positions[:, 0], tag_positions[:, 1], s=50, color="black", label="tags", zorder=99)
        ax.hlines(0, -4, 4, color="gray")
        ax.vlines(0, -6, 6, color="gray")
        ax.grid(visible=True)
        ax.set_xlim(-4, 4)
        ax.set_ylim(-6, 6)
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc="upper right")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

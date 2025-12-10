import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import rerun as rr
from scipy.spatial.transform import Rotation as Rot

ROOT = Path(__file__).parent


def init_rerun():
    rr.init("maze")
    rr.connect_tcp()

    blueprint = ROOT / "rerun_blueprints" / "pointcloud_preprocessing.rbl"
    rr.log_file_from_path(blueprint)


def mask_pointcloud(pointcloud, mask):
    filtered_points = np.array(pointcloud.points)[mask]
    filtered_pcd = o3d.geometry.PointCloud()
    filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

    if pointcloud.has_colors():
        colors = np.asarray(pointcloud.colors)[mask]
        filtered_pcd.colors = o3d.utility.Vector3dVector(colors)

    return filtered_pcd


def mean_std_pointcloud(pointcloud):
    mean, cov = pointcloud.compute_mean_and_covariance()
    return mean, cov


def mahalanobis_distance_2d(pointcloud):
    points = np.asarray(pointcloud.points)
    xy = points[:, :2]

    mean_3d, cov_3d = mean_std_pointcloud(pointcloud)
    mean_xy = mean_3d[:2]
    cov_xy = cov_3d[:2, :2]

    diff = xy - mean_xy
    inv_cov = np.linalg.inv(cov_xy)

    mahalanobis_distances = np.sqrt(np.einsum("ij,jk,ik->i", diff, inv_cov, diff))
    return mahalanobis_distances


def mahalanobis_filter(pointcloud, max_distance):
    mahalanobis_distances = mahalanobis_distance_2d(pointcloud)
    mask = mahalanobis_distances <= max_distance
    filtered_pointcloud = mask_pointcloud(pointcloud, mask)
    return filtered_pointcloud


def log_mean_std_pointcloud(pointcloud, entity="cloud"):
    mean, cov = mean_std_pointcloud(pointcloud)
    pointcloud_entity = f"world/{entity}"
    rr.log(
        f"{pointcloud_entity}/stats/mean",
        rr.Points3D([mean], radii=0.04, colors=[[255, 0, 0]], labels=["mean"]),
    )

    eigvals, eigvecs = np.linalg.eigh(cov)

    n_sigmas = [1, 2, 3]
    colors_list = [
        [0, 180, 255, 128],  # light blue
        [255, 255, 0, 128],  # yellow
        [255, 100, 0, 128],  # red-orange
    ]
    for n_sigma, color in zip(n_sigmas, colors_list):
        half_sizes = n_sigma * np.sqrt(eigvals)

        if np.linalg.det(eigvecs) < 0:
            eigvecs[:, 2] *= -1

        R = eigvecs
        quat = Rot.from_matrix(R).as_quat()

        rr.log(
            f"{pointcloud_entity}/stats/cov/{n_sigma}σ",
            rr.Ellipsoids3D(
                centers=[mean],
                half_sizes=[half_sizes],
                quaternions=[quat],
                colors=[color],
                fill_mode=rr.components.FillMode.MajorWireframe,
            ),
        )


def log_pointcloud(pointcloud, entity="cloud"):
    points = np.asarray(pointcloud.points)
    colors = np.asarray(pointcloud.colors) if pointcloud.has_colors() else None
    rr.log("world/" + entity, rr.Points3D(positions=points, colors=colors, radii=0.005))


def main(args=None):
    if args.rerun:
        init_rerun()

    # Load pointcloud from file
    pointcloud_file = ROOT / "clouds" / "cloud.ply"
    pointcloud = o3d.io.read_point_cloud(pointcloud_file)
    # Log pointcloud
    log_pointcloud(pointcloud, "cloud")
    log_mean_std_pointcloud(pointcloud, "cloud")

    # Filter the pointcloud to only keep points that are no further than 1 sigma from
    # the mean
    mahalanobis_cloud = mahalanobis_filter(pointcloud, max_distance=1)
    log_pointcloud(mahalanobis_cloud, "mahalanobis_cloud")
    log_mean_std_pointcloud(mahalanobis_cloud, "mahalanobis_cloud")

    loose_cl, ind = mahalanobis_cloud.remove_radius_outlier(nb_points=32, radius=0.075)
    strict_cl, ind = mahalanobis_cloud.remove_radius_outlier(nb_points=32, radius=0.05)

    # Filter the pointcloud to only keep points that are no further than 3 sigma from
    # the mean
    loose_cl = mahalanobis_filter(loose_cl, max_distance=3)
    strict_cl = mahalanobis_filter(strict_cl, max_distance=3)

    log_pointcloud(loose_cl, "loose_cloud")
    log_pointcloud(strict_cl, "strict_cloud")

    if args.write:
        out_cloud_path = ROOT/ "clouds" / "filtered_cloud.ply"
        o3d.io.write_point_cloud(out_cloud_path, strict_cl)
        print(f"Written filtered pointcloud: `{out_cloud_path}`")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--rerun", action="store_true", help="Enable Rerun logging")
    parser.add_argument("-w", "--write", action="store_true", help="Write filtered point cloud to `clouds/filtered_cloud.ply`")
    parsed_args = parser.parse_args()
    main(parsed_args)

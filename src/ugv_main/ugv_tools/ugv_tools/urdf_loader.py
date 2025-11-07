#!/usr/bin/env python3
import contextlib
import io
from pathlib import Path
from typing import Optional

import numpy as np
import scipy.spatial.transform as st
import trimesh
import trimesh.visual
from PIL import Image
from urdf_parser_py import urdf as urdf_parser

import rerun as rr


def resolve_ros_path(path_str: str) -> str:
    """Resolve a ROS path to an absolute path."""
    if path_str.startswith("package://"):
        path = Path(path_str)
        package_name = path.parts[1]
        relative_path = Path(*path.parts[2:])

        package_path = resolve_ros1_package(package_name) or resolve_ros2_package(
            package_name
        )

        if package_path is None:
            raise ValueError(
                f"Could not resolve {path}."
                f"Replace with relative / absolute path, source the correct ROS environment, or install {package_name}."
            )

        return str(package_path / relative_path)
    elif path_str.startswith("file://"):
        return path_str[len("file://") :]
    else:
        return path_str


def resolve_ros2_package(package_name: str) -> Optional[str]:
    try:
        import ament_index_python

        try:
            return ament_index_python.get_package_share_directory(package_name)
        except ament_index_python.packages.PackageNotFoundError:
            return None
    except ImportError:
        return None


def resolve_ros1_package(package_name: str) -> Optional[str]:
    return None


class URDFLogger:
    """Class to log a URDF to Rerun."""

    def __init__(self, filepath: str, entity_path_prefix: str = "") -> None:
        # Suppress all output including warnings
        with (
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            if str(filepath).endswith("xacro"):
                import xacro

                xacro_doc = xacro.parse(open(filepath, encoding="utf-8"))
                xacro.process_doc(xacro_doc)
                self.urdf = urdf_parser.URDF.from_xml_string(xacro_doc.toxml())
            else:
                self.urdf = urdf_parser.URDF.from_xml_file(filepath)

        self.filepath = filepath
        self.root_filepath = Path(filepath).parent
        self.entity_path_prefix = entity_path_prefix
        self.mat_name_to_mat = {mat.name: mat for mat in self.urdf.materials}

    def link_entity_path(self, link: urdf_parser.Link) -> str:
        """Return the entity path for the URDF link."""
        root_name = self.urdf.get_root()
        link_names = self.urdf.get_chain(root_name, link.name)[0::2]  # skip the joints
        return self.add_entity_path_prefix("/".join(link_names))

    def joint_entity_path(self, joint: urdf_parser.Joint) -> str:
        """Return the entity path for the URDF joint."""
        root_name = self.urdf.get_root()
        joint_names = self.urdf.get_chain(root_name, joint.child)[
            0::2
        ]  # skip the links
        return self.add_entity_path_prefix("/".join(joint_names))

    def add_entity_path_prefix(self, entity_path: str) -> str:
        """Add prefix (if passed) to entity path."""
        if self.entity_path_prefix is not None:
            return f"{self.entity_path_prefix}/{entity_path}"
        return entity_path

    def log(self, recording: rr.RecordingStream) -> None:
        """Log a URDF file to Rerun."""
        for joint in self.urdf.joints:
            entity_path = self.joint_entity_path(joint)
            self.log_joint(entity_path, joint, recording)

        for link in self.urdf.links:
            entity_path = self.link_entity_path(link)
            self.log_link(entity_path, link, recording)

    def log_link(
        self, entity_path: str, link: urdf_parser.Link, recording: rr.RecordingStream
    ) -> None:
        """Log a URDF link to Rerun."""

        # create one mesh out of all visuals
        for i, visual in enumerate(link.visuals):
            self.log_visual(entity_path + f"/visual_{i}", visual, recording=recording)

    def log_joint(
        self, entity_path: str, joint: urdf_parser.Joint, recording: rr.RecordingStream
    ) -> None:
        """Log a URDF joint to Rerun."""
        # origin_to_transform(joint.origin)
        # if joint.origin is not None and joint.origin.xyz is not None:
        #     translation = joint.origin.xyz
        # if joint.origin is not None and joint.origin.rpy is not None:
        #     rotation = st.Rotation.from_euler("xyz", joint.origin.rpy).as_matrix()
        transform = origin_to_transform(joint.origin)

        if transform is not None:
            recording.log(
                entity_path,
                transform,
                static=True,
            )

    def log_visual(
        self,
        entity_path: str,
        visual: urdf_parser.Visual,
        recording: rr.RecordingStream,
    ) -> None:
        """Log a URDF visual to Rerun."""
        material = None
        if visual.material is not None:
            if visual.material.color is None and visual.material.texture is None:
                # use globally defined material
                material = self.mat_name_to_mat[visual.material.name]
            else:
                material = visual.material

        transform = origin_to_transform(visual.origin)

        if isinstance(visual.geometry, urdf_parser.Mesh):
            resolved_path = self.root_filepath / resolve_ros_path(
                visual.geometry.filename
            )
            mesh_scale = visual.geometry.scale
            mesh_or_scene = trimesh.load_mesh(resolved_path)
            if mesh_scale is not None:
                if transform is not None:
                    transform.scale = mesh_scale
                else:
                    transform = rr.Transform3D(scale=mesh_scale)
        elif isinstance(visual.geometry, urdf_parser.Box):
            mesh_or_scene = trimesh.creation.box(extents=visual.geometry.size)
        elif isinstance(visual.geometry, urdf_parser.Cylinder):
            mesh_or_scene = trimesh.creation.cylinder(
                radius=visual.geometry.radius,
                height=visual.geometry.length,
            )
        elif isinstance(visual.geometry, urdf_parser.Sphere):
            mesh_or_scene = trimesh.creation.icosphere(
                radius=visual.geometry.radius,
            )
        else:
            recording.log(
                "",
                rr.TextLog("Unsupported geometry type: " + str(type(visual.geometry))),
            )
            mesh_or_scene = trimesh.Trimesh()

        if isinstance(mesh_or_scene, trimesh.Scene):
            scene = mesh_or_scene
            # use dump to apply scene graph transforms and get a list of transformed meshes
            for i, mesh in enumerate(scene_to_trimeshes(scene)):
                if material is not None and not isinstance(
                    mesh.visual, trimesh.visual.texture.TextureVisuals
                ):
                    if material.color is not None:
                        mesh.visual = trimesh.visual.ColorVisuals()
                        mesh.visual.vertex_colors = material.color.rgba
                    elif material.texture is not None:
                        texture_path = resolve_ros_path(material.texture.filename)
                        mesh.visual = trimesh.visual.texture.TextureVisuals(
                            image=Image.open(texture_path)
                        )
                log_trimesh(entity_path + f"/{i}", mesh, transform, recording=recording)
        else:
            mesh = mesh_or_scene
            if material is not None and not isinstance(
                mesh.visual, trimesh.visual.texture.TextureVisuals
            ):
                if material.color is not None:
                    mesh.visual = trimesh.visual.ColorVisuals()
                    mesh.visual.vertex_colors = material.color.rgba
                elif material.texture is not None:
                    texture_path = resolve_ros_path(material.texture.filename)
                    mesh.visual = trimesh.visual.texture.TextureVisuals(
                        image=Image.open(texture_path)
                    )
            log_trimesh(entity_path, mesh, transform, recording=recording)


def scene_to_trimeshes(scene: trimesh.Scene) -> list[trimesh.Trimesh]:
    """
    Convert a trimesh.Scene to a list of trimesh.Trimesh.

    Skips objects that are not an instance of trimesh.Trimesh.
    """
    trimeshes = []
    scene_dump = scene.dump()
    geometries = [scene_dump] if not isinstance(scene_dump, list) else scene_dump
    for geometry in geometries:
        if isinstance(geometry, trimesh.Trimesh):
            trimeshes.append(geometry)
        elif isinstance(geometry, trimesh.Scene):
            trimeshes.extend(scene_to_trimeshes(geometry))
    return trimeshes


def origin_to_transform(origin: urdf_parser.Pose | None) -> rr.Transform3D | None:
    """Convert a URDF origin to a Rerun transform."""
    if origin is None:
        return None

    if origin.xyz is not None:
        translation = origin.xyz
    if origin.rpy is not None:
        rotation = st.Rotation.from_euler("xyz", origin.rpy).as_quat()

    if translation is not None and rotation is not None:
        return rr.Transform3D(translation=translation, quaternion=rotation)
    else:
        return None


def log_trimesh(
    entity_path: str,
    mesh: trimesh.Trimesh,
    transform: rr.Transform3D | None,
    recording: rr.RecordingStream,
) -> None:
    vertex_colors = albedo_texture = vertex_texcoords = None

    if isinstance(mesh.visual, trimesh.visual.color.ColorVisuals):
        vertex_colors = mesh.visual.vertex_colors
    elif isinstance(mesh.visual, trimesh.visual.texture.TextureVisuals):
        trimesh_material = mesh.visual.material

        if mesh.visual.uv is not None:
            vertex_texcoords = mesh.visual.uv
            # Trimesh uses the OpenGL convention for UV coordinates, so we need to flip the V coordinate
            # since Rerun uses the Vulkan/Metal/DX12/WebGPU convention.
            vertex_texcoords[:, 1] = 1.0 - vertex_texcoords[:, 1]

        if isinstance(trimesh_material, trimesh.visual.material.PBRMaterial):
            if trimesh_material.baseColorTexture is not None:
                albedo_texture = pil_image_to_albedo_texture(
                    trimesh_material.baseColorTexture
                )
            elif trimesh_material.baseColorFactor is not None:
                vertex_colors = trimesh_material.baseColorFactor
        elif isinstance(trimesh_material, trimesh.visual.material.SimpleMaterial):
            if trimesh_material.image is not None:
                albedo_texture = pil_image_to_albedo_texture(trimesh_material.image)
            else:
                vertex_colors = mesh.visual.to_color().vertex_colors

    recording.log(
        entity_path,
        rr.Mesh3D(
            vertex_positions=mesh.vertices,
            triangle_indices=mesh.faces,
            vertex_normals=mesh.vertex_normals,
            vertex_colors=vertex_colors,
            albedo_texture=albedo_texture,
            vertex_texcoords=vertex_texcoords,
        ),
        static=True,
    )

    if transform is not None:
        recording.log(entity_path, transform, static=True)


def pil_image_to_albedo_texture(image: Image.Image) -> np.ndarray:
    """Convert a PIL image to an albedo texture."""
    albedo_texture = np.asarray(image)
    if albedo_texture.ndim == 2:
        # If the texture is grayscale, we need to convert it to RGB since
        # Rerun expects a 3-channel texture.
        # See: https://github.com/rerun-io/rerun/issues/4878
        albedo_texture = np.stack([albedo_texture] * 3, axis=-1)
    return albedo_texture

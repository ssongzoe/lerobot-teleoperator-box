"""Coordinate-frame conversions for Meta Quest controller poses."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from .vr_protocol import VrPose


# Coordinate conversion used by the original RBY1 VR teleoperation example.
#
# Quest pose:
#     T_quest
#
# RBY1-compatible controller pose:
#     T_conv.T @ T_quest @ T_conv
#
# Axis correspondence:
#     Quest X -> RBY1 Z
#     Quest Y -> RBY1 -X
#     Quest Z -> RBY1 Y
T_CONV = np.array(
    [
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def controller_pose_to_rby1(
    pose: VrPose,
    *,
    side: str,
) -> np.ndarray:
    """Convert a Quest controller pose to the RBY1 controller frame.

    Args:
        pose:
            Controller position and quaternion reported by Meta Quest.

        side:
            Controller side. Must be ``"right"`` or ``"left"``.

            Both sides currently use the same coordinate transformation. The
            argument is retained so side-specific transforms can be introduced
            later without changing the public interface.

    Returns:
        A 4×4 homogeneous transformation matrix.
    """

    if side not in {"right", "left"}:
        raise ValueError(
            f"side must be 'right' or 'left', received {side!r}."
        )

    quest_pose = pose_to_matrix(pose)

    converted_pose = (
        T_CONV.T
        @ quest_pose
        @ T_CONV
    )

    return normalize_transform(converted_pose)


def pose_to_matrix(pose: VrPose) -> np.ndarray:
    """Convert a ``VrPose`` into a homogeneous transformation matrix.

    Quest quaternions are expected in SciPy order:

        [x, y, z, w]
    """

    position = np.asarray(
        pose.position,
        dtype=np.float64,
    )

    quaternion = np.asarray(
        pose.rotation,
        dtype=np.float64,
    )

    if position.shape != (3,):
        raise ValueError(
            "VR position must have shape (3,), "
            f"received {position.shape}."
        )

    if quaternion.shape != (4,):
        raise ValueError(
            "VR quaternion must have shape (4,), "
            f"received {quaternion.shape}."
        )

    if not np.all(np.isfinite(position)):
        raise ValueError(
            "VR position contains non-finite values."
        )

    if not np.all(np.isfinite(quaternion)):
        raise ValueError(
            "VR quaternion contains non-finite values."
        )

    quaternion_norm = float(
        np.linalg.norm(quaternion)
    )

    if quaternion_norm < 1e-8:
        raise ValueError(
            "VR quaternion has zero length."
        )

    quaternion = quaternion / quaternion_norm

    transform = np.eye(
        4,
        dtype=np.float64,
    )

    transform[:3, :3] = R.from_quat(
        quaternion
    ).as_matrix()

    transform[:3, 3] = position

    return transform


def matrix_to_pose(
    transform: np.ndarray,
) -> VrPose:
    """Convert a homogeneous transformation matrix to ``VrPose``."""

    transform = normalize_transform(transform)

    position = tuple(
        float(value)
        for value in transform[:3, 3]
    )

    quaternion = tuple(
        float(value)
        for value in R.from_matrix(
            transform[:3, :3]
        ).as_quat()
    )

    return VrPose(
        position=position,
        rotation=quaternion,
    )


def normalize_transform(
    transform: np.ndarray,
) -> np.ndarray:
    """Validate and normalize a homogeneous transformation matrix."""

    matrix = np.asarray(
        transform,
        dtype=np.float64,
    )

    if matrix.shape != (4, 4):
        raise ValueError(
            "Transformation matrix must have shape (4, 4), "
            f"received {matrix.shape}."
        )

    if not np.all(np.isfinite(matrix)):
        raise ValueError(
            "Transformation matrix contains non-finite values."
        )

    normalized = matrix.copy()

    # Project the rotation block onto the nearest valid rotation matrix.
    u, _, vh = np.linalg.svd(
        normalized[:3, :3]
    )

    rotation = u @ vh

    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vh

    normalized[:3, :3] = rotation
    normalized[3] = [0.0, 0.0, 0.0, 1.0]

    return normalized
"""Utilities for converting Cartesian poses to and from LeRobot dictionaries."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as R


def observation_to_pose(
    observation: Mapping[str, Any],
    *,
    prefix: str,
) -> np.ndarray | None:
    """Extract a Cartesian end-effector pose from a robot observation.

    Supported representations, in priority order:

    1. A homogeneous transformation matrix::

           observation[f"{prefix}.pose"]       -> shape (4, 4)
           observation[f"{prefix}.transform"]  -> shape (4, 4)
           observation[prefix]                 -> shape (4, 4)

    2. Position and rotation-vector features::

           {prefix}.x
           {prefix}.y
           {prefix}.z
           {prefix}.wx
           {prefix}.wy
           {prefix}.wz

    3. Position and quaternion features::

           {prefix}.x
           {prefix}.y
           {prefix}.z
           {prefix}.qx
           {prefix}.qy
           {prefix}.qz
           {prefix}.qw

    4. A six-value pose vector::

           observation[f"{prefix}.pose"]
               = [x, y, z, wx, wy, wz]

    Args:
        observation:
            Latest observation returned by the robot.

        prefix:
            End-effector feature prefix such as ``"right_ee"`` or
            ``"left_ee"``.

    Returns:
        A 4×4 homogeneous transformation matrix, or ``None`` when the
        observation does not contain a supported pose representation.
    """

    for key in (
        f"{prefix}.pose",
        f"{prefix}.transform",
        prefix,
    ):
        if key not in observation:
            continue

        value = _to_numpy(observation[key])

        if value.shape == (4, 4):
            return validate_transform(
                value,
                name=key,
            )

        if value.shape == (16,):
            return validate_transform(
                value.reshape(4, 4),
                name=key,
            )

        if value.shape == (6,):
            return xyz_rotvec_to_pose(
                value[:3],
                value[3:],
            )

        if value.shape == (7,):
            return xyz_quaternion_to_pose(
                value[:3],
                value[3:],
            )

    rotvec_feature_names = (
        f"{prefix}.x",
        f"{prefix}.y",
        f"{prefix}.z",
        f"{prefix}.wx",
        f"{prefix}.wy",
        f"{prefix}.wz",
    )

    if all(name in observation for name in rotvec_feature_names):
        values = np.array(
            [
                _to_scalar(observation[name], name=name)
                for name in rotvec_feature_names
            ],
            dtype=np.float64,
        )

        return xyz_rotvec_to_pose(
            values[:3],
            values[3:],
        )

    quaternion_feature_names = (
        f"{prefix}.x",
        f"{prefix}.y",
        f"{prefix}.z",
        f"{prefix}.qx",
        f"{prefix}.qy",
        f"{prefix}.qz",
        f"{prefix}.qw",
    )

    if all(name in observation for name in quaternion_feature_names):
        values = np.array(
            [
                _to_scalar(observation[name], name=name)
                for name in quaternion_feature_names
            ],
            dtype=np.float64,
        )

        return xyz_quaternion_to_pose(
            values[:3],
            values[3:],
        )

    position_key = f"{prefix}.position"
    rotation_key = f"{prefix}.rotation"

    if (
        position_key in observation
        and rotation_key in observation
    ):
        position = _to_numpy(
            observation[position_key]
        ).reshape(-1)

        rotation = _to_numpy(
            observation[rotation_key]
        ).reshape(-1)

        if rotation.shape == (3,):
            return xyz_rotvec_to_pose(
                position,
                rotation,
            )

        if rotation.shape == (4,):
            return xyz_quaternion_to_pose(
                position,
                rotation,
            )

        raise ValueError(
            f"'{rotation_key}' must contain either 3 rotation-vector "
            f"values or 4 quaternion values, received shape {rotation.shape}."
        )

    return None


def pose_to_action(
    pose: np.ndarray,
    *,
    prefix: str,
) -> dict[str, float]:
    """Convert a Cartesian pose to the robot action representation.

    The returned representation is:

        x, y, z, wx, wy, wz

    where ``wx, wy, wz`` are an axis-angle rotation vector in radians.

    Args:
        pose:
            Cartesian target as a 4×4 homogeneous transformation matrix.

        prefix:
            Action feature prefix such as ``"right_ee"`` or ``"left_ee"``.

    Returns:
        Dictionary containing six scalar action features.
    """

    pose = validate_transform(
        pose,
        name="pose",
    )

    position = pose[:3, 3]
    rotation_vector = R.from_matrix(
        pose[:3, :3]
    ).as_rotvec()

    return {
        f"{prefix}.x": float(position[0]),
        f"{prefix}.y": float(position[1]),
        f"{prefix}.z": float(position[2]),
        f"{prefix}.wx": float(rotation_vector[0]),
        f"{prefix}.wy": float(rotation_vector[1]),
        f"{prefix}.wz": float(rotation_vector[2]),
    }


def action_to_pose(
    action: Mapping[str, Any],
    *,
    prefix: str,
) -> np.ndarray:
    """Convert six Cartesian action features into a pose matrix."""

    feature_names = (
        f"{prefix}.x",
        f"{prefix}.y",
        f"{prefix}.z",
        f"{prefix}.wx",
        f"{prefix}.wy",
        f"{prefix}.wz",
    )

    missing = [
        name
        for name in feature_names
        if name not in action
    ]

    if missing:
        raise KeyError(
            "Cartesian action is missing required features: "
            + ", ".join(missing)
        )

    values = np.array(
        [
            _to_scalar(action[name], name=name)
            for name in feature_names
        ],
        dtype=np.float64,
    )

    return xyz_rotvec_to_pose(
        values[:3],
        values[3:],
    )


def xyz_rotvec_to_pose(
    position: Sequence[float] | np.ndarray,
    rotation_vector: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """Build a pose matrix from XYZ position and rotation vector."""

    position_array = _validate_vector(
        position,
        length=3,
        name="position",
    )

    rotation_array = _validate_vector(
        rotation_vector,
        length=3,
        name="rotation_vector",
    )

    pose = np.eye(
        4,
        dtype=np.float64,
    )

    pose[:3, :3] = R.from_rotvec(
        rotation_array
    ).as_matrix()

    pose[:3, 3] = position_array

    return pose


def xyz_quaternion_to_pose(
    position: Sequence[float] | np.ndarray,
    quaternion: Sequence[float] | np.ndarray,
) -> np.ndarray:
    """Build a pose matrix from XYZ position and XYZW quaternion."""

    position_array = _validate_vector(
        position,
        length=3,
        name="position",
    )

    quaternion_array = _validate_vector(
        quaternion,
        length=4,
        name="quaternion",
    )

    quaternion_norm = float(
        np.linalg.norm(quaternion_array)
    )

    if quaternion_norm < 1e-8:
        raise ValueError(
            "quaternion must not have zero length."
        )

    quaternion_array = (
        quaternion_array / quaternion_norm
    )

    pose = np.eye(
        4,
        dtype=np.float64,
    )

    pose[:3, :3] = R.from_quat(
        quaternion_array
    ).as_matrix()

    pose[:3, 3] = position_array

    return pose


def pose_to_xyz_rotvec(
    pose: np.ndarray,
) -> np.ndarray:
    """Convert a pose matrix to ``[x, y, z, wx, wy, wz]``."""

    pose = validate_transform(
        pose,
        name="pose",
    )

    return np.concatenate(
        [
            pose[:3, 3],
            R.from_matrix(
                pose[:3, :3]
            ).as_rotvec(),
        ]
    )


def validate_transform(
    transform: np.ndarray,
    *,
    name: str,
) -> np.ndarray:
    """Validate and return a normalized homogeneous transformation."""

    matrix = np.asarray(
        transform,
        dtype=np.float64,
    )

    if matrix.shape != (4, 4):
        raise ValueError(
            f"{name} must have shape (4, 4), "
            f"received {matrix.shape}."
        )

    if not np.all(np.isfinite(matrix)):
        raise ValueError(
            f"{name} contains non-finite values."
        )

    if not np.allclose(
        matrix[3],
        [0.0, 0.0, 0.0, 1.0],
        atol=1e-6,
    ):
        raise ValueError(
            f"{name} is not a valid homogeneous transformation."
        )

    normalized = matrix.copy()

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


def _validate_vector(
    value: Sequence[float] | np.ndarray,
    *,
    length: int,
    name: str,
) -> np.ndarray:
    """Validate a finite one-dimensional numeric vector."""

    array = np.asarray(
        value,
        dtype=np.float64,
    ).reshape(-1)

    if array.shape != (length,):
        raise ValueError(
            f"{name} must contain {length} values, "
            f"received shape {array.shape}."
        )

    if not np.all(np.isfinite(array)):
        raise ValueError(
            f"{name} contains non-finite values."
        )

    return array


def _to_numpy(
    value: Any,
) -> np.ndarray:
    """Convert a tensor-like value to a NumPy array."""

    if hasattr(value, "detach"):
        value = value.detach()

    if hasattr(value, "cpu"):
        value = value.cpu()

    if hasattr(value, "numpy"):
        value = value.numpy()

    return np.asarray(
        value,
        dtype=np.float64,
    )


def _to_scalar(
    value: Any,
    *,
    name: str,
) -> float:
    """Convert a scalar or one-element tensor-like value to float."""

    array = _to_numpy(value)

    if array.size != 1:
        raise ValueError(
            f"'{name}' must be scalar, received shape {array.shape}."
        )

    result = float(
        array.reshape(-1)[0]
    )

    if not np.isfinite(result):
        raise ValueError(
            f"'{name}' must be finite."
        )

    return result
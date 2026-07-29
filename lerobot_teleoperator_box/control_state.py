"""Clutch and Cartesian target state for VR teleoperation."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R


class ArmControlState:
    """Maintain clutch anchors and Cartesian targets for one robot arm.

    When the controller grip is pressed, the current VR controller pose and
    measured robot end-effector pose are stored as anchors.

    While the grip remains pressed, controller motion relative to the anchor is
    applied to the robot end-effector anchor.

    When the grip is released, the most recent target remains locked.
    """

    def __init__(self, side: str):
        if side not in {"right", "left"}:
            raise ValueError(
                f"side must be 'right' or 'left', received {side!r}."
            )

        self.side = side

        self._is_following = False

        self._controller_start_pose: np.ndarray | None = None
        self._robot_start_pose: np.ndarray | None = None
        self._last_target_pose: np.ndarray | None = None

    @property
    def is_following(self) -> bool:
        """Whether the controller currently drives this arm."""

        return self._is_following

    @property
    def is_initialized(self) -> bool:
        """Whether this arm has produced at least one valid target."""

        return self._last_target_pose is not None

    @property
    def last_target_pose(self) -> np.ndarray | None:
        """Return a copy of the most recent Cartesian target."""

        if self._last_target_pose is None:
            return None

        return self._last_target_pose.copy()

    def reset(self) -> None:
        """Clear clutch anchors and the previous target."""

        self._is_following = False

        self._controller_start_pose = None
        self._robot_start_pose = None
        self._last_target_pose = None

    def set_hold_target(self, pose: np.ndarray) -> np.ndarray:
        """Replace the arm hold target and clear VR clutch anchors."""

        pose = self._validate_pose(
            pose,
            name="pose",
        )
        self._is_following = False
        self._controller_start_pose = None
        self._robot_start_pose = None
        self._last_target_pose = pose.copy()
        return self.last_target_pose

    def update(
        self,
        *,
        controller_pose: np.ndarray,
        robot_pose: np.ndarray | None,
        clutch_pressed: bool,
        position_scale: float,
        rotation_scale: float,
        max_position_delta_m: float,
        max_rotation_delta_rad: float,
        require_initialization_button: bool,
    ) -> np.ndarray | None:
        """Update and return the Cartesian target pose.

        Args:
            controller_pose:
                Current controller pose as a 4×4 homogeneous transformation.

            robot_pose:
                Current measured robot end-effector pose as a 4×4 homogeneous
                transformation. This is used when establishing a new clutch
                anchor.

            clutch_pressed:
                Whether the Quest grip button is currently pressed.

            position_scale:
                Scale applied to controller translation relative to the clutch
                anchor.

            rotation_scale:
                Scale applied to controller rotation relative to the clutch
                anchor.

            max_position_delta_m:
                Maximum translation change allowed between consecutive targets.

            max_rotation_delta_rad:
                Maximum rotation change allowed between consecutive targets.

            require_initialization_button:
                When true, the arm begins following only after the grip button
                is pressed. When false, following begins automatically as soon
                as valid controller and robot poses are available.

        Returns:
            A 4×4 Cartesian target pose, or ``None`` if no valid target has
            been initialized yet.
        """

        controller_pose = self._validate_pose(
            controller_pose,
            name="controller_pose",
        )

        if robot_pose is not None:
            robot_pose = self._validate_pose(
                robot_pose,
                name="robot_pose",
            )

        self._validate_parameters(
            position_scale=position_scale,
            rotation_scale=rotation_scale,
            max_position_delta_m=max_position_delta_m,
            max_rotation_delta_rad=max_rotation_delta_rad,
        )

        should_follow = (
            clutch_pressed
            if require_initialization_button
            else True
        )

        if not should_follow:
            self._is_following = False
            self._controller_start_pose = None
            self._robot_start_pose = None

            return self.last_target_pose

        if not self._is_following:
            anchor_pose = self._select_robot_anchor(robot_pose)

            if anchor_pose is None:
                return None

            self._controller_start_pose = controller_pose.copy()
            self._robot_start_pose = anchor_pose.copy()
            self._last_target_pose = anchor_pose.copy()
            self._is_following = True

            return self.last_target_pose

        if (
            self._controller_start_pose is None
            or self._robot_start_pose is None
        ):
            self._is_following = False
            return self.last_target_pose

        controller_delta = (
            np.linalg.inv(self._controller_start_pose)
            @ controller_pose
        )

        scaled_delta = self._scale_pose_delta(
            controller_delta,
            position_scale=position_scale,
            rotation_scale=rotation_scale,
        )

        target_pose = self._apply_controller_delta(
            robot_start_pose=self._robot_start_pose,
            controller_delta=scaled_delta,
        )

        if self._last_target_pose is not None:
            target_pose = self._limit_target_step(
                previous_pose=self._last_target_pose,
                target_pose=target_pose,
                max_position_delta_m=max_position_delta_m,
                max_rotation_delta_rad=max_rotation_delta_rad,
            )

        self._last_target_pose = target_pose

        return self.last_target_pose

    def _select_robot_anchor(
        self,
        robot_pose: np.ndarray | None,
    ) -> np.ndarray | None:
        """Choose the robot pose used when a new clutch begins."""

        if robot_pose is not None:
            return robot_pose

        if self._last_target_pose is not None:
            return self._last_target_pose

        return None

    @staticmethod
    def _apply_controller_delta(
        *,
        robot_start_pose: np.ndarray,
        controller_delta: np.ndarray,
    ) -> np.ndarray:
        """Apply controller-relative motion in the robot base frame.

        This follows the transformation used in the original RBY1 VR example.
        The controller-relative motion is rotated using the initial robot
        end-effector orientation before being applied to the robot pose.
        """

        global_to_start = np.eye(4, dtype=np.float64)
        global_to_start[:3, :3] = robot_start_pose[:3, :3].T

        delta_in_base = (
            global_to_start
            @ controller_delta
            @ global_to_start.T
        )

        target_pose = robot_start_pose @ delta_in_base
        target_pose[3] = [0.0, 0.0, 0.0, 1.0]

        return target_pose

    @staticmethod
    def _scale_pose_delta(
        pose_delta: np.ndarray,
        *,
        position_scale: float,
        rotation_scale: float,
    ) -> np.ndarray:
        """Scale relative controller translation and rotation."""

        scaled_delta = np.eye(4, dtype=np.float64)

        scaled_delta[:3, 3] = (
            pose_delta[:3, 3] * position_scale
        )

        rotation_vector = R.from_matrix(
            pose_delta[:3, :3]
        ).as_rotvec()

        scaled_delta[:3, :3] = R.from_rotvec(
            rotation_vector * rotation_scale
        ).as_matrix()

        return scaled_delta

    @staticmethod
    def _limit_target_step(
        *,
        previous_pose: np.ndarray,
        target_pose: np.ndarray,
        max_position_delta_m: float,
        max_rotation_delta_rad: float,
    ) -> np.ndarray:
        """Limit Cartesian movement between consecutive targets."""

        limited_pose = target_pose.copy()

        position_delta = (
            target_pose[:3, 3]
            - previous_pose[:3, 3]
        )

        position_distance = float(
            np.linalg.norm(position_delta)
        )

        if (
            max_position_delta_m > 0.0
            and position_distance > max_position_delta_m
        ):
            position_delta *= (
                max_position_delta_m
                / position_distance
            )

            limited_pose[:3, 3] = (
                previous_pose[:3, 3]
                + position_delta
            )

        relative_rotation = (
            previous_pose[:3, :3].T
            @ target_pose[:3, :3]
        )

        rotation_vector = R.from_matrix(
            relative_rotation
        ).as_rotvec()

        rotation_angle = float(
            np.linalg.norm(rotation_vector)
        )

        if (
            max_rotation_delta_rad > 0.0
            and rotation_angle > max_rotation_delta_rad
        ):
            rotation_vector *= (
                max_rotation_delta_rad
                / rotation_angle
            )

            limited_pose[:3, :3] = (
                previous_pose[:3, :3]
                @ R.from_rotvec(rotation_vector).as_matrix()
            )

        limited_pose[3] = [0.0, 0.0, 0.0, 1.0]

        return limited_pose

    @staticmethod
    def _validate_pose(
        pose: np.ndarray,
        *,
        name: str,
    ) -> np.ndarray:
        """Validate and copy a homogeneous transformation matrix."""

        array = np.asarray(
            pose,
            dtype=np.float64,
        )

        if array.shape != (4, 4):
            raise ValueError(
                f"{name} must have shape (4, 4), "
                f"received {array.shape}."
            )

        if not np.all(np.isfinite(array)):
            raise ValueError(
                f"{name} contains non-finite values."
            )

        if not np.allclose(
            array[3],
            [0.0, 0.0, 0.0, 1.0],
            atol=1e-6,
        ):
            raise ValueError(
                f"{name} is not a valid homogeneous transformation."
            )

        rotation = array[:3, :3]

        if not np.allclose(
            rotation.T @ rotation,
            np.eye(3),
            atol=1e-4,
        ):
            raise ValueError(
                f"{name} contains an invalid rotation matrix."
            )

        if not np.isclose(
            np.linalg.det(rotation),
            1.0,
            atol=1e-4,
        ):
            raise ValueError(
                f"{name} rotation determinant must be 1."
            )

        return array.copy()

    @staticmethod
    def _validate_parameters(
        *,
        position_scale: float,
        rotation_scale: float,
        max_position_delta_m: float,
        max_rotation_delta_rad: float,
    ) -> None:
        """Validate scaling and safety-limit parameters."""

        parameters = {
            "position_scale": position_scale,
            "rotation_scale": rotation_scale,
            "max_position_delta_m": max_position_delta_m,
            "max_rotation_delta_rad": max_rotation_delta_rad,
        }

        for name, value in parameters.items():
            if not np.isfinite(value):
                raise ValueError(
                    f"{name} must be finite."
                )

            if value < 0.0:
                raise ValueError(
                    f"{name} must be greater than or equal to zero."
                )

class TorsoControlState:
    """Maintain the HMD clutch anchors and Cartesian torso target.

    The torso follows only while both Quest grip buttons are pressed.  At the
    rising edge, the current HMD pose and measured ``link_torso_5`` pose are
    captured as anchors.  Relative HMD rotation and vertical translation are
    then applied to the torso anchor, matching the original RB-Y1 VR example.

    Horizontal HMD translation is intentionally ignored so leaning forward or
    sideways does not translate the torso target in the robot base frame.
    """

    def __init__(self) -> None:
        self._is_following = False
        self._head_start_pose: np.ndarray | None = None
        self._torso_start_pose: np.ndarray | None = None
        self._last_target_pose: np.ndarray | None = None

    @property
    def is_following(self) -> bool:
        return self._is_following

    @property
    def last_target_pose(self) -> np.ndarray | None:
        if self._last_target_pose is None:
            return None
        return self._last_target_pose.copy()

    def reset(self) -> None:
        self._is_following = False
        self._head_start_pose = None
        self._torso_start_pose = None
        self._last_target_pose = None

    def set_hold_target(self, pose: np.ndarray) -> np.ndarray:
        """Replace the torso hold target and clear HMD-follow anchors."""

        pose = ArmControlState._validate_pose(
            pose,
            name="pose",
        )
        self._is_following = False
        self._head_start_pose = None
        self._torso_start_pose = None
        self._last_target_pose = pose.copy()
        return self.last_target_pose

    def update(
        self,
        *,
        head_pose: np.ndarray | None,
        robot_pose: np.ndarray | None,
        both_grips_pressed: bool,
        position_scale: float,
        rotation_scale: float,
    ) -> np.ndarray | None:
        """Update and return the torso Cartesian target.

        ``head_pose`` and ``robot_pose`` are 4×4 transforms expressed in the
        converted Quest frame and RB-Y1 base frame respectively.  Only HMD Z
        translation and relative rotation are used.
        """

        if head_pose is not None:
            head_pose = ArmControlState._validate_pose(
                head_pose,
                name="head_pose",
            )
        if robot_pose is not None:
            robot_pose = ArmControlState._validate_pose(
                robot_pose,
                name="robot_pose",
            )

        for name, value in {
            "position_scale": position_scale,
            "rotation_scale": rotation_scale,
        }.items():
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be a finite value greater than or equal to zero."
                )
            
        if not both_grips_pressed or head_pose is None:
            self._is_following = False
            self._head_start_pose = None
            self._torso_start_pose = None

            # Initialize the torso hold target once from the measured pose.
            # Without this, BoxVr falls back to the current measured pose every
            # frame, causing any physical drift to become the next command target.
            if self._last_target_pose is None and robot_pose is not None:
                self._last_target_pose = robot_pose.copy()

            return self.last_target_pose

        if not self._is_following:
            anchor_pose = robot_pose
            if anchor_pose is None:
                anchor_pose = self._last_target_pose
            if anchor_pose is None:
                return None

            self._head_start_pose = head_pose.copy()
            self._torso_start_pose = anchor_pose.copy()
            self._last_target_pose = anchor_pose.copy()
            self._is_following = True
            return self.last_target_pose

        if self._head_start_pose is None or self._torso_start_pose is None:
            self._is_following = False
            return self.last_target_pose

        delta = np.linalg.inv(self._head_start_pose) @ head_pose

        # Match the original example: ignore HMD X/Y translation, retain Z.
        delta[0, 3] = 0.0
        delta[1, 3] = 0.0

        scaled_delta = ArmControlState._scale_pose_delta(
            delta,
            position_scale=position_scale,
            rotation_scale=rotation_scale,
        )

        target_pose = self._torso_start_pose @ scaled_delta
        target_pose[3] = [0.0, 0.0, 0.0, 1.0]
        self._last_target_pose = target_pose

        return self.last_target_pose

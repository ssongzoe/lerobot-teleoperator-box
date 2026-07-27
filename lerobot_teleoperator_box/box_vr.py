"""Meta Quest VR teleoperator for bimanual RBY1 Cartesian control."""

from __future__ import annotations

from typing import Any

import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction

from .config_box_vr import BoxVrConfig
from .constants import (
    LEFT_EE_FEATURES,
    LEFT_GRIPPER_FEATURE,
    RIGHT_EE_FEATURES,
    RIGHT_GRIPPER_FEATURE,
)
from .control_state import ArmControlState
from .frame_transforms import controller_pose_to_rby1
from .pose_utils import observation_to_pose, pose_to_action
from .vr_receiver import VrReceiver


class BoxVr(Teleoperator):
    """Bimanual Cartesian teleoperator using Meta Quest controllers.

    Each arm produces a 6D Cartesian target:

        x, y, z, wx, wy, wz

    where ``wx, wy, wz`` form a rotation vector.

    The teleoperator itself does not perform inverse kinematics. The returned
    Cartesian targets are passed directly to the RBY1 robot implementation,
    which uses its existing Cartesian controller.
    """

    config_class = BoxVrConfig
    name = "box_vr"

    def __init__(self, config: BoxVrConfig):
        super().__init__(config)

        self.config = config
        self.receiver = VrReceiver(config)

        self.right_state = ArmControlState(side="right")
        self.left_state = ArmControlState(side="left")

        self._is_connected = False

        # Most recent measured robot end-effector poses.
        self._right_robot_pose: np.ndarray | None = None
        self._left_robot_pose: np.ndarray | None = None

        # Most recently returned action.
        self._last_action: RobotAction | None = None

    @property
    def action_features(self) -> dict[str, type]:
        """Features produced by :meth:`get_action`."""

        features: dict[str, type] = {}

        if self.config.use_right_arm:
            features.update({name: float for name in RIGHT_EE_FEATURES})

        if self.config.use_left_arm:
            features.update({name: float for name in LEFT_EE_FEATURES})

        if self.config.use_gripper:
            if self.config.use_right_arm:
                features[RIGHT_GRIPPER_FEATURE] = float

            if self.config.use_left_arm:
                features[LEFT_GRIPPER_FEATURE] = float

        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        """Box VR currently does not accept force or haptic feedback."""

        return {}

    @property
    def is_connected(self) -> bool:
        """Whether the UDP receiver is active."""

        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """No hardware calibration is required for the VR receiver."""

        return True

    def connect(self, calibrate: bool = True) -> None:
        """Open the UDP receiver and optionally send the Quest handshake."""

        if self.is_connected:
            raise RuntimeError(f"{self} is already connected.")

        self.receiver.connect()
        self._is_connected = True

        if self.config.send_handshake:
            self.receiver.send_handshake()

        self.configure()

    def calibrate(self) -> None:
        """No-op because this teleoperator has no motor calibration."""

        return None

    def configure(self) -> None:
        """Reset transient clutch and target state."""

        self.right_state.reset()
        self.left_state.reset()

        self._right_robot_pose = None
        self._left_robot_pose = None
        self._last_action = None

    def update_robot_observation(
        self,
        observation: dict[str, Any],
    ) -> None:
        """Update the measured robot poses used as clutch anchors.

        This method should be called with the latest observation before
        :meth:`get_action` whenever the recording loop supports it.

        Args:
            observation: Observation returned by ``robot.get_observation()``.
        """

        if self.config.use_right_arm:
            self._right_robot_pose = observation_to_pose(
                observation=observation,
                prefix="right_ee",
            )

        if self.config.use_left_arm:
            self._left_robot_pose = observation_to_pose(
                observation=observation,
                prefix="left_ee",
            )

    def get_action(self) -> RobotAction:
        """Receive one VR packet and produce a Cartesian robot action."""

        if not self.is_connected:
            raise RuntimeError(
                f"{self} is not connected. Call connect() before get_action()."
            )

        packet = self.receiver.receive()

        if packet is None:
            return self._handle_missing_packet()

        action: RobotAction = {}

        if self.config.use_right_arm:
            right_controller_pose = controller_pose_to_rby1(
                packet.right.pose,
                side="right",
            )

            right_target = self.right_state.update(
                controller_pose=right_controller_pose,
                robot_pose=self._right_robot_pose,
                clutch_pressed=packet.right.grip_pressed,
                position_scale=self.config.position_scale,
                rotation_scale=self.config.rotation_scale,
                max_position_delta_m=self.config.max_position_delta_m,
                max_rotation_delta_rad=self.config.max_rotation_delta_rad,
                require_initialization_button=(
                    self.config.require_initialization_button
                ),
            )

            if right_target is not None:
                action.update(
                    pose_to_action(
                        pose=right_target,
                        prefix="right_ee",
                    )
                )

            if self.config.use_gripper:
                action[RIGHT_GRIPPER_FEATURE] = float(packet.right.trigger)

        if self.config.use_left_arm:
            left_controller_pose = controller_pose_to_rby1(
                packet.left.pose,
                side="left",
            )

            left_target = self.left_state.update(
                controller_pose=left_controller_pose,
                robot_pose=self._left_robot_pose,
                clutch_pressed=packet.left.grip_pressed,
                position_scale=self.config.position_scale,
                rotation_scale=self.config.rotation_scale,
                max_position_delta_m=self.config.max_position_delta_m,
                max_rotation_delta_rad=self.config.max_rotation_delta_rad,
                require_initialization_button=(
                    self.config.require_initialization_button
                ),
            )

            if left_target is not None:
                action.update(
                    pose_to_action(
                        pose=left_target,
                        prefix="left_ee",
                    )
                )

            if self.config.use_gripper:
                action[LEFT_GRIPPER_FEATURE] = float(packet.left.trigger)

        action = self._fill_missing_features(action)
        self._last_action = dict(action)

        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        """Ignore feedback because Quest haptics are not implemented yet."""

        if not self.is_connected:
            raise RuntimeError(
                f"{self} is not connected. Call connect() before send_feedback()."
            )

        if feedback:
            raise ValueError(
                "BoxVr does not currently support feedback features."
            )

    def disconnect(self) -> None:
        """Close the UDP receiver and clear transient state."""

        if not self.is_connected:
            return

        self.receiver.close()
        self._is_connected = False

        self.right_state.reset()
        self.left_state.reset()

        self._right_robot_pose = None
        self._left_robot_pose = None
        self._last_action = None

    def _handle_missing_packet(self) -> RobotAction:
        """Return the previous action when a UDP packet is unavailable."""

        if self.config.hold_last_target and self._last_action is not None:
            return dict(self._last_action)

        raise RuntimeError(
            "No valid VR packet was received and no previous target is available."
        )

    def _fill_missing_features(
        self,
        action: RobotAction,
    ) -> RobotAction:
        """Fill temporarily unavailable targets from the previous action.

        An arm may not yet produce a target when its clutch has never been
        initialized. Previously valid values are reused when available.
        """

        expected_features = self.action_features

        for feature_name in expected_features:
            if feature_name in action:
                continue

            if (
                self.config.hold_last_target
                and self._last_action is not None
                and feature_name in self._last_action
            ):
                action[feature_name] = self._last_action[feature_name]
                continue

            raise RuntimeError(
                f"Action feature '{feature_name}' is unavailable. "
                "Provide the latest robot observation and initialize the "
                "corresponding VR controller clutch."
            )

        return action
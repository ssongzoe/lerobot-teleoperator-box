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
        self._latest_robot_observation = None


        # Read-only RB-Y1 connection used only for state reading and FK.
        self._robot = None
        self._model = None
        self._dyn_robot = None
        self._dyn_state = None

        self._idx_base = 0
        self._idx_right_arm_6 = 1
        self._idx_left_arm_6 = 2


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
        if self.is_connected:
            raise RuntimeError(f"{self} is already connected.")

        self._connect_robot_state_reader()

        try:
            self.receiver.connect()
        except Exception:
            self._disconnect_robot_state_reader()
            raise

        self._is_connected = True

        if self.config.send_handshake:
            self.receiver.send_handshake()

        self.configure()

    def _connect_robot_state_reader(self) -> None:
        """Open a read-only RB-Y1 connection for current EE pose calculation."""

        try:
            import rby1_sdk as rby
        except ImportError as exc:
            raise ImportError(
                "rby1_sdk is required for BoxVr robot state reading."
            ) from exc

        self._robot = rby.create_robot(
            self.config.robot_address,
            self.config.robot_model,
        )

        if not self._robot.connect():
            self._robot = None
            raise ConnectionError(
                f"Failed to connect to RB-Y1 at "
                f"{self.config.robot_address} for state reading."
            )

        self._model = self._robot.model()
        self._dyn_robot = self._robot.get_dynamics()

        self._dyn_state = self._dyn_robot.make_state(
            [
                "base",
                "link_right_arm_6",
                "link_left_arm_6",
            ],
            self._model.robot_joint_names,
        )


    def _update_robot_poses_from_state(self) -> None:
        """Read the current robot state and calculate both EE poses with FK."""

        if (
            self._robot is None
            or self._dyn_robot is None
            or self._dyn_state is None
        ):
            raise RuntimeError(
                "RB-Y1 state reader is not connected."
            )

        state = self._robot.get_state()

        self._dyn_state.set_q(
            np.asarray(state.position, dtype=np.float64).copy()
        )
        self._dyn_robot.compute_forward_kinematics(self._dyn_state)

        if self.config.use_right_arm:
            self._right_robot_pose = np.asarray(
                self._dyn_robot.compute_transformation(
                    self._dyn_state,
                    self._idx_base,
                    self._idx_right_arm_6,
                ),
                dtype=np.float64,
            )

        if self.config.use_left_arm:
            self._left_robot_pose = np.asarray(
                self._dyn_robot.compute_transformation(
                    self._dyn_state,
                    self._idx_base,
                    self._idx_left_arm_6,
                ),
                dtype=np.float64,
            )


    def _disconnect_robot_state_reader(self) -> None:
        """Close the read-only RB-Y1 connection."""

        if self._robot is not None:
            self._robot.disconnect()

        self._robot = None
        self._model = None
        self._dyn_robot = None
        self._dyn_state = None





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

    def set_robot_observation(self, observation) -> None:
        self._latest_robot_observation = dict(observation)


    def get_action(self) -> RobotAction:
        """Receive one VR packet and produce a Cartesian robot action."""

        if not self.is_connected:
            raise RuntimeError(
                f"{self} is not connected. Call connect() before get_action()."
            )

        packet = self.receiver.receive()

        if packet is None:
            return self._handle_missing_packet()

        # 현재 joint state를 읽고 FK로 양팔 EE pose 계산
        self._update_robot_poses_from_state()

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

            # 클러치 초기화 전에는 현재 로봇 EE pose 유지
            if right_target is None:
                if self._right_robot_pose is None:
                    raise RuntimeError(
                        "Current right EE pose is unavailable."
                    )
                right_target = self._right_robot_pose.copy()

            action.update(
                pose_to_action(
                    pose=right_target,
                    prefix="right_ee",
                )
            )

            if self.config.use_gripper:
                action[RIGHT_GRIPPER_FEATURE] = float(
                    packet.right.trigger
                )

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

            # 클러치 초기화 전에는 현재 로봇 EE pose 유지
            if left_target is None:
                if self._left_robot_pose is None:
                    raise RuntimeError(
                        "Current left EE pose is unavailable."
                    )
                left_target = self._left_robot_pose.copy()

            action.update(
                pose_to_action(
                    pose=left_target,
                    prefix="left_ee",
                )
            )

            if self.config.use_gripper:
                action[LEFT_GRIPPER_FEATURE] = float(
                    packet.left.trigger
                )

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
        self._disconnect_robot_state_reader()
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
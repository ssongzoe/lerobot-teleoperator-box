"""Meta Quest VR teleoperator for bimanual RBY1 Cartesian control."""

from __future__ import annotations

from typing import Any

import logging
import threading
import time

import numpy as np

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot.types import RobotAction

from .config_box_vr import BoxVrConfig
from .constants import (
    BASE_VEL_FEATURES,
    TORSO_EE_FEATURES,
    LEFT_EE_FEATURES,
    LEFT_GRIPPER_FEATURE,
    RIGHT_EE_FEATURES,
    RIGHT_GRIPPER_FEATURE,
)
from .control_state import ArmControlState, TorsoControlState
from .frame_transforms import controller_pose_to_rby1, vr_pose_to_rby1
from .pose_utils import observation_to_pose, pose_to_action
from .vr_receiver import VrReceiver


logger = logging.getLogger(__name__)


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

        self.torso_state = TorsoControlState()
        self.right_state = ArmControlState(side="right")
        self.left_state = ArmControlState(side="left")

        self._is_connected = False

        # Most recent measured robot Cartesian poses.
        self._torso_robot_pose: np.ndarray | None = None
        self._right_robot_pose: np.ndarray | None = None
        self._left_robot_pose: np.ndarray | None = None

        # Most recently returned action.
        self._last_action: RobotAction | None = None
        self._latest_robot_observation = None

        # Smoothed mobile-base velocity state.
        self._mobile_linear_velocity = np.zeros(2, dtype=np.float64)
        self._mobile_angular_velocity = 0.0
        self._last_mobile_update_time: float | None = None


        # Read-only RB-Y1 connection used only for asynchronous state
        # subscription and FK. get_action() never performs an SDK RPC.
        self._robot = None
        self._model = None
        self._dyn_robot = None
        self._dyn_state = None

        self._robot_pose_lock = threading.Lock()
        self._first_robot_state_event = threading.Event()
        self._last_robot_state_time: float | None = None
        self._state_update_started = False

        self._idx_base = 0
        self._idx_torso_5 = 1
        self._idx_right_arm_6 = 2
        self._idx_left_arm_6 = 3


    @property
    def action_features(self) -> dict[str, type]:
        """Features produced by :meth:`get_action`."""

        features: dict[str, type] = {}

        if self.config.use_torso:
            features.update({name: float for name in TORSO_EE_FEATURES})

        if self.config.use_right_arm:
            features.update({name: float for name in RIGHT_EE_FEATURES})

        if self.config.use_left_arm:
            features.update({name: float for name in LEFT_EE_FEATURES})

        if self.config.use_mobile_base:
            features.update({name: float for name in BASE_VEL_FEATURES})

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
        """Subscribe to RB-Y1 state updates and cache EE poses asynchronously.

        The additional SDK connection is read-only. Unlike the previous
        implementation, :meth:`get_action` does not call ``get_state()`` or run
        FK synchronously; the SDK callback updates a small pose cache instead.
        """

        try:
            import rby1_sdk as rby
        except ImportError as exc:
            raise ImportError(
                "rby1_sdk is required for BoxVr robot state reading."
            ) from exc

        update_rate_hz = float(self.config.robot_state_update_rate_hz)
        if not np.isfinite(update_rate_hz) or update_rate_hz <= 0.0:
            raise ValueError(
                "robot_state_update_rate_hz must be a positive finite value."
            )

        initial_timeout_s = float(self.config.robot_state_initial_timeout_s)
        if not np.isfinite(initial_timeout_s) or initial_timeout_s <= 0.0:
            raise ValueError(
                "robot_state_initial_timeout_s must be a positive finite value."
            )

        self._first_robot_state_event.clear()
        with self._robot_pose_lock:
            self._torso_robot_pose = None
            self._right_robot_pose = None
            self._left_robot_pose = None
            self._last_robot_state_time = None

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

        try:
            self._model = self._robot.model()
            self._dyn_robot = self._robot.get_dynamics()

            self._dyn_state = self._dyn_robot.make_state(
                [
                    "base",
                    "link_torso_5",
                    "link_right_arm_6",
                    "link_left_arm_6",
                ],
                self._model.robot_joint_names,
            )

            started = self._robot.start_state_update(
                self._robot_state_callback,
                update_rate_hz,
            )
            if started is False:
                raise RuntimeError(
                    "RB-Y1 start_state_update() returned False."
                )
            self._state_update_started = True

            if not self._first_robot_state_event.wait(
                timeout=initial_timeout_s
            ):
                raise TimeoutError(
                    "Timed out waiting for the first RB-Y1 state update "
                    f"after {initial_timeout_s:.2f}s."
                )

        except Exception:
            self._disconnect_robot_state_reader()
            raise

        logger.info(
            "RB-Y1 asynchronous state reader started at %.1f Hz.",
            update_rate_hz,
        )

    def _robot_state_callback(self, state: Any) -> None:
        """Run FK from an SDK state callback and atomically cache EE poses."""

        if self._dyn_robot is None or self._dyn_state is None:
            return

        try:
            self._dyn_state.set_q(
                np.asarray(state.position, dtype=np.float64).copy()
            )
            self._dyn_robot.compute_forward_kinematics(self._dyn_state)

            torso_pose = None
            right_pose = None
            left_pose = None

            if self.config.use_torso:
                torso_pose = np.asarray(
                    self._dyn_robot.compute_transformation(
                        self._dyn_state,
                        self._idx_base,
                        self._idx_torso_5,
                    ),
                    dtype=np.float64,
                ).copy()

            if self.config.use_right_arm:
                right_pose = np.asarray(
                    self._dyn_robot.compute_transformation(
                        self._dyn_state,
                        self._idx_base,
                        self._idx_right_arm_6,
                    ),
                    dtype=np.float64,
                ).copy()

            if self.config.use_left_arm:
                left_pose = np.asarray(
                    self._dyn_robot.compute_transformation(
                        self._dyn_state,
                        self._idx_base,
                        self._idx_left_arm_6,
                    ),
                    dtype=np.float64,
                ).copy()

            with self._robot_pose_lock:
                if torso_pose is not None:
                    self._torso_robot_pose = torso_pose
                if right_pose is not None:
                    self._right_robot_pose = right_pose
                if left_pose is not None:
                    self._left_robot_pose = left_pose
                self._last_robot_state_time = time.monotonic()

            self._first_robot_state_event.set()

        except Exception:
            logger.exception(
                "Failed to update cached RB-Y1 EE poses from state callback."
            )

    def _get_cached_robot_poses(
        self,
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        """Return fresh copies of the latest callback-computed Cartesian poses."""

        max_age_s = float(self.config.robot_state_max_age_s)
        if not np.isfinite(max_age_s) or max_age_s <= 0.0:
            raise ValueError(
                "robot_state_max_age_s must be a positive finite value."
            )

        with self._robot_pose_lock:
            torso_pose = (
                None
                if self._torso_robot_pose is None
                else self._torso_robot_pose.copy()
            )
            right_pose = (
                None
                if self._right_robot_pose is None
                else self._right_robot_pose.copy()
            )
            left_pose = (
                None
                if self._left_robot_pose is None
                else self._left_robot_pose.copy()
            )
            last_update_time = self._last_robot_state_time

        if last_update_time is None:
            raise RuntimeError(
                "No RB-Y1 state has been received by the asynchronous reader."
            )

        age_s = time.monotonic() - last_update_time
        if age_s > max_age_s:
            raise RuntimeError(
                "Cached RB-Y1 state is stale: "
                f"age={age_s:.3f}s, limit={max_age_s:.3f}s."
            )

        return torso_pose, right_pose, left_pose

    def _disconnect_robot_state_reader(self) -> None:
        """Stop state subscription and close the read-only RB-Y1 handle."""

        robot = self._robot

        if robot is not None and self._state_update_started:
            stop_state_update = getattr(robot, "stop_state_update", None)
            if callable(stop_state_update):
                try:
                    stop_state_update()
                except Exception:
                    logger.exception(
                        "Failed to stop the RB-Y1 state update callback."
                    )

        self._state_update_started = False

        if robot is not None:
            try:
                robot.disconnect()
            except Exception:
                logger.exception(
                    "Failed to disconnect the RB-Y1 state reader."
                )

        self._robot = None
        self._model = None
        self._dyn_robot = None
        self._dyn_state = None

        self._first_robot_state_event.clear()
        with self._robot_pose_lock:
            self._torso_robot_pose = None
            self._right_robot_pose = None
            self._left_robot_pose = None
            self._last_robot_state_time = None




    def calibrate(self) -> None:
        """No-op because this teleoperator has no motor calibration."""

        return None

    def configure(self) -> None:
        """Reset transient clutch and target state."""

        self.torso_state.reset()
        self.right_state.reset()
        self.left_state.reset()

        # Robot-pose cache is owned by the asynchronous state callback and is
        # intentionally preserved across transient teleoperator resets.
        self._mobile_linear_velocity.fill(0.0)
        self._mobile_angular_velocity = 0.0
        self._last_mobile_update_time = None
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

        torso_pose = None
        right_pose = None
        left_pose = None

        if self.config.use_torso:
            torso_pose = observation_to_pose(
                observation=observation,
                prefix="torso_ee",
            )

        if self.config.use_right_arm:
            right_pose = observation_to_pose(
                observation=observation,
                prefix="right_ee",
            )

        if self.config.use_left_arm:
            left_pose = observation_to_pose(
                observation=observation,
                prefix="left_ee",
            )

        if torso_pose is None and right_pose is None and left_pose is None:
            return

        with self._robot_pose_lock:
            if torso_pose is not None:
                self._torso_robot_pose = torso_pose.copy()
            if right_pose is not None:
                self._right_robot_pose = right_pose.copy()
            if left_pose is not None:
                self._left_robot_pose = left_pose.copy()
            self._last_robot_state_time = time.monotonic()

        self._first_robot_state_event.set()

    def set_robot_observation(self, observation: dict[str, Any]) -> None:
        """Accept an observation when a custom LeRobot loop provides one."""

        self._latest_robot_observation = dict(observation)
        self.update_robot_observation(self._latest_robot_observation)


    def _torso_clutch_pressed(self, packet: Any) -> bool:
        """Return whether the configured torso clutch input is held.

        Quest button naming follows the packet convention:
        left primary/secondary are X/Y and right primary/secondary are A/B.
        ``both_grips`` preserves the previous behaviour for compatibility.
        """

        button = str(self.config.torso_clutch_button).strip().lower()

        if button == "left_secondary":
            return bool(packet.left.secondary_button)
        if button == "left_primary":
            return bool(packet.left.primary_button)
        if button == "right_secondary":
            return bool(packet.right.secondary_button)
        if button == "right_primary":
            return bool(packet.right.primary_button)
        if button == "both_grips":
            return bool(
                packet.right.grip_pressed
                and packet.left.grip_pressed
            )

        raise ValueError(
            "Unsupported torso_clutch_button="
            f"{self.config.torso_clutch_button!r}. Expected one of: "
            "left_secondary, left_primary, right_secondary, "
            "right_primary, both_grips."
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

        # Copies only: no SDK RPC or FK runs on the LeRobot action thread.
        torso_robot_pose, right_robot_pose, left_robot_pose = (
            self._get_cached_robot_poses()
        )

        action: RobotAction = {}

        # Compute the base command before the torso target so the torso can
        # decide whether to hold a fixed target or follow the measured pose.
        mobile_action: RobotAction = {}
        base_is_moving = False
        if self.config.use_mobile_base:
            mobile_action = self._update_mobile_base_action(packet)

            motion_threshold = float(
                self.config.torso_base_motion_threshold
            )
            if not np.isfinite(motion_threshold) or motion_threshold < 0.0:
                raise ValueError(
                    "torso_base_motion_threshold must be a finite, "
                    "non-negative value."
                )

            base_is_moving = any(
                abs(float(mobile_action[name])) > motion_threshold
                for name in BASE_VEL_FEATURES
            )

        if self.config.use_torso:
            head_pose = (
                None
                if packet.head is None
                else vr_pose_to_rby1(packet.head.pose)
            )

            torso_clutch_pressed = self._torso_clutch_pressed(packet)

            if (
                self.config.torso_follow_measured_pose_while_base_moving
                and base_is_moving
                and not torso_clutch_pressed
            ):
                if torso_robot_pose is None:
                    raise RuntimeError(
                        "Current torso EE pose is unavailable while the "
                        "mobile base is moving."
                    )

                # Make the latest measured pose the new hold target. This
                # minimizes Cartesian error during base acceleration and also
                # prevents a jump back to the pre-drive target after stopping.
                torso_target = self.torso_state.synchronize_to_robot_pose(
                    torso_robot_pose
                )
            else:
                torso_target = self.torso_state.update(
                    head_pose=head_pose,
                    robot_pose=torso_robot_pose,
                    # TorsoControlState keeps its legacy argument name, but
                    # the clutch source is configurable. Default: left Y.
                    both_grips_pressed=torso_clutch_pressed,
                    position_scale=self.config.torso_position_scale,
                    rotation_scale=self.config.torso_rotation_scale,
                )

            if torso_target is None:
                if torso_robot_pose is None:
                    raise RuntimeError(
                        "Current torso EE pose is unavailable."
                    )
                torso_target = torso_robot_pose.copy()

            action.update(
                pose_to_action(
                    pose=torso_target,
                    prefix="torso_ee",
                )
            )

        if self.config.use_right_arm:
            right_controller_pose = controller_pose_to_rby1(
                packet.right.pose,
                side="right",
            )

            right_target = self.right_state.update(
                controller_pose=right_controller_pose,
                robot_pose=right_robot_pose,
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
                if right_robot_pose is None:
                    raise RuntimeError(
                        "Current right EE pose is unavailable."
                    )
                right_target = right_robot_pose.copy()

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
                robot_pose=left_robot_pose,
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
                if left_robot_pose is None:
                    raise RuntimeError(
                        "Current left EE pose is unavailable."
                    )
                left_target = left_robot_pose.copy()

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

        if self.config.use_mobile_base:
            action.update(mobile_action)

        action = self._fill_missing_features(action)
        self._last_action = dict(action)

        return action


    @staticmethod
    def _apply_thumbstick_deadzone(
        axis: tuple[float, float],
        deadzone: float,
    ) -> np.ndarray:
        """Return a 2D thumbstick value with radial deadzone removal."""

        value = np.asarray(axis, dtype=np.float64)
        magnitude = float(np.linalg.norm(value))

        if magnitude <= deadzone:
            return np.zeros(2, dtype=np.float64)

        if magnitude > 1.0:
            value = value / magnitude
            magnitude = 1.0

        # Re-scale the remaining range so motion begins continuously at zero.
        scaled_magnitude = (magnitude - deadzone) / max(1.0 - deadzone, 1e-6)
        return value / max(magnitude, 1e-9) * scaled_magnitude

    @staticmethod
    def _move_vector_towards(
        current: np.ndarray,
        target: np.ndarray,
        max_delta: float,
    ) -> np.ndarray:
        """Move a vector toward ``target`` by at most ``max_delta``."""

        delta = target - current
        distance = float(np.linalg.norm(delta))
        if distance <= max_delta or distance <= 1e-12:
            return target.copy()
        return current + delta * (max_delta / distance)

    @staticmethod
    def _move_scalar_towards(
        current: float,
        target: float,
        max_delta: float,
    ) -> float:
        """Move a scalar toward ``target`` by at most ``max_delta``."""

        delta = float(target - current)
        if abs(delta) <= max_delta:
            return float(target)
        return float(current + np.sign(delta) * max_delta)

    def _mobile_update_dt(self) -> float:
        """Return a bounded elapsed time for the mobile velocity ramp."""

        now = time.monotonic()
        if self._last_mobile_update_time is None:
            # Use the expected state/action period for the first update so the
            # first non-zero command also starts smoothly.
            rate_hz = max(float(self.config.robot_state_update_rate_hz), 1.0)
            dt = 1.0 / rate_hz
        else:
            dt = max(0.0, now - self._last_mobile_update_time)

        self._last_mobile_update_time = now
        return min(dt, max(float(self.config.mobile_max_update_dt_s), 1e-3))

    def _update_mobile_base_action(self, packet) -> RobotAction:
        """Map Quest thumbsticks to smoothly ramped body-frame velocity.

        Mapping:

        * right stick Y -> forward/backward ``x.vel``
        * right stick X -> lateral ``-y.vel``
        * left stick X  -> yaw ``-theta.vel``

        Thumbstick input defines a target velocity. The commanded velocity
        approaches that target over ``mobile_acceleration_time_s`` and returns
        to zero over ``mobile_deceleration_time_s`` when the stick is released.
        """

        deadzone = float(self.config.mobile_thumbstick_deadzone)
        if not 0.0 <= deadzone < 1.0:
            raise ValueError(
                "mobile_thumbstick_deadzone must be in [0, 1)."
            )

        acceleration_time = float(self.config.mobile_acceleration_time_s)
        deceleration_time = float(self.config.mobile_deceleration_time_s)
        if acceleration_time <= 0.0:
            raise ValueError("mobile_acceleration_time_s must be greater than 0.")
        if deceleration_time <= 0.0:
            raise ValueError("mobile_deceleration_time_s must be greater than 0.")

        max_linear = float(self.config.mobile_max_linear_velocity_mps)
        max_angular = float(self.config.mobile_max_angular_velocity_rps)
        if max_linear < 0.0:
            raise ValueError("mobile_max_linear_velocity_mps must be non-negative.")
        if max_angular < 0.0:
            raise ValueError("mobile_max_angular_velocity_rps must be non-negative.")

        right_axis = self._apply_thumbstick_deadzone(
            packet.right.thumbstick_axis,
            deadzone,
        )
        left_axis = self._apply_thumbstick_deadzone(
            packet.left.thumbstick_axis,
            deadzone,
        )

        linear_input = np.array(
            [right_axis[1], -right_axis[0]],
            dtype=np.float64,
        )
        angular_input = float(-left_axis[0])

        target_linear = linear_input * max_linear
        target_angular = angular_input * max_angular

        dt = self._mobile_update_dt()

        linear_released = float(np.linalg.norm(linear_input)) <= 1e-9
        angular_released = abs(angular_input) <= 1e-9

        linear_ramp_time = (
            deceleration_time if linear_released else acceleration_time
        )
        angular_ramp_time = (
            deceleration_time if angular_released else acceleration_time
        )

        linear_max_delta = (
            max_linear * dt / linear_ramp_time if max_linear > 0.0 else 0.0
        )
        angular_max_delta = (
            max_angular * dt / angular_ramp_time if max_angular > 0.0 else 0.0
        )

        self._mobile_linear_velocity = self._move_vector_towards(
            self._mobile_linear_velocity,
            target_linear,
            linear_max_delta,
        )
        self._mobile_angular_velocity = self._move_scalar_towards(
            self._mobile_angular_velocity,
            target_angular,
            angular_max_delta,
        )

        return {
            "x.vel": float(self._mobile_linear_velocity[0]),
            "y.vel": float(self._mobile_linear_velocity[1]),
            "theta.vel": float(self._mobile_angular_velocity),
        }



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

        self.torso_state.reset()
        self.right_state.reset()
        self.left_state.reset()

        self._mobile_linear_velocity.fill(0.0)
        self._mobile_angular_velocity = 0.0
        self._last_mobile_update_time = None
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
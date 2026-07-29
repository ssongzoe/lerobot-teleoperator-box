"""Configuration for the Box VR teleoperator."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("box_vr")
@dataclass
class BoxVrConfig(TeleoperatorConfig):
    """Configuration for bimanual Cartesian control using Meta Quest VR.

    The teleoperator outputs Cartesian end-effector targets using the
    existing RBY1 action representation:

        position: x, y, z
        rotation: wx, wy, wz

    Each arm therefore outputs 6 values. Both arms output 12 values in total.
    Gripper features are added only when ``use_gripper`` is enabled.
    """
    # RB-Y1 read-only connection for state and forward kinematics.
    robot_address: str = "192.168.30.1:50051"
    robot_model: str = "m"  # "a" | "m" | "ub"

    # The read-only SDK handle subscribes to state updates on its own thread.
    # get_action() only copies the cached FK poses and never calls get_state().
    robot_state_update_rate_hz: float = 30.0
    robot_state_initial_timeout_s: float = 3.0
    robot_state_max_age_s: float = 0.5

    # UDP address on this computer.
    local_ip: str = "0.0.0.0"
    local_port: int = 5005

    # Meta Quest address used for handshake messages.
    meta_quest_ip: str = ""
    meta_quest_port: int = 6000 #5005

    # Send an initial UDP handshake to the Meta Quest application.
    send_handshake: bool = True

    # Maximum time to wait for a VR packet.
    receive_timeout_s: float = 0.1

    # Cartesian component selection.
    use_torso: bool = False
    use_right_arm: bool = True
    use_left_arm: bool = True

    # Fixed torso-pose toggle. By default, each rising edge of the Meta Quest
    # left-controller Y button alternates A -> B -> A. Values are the six
    # torso joint angles in degrees and are converted to Cartesian targets by FK.
    torso_toggle_button: str = "left_secondary"
    torso_pose_a_deg: tuple[float, ...] = (0.0, 55.0, -60.0, 7.0, 0.0, 0.0)
    torso_pose_b_deg: tuple[float, ...] = (0.0, 82.0, -91.0, 30.0, -2.0, 0.0)
    torso_preset_duration_s: float = 1.5

    # During a torso preset transition, translate both arm Cartesian targets
    # by the same base-frame XYZ displacement as the torso EE. Arm orientations
    # remain unchanged, so the relative pose between both hands is preserved.
    torso_preset_arm_translation_follow_ratio: float = 1.0

    # Optional gripper control.
    use_gripper: bool = False

    # Optional omnidirectional mobile-base control. The right thumbstick
    # controls body-frame x/y velocity and the left thumbstick X axis
    # controls yaw velocity, matching the original RB-Y1 VR example.
    use_mobile_base: bool = False

    # Ignore small thumbstick drift around the neutral position.
    mobile_thumbstick_deadzone: float = 0.10

    # Time required to ramp from zero to the configured maximum speed while
    # the thumbstick is held fully. Releasing the stick ramps back to zero
    # over ``mobile_deceleration_time_s`` instead of stopping abruptly.
    mobile_acceleration_time_s: float = 0.50
    mobile_deceleration_time_s: float = 0.50

    # Tuned body-frame velocity limits for comfortable whole-body teleoperation.
    mobile_max_linear_velocity_mps: float = 0.34
    mobile_max_angular_velocity_rps: float = 0.39

    # Cap the elapsed time used by one velocity update. This prevents a single
    # delayed control-loop iteration from causing a large velocity jump.
    mobile_max_update_dt_s: float = 0.10

    # Require the controller clutch button before updating a target.
    require_initialization_button: bool = True

    # Keep the previous Cartesian target when no valid packet is received
    # or when the clutch button is released.
    hold_last_target: bool = True

    # Scale controller translation and rotation relative to the clutch anchor.
    position_scale: float = 1.0
    rotation_scale: float = 1.0

    # Per-update safety limits applied to the Cartesian target.
    max_position_delta_m: float = 0.05
    max_rotation_delta_rad: float = 0.20
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

    # Torso Cartesian clutch input. The default ``left_secondary`` is the
    # Meta Quest left-controller Y button. The torso follows only while the
    # selected input is held; releasing it keeps the last torso target.
    #
    # Supported values:
    #   left_secondary  = Y (default)
    #   left_primary    = X
    #   right_secondary = B
    #   right_primary   = A
    #   both_grips      = legacy behaviour
    torso_clutch_button: str = "left_secondary"

    # Relative HMD rotation and vertical translation are applied to the torso
    # anchor while horizontal HMD translation is ignored.
    torso_position_scale: float = 1.0
    torso_rotation_scale: float = 1.0

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

    # Final body-frame velocity limits. These defaults are 70% of the previous
    # 0.70 m/s and 0.70 rad/s limits.
    mobile_max_linear_velocity_mps: float = 0.49
    mobile_max_angular_velocity_rps: float = 0.49

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
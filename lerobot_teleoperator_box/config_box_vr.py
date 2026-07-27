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

    # Arm selection.
    use_right_arm: bool = True
    use_left_arm: bool = True

    # Optional gripper control.
    use_gripper: bool = False

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
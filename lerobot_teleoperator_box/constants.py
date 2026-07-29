"""Shared constants for the Box VR teleoperator."""

from __future__ import annotations

# Cartesian end-effector action representation:
#
#     position      = x, y, z        [m]
#     rotation      = wx, wy, wz     [rotation vector, rad]
#
# Each arm therefore contributes six action values.

TORSO_EE_PREFIX = "torso_ee"
RIGHT_EE_PREFIX = "right_ee"
LEFT_EE_PREFIX = "left_ee"


# Mobile-base velocity action representation in the robot body frame.
BASE_VEL_FEATURES: tuple[str, ...] = (
    "x.vel",
    "y.vel",
    "theta.vel",
)


TORSO_EE_FEATURES: tuple[str, ...] = (
    f"{TORSO_EE_PREFIX}.x",
    f"{TORSO_EE_PREFIX}.y",
    f"{TORSO_EE_PREFIX}.z",
    f"{TORSO_EE_PREFIX}.wx",
    f"{TORSO_EE_PREFIX}.wy",
    f"{TORSO_EE_PREFIX}.wz",
)


RIGHT_EE_FEATURES: tuple[str, ...] = (
    f"{RIGHT_EE_PREFIX}.x",
    f"{RIGHT_EE_PREFIX}.y",
    f"{RIGHT_EE_PREFIX}.z",
    f"{RIGHT_EE_PREFIX}.wx",
    f"{RIGHT_EE_PREFIX}.wy",
    f"{RIGHT_EE_PREFIX}.wz",
)


LEFT_EE_FEATURES: tuple[str, ...] = (
    f"{LEFT_EE_PREFIX}.x",
    f"{LEFT_EE_PREFIX}.y",
    f"{LEFT_EE_PREFIX}.z",
    f"{LEFT_EE_PREFIX}.wx",
    f"{LEFT_EE_PREFIX}.wy",
    f"{LEFT_EE_PREFIX}.wz",
)


# Normalized gripper targets.
#
# Expected range:
#
#     0.0 = open
#     1.0 = closed
#
# The exact direction can later be adjusted in BoxVr.get_action() if the
# physical left and right grippers use opposite conventions.

RIGHT_GRIPPER_FEATURE = "right_gripper_0"
LEFT_GRIPPER_FEATURE = "left_gripper_0"


BIMANUAL_EE_FEATURES: tuple[str, ...] = (
    *RIGHT_EE_FEATURES,
    *LEFT_EE_FEATURES,
)


BIMANUAL_ACTION_FEATURES: tuple[str, ...] = (
    *BIMANUAL_EE_FEATURES,
    RIGHT_GRIPPER_FEATURE,
    LEFT_GRIPPER_FEATURE,
)


# Meta Quest grip threshold used to activate Cartesian clutch control.
QUEST_GRIP_PRESSED_THRESHOLD = 0.8


# Expected dimensions.
CARTESIAN_FEATURES_PER_ARM = 6
BIMANUAL_CARTESIAN_ACTION_DIM = 12
BIMANUAL_CARTESIAN_GRIPPER_ACTION_DIM = 14
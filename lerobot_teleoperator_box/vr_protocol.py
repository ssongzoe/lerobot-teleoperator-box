"""Meta Quest VR controller packet definitions and JSON parser."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


@dataclass(frozen=True)
class VrPose:
    """Position and orientation reported by the Meta Quest.

    Attributes:
        position:
            XYZ position in the Quest coordinate frame, in meters.

        rotation:
            Quaternion in the order ``[x, y, z, w]``.
    """

    position: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True)
class VrButtons:
    """Button and analog input state for one Quest controller."""

    primary_button: bool
    secondary_button: bool

    trigger: float
    grip: float

    thumbstick_axis: tuple[float, float]


@dataclass(frozen=True)
class VrController:
    """Pose and button state for one Quest controller."""

    pose: VrPose
    buttons: VrButtons

    @property
    def grip_pressed(self) -> bool:
        """Whether the grip button is pressed strongly enough for clutching."""

        return self.buttons.grip > 0.8

    @property
    def trigger(self) -> float:
        """Normalized trigger value used for gripper control."""

        return self.buttons.trigger

    @property
    def primary_button(self) -> bool:
        return self.buttons.primary_button

    @property
    def secondary_button(self) -> bool:
        return self.buttons.secondary_button

    @property
    def thumbstick_axis(self) -> tuple[float, float]:
        return self.buttons.thumbstick_axis


@dataclass(frozen=True)
class VrHead:
    """Optional headset pose."""

    pose: VrPose


@dataclass(frozen=True)
class VrPacket:
    """One complete Meta Quest controller packet.

    Both hand controllers are required. If either controller is absent from a
    received JSON message, parsing fails and the receiver discards that packet.
    This prevents an untracked controller from producing an invalid robot
    target.
    """

    right: VrController
    left: VrController
    head: VrHead | None = None


def parse_vr_packet(
    payload: bytes | bytearray | memoryview | str | Mapping[str, Any],
) -> VrPacket:
    """Parse one Meta Quest JSON packet.

    Expected JSON structure::

        {
            "hands": {
                "right": {
                    "position": [x, y, z],
                    "rotation": [qx, qy, qz, qw],
                    "buttons": {
                        "primaryButton": false,
                        "secondaryButton": false,
                        "trigger": 0.0,
                        "grip": 0.0,
                        "thumbstickAxis": [x, y]
                    }
                },
                "left": {
                    "position": [x, y, z],
                    "rotation": [qx, qy, qz, qw],
                    "buttons": {
                        "primaryButton": false,
                        "secondaryButton": false,
                        "trigger": 0.0,
                        "grip": 0.0,
                        "thumbstickAxis": [x, y]
                    }
                }
            },
            "head": {
                "position": [x, y, z],
                "rotation": [qx, qy, qz, qw]
            }
        }

    Args:
        payload:
            Raw UTF-8 JSON bytes, a JSON string, or an already decoded mapping.

    Returns:
        Parsed and validated ``VrPacket``.

    Raises:
        TypeError:
            If the input or one of its fields has an unsupported type.

        ValueError:
            If required fields are missing, vectors have incorrect lengths,
            or values are not finite.
    """

    message = _decode_payload(payload)

    hands = _require_mapping(message, "hands")

    right_data = _require_mapping(hands, "right")
    left_data = _require_mapping(hands, "left")

    right = _parse_controller(right_data, side="right")
    left = _parse_controller(left_data, side="left")

    head = None
    if "head" in message and message["head"] is not None:
        head_data = _as_mapping(message["head"], field_name="head")
        head = VrHead(pose=_parse_pose(head_data, prefix="head"))

    return VrPacket(
        right=right,
        left=left,
        head=head,
    )


def _decode_payload(
    payload: bytes | bytearray | memoryview | str | Mapping[str, Any],
) -> Mapping[str, Any]:
    """Decode an input payload into a mapping."""

    if isinstance(payload, Mapping):
        return payload

    if isinstance(payload, (bytes, bytearray, memoryview)):
        try:
            payload = bytes(payload).decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ValueError("VR packet is not valid UTF-8.") from exc

    if not isinstance(payload, str):
        raise TypeError(
            "VR packet must be bytes, a JSON string, or a mapping. "
            f"Received {type(payload).__name__}."
        )

    if not payload.strip():
        raise ValueError("VR packet is empty.")

    decoded = json.loads(payload)

    if not isinstance(decoded, Mapping):
        raise TypeError(
            "The top-level VR JSON value must be an object."
        )

    return decoded


def _parse_controller(
    controller_data: Mapping[str, Any],
    *,
    side: str,
) -> VrController:
    """Parse one hand controller."""

    pose = _parse_pose(
        controller_data,
        prefix=f"hands.{side}",
    )

    buttons_data = _require_mapping(
        controller_data,
        "buttons",
        prefix=f"hands.{side}",
    )

    buttons = VrButtons(
        primary_button=_as_bool(
            buttons_data.get("primaryButton", False),
            field_name=f"hands.{side}.buttons.primaryButton",
        ),
        secondary_button=_as_bool(
            buttons_data.get("secondaryButton", False),
            field_name=f"hands.{side}.buttons.secondaryButton",
        ),
        trigger=_as_normalized_float(
            buttons_data.get("trigger", 0.0),
            field_name=f"hands.{side}.buttons.trigger",
        ),
        grip=_as_normalized_float(
            buttons_data.get("grip", 0.0),
            field_name=f"hands.{side}.buttons.grip",
        ),
        thumbstick_axis=_as_vector(
            buttons_data.get("thumbstickAxis", [0.0, 0.0]),
            length=2,
            field_name=f"hands.{side}.buttons.thumbstickAxis",
        ),
    )

    return VrController(
        pose=pose,
        buttons=buttons,
    )


def _parse_pose(
    pose_data: Mapping[str, Any],
    *,
    prefix: str,
) -> VrPose:
    """Parse a Quest pose containing position and quaternion rotation."""

    position = _as_vector(
        _require_value(pose_data, "position", prefix=prefix),
        length=3,
        field_name=f"{prefix}.position",
    )

    rotation = _as_vector(
        _require_value(pose_data, "rotation", prefix=prefix),
        length=4,
        field_name=f"{prefix}.rotation",
    )

    quaternion_norm = math.sqrt(sum(value * value for value in rotation))

    if quaternion_norm < 1e-8:
        raise ValueError(
            f"{prefix}.rotation contains a zero-length quaternion."
        )

    normalized_rotation = tuple(
        value / quaternion_norm
        for value in rotation
    )

    return VrPose(
        position=position,
        rotation=normalized_rotation,
    )


def _require_mapping(
    mapping: Mapping[str, Any],
    key: str,
    *,
    prefix: str = "",
) -> Mapping[str, Any]:
    """Read a required mapping field."""

    field_name = f"{prefix}.{key}" if prefix else key
    value = _require_value(mapping, key, prefix=prefix)

    return _as_mapping(
        value,
        field_name=field_name,
    )


def _as_mapping(
    value: Any,
    *,
    field_name: str,
) -> Mapping[str, Any]:
    """Validate that a value is a mapping."""

    if not isinstance(value, Mapping):
        raise TypeError(
            f"'{field_name}' must be a JSON object. "
            f"Received {type(value).__name__}."
        )

    return value


def _require_value(
    mapping: Mapping[str, Any],
    key: str,
    *,
    prefix: str = "",
) -> Any:
    """Read a required value from a mapping."""

    if key not in mapping:
        field_name = f"{prefix}.{key}" if prefix else key
        raise ValueError(
            f"VR packet is missing required field '{field_name}'."
        )

    return mapping[key]


def _as_vector(
    value: Any,
    *,
    length: int,
    field_name: str,
) -> tuple[float, ...]:
    """Validate and convert a numeric vector."""

    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes, bytearray))
    ):
        raise TypeError(
            f"'{field_name}' must be an array with {length} values."
        )

    if len(value) != length:
        raise ValueError(
            f"'{field_name}' must contain {length} values, "
            f"but received {len(value)}."
        )

    result = tuple(
        _as_finite_float(
            item,
            field_name=f"{field_name}[{index}]",
        )
        for index, item in enumerate(value)
    )

    return result


def _as_normalized_float(
    value: Any,
    *,
    field_name: str,
) -> float:
    """Convert an analog input to a value in the range [0, 1]."""

    result = _as_finite_float(
        value,
        field_name=field_name,
    )

    return min(max(result, 0.0), 1.0)


def _as_finite_float(
    value: Any,
    *,
    field_name: str,
) -> float:
    """Convert a numeric JSON value to a finite float."""

    if isinstance(value, bool):
        raise TypeError(
            f"'{field_name}' must be numeric, not boolean."
        )

    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(
            f"'{field_name}' must be numeric."
        ) from exc

    if not math.isfinite(result):
        raise ValueError(
            f"'{field_name}' must be finite."
        )

    return result


def _as_bool(
    value: Any,
    *,
    field_name: str,
) -> bool:
    """Convert a JSON boolean or zero/one value to bool."""

    if isinstance(value, bool):
        return value

    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)

    raise TypeError(
        f"'{field_name}' must be a boolean or zero/one value."
    )
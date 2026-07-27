"""UDP receiver for Meta Quest VR controller packets."""

from __future__ import annotations

import json
import socket
import time
from typing import Any

from .config_box_vr import BoxVrConfig
from .vr_protocol import VrPacket, parse_vr_packet


class VrReceiver:
    """Receive Meta Quest controller data over UDP.

    This class is responsible only for network communication:

    - opening and closing the UDP socket
    - receiving raw packets
    - validating the packet sender
    - parsing packets through ``vr_protocol.py``
    - sending optional handshake messages

    Coordinate conversion and robot target generation are intentionally handled
    outside this class.
    """

    def __init__(self, config: BoxVrConfig):
        self.config = config

        self._socket: socket.socket | None = None
        self._is_connected = False

        self._last_handshake_time = 0.0
        self._last_sender_address: tuple[str, int] | None = None

    @property
    def is_connected(self) -> bool:
        """Whether the UDP socket is currently open."""

        return self._is_connected

    @property
    def last_sender_address(self) -> tuple[str, int] | None:
        """Address of the most recent valid packet sender."""

        return self._last_sender_address

    def connect(self) -> None:
        """Create and bind the UDP socket."""

        if self.is_connected:
            raise RuntimeError("VR receiver is already connected.")

        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp_socket.settimeout(self.config.receive_timeout_s)
            udp_socket.bind(
                (
                    self.config.local_ip,
                    self.config.local_port,
                )
            )
        except Exception:
            udp_socket.close()
            raise

        self._socket = udp_socket
        self._is_connected = True

    def receive(self) -> VrPacket | None:
        """Receive and parse one VR packet.

        Returns:
            Parsed ``VrPacket`` when a valid packet is received.
            ``None`` when the socket times out or the packet is invalid.
        """

        self._require_connected()

        try:
            payload, sender_address = self._socket.recvfrom(65535)
        except socket.timeout:
            return None
        except BlockingIOError:
            return None
        except OSError as exc:
            if not self.is_connected:
                return None

            raise RuntimeError(
                f"Failed to receive VR UDP packet: {exc}"
            ) from exc

        if not payload:
            return None

        if not self._is_allowed_sender(sender_address):
            return None

        try:
            packet = parse_vr_packet(payload)
        except (ValueError, TypeError, KeyError, json.JSONDecodeError):
            return None

        self._last_sender_address = sender_address
        return packet

    def send_handshake(self) -> None:
        """Send one handshake packet to the configured Meta Quest address."""

        self._require_connected()

        if not self.config.meta_quest_ip:
            raise ValueError(
                "meta_quest_ip must be configured when send_handshake is enabled."
            )

        payload = self._build_handshake_payload()

        try:
            self._socket.sendto(
                payload,
                (
                    self.config.meta_quest_ip,
                    self.config.meta_quest_port,
                ),
            )
        except OSError as exc:
            raise RuntimeError(
                "Failed to send VR handshake to "
                f"{self.config.meta_quest_ip}:"
                f"{self.config.meta_quest_port}: {exc}"
            ) from exc

        self._last_handshake_time = time.monotonic()

    def close(self) -> None:
        """Close the UDP socket."""

        if self._socket is not None:
            try:
                self._socket.close()
            finally:
                self._socket = None

        self._is_connected = False
        self._last_sender_address = None
        self._last_handshake_time = 0.0

    def _is_allowed_sender(
        self,
        sender_address: tuple[str, int],
    ) -> bool:
        """Validate the source IP of an incoming UDP packet."""

        sender_ip, _ = sender_address

        if not self.config.meta_quest_ip:
            return True

        return sender_ip == self.config.meta_quest_ip

    def _build_handshake_payload(self) -> bytes:
        """Tell the Quest app where to send controller packets."""

        target_info = {
            "ip": self.config.local_ip,
            "port": self.config.local_port,
        }

        return json.dumps(target_info).encode("utf-8")

    def _require_connected(self) -> None:
        """Raise when a socket operation is attempted before connection."""

        if not self.is_connected or self._socket is None:
            raise RuntimeError(
                "VR receiver is not connected. Call connect() first."
            )

    def __enter__(self) -> VrReceiver:
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: object,
        exc_value: object,
        traceback: object,
    ) -> None:
        self.close()
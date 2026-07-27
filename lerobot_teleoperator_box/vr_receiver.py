"""Thread-safe UDP receiver for Meta Quest controller data."""

from __future__ import annotations

import json
import logging
import socket
import threading
import time

from .vr_protocol import VrPacket, parse_vr_packet

logger = logging.getLogger(__name__)


class VrReceiver:
    """Continuously receive Meta Quest packets in a background thread.

    The receiver binds a UDP socket on ``0.0.0.0:local_port`` and stores the
    latest valid :class:`VrPacket`.

    ``local_ip`` is not used for socket binding. It is advertised to the
    Meta Quest through the handshake so that the Quest knows where to send
    controller packets.
    """

    def __init__(
        self,
        *,
        local_ip: str,
        local_port: int,
        meta_quest_ip: str,
        meta_quest_port: int,
        receive_timeout_s: float = 0.1,
    ) -> None:
        self.local_ip = local_ip
        self.local_port = local_port
        self.meta_quest_ip = meta_quest_ip
        self.meta_quest_port = meta_quest_port
        self.receive_timeout_s = receive_timeout_s

        self._lock = threading.Lock()

        self._latest_packet: VrPacket | None = None
        self._last_packet_time: float | None = None

        # Used to block only until the first valid VR packet arrives.
        self._first_packet_event = threading.Event()

        self._running = False
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None

    def connect(self) -> None:
        """Bind the UDP socket and start the receiver thread."""

        if self._running:
            return

        # Reset cached state in case this object is reconnected.
        with self._lock:
            self._latest_packet = None
            self._last_packet_time = None

        self._first_packet_event.clear()

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        try:
            # Accept packets arriving through any network interface.
            sock.bind(("0.0.0.0", self.local_port))

            # The timeout lets the thread periodically check self._running.
            sock.settimeout(max(self.receive_timeout_s, 0.1))

        except Exception:
            sock.close()
            raise

        self._socket = sock
        self._running = True

        self._thread = threading.Thread(
            target=self._receive_loop,
            name="box-vr-receiver",
            daemon=True,
        )
        self._thread.start()

        logger.info(
            "VR receiver listening on 0.0.0.0:%d",
            self.local_port,
        )

    def send_handshake(self) -> None:
        """Tell Meta Quest where it should send controller packets."""

        target_info = {
            "ip": self.local_ip,
            "port": self.local_port,
        }

        message = json.dumps(target_info).encode("utf-8")

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.sendto(
                message,
                (
                    self.meta_quest_ip,
                    self.meta_quest_port,
                ),
            )

        logger.info(
            "VR handshake sent to %s:%d: %s",
            self.meta_quest_ip,
            self.meta_quest_port,
            target_info,
        )

    def receive(self) -> VrPacket | None:
        """Return the latest valid packet.

        Before the first valid packet arrives, this method waits for up to
        10 seconds. After the first packet has arrived, it returns the latest
        cached packet immediately without reading directly from the socket.
        """

        if not self._first_packet_event.wait(timeout=10.0):
            logger.warning(
                "No valid VR packet was received within 10 seconds."
            )
            return None

        with self._lock:
            return self._latest_packet

    def get_state(self) -> VrPacket | None:
        """Return the latest valid packet."""

        return self.receive()

    @property
    def last_packet_age_s(self) -> float | None:
        """Return elapsed seconds since the latest valid packet."""

        with self._lock:
            if self._last_packet_time is None:
                return None

            return time.monotonic() - self._last_packet_time

    def close(self) -> None:
        """Stop the receiver thread and release the UDP socket."""

        self._running = False

        # Wake receive() if it is waiting for the first packet.
        self._first_packet_event.set()

        sock = self._socket
        self._socket = None

        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

        thread = self._thread
        self._thread = None

        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)

        with self._lock:
            self._latest_packet = None
            self._last_packet_time = None

        logger.info("VR receiver stopped.")

    def _receive_loop(self) -> None:
        """Continuously receive, decode and cache Meta Quest packets."""

        while self._running:
            sock = self._socket

            if sock is None:
                break

            try:
                data, address = sock.recvfrom(65535)

            except socket.timeout:
                continue

            except OSError:
                # Normally occurs when close() releases the socket.
                if self._running:
                    logger.exception("VR UDP socket error.")
                break

            except Exception:
                logger.exception(
                    "Unexpected error while receiving a VR UDP packet."
                )
                continue

            try:
                payload = json.loads(data.decode("utf-8"))
                packet = parse_vr_packet(payload)

            except Exception as exc:
                logger.warning(
                    "Invalid VR packet from %s: %s: %s",
                    address,
                    type(exc).__name__,
                    exc,
                )
                continue

            with self._lock:
                self._latest_packet = packet
                self._last_packet_time = time.monotonic()

            # Wake the first receive() call after a valid packet is cached.
            self._first_packet_event.set()
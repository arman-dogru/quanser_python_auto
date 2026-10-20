"""Detection stream on UDP port 5010, matching the Stage 6 Stream Server.

* :class:`UdpDetectionReceiver` stands in for the model's receiving block
  (standard library sockets): it drains the socket each poll and keeps the
  most recent complete sample, outputs the ``[0 0 0]`` default before any
  data, and reports ``new`` like the block does.
* :class:`UdpDetectionSender` is the sending side (the original sender was
  not supplied): it sends one ``<3d`` datagram per detection.

The model binds ``udp://localhost:5010?peer=any``; QUARC's ``peer=any``
accepts datagrams from any sender address. Bind address defaults to
127.0.0.1 here; pass ``0.0.0.0`` to accept remote senders.
"""
from __future__ import annotations

import socket
import time

from ..detection import packet
from .interfaces import DetectionSample

STAGE6_PORT = 5010


class UdpDetectionReceiver:
    def __init__(self, host: str = "127.0.0.1", port: int = STAGE6_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.address = self.sock.getsockname()
        self._last = packet.DEFAULT_VALUE
        self._received_at: float | None = None
        self.bad_datagrams = 0

    def poll(self) -> DetectionSample:
        new = False
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except BlockingIOError:
                break
            try:
                self._last = packet.decode(data)
            except ValueError:
                self.bad_datagrams += 1
                continue
            self._received_at = time.monotonic()
            new = True
        return DetectionSample(values=self._last, new=new, state=2, error=0, received_at=self._received_at)

    def close(self) -> None:
        self.sock.close()


class UdpDetectionSender:
    def __init__(self, host: str = "127.0.0.1", port: int = STAGE6_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target = (host, port)

    def send(self, detected: float, error_x: float, error_y: float) -> int:
        return self.sock.sendto(packet.encode(detected, error_x, error_y), self.target)

    def close(self) -> None:
        self.sock.close()

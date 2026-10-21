"""In-process stand-ins for the two network peers, for loopback tests and demos.

* :class:`FakeCommander` plays the Commander's side of TCP 18001: a Stream
  Client that receives 14 doubles ("Receive most recent data") and sends back
  element 14 as one double (the Memory-block echo used for loopback time).
* :class:`FakePublisher` plays ``yolo_pad_direct_stream_fresh.py``'s side of
  TCP 18901 with the same accept-on-publish, non-blocking ``<6d`` behaviour as
  ``LatestObservationServer``. Tests also run against the real class.
"""
from __future__ import annotations

import socket
import threading
import time

from ..protocol import COMMAND, LOOPBACK, VISION, CommandPacket, RecordBuffer


class FakeCommander(threading.Thread):
    def __init__(self, port: int, host: str = "127.0.0.1") -> None:
        super().__init__(daemon=True)
        self.addr = (host, port)
        self.packets: list[CommandPacket] = []
        self.records = 0
        self.stop_flag = threading.Event()
        self.connected = threading.Event()

    def run(self) -> None:
        deadline = time.monotonic() + 10
        sock = None
        while sock is None and time.monotonic() < deadline and not self.stop_flag.is_set():
            try:
                sock = socket.create_connection(self.addr, timeout=0.2)
            except OSError:
                time.sleep(0.02)
        if sock is None:
            return
        self.connected.set()
        sock.settimeout(0.05)
        buf = RecordBuffer(COMMAND.size)
        with sock:
            while not self.stop_flag.is_set():
                try:
                    data = sock.recv(65536)
                except TimeoutError:
                    continue
                except OSError:
                    break
                if not data:
                    break
                before = buf.dropped
                newest = buf.feed(data)
                if newest is None:
                    continue
                self.records += 1 + (buf.dropped - before)
                pkt = CommandPacket.unpack(newest)
                self.packets.append(pkt)
                try:
                    sock.sendall(LOOPBACK.pack(pkt.timestamp))
                except OSError:
                    break

    def stop(self) -> None:
        self.stop_flag.set()
        self.join(timeout=2)


class FakePublisher:
    def __init__(self, port: int = 0, host: str = "127.0.0.1") -> None:
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(1)
        self.listener.setblocking(False)
        self.client: socket.socket | None = None
        self.published = 0

    @property
    def port(self) -> int:
        return self.listener.getsockname()[1]

    def publish(self, detected, cx, cy, conf, frame, timestamp) -> bool:
        if self.client is None:
            try:
                self.client, _ = self.listener.accept()
                self.client.setblocking(False)
            except BlockingIOError:
                return False
        try:
            self.client.sendall(VISION.pack(float(bool(detected)), cx if detected else 0.0,
                                            cy if detected else 0.0, conf if detected else 0.0,
                                            float(frame), float(timestamp)))
            self.published += 1
            return True
        except OSError:
            self.client.close()
            self.client = None
            return False

    def send_raw(self, data: bytes) -> None:
        if self.client is not None:
            self.client.sendall(data)

    def close(self) -> None:
        if self.client:
            self.client.close()
        self.listener.close()

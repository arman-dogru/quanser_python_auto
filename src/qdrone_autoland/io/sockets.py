"""TCP endpoints that replace the QUARC Stream blocks of the mission server.

* :class:`VisionClient` replaces "YOLO Observation TCP Client"
  (``tcpip://localhost:18901``, receive only, most recent data).
* :class:`CommanderServer` replaces "Communication - QDrone/Stream Server"
  (``URI_Follower = tcpip://localhost:18001``): sends the 14-double packet,
  receives the Commander's 1-double timestamp echo.

Both are non-blocking and never raise into the control loop; they reconnect
on their own. The QUARC "state"/"error" outputs have no socket equivalent;
:attr:`CommanderServer.state` reports 2 when a Commander is connected and 0
otherwise, which is all the mission comm-loss chart tests (``state == 2``).
"""
from __future__ import annotations

import errno
import socket

from ..protocol import COMMAND, LOOPBACK, RecordBuffer

_WOULD_BLOCK = (BlockingIOError, InterruptedError)


class VisionClient:
    def __init__(self, host: str = "127.0.0.1", port: int = 18901, reconnect_s: float = 0.5) -> None:
        self.addr = (host, port)
        self.reconnect_s = reconnect_s
        self.sock: socket.socket | None = None
        self.next_attempt = 0.0
        self.connected = False

    def _connect(self, now: float) -> None:
        if now < self.next_attempt:
            return
        self.next_attempt = now + self.reconnect_s
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setblocking(False)
        err = s.connect_ex(self.addr)
        if err in (0, errno.EINPROGRESS, errno.EWOULDBLOCK, errno.EALREADY):
            self.sock = s
        else:
            s.close()

    def read(self, now: float) -> bytes:
        """Return every byte available now (the backlog is drained)."""
        if self.sock is None:
            self._connect(now)
            return b""
        chunks = []
        while True:
            try:
                data = self.sock.recv(65536)
            except _WOULD_BLOCK:
                break
            except OSError:
                self.close()
                break
            if not data:
                self.close()
                break
            self.connected = True
            chunks.append(data)
        return b"".join(chunks)

    def close(self) -> None:
        if self.sock is not None:
            self.sock.close()
        self.sock = None
        self.connected = False


class CommanderServer:
    def __init__(self, host: str = "0.0.0.0", port: int = 18001) -> None:
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((host, port))
        self.listener.listen(1)
        self.listener.setblocking(False)
        self.client: socket.socket | None = None
        self.rx = RecordBuffer(LOOPBACK.size)
        self.loopback_timestamp = 0.0
        self.sent = 0
        self.dropped = 0
        self.pending = b""

    @property
    def port(self) -> int:
        return self.listener.getsockname()[1]

    @property
    def state(self) -> float:
        return 2.0 if self.client is not None else 0.0

    @property
    def error(self) -> float:
        return 0.0

    def poll(self) -> None:
        if self.client is None:
            try:
                self.client, _ = self.listener.accept()
                self.client.setblocking(False)
                self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.rx = RecordBuffer(LOOPBACK.size)
                self.pending = b""
            except _WOULD_BLOCK:
                return
        while self.client is not None:
            try:
                data = self.client.recv(4096)
            except _WOULD_BLOCK:
                break
            except OSError:
                self.disconnect()
                break
            if not data:
                self.disconnect()
                break
            newest = self.rx.feed(data)
            if newest is not None:
                (self.loopback_timestamp,) = LOOPBACK.unpack(newest)

    def _write(self, data: bytes) -> bytes | None:
        """Write as much as possible; return the unwritten tail (None on error)."""
        try:
            n = self.client.send(data)
        except _WOULD_BLOCK:
            return data
        except OSError:
            self.disconnect()
            return None
        return data[n:]

    def send(self, payload: bytes) -> bool:
        """Send one packet. If the socket cannot take it, the sample is dropped
        ("Send most recent data"): it is never queued behind older samples. A
        partially written packet is always completed first so the 112-byte
        framing on the Commander side never desynchronises."""
        if len(payload) != COMMAND.size:
            raise ValueError("commander payload must be 112 bytes")
        if self.client is None:
            return False
        if self.pending:
            rest = self._write(self.pending)
            if rest is None:
                return False
            self.pending = rest
            if self.pending:
                self.dropped += 1
                return False
        rest = self._write(payload)
        if rest is None:
            return False
        if len(rest) == len(payload):
            self.dropped += 1
            return False
        self.pending = rest
        self.sent += 1
        return True

    def disconnect(self) -> None:
        if self.client is not None:
            self.client.close()
        self.client = None
        self.pending = b""

    def close(self) -> None:
        self.disconnect()
        self.listener.close()

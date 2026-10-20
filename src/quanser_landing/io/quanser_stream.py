"""Optional adapters on Quanser's official Python API.

``quanser.communications.Stream`` wraps the same Stream API the QUARC
Stream Server block uses, so it is the closest match to the model. It is not
on PyPI: it ships with QUARC / Quanser SDK (``quanser_api`` wheel on
Windows/macOS, ``python3-quanser-apis`` apt package on Linux, preinstalled on
QDrone 2). Import lazily so the rest of the package works without it.

Only the detection stream is implemented: the model fully specifies it
(``udp://localhost:5010?peer=any``, three little-endian doubles). Not tested
here: the API is not installed on the build machine.
"""
from __future__ import annotations

from .interfaces import DetectionSample

STAGE6_DETECTION_URI = "udp://localhost:5010?peer=any"


def quanser_api_available() -> bool:
    try:
        import quanser.communications  # noqa: F401
    except ImportError:
        return False
    return True


class QuanserStreamDetectionReceiver:  # pragma: no cover - needs Quanser SDK
    """Listen like the Stage 6 block: non-blocking, most recent sample."""

    def __init__(self, uri: str = STAGE6_DETECTION_URI, receive_buffer_size: int = 256):
        from array import array

        from quanser.communications import Stream

        self._Stream = Stream
        self._server = Stream()
        self._server.listen(uri, True)
        self._client = None
        self._buf = array("d", [0.0, 0.0, 0.0])
        self._last = (0.0, 0.0, 0.0)
        self._rbs = receive_buffer_size

    def poll(self) -> DetectionSample:
        from quanser.common import ErrorCode

        new = False
        if self._client is None:
            try:
                self._client = self._server.accept(1460, self._rbs)
            except Exception:  # would block: no peer yet
                return DetectionSample(self._last, False, 1, 0, None)
        while True:
            try:
                r = self._client.receive_double_array(self._buf, 3)
            except Exception:
                break
            if r == 1:
                self._last = tuple(self._buf)
                new = True
                continue
            if r == -ErrorCode.WOULD_BLOCK or r == 0:
                break
        return DetectionSample(self._last, new, 2, 0, None)

    def close(self) -> None:
        for s in (self._client, self._server):
            if s is not None:
                s.close()

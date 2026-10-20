"""Real-time mission-server loop: sockets + sources -> MissionRuntime -> Commander.

Timing: the model's dwell counters are tick based (e.g. 100 ticks = 0.1 s at
the 1 kHz FixedStep), so the loop keeps *model time* exactly ``k * step``.
When the host falls behind it runs the missed ticks back to back (catch-up)
and counts an overrun; if it falls more than ``max_lag`` behind, it stops
(fail-stop) rather than silently stretching time.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field

from ..config import DEFAULT, Config
from ..runtime import MissionRuntime, RawInputs, TickRecord
from ..safety import Mode
from .sockets import CommanderServer, VisionClient
from .sources import JoystickSource, NoJoystick, NoPose, PoseSource

log = logging.getLogger(__name__)


@dataclass
class ServeStats:
    ticks: int = 0
    sent: int = 0
    send_dropped: int = 0
    catch_up_ticks: int = 0
    max_lag_s: float = 0.0
    stopped_reason: str | None = None
    vision_bytes: int = 0
    faults: dict = field(default_factory=dict)


class MissionServer:
    def __init__(self, cfg: Config = DEFAULT, mode: Mode = Mode.MONITOR, vision: VisionClient | None = None,
                 commander: CommanderServer | None = None, pose: PoseSource | None = None,
                 joystick: JoystickSource | None = None, wall=time.time) -> None:
        self.cfg = cfg
        self.runtime = MissionRuntime(cfg, mode)
        self.vision = vision if vision is not None else VisionClient(port=cfg.auto.yolo_tcp_port)
        self.commander = commander if commander is not None else CommanderServer()
        self.pose = pose or NoPose()
        self.joystick = joystick or NoJoystick()
        self.wall = wall
        self.stats = ServeStats()
        self.k = 0
        self.last: TickRecord | None = None

    def step_once(self, t: float | None = None) -> TickRecord | None:
        """Run exactly one model tick at model time ``t`` (default ``k * step``)."""
        if t is None:
            t = self.k * self.cfg.model.step_size
        wall = self.wall()
        self.commander.poll()
        data = self.vision.read(wall)
        self.stats.vision_bytes += len(data)
        raw = RawInputs(t, wall, self.pose.poll(t), self.joystick.poll(t), data, self.commander.state,
                        self.commander.error, self.commander.loopback_timestamp)
        rec = self.runtime.tick(raw)
        if self.runtime.vision_guard.resync_requested:
            self.runtime.vision_guard.resync_requested = False
            self.vision.close()                     # reconnect re-establishes 48-byte framing
        self.k += 1
        self.stats.ticks += 1
        self.last = rec
        if rec is None:
            self._fail(f"runtime: {self.runtime.failed}")
            return None
        if rec.decision.packet is not None:
            if self.commander.send(rec.decision.packet.pack()):
                self.stats.sent += 1
            else:
                self.stats.send_dropped += 1
        return rec

    def _fail(self, reason: str) -> None:
        if self.stats.stopped_reason is None:
            self.stats.stopped_reason = reason
            log.error("mission server stopped: %s", reason)
        # Closing the stream makes the Commander's own comm-loss path take over.
        self.commander.disconnect()

    def run(self, duration: float | None = None, max_lag: float = 0.05, clock=time.perf_counter,
            sleep=time.sleep, should_stop=lambda: False) -> ServeStats:
        dt = self.cfg.model.step_size
        start = clock() - self.k * dt          # resumable: model time continues from k
        while self.stats.stopped_reason is None and not should_stop():
            t = self.k * dt
            if duration is not None and t >= duration:
                break
            lag = clock() - start - t
            if lag < 0:
                if -lag > 0.0015:
                    sleep(-lag - 0.001)
                continue
            if lag > dt:
                self.stats.catch_up_ticks += 1
            self.stats.max_lag_s = max(self.stats.max_lag_s, lag)
            if lag > max_lag:
                self._fail(f"real-time overrun: {lag * 1000:.1f} ms behind model time")
                break
            self.step_once(t)
        self.stats.faults = dict(self.runtime.gate.faults)
        return self.stats

    def close(self) -> None:
        self.vision.close()
        self.commander.close()

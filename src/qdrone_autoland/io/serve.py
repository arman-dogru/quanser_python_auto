"""Real-time mission-server loop: sockets + sources -> MissionRuntime -> Commander.

Timing: the model's dwell counters are tick based (e.g. 100 ticks = 0.1 s at
the 1 kHz FixedStep), so the loop keeps *model time* exactly ``k * step``.
When the host falls behind it runs the missed ticks back to back (catch-up)
and counts an overrun; if it falls more than ``max_lag`` behind, it stops
(fail-stop) rather than silently stretching time.

Waiting for the next tick is a hybrid: an OS sleep until ``spin_s`` before
the tick, then a short yielding spin (``sleep(0)``) to absorb the OS timer's
wake-up latency. This keeps one core well below 100 % without giving up
sub-millisecond tick timing (see ``ServeStats.lag_*`` and ``cpu_s``).

Every input (Commander socket, vision socket, pose and joystick sources) is
read inside the protected tick: an exception from any of them is a
fail-stop with the reason recorded, exactly like an exception in the
mission logic.
"""
from __future__ import annotations

import logging
import time
from collections import deque
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
    cpu_s: float = 0.0
    wall_s: float = 0.0
    lag_p50_s: float = 0.0
    lag_p99_s: float = 0.0
    lag_p999_s: float = 0.0


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
        try:
            wall = self.wall()
            self.commander.poll()
            data = self.vision.read(wall)
            pose = self.pose.poll(t)
            joystick = self.joystick.poll(t)
            raw = RawInputs(t, wall, pose, joystick, data, self.commander.state, self.commander.error,
                            self.commander.loopback_timestamp)
        except Exception as exc:  # noqa: BLE001 - any input failure is a fail-stop
            self.k += 1
            self.stats.ticks += 1
            self.last = None
            self._fail(f"input: {type(exc).__name__}: {exc}")
            return None
        self.stats.vision_bytes += len(data)
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
            sleep=time.sleep, should_stop=lambda: False, spin_s: float = 0.0004) -> ServeStats:
        dt = self.cfg.model.step_size
        start = clock() - self.k * dt          # resumable: model time continues from k
        cpu0, wall0 = time.process_time(), clock()
        lags: deque[float] = deque(maxlen=600_000)
        try:
            while self.stats.stopped_reason is None and not should_stop():
                t = self.k * dt
                if duration is not None and t >= duration:
                    break
                lag = clock() - start - t
                if lag < 0:
                    # Early: OS sleep until spin_s before the tick, then yield-spin.
                    sleep(-lag - spin_s if -lag > spin_s else 0)
                    continue
                lags.append(lag)
                if lag > dt:
                    self.stats.catch_up_ticks += 1
                self.stats.max_lag_s = max(self.stats.max_lag_s, lag)
                if lag > max_lag:
                    self._fail(f"real-time overrun: {lag * 1000:.1f} ms behind model time")
                    break
                self.step_once(t)
        finally:
            self.stats.faults = dict(self.runtime.gate.faults)
            self.stats.cpu_s += time.process_time() - cpu0
            self.stats.wall_s += clock() - wall0
            if lags:
                q = sorted(lags)
                pick = lambda f: q[min(len(q) - 1, int(f * len(q)))]  # noqa: E731
                self.stats.lag_p50_s, self.stats.lag_p99_s, self.stats.lag_p999_s = pick(0.5), pick(0.99), pick(0.999)
        return self.stats

    def close(self) -> None:
        self.vision.close()
        self.commander.close()

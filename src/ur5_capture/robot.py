"""Robot motion ownership lives in a spawned process, isolated from recorder stalls."""
import multiprocessing as mp
import time
import traceback
from dataclasses import dataclass

import numpy as np

from .config import RobotConfig
from .geometry import check_target, interpolate, vector


@dataclass
class RobotState:
    host_time: float
    device_time: float
    pose: np.ndarray
    joints: np.ndarray


def _worker(config, mock, shared, connection, quit_event):
    cfg = RobotConfig.model_validate(config)
    control = receive = None
    active = False
    try:
        if not mock:
            import rtde_control
            import rtde_receive
            receive = rtde_receive.RTDEReceiveInterface(cfg.ip, cfg.control_hz)
            if not receive.isConnected() or receive.getRobotMode() != 7:
                raise RuntimeError("UR controller must be powered on and in RUNNING mode")
            # Reuse pendant External Control program.  The default ur_rtde flag
            # uploads/replaces a control script and would remove rg2_worker_thread().
            control = rtde_control.RTDEControlInterface(
                cfg.ip, cfg.control_hz,
                rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP,
                cfg.ur_cap_port,
            )
            if not control.isConnected():
                raise RuntimeError("RTDE control connection failed")
            if not control.setTcp(list(cfg.tcp_offset)):
                raise RuntimeError(
                    "ur_rtde External Control API rejected setTcp; load an External Control "
                    "URCap program compatible with ur_rtde on the pendant and set its "
                    f"server port to {cfg.ur_cap_port}"
                )
            if not control.setWatchdog(cfg.watchdog_hz):
                raise RuntimeError("Controller watchdog could not be enabled")
        pose = vector(cfg.mock_initial_pose)
        joints = np.zeros(6)
        start = target = pose.copy()
        segment_start = last_command = time.monotonic()
        duration = 0.0
        previous_device_time = None
        last_device_change = time.monotonic()
        next_tick = time.monotonic()
        connection.send({"ready": True})
        while not quit_event.is_set():
            now = time.monotonic()
            if mock:
                if active:
                    pose = interpolate(start, target, (now - segment_start) / max(duration, 1e-9))
                device_time = now
            else:
                if receive.isProtectiveStopped() or receive.isEmergencyStopped():
                    raise RuntimeError("Controller protective/emergency stop")
                if not receive.isConnected():
                    raise RuntimeError("RTDE receive disconnected")
                device_time = receive.getTimestamp()
                if device_time != previous_device_time:
                    last_device_change = now
                    previous_device_time = device_time
                elif now - last_device_change > cfg.command_timeout_s:
                    raise RuntimeError("RTDE state packets stalled")
                pose = vector(receive.getActualTCPPose())
                joints = vector(receive.getActualQ())
                if not control.kickWatchdog():
                    raise RuntimeError("Controller watchdog kick failed")
            with shared.get_lock():
                shared[:] = [now, device_time, *pose, *joints]
            if connection.poll():
                cmd = connection.recv()
                if cmd["kind"] == "stop":
                    if active and control is not None:
                        if not control.servoStop():
                            raise RuntimeError("servoStop failed")
                    active = False
                    target = pose.copy()
                    connection.send({"ok": True})
                else:
                    if now - cmd["sent"] > cfg.command_timeout_s:
                        raise RuntimeError("Expired motion command")
                    new_target = vector(cmd["target"])
                    duration = float(cmd["duration"])
                    if duration <= 0:
                        raise ValueError("Invalid segment duration")
                    check_target(pose, new_target, cfg)
                    enabled = bool(cmd["enabled"])
                    if enabled and not (mock or cfg.motion_enabled):
                        raise RuntimeError("Motion is disabled by config")
                    if enabled and control is not None and not control.isPoseWithinSafetyLimits(new_target.tolist()):
                        raise RuntimeError("Controller rejected target pose")
                    if enabled:
                        # No target windup: interpolate from current measured TCP on every command.
                        start, target, segment_start = pose.copy(), new_target, now
                        active = True
                    else:
                        if active and control is not None:
                            if not control.servoStop():
                                raise RuntimeError("servoStop failed")
                        active = False
                        target = pose.copy()
                    last_command = now
                    connection.send({"ok": True, "accepted_host_time": now})
            if active:
                if now - last_command > cfg.command_timeout_s:
                    raise RuntimeError("Control command heartbeat expired")
                check_target(pose, target, cfg)
                if control is not None:
                    command_pose = interpolate(start, target, (now - segment_start + 1 / cfg.control_hz) / duration)
                    if not control.servoL(command_pose.tolist(), cfg.high_translation_speed,
                                          0.1, 1 / cfg.control_hz, cfg.lookahead_time, cfg.gain):
                        raise RuntimeError("servoL failed")
            next_tick += 1 / cfg.control_hz
            delay = next_tick - time.monotonic()
            if delay > 0:
                quit_event.wait(delay)
            elif delay < -cfg.command_timeout_s and not mock:
                # Preserve strict timing safety on the physical robot. Mock workers
                # can be descheduled on oversubscribed/shared compute instances.
                raise RuntimeError("Servo process scheduling deadline missed")
            elif delay < -1 / cfg.control_hz:
                next_tick = time.monotonic()
    except BaseException as exc:
        try:
            connection.send({"error": f"{type(exc).__name__}: {exc}", "traceback": traceback.format_exc()})
        except (BrokenPipeError, EOFError, OSError):
            pass
    finally:
        if control is not None:
            for action in (control.servoStop, control.stopScript, control.disconnect):
                try:
                    action()
                except Exception:
                    pass
        if receive is not None:
            try:
                receive.disconnect()
            except Exception:
                pass
        connection.close()


class UR5Backend:
    def __init__(self, cfg: RobotConfig, mock=False):
        self.cfg, self.mock = cfg, mock
        ctx = mp.get_context("spawn")
        self.shared = ctx.Array("d", 14)
        self.connection, self.child_connection = ctx.Pipe()
        self.quit_event = ctx.Event()
        self.process = ctx.Process(target=_worker, args=(cfg.model_dump(), mock, self.shared,
                                                        self.child_connection, self.quit_event), daemon=True)
        self.started = False

    def connect(self):
        self.process.start()
        self.started = True
        self.child_connection.close()
        self._response(20)
        deadline = time.monotonic() + 2
        while self.shared[0] == 0:
            if time.monotonic() > deadline:
                raise TimeoutError("No robot state received")
            self.check_health()
            time.sleep(0.005)

    def _response(self, timeout=0.5):
        if not self.connection.poll(timeout):
            raise TimeoutError("RTDE process did not acknowledge command")
        try:
            response = self.connection.recv()
        except (EOFError, OSError) as exc:
            raise RuntimeError("RTDE process connection closed") from exc
        if "error" in response:
            raise RuntimeError(response["error"])
        return response

    def check_health(self):
        if self.connection.poll():
            self._response()
        if not self.process.is_alive():
            raise RuntimeError("RTDE process exited")

    def get_state(self, max_age=0.05):
        if not np.isfinite(max_age) or max_age <= 0:
            raise ValueError("max_age must be a positive finite number")
        # On heavily shared CPUs the spawned mock RTDE worker may briefly not run
        # within the default 50 ms freshness window. Wait for an ACTUALLY NEW
        # sample instead of accepting an old one or relaxing max_age. The real
        # robot must continue to fail immediately on stale feedback for safety.
        deadline = time.monotonic() + (1.0 if self.mock else 0.0)
        while True:
            self.check_health()
            with self.shared.get_lock():
                data = np.array(self.shared[:])
            now = time.monotonic()
            age = now - data[0]
            if data[0] > 0 and 0 <= age <= max_age:
                return RobotState(float(data[0]), float(data[1]), data[2:8], data[8:14])
            if not self.mock or now >= deadline:
                raise RuntimeError(
                    f"Robot state is stale (age={age:.3f}s, allowed={max_age:.3f}s, "
                    f"mode={'mock' if self.mock else 'real'})"
                )
            time.sleep(min(0.005, max(0.0, deadline - now)))

    def send_target(self, target, duration, enabled):
        self.check_health()
        self.connection.send({"kind": "target", "target": vector(target).tolist(),
                              "duration": duration, "enabled": enabled, "sent": time.monotonic()})
        return self._response()

    def stop(self):
        if not self.started:
            return
        if not self.process.is_alive():
            return
        self.connection.send({"kind": "stop"})
        self._response()

    def disconnect(self):
        try:
            self.stop()
        except (BrokenPipeError, EOFError, OSError, RuntimeError):
            # Cleanup after a failed watchdog/worker is best effort; always release IPC.
            pass
        finally:
            self.quit_event.set()
            if self.started:
                self.process.join(3)
                if self.process.is_alive():
                    # Controller-side watchdog is the fallback when Python cannot clean up.
                    self.process.terminate()
                    self.process.join(2)
            self.connection.close()

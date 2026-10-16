"""Episode loop with prompt/record/keep/discard state transitions."""
import contextlib
import json
import select
import sys
import time
from pathlib import Path

from .cameras import CameraRig
from .dataset import V3Writer, build_frame
from .geometry import check_target, target_from_axes


class TerminalKeys:
    def __enter__(self):
        self.saved = None
        if sys.stdin.isatty() and sys.platform != "win32":
            import termios
            import tty
            self.saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def read(self):
        if sys.platform == "win32" and sys.stdin.isatty():
            import msvcrt
            if msvcrt.kbhit():
                key = msvcrt.getwch()
                if key in ("\x00", "\xe0"):
                    msvcrt.getwch()  # consume the second half of a Windows special key
                    return ""
                if key == "\x03":
                    raise KeyboardInterrupt
                return key.lower()
        if self.saved is not None and select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1).lower()
        return ""

    def __exit__(self, *args):
        if self.saved is not None:
            import termios
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.saved)


def record(cfg, unattended=False):
    from .lerobot_bridge import UR5Robot, UR5RobotConfig, SpaceMouseTeleoperator, SpaceMouseTeleoperatorConfig
    if cfg.mode == "real":
        if not cfg.gripper.feedback_plugin:
            raise ValueError("Live 8D capture requires gripper.feedback_plugin (real position feedback)")
        if unattended:
            raise ValueError("--unattended is supported only in mock mode")
        if not cfg.robot.motion_enabled:
            raise ValueError("Live recording requires robot.motion_enabled=true; use devices for diagnostics")
        if any(not c.serial.isdigit() for c in cfg.cameras):
            raise ValueError("Replace all camera serial placeholders before live recording")
    if Path(cfg.dataset.root).exists():
        raise FileExistsError(f"Output already exists: {cfg.dataset.root}")
    rig = CameraRig(cfg)
    robot = UR5Robot(UR5RobotConfig(id="ur5"), capture_config=cfg)
    mouse = SpaceMouseTeleoperator(SpaceMouseTeleoperatorConfig(id="spacemouse"), capture_config=cfg)
    writer = None
    kept = attempted = 0
    try:
        rig.connect()
        mouse.connect()
        robot.connect()
        state = robot.get_state(cfg.synchronization.max_robot_age_s)
        check_target(state.pose, state.pose, cfg.robot)
        writer = V3Writer(cfg, rig.metadata())
        while kept < cfg.dataset.episodes:
            if not unattended:
                if input(f"Episode {kept + 1}: Enter starts; q quits > ").strip().lower() == "q":
                    break
                print("Hold SpaceMouse deadman to move. g: toggle commanded grasp; s: end, d: discard, q: quit.")
            attempted += 1
            decision = "s"
            start = deadline = time.monotonic()
            frames = round(cfg.dataset.episode_seconds * cfg.dataset.fps)
            try:
                with TerminalKeys() as keys:
                    for index in range(frames):
                        key = keys.read()
                        if key == "g":
                            mouse.toggle_grasp()  # physical execution is gated below by deadman
                        if key in ("s", "d", "q"):
                            decision = key
                            break
                        if time.monotonic() - deadline > cfg.synchronization.max_loop_lateness_s:
                            raise RuntimeError("Recorder missed its timing deadline; episode discarded")
                        images = rig.read()
                        state = robot.get_state(cfg.synchronization.max_robot_age_s)
                        gripper_sample = robot.read_gripper_qpos()  # absolute feedback, BEFORE current action
                        sample = mouse.read()
                        if sample.stop:
                            raise RuntimeError("SpaceMouse stop button pressed")
                        target, delta = target_from_axes(state.pose, sample.axes, cfg, sample.enabled)
                        response = robot.send_target(target, 1 / cfg.dataset.fps, sample.enabled)
                        robot.update_gripper_command(sample.grasp, enabled=sample.enabled)
                        frame = build_frame(cfg, state, images, target, delta, sample.grasp, sample.enabled,
                                            response["accepted_host_time"], gripper_sample)
                        writer.add(frame)
                        deadline = start + (index + 1) / cfg.dataset.fps
                        time.sleep(max(0, deadline - time.monotonic()))
            finally:
                # Stop before episode encoding or any blocking input prompt.
                robot.stop()
            if writer.count < 2:
                decision = "d" if decision != "q" else "q"
            if decision == "s" and not unattended:
                decision = input("Keep episode? Enter/s=save, d=discard, q=discard and quit > ").strip().lower() or "s"
            if decision == "s":
                writer.save()
                kept += 1
                print(f"Saved episode {kept}/{cfg.dataset.episodes}")
            else:
                writer.discard()
                print("Discarded current episode")
            if decision == "q":
                break
        return {"saved_episodes": kept, "attempted_episodes": attempted, "root": cfg.dataset.root}
    except BaseException as exc:
        if writer is not None:
            log = {"error": f"{type(exc).__name__}: {exc}", "saved_episodes": kept,
                   "partial_episode_discarded": True, "monotonic_time": time.monotonic()}
            (writer.root / "meta" / "capture_failure.json").write_text(json.dumps(log, indent=2), encoding="utf-8")
        raise
    finally:
        # Motion cleanup takes priority over file cleanup.
        with contextlib.suppress(Exception):
            robot.disconnect()
        with contextlib.suppress(Exception):
            mouse.disconnect()
        rig.disconnect()
        if writer is not None:
            writer.close()

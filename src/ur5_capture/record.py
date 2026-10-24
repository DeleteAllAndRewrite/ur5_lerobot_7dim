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


def prompt(text, display=None):
    """input() that also accepts keys typed in the preview window."""
    if display is None or sys.platform == "win32" or not sys.stdin.isatty():
        return input(text)
    display.clear_keys()
    print(text, end="", flush=True)
    while True:
        if select.select([sys.stdin], [], [], 0.05)[0]:
            line = sys.stdin.readline()
            if not line:
                raise EOFError
            return line.rstrip("\n")
        key = display.read_key()
        if key:
            # Window keys answer immediately: Enter = default, q/s/d = choice.
            print("" if key == "\n" else key, flush=True)
            return "" if key == "\n" else key


def next_dataset_root(base: str) -> Path:
    """Return base_0000, base_0001, ... after the highest existing run number."""
    root = Path(base)
    prefix = f"{root.name}_"
    latest = -1
    if root.parent.exists():
        for entry in root.parent.iterdir():
            if entry.name.startswith(prefix):
                suffix = entry.name[len(prefix):]
                if suffix.isascii() and suffix.isdigit():
                    latest = max(latest, int(suffix))
    return root.with_name(f"{root.name}_{latest + 1:04d}")


def record(cfg, unattended=False, *, preview=False, preview_tile_width=480):
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
    # Keep the configured base reusable, including repeated calls with the same cfg.
    cfg = cfg.model_copy(deep=True)
    cfg.dataset.root = str(next_dataset_root(cfg.dataset.root))
    print(f"Dataset output: {cfg.dataset.root}")
    rig = CameraRig(cfg)
    robot = UR5Robot(UR5RobotConfig(id="ur5"), capture_config=cfg)
    mouse = SpaceMouseTeleoperator(SpaceMouseTeleoperatorConfig(id="spacemouse"), capture_config=cfg)
    writer = None
    display = None
    kept = attempted = 0
    stage = "preview startup"
    try:
        if preview:
            from .preview import RecordingPreview
            display = RecordingPreview(cfg, tile_width=preview_tile_width)
            display.start()
        stage = "camera connection"
        print("Connecting cameras...", flush=True)
        rig.connect()
        if display is not None:
            display.watch_idle(rig)
        stage = "SpaceMouse connection"
        print("Connecting SpaceMouse...", flush=True)
        mouse.connect()
        stage = "UR5 / RG2 connection"
        print(f"Connecting UR5 / RG2 at {cfg.robot.ip} "
              f"(External Control port {cfg.robot.ur_cap_port})...", flush=True)
        robot.connect()
        state = robot.get_state(cfg.synchronization.max_robot_age_s)
        check_target(state.pose, state.pose, cfg.robot)
        stage = "dataset initialization"
        writer = V3Writer(cfg, rig.metadata())
        while kept < cfg.dataset.episodes:
            stage = "waiting for episode"
            if not unattended:
                if prompt(f"Episode {kept + 1}: Enter starts; q quits > ", display).strip().lower() == "q":
                    break
                print("Move SpaceMouse cap to move. Button 0 or g: toggle gripper; button 1: speed low/high.")
                print("Terminal or camera window: s ends episode, d discards, q discards and quits. Use pendant E-stop for emergency stop.")
            attempted += 1
            decision = "s"
            timed_out = False
            deadline = time.monotonic()
            frames = round(cfg.dataset.episode_seconds * cfg.dataset.fps)
            # RealSense may spend several frames finishing USB/codec startup even
            # after connect() succeeds. Establish the strict schedule only after
            # this bounded warm-up, so startup latency cannot discard an episode.
            warmup_frames = min(frames - 1, max(5, cfg.dataset.fps // 2))
            try:
                stage = "episode capture"
                if display is not None:
                    display.update()
                mouse.start_episode()
                if display is not None:
                    display.clear_keys()
                with TerminalKeys() as keys:
                    for index in range(frames):
                        key = keys.read() or (display.read_key() if display is not None else "")
                        if key == "g":
                            mouse.toggle_grasp()
                        if key in ("s", "d", "q"):
                            decision = key
                            break
                        lateness = time.monotonic() - deadline
                        if index >= warmup_frames and lateness > cfg.synchronization.max_loop_lateness_s:
                            raise RuntimeError(
                                "Recorder missed its timing deadline; "
                                f"frame={index} lateness={lateness:.3f}s; episode discarded"
                            )
                        images = rig.read()
                        state = robot.get_state(cfg.synchronization.max_robot_age_s)
                        gripper_sample = robot.read_gripper_qpos()  # absolute feedback, BEFORE current action
                        sample = mouse.read()
                        target, delta = target_from_axes(state.pose, sample.axes, cfg, sample.enabled,
                                                         high_speed=sample.high_speed)
                        response = robot.send_target(target, 1 / cfg.dataset.fps, sample.enabled)
                        robot.update_gripper_command(sample.grasp, enabled=sample.enabled)
                        frame = build_frame(cfg, state, images, target, delta, sample.grasp, sample.enabled,
                                            response["accepted_host_time"], gripper_sample)
                        writer.add(frame)
                        if display is not None:
                            display.update(images)
                        if index == warmup_frames - 1:
                            # Discard startup delay from the schedule while keeping
                            # all warm-up frames in the recorded episode.
                            deadline = time.monotonic()
                        else:
                            deadline += 1 / cfg.dataset.fps
                            now = time.monotonic()
                            if deadline < now:
                                # Rebase after a small overrun; otherwise several
                                # sub-period overruns accumulate into a false miss.
                                deadline = now
                        time.sleep(max(0, deadline - time.monotonic()))
                    else:
                        timed_out = True
            finally:
                # Stop before episode encoding or any blocking input prompt.
                robot.stop()
                if display is not None:
                    display.update(recording=False)
            if timed_out and writer.count >= 2:
                print(f"Episode time limit reached ({cfg.dataset.episode_seconds:g}s).")
            if writer.count < 2:
                decision = "d" if decision != "q" else "q"
            if decision == "s" and not unattended:
                decision = prompt("Keep episode? Enter/s=save, d=discard, q=discard and quit > ", display).strip().lower() or "s"
            if decision == "s":
                stage = "episode saving"
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
        print(f"Capture failed during {stage}: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        if writer is not None:
            log = {"error": f"{type(exc).__name__}: {exc}", "saved_episodes": kept,
                   "stage": stage, "partial_episode_discarded": True, "monotonic_time": time.monotonic()}
            (writer.root / "meta" / "capture_failure.json").write_text(json.dumps(log, indent=2), encoding="utf-8")
        raise
    finally:
        # Motion cleanup takes priority over file cleanup.
        with contextlib.suppress(Exception):
            robot.disconnect()
        with contextlib.suppress(Exception):
            mouse.disconnect()
        rig.disconnect()
        if display is not None:
            display.close()
        if writer is not None:
            writer.close()

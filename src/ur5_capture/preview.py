"""Live, camera-only preview of the three configured RGB streams."""
import argparse
from contextlib import suppress
import math
import multiprocessing as mp
import os
import shutil
import subprocess
from pathlib import Path
from queue import Empty, Full
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np

from .cameras import CameraStream
from .config import load_config


def render_views(cv2, streams, frames, now, tile_width, max_age):
    """Render RGB frames as a BGR strip, preserving each camera's aspect ratio."""
    tile_height = max(round(tile_width * s.cfg.height / s.cfg.width) for s in streams)
    tiles = []
    for stream in streams:
        cfg = stream.cfg
        frame = frames[cfg.name]
        tile = np.zeros((tile_height + 76, tile_width, 3), dtype=np.uint8)
        color = (0, 200, 255)
        status = "WAITING"
        if frame is not None:
            height = round(tile_width * frame.rgb.shape[0] / frame.rgb.shape[1])
            bgr = cv2.cvtColor(frame.rgb, cv2.COLOR_RGB2BGR)
            y = 76 + (tile_height - height) // 2
            tile[y:y + height] = cv2.resize(bgr, (tile_width, height))
            age = max(0, now - frame.host_time)
            status = f"{'LIVE' if age <= max_age else 'STALE'}  age {age * 1000:.0f} ms"
            color = (0, 220, 0) if age <= max_age else (0, 0, 255)
        for text, y in ((f"{cfg.name} ({cfg.role})", 20),
                        (f"SN {cfg.serial} | {cfg.width}x{cfg.height} @ {cfg.fps}", 43),
                        (status, 66)):
            cv2.putText(tile, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        tiles.append(tile)
    return np.concatenate(tiles, axis=1)


def _load_cv2(*, headless=False):
    import cv2

    if not headless and any(
        line.strip().startswith("GUI:") and line.split(":", 1)[1].strip() == "NONE"
        for line in cv2.getBuildInformation().splitlines()
    ):
        raise RuntimeError("OpenCV has no GUI support. Run: python -m pip uninstall -y opencv-python-headless; "
                           "then python -m pip install --force-reinstall --no-deps 'opencv-python>=4.10,<5'")
    if not headless and sys.platform.startswith("linux") and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        raise RuntimeError("No desktop display. Run in a desktop terminal, or use --headless --duration 5.")
    return cv2


def _keep_on_top(cv2, window):
    """Best effort: keep the preview above other windows."""
    with suppress(Exception):
        cv2.setWindowProperty(window, cv2.WND_PROP_TOPMOST, 1)
    # OpenCV's GTK backend ignores WND_PROP_TOPMOST; ask the window manager.
    if sys.platform.startswith("linux") and shutil.which("wmctrl"):
        with suppress(Exception):
            subprocess.run(["wmctrl", "-r", window, "-b", "add,above"], timeout=1,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def _record_preview_worker(config, tile_width, frames_queue, stop, recording, status, keys_queue):
    """Own the GUI in a separate process; never open camera devices here."""
    window = "Episode cameras | keys go to recorder | close window to hide"
    ready = False
    try:
        cv2 = _load_cv2()
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        streams = [SimpleNamespace(cfg=c) for c in config.cameras]
        frames = {c.name: None for c in config.cameras}
        while not stop.is_set():
            tick = time.monotonic()
            try:
                frames = frames_queue.get(timeout=0.03)
            except Empty:
                pass
            strip = render_views(cv2, streams, frames, time.monotonic(), tile_width,
                                 config.synchronization.max_camera_age_s)
            strip = cv2.copyMakeBorder(strip, 28, 0, 0, 0, cv2.BORDER_CONSTANT)
            label = "RECORDING" if recording.is_set() else "PAUSED - live cameras / not recording"
            cv2.putText(strip, label + " | Keys here or in terminal: Enter / g / s / d / q", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 0), 1, cv2.LINE_AA)
            cv2.imshow(window, strip)
            key = cv2.waitKey(1) & 0xFF
            if not ready:
                # Report success only after the first frame and GUI events work,
                # so startup failures reach the recorder before hardware connects.
                status.send(None)
                ready = True
                _keep_on_top(cv2, window)
            if key in (10, 13):
                key = ord("\n")
            if key == ord("\n") or 32 <= key < 127:
                # Forward keys to the recorder instead of acting on them here.
                with suppress(Full):
                    keys_queue.put_nowait(chr(key).lower())
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            stop.wait(max(0, 1 / 15 - (time.monotonic() - tick)))
    except Exception as exc:
        print(f"Camera preview stopped: {exc}", file=sys.stderr, flush=True)
        if not ready:
            status.send(f"{type(exc).__name__}: {exc}")
    finally:
        stop.set()
        with suppress(Exception):
            cv2.destroyAllWindows()
        status.close()


class RecordingPreview:
    """Best-effort display of recorded frames, with a bounded, nonblocking queue."""

    def __init__(self, config, tile_width=480):
        if tile_width < 160:
            raise ValueError("Preview tile width must be at least 160")
        context = mp.get_context("spawn")
        self.frames = context.Queue(maxsize=1)
        # A stuck/dead GUI must never make interpreter shutdown wait for its queue.
        self.frames.cancel_join_thread()
        self.stop = context.Event()
        self.recording = context.Event()
        self.keys = context.Queue(maxsize=64)
        self.keys.cancel_join_thread()
        self.status, sender = context.Pipe(duplex=False)
        self.process = context.Process(target=_record_preview_worker,
            args=(config, tile_width, self.frames, self.stop, self.recording, sender, self.keys), daemon=True)
        self.sender = sender
        self.last_update = -math.inf
        self._lock = threading.Lock()
        self._idle_stop = threading.Event()
        self._idle_thread = None

    def start(self):
        self.process.start()
        self.sender.close()
        if not self.status.poll(10):
            raise RuntimeError("Camera preview failed to open within 10 seconds")
        try:
            error = self.status.recv()
        except EOFError as exc:
            raise RuntimeError("Camera preview process exited before opening its window") from exc
        if error is not None:
            raise RuntimeError(error)

    def watch_idle(self, rig):
        """Show latest camera buffers during robot connection, prompts and saving.

        Reading buffers does not consume synchronized recording frames or reopen
        devices. During an episode only update() supplies the recorded frames.
        """
        def pump():
            while not self._idle_stop.is_set() and not self.stop.is_set():
                with self._lock:
                    if not self.recording.is_set():
                        images = {}
                        for stream in rig.streams:
                            with stream.lock:
                                images[stream.cfg.name] = stream.buffer[-1] if stream.buffer else None
                        self._enqueue(images)
                self._idle_stop.wait(1 / 15)

        self._idle_thread = threading.Thread(target=pump, name="idle-camera-preview", daemon=True)
        self._idle_thread.start()

    def read_key(self):
        """Return the next key pressed in the preview window, or ""."""
        try:
            return self.keys.get_nowait()
        except (Empty, OSError, ValueError):
            return ""

    def clear_keys(self):
        while self.read_key():
            pass

    def _enqueue(self, images):
        now = time.monotonic()
        # Limit IPC/rendering work independently of the recording frame rate.
        if now - self.last_update < 1 / 15:
            return
        try:
            self.frames.put_nowait(images)
        except Full:
            return
        self.last_update = now

    def update(self, images=None, *, recording=True):
        with self._lock:
            if self.stop.is_set():
                return
            if recording:
                self.recording.set()
            else:
                self.recording.clear()
            if images is not None:
                self._enqueue(images)

    def close(self):
        self._idle_stop.set()
        if self._idle_thread is not None:
            self._idle_thread.join(timeout=1)
        self.stop.set()
        if self.process.pid is not None:
            self.process.join(timeout=1)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1)
        self.frames.close()
        self.keys.close()
        self.status.close()
        self.sender.close()


def preview(config, *, mock=False, tile_width=480, duration=None, headless=False):
    cv2 = _load_cv2(headless=headless)

    streams = [CameraStream(c, mock=mock or config.mode == "mock") for c in config.cameras]
    frames = {s.cfg.name: None for s in streams}
    window = "Three RGB cameras | Q / Esc to quit"
    window_open = False
    try:
        if not headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            window_open = True
        for stream in streams:
            print(f"Opening {stream.cfg.name}: {stream.cfg.serial}", flush=True)
            stream.connect()
        print("Preview started. " + ("Press Ctrl+C to quit." if headless else
                                     "Press Q / Esc in the window or Ctrl+C to quit."), flush=True)
        started = time.monotonic()
        while True:
            tick = time.monotonic()
            for stream in streams:
                if stream.error is not None:
                    raise RuntimeError(f"Camera {stream.cfg.name} failed: {stream.error}") from stream.error
                with stream.lock:
                    if stream.buffer:
                        frames[stream.cfg.name] = stream.buffer[-1]
            now = time.monotonic()
            missing = [name for name, frame in frames.items() if frame is None]
            if missing and now - started > config.synchronization.startup_timeout_s:
                raise TimeoutError(f"No frames received from: {', '.join(missing)}")
            strip = render_views(cv2, streams, frames, now, tile_width,
                                 config.synchronization.max_camera_age_s)
            if not headless:
                cv2.imshow(window, strip)
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    break
                if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    break
            if duration is not None and now - started >= duration:
                if missing:
                    raise TimeoutError(f"No frames received from: {', '.join(missing)}")
                break
            time.sleep(max(0, 1 / max(s.cfg.fps for s in streams) - (time.monotonic() - tick)))
        print("Preview stopped.")
    finally:
        for stream in reversed(streams):
            with suppress(Exception):
                stream.disconnect()
        if window_open:
            cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Preview three RealSense RGB cameras without robot control")
    parser.add_argument("--config", type=Path,
                        default=Path(__file__).resolve().parents[2] / "configs/ur5_three_rgb.yaml")
    parser.add_argument("--mock", action="store_true", help="Generate test images instead of opening cameras")
    parser.add_argument("--tile-width", type=int, default=480, help="Display width per camera (default: 480)")
    parser.add_argument("--duration", type=float, help="Stop automatically after this many seconds")
    parser.add_argument("--headless", action="store_true", help="Read and render frames without opening a window")
    args = parser.parse_args()
    if args.tile_width < 160:
        parser.error("--tile-width must be at least 160")
    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("--duration must be finite and positive")
    try:
        preview(load_config(args.config), mock=args.mock, tile_width=args.tile_width,
                duration=args.duration, headless=args.headless)
    except KeyboardInterrupt:
        print("\nPreview interrupted.")
    except Exception as exc:
        parser.exit(1, f"Preview failed: {exc}\n")


if __name__ == "__main__":
    main()

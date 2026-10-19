"""Live, camera-only preview of the three configured RGB streams."""
import argparse
from contextlib import suppress
import math
import os
from pathlib import Path
import sys
import time

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


def preview(config, *, mock=False, tile_width=480, duration=None, headless=False):
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

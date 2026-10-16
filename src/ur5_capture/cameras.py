"""Parallel RGB camera acquisition with explicit host-arrival timestamps."""
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class CameraFrame:
    rgb: np.ndarray
    host_time: float
    device_time_ms: float
    frame_number: int


def select_frames(buffers, previous, now, sync):
    candidates = {name: [f for f in frames if f.frame_number > previous.get(name, -1)]
                  for name, frames in buffers.items()}
    if any(not fs for fs in candidates.values()):
        raise RuntimeError("Waiting for a fresh frame from every camera")
    anchor = min(fs[-1].host_time for fs in candidates.values())
    selected = {name: min(fs, key=lambda f: abs(f.host_time - anchor)) for name, fs in candidates.items()}
    times = [f.host_time for f in selected.values()]
    if now - min(times) > sync.max_camera_age_s:
        raise RuntimeError("Camera frame is stale")
    if max(times) - min(times) > sync.max_camera_skew_s:
        raise RuntimeError("Camera host-arrival skew exceeded")
    return selected


class CameraStream:
    def __init__(self, cfg, mock=False):
        self.cfg, self.mock = cfg, mock
        self.buffer = deque(maxlen=8)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error = None
        self.pipeline = None
        self.thread = None
        self.metadata = {"serial": cfg.serial, "role": cfg.role, "color_order": "RGB"}

    def connect(self):
        if not self.mock:
            import pyrealsense2 as rs
            self.pipeline = rs.pipeline()
            options = rs.config()
            options.enable_device(self.cfg.serial)
            options.enable_stream(rs.stream.color, self.cfg.width, self.cfg.height, rs.format.rgb8, self.cfg.fps)
            profile = self.pipeline.start(options)
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            self.metadata["intrinsics"] = {"width": intr.width, "height": intr.height,
                "fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                "model": str(intr.model), "coeffs": list(intr.coeffs)}
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        frame_number = 0
        try:
            while not self.stop_event.is_set():
                if self.mock:
                    now = time.monotonic()
                    h, w = self.cfg.height, self.cfg.width
                    rgb = np.empty((h, w, 3), dtype=np.uint8)
                    rgb[:, :, 0] = np.arange(w, dtype=np.uint16)[None, :] % 256
                    rgb[:, :, 1] = np.arange(h, dtype=np.uint16)[:, None] % 256
                    rgb[:, :, 2] = (frame_number * 7 + sum(map(ord, self.cfg.name))) % 256
                    frame = CameraFrame(rgb, now, now * 1000, frame_number)
                    frame_number += 1
                    self.metadata["timestamp_domain"] = "mock_monotonic"
                else:
                    frames = self.pipeline.wait_for_frames(timeout_ms=500)
                    color = frames.get_color_frame()
                    if not color:
                        continue
                    now = time.monotonic()
                    frame = CameraFrame(np.asanyarray(color.get_data()).copy(), now,
                                        color.get_timestamp(), color.get_frame_number())
                    self.metadata["timestamp_domain"] = str(color.get_frame_timestamp_domain())
                with self.lock:
                    self.buffer.append(frame)
                if self.mock:
                    self.stop_event.wait(max(0, 1 / self.cfg.fps - (time.monotonic() - now)))
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error = exc

    def disconnect(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(1)
        if self.pipeline is not None:
            self.pipeline.stop()


class CameraRig:
    def __init__(self, config):
        self.cfg = config
        self.streams = [CameraStream(c, config.mode == "mock") for c in config.cameras]
        self.previous = {}

    def connect(self):
        try:
            for stream in self.streams:
                stream.connect()
            self.read(self.cfg.synchronization.startup_timeout_s)
        except BaseException:
            self.disconnect()
            raise

    def read(self, timeout=None):
        sync = self.cfg.synchronization
        deadline = time.monotonic() + (sync.max_camera_age_s if timeout is None else timeout)
        last_error = None
        while time.monotonic() < deadline:
            buffers = {}
            for stream in self.streams:
                if stream.error is not None:
                    raise RuntimeError(f"Camera {stream.cfg.name} failed: {stream.error}") from stream.error
                with stream.lock:
                    buffers[stream.cfg.name] = list(stream.buffer)
            try:
                selected = select_frames(buffers, self.previous, time.monotonic(), sync)
                self.previous = {name: frame.frame_number for name, frame in selected.items()}
                return selected
            except RuntimeError as exc:
                last_error = exc
            time.sleep(0.001)
        raise TimeoutError(f"Three-view synchronization failed: {last_error}")

    def metadata(self):
        return {s.cfg.name: s.metadata.copy() for s in self.streams}

    def disconnect(self):
        for stream in reversed(self.streams):
            try:
                stream.disconnect()
            except Exception:
                pass

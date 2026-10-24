"""The recording display must not acquire cameras or wait for a slow GUI."""
from queue import Empty, Queue
from types import SimpleNamespace
import threading
import time

import numpy as np
import pytest

from ur5_capture import preview
from ur5_capture.config import load_config
from ur5_capture.cameras import CameraRig


def test_slow_preview_drops_display_frames_without_changing_images(monkeypatch):
    cfg = load_config("configs/mock.yaml")
    display = preview.RecordingPreview(cfg)
    images = {c.name: np.zeros((c.height, c.width, 3), np.uint8) for c in cfg.cameras}
    ticks = iter(range(100))
    monkeypatch.setattr(preview.time, "monotonic", lambda: next(ticks))
    try:
        # No reader: the queue stays full, simulating a stalled GUI process.
        started = time.perf_counter()
        for _ in range(100):
            display.update(images)
        assert time.perf_counter() - started < 1
        assert all(not image.any() for image in images.values())
        display.update(recording=False)
        assert not display.recording.is_set()
    finally:
        display.close()


def test_idle_preview_refreshes_without_consuming_recording_frames():
    cfg = load_config("configs/mock.yaml")
    rig = CameraRig(cfg)
    display = preview.RecordingPreview(cfg)
    try:
        rig.connect()
        previous = rig.previous.copy()
        display.watch_idle(rig)
        first = display.frames.get(timeout=2)
        second = display.frames.get(timeout=2)
        assert set(first) == {c.name for c in cfg.cameras}
        assert all(second[name].frame_number > first[name].frame_number for name in first)
        assert rig.previous == previous
        assert not display.recording.is_set()

        display.update()  # Episode starts: idle thread must stop sending images.
        try:
            display.frames.get(timeout=0.15)  # allow an already queued idle frame
        except Empty:
            pass
        with pytest.raises(Empty):
            display.frames.get(timeout=0.15)
        recorded = rig.read()
        display.update(recorded)
        shown = display.frames.get(timeout=2)
        assert {k: f.frame_number for k, f in shown.items()} == {
            k: f.frame_number for k, f in recorded.items()}

        display.update(recording=False)
        resumed = display.frames.get(timeout=2)
        assert all(resumed[name].frame_number >= shown[name].frame_number for name in shown)
    finally:
        display.close()
        rig.disconnect()


@pytest.mark.parametrize("render_fails", [False, True])
def test_window_close_only_stops_preview_and_uses_supplied_frames(monkeypatch, render_fails):
    cfg = load_config("configs/mock.yaml")
    images = {c.name: object() for c in cfg.cameras}
    received = []
    frames_queue = Queue(maxsize=1)
    frames_queue.put(images)
    stop, recording = threading.Event(), threading.Event()
    recording.set()
    status = SimpleNamespace(send=lambda value: received.append(value), close=lambda: None)
    rendered = []

    def render(cv2, streams, frames, *args):
        assert [s.cfg.name for s in streams] == [c.name for c in cfg.cameras]
        rendered.append(frames)
        return np.zeros((10, 10, 3), np.uint8)

    def show(*args):
        if render_fails:
            raise RuntimeError("Cannot display first frame")

    cv2 = SimpleNamespace(
        WINDOW_NORMAL=0, BORDER_CONSTANT=0, FONT_HERSHEY_SIMPLEX=0, LINE_AA=0,
        namedWindow=lambda *args: None, copyMakeBorder=lambda image, *args: image,
        putText=lambda *args: None, imshow=show,
        WND_PROP_VISIBLE=0, WND_PROP_TOPMOST=1, setWindowProperty=lambda *args: None,
        # Keys are forwarded to the recorder; only closing the window ends the preview.
        waitKey=lambda *args: ord("Q"), getWindowProperty=lambda *args: 0, destroyAllWindows=lambda: None,
    )
    keys_queue = Queue()
    monkeypatch.setattr(preview, "_load_cv2", lambda: cv2)
    monkeypatch.setattr(preview, "render_views", render)
    monkeypatch.setattr(preview.CameraStream, "connect",
                        lambda self: (_ for _ in ()).throw(AssertionError("Camera reopened")))
    preview._record_preview_worker(cfg, 480, frames_queue, stop, recording, status, keys_queue)
    assert rendered == [images]
    assert [keys_queue.get_nowait() for _ in range(keys_queue.qsize())] == ([] if render_fails else ["q"])
    assert received == (["RuntimeError: Cannot display first frame"] if render_fails else [None])
    assert stop.is_set()
    assert recording.is_set()  # Closing the window does not request recording to stop.


@pytest.mark.parametrize("capture_fails", [False, True])
def test_recorder_previews_saved_images_and_cleans_up(tmp_path, monkeypatch, capture_fails):
    from ur5_capture import record as recorder

    cfg = load_config("configs/mock.yaml")
    cfg.dataset.root = str(tmp_path / "data")
    cfg.dataset.episodes = 1
    cfg.dataset.episode_seconds = 0.2
    stored = []
    displayed = []
    events = []

    class Writer:
        def __init__(self, config, metadata):
            self.root = tmp_path
            (self.root / "meta").mkdir()
            self.count = 0

        def add(self, frame):
            if capture_fails and self.count == 2:
                raise RuntimeError("Capture interrupted")
            stored.append(frame)
            self.count += 1

        def save(self):
            assert events[-1] == "paused"
            events.append("saved")
            self.count = 0

        def close(self):
            events.append("writer_closed")

    class Display:
        def __init__(self, config, tile_width):
            pass

        def start(self):
            events.append("opened")

        def watch_idle(self, rig):
            pass

        def read_key(self):
            return ""

        def clear_keys(self):
            pass

        def update(self, images=None, *, recording=True):
            if recording and images is not None:
                displayed.append(images)
            elif not recording:
                events.append("paused")

        def close(self):
            events.append("display_closed")

    monkeypatch.setattr(recorder, "V3Writer", Writer)
    monkeypatch.setattr(preview, "RecordingPreview", Display)
    if capture_fails:
        with pytest.raises(RuntimeError, match="Capture interrupted"):
            recorder.record(cfg, unattended=True, preview=True)
        assert "saved" not in events
        assert (tmp_path / "meta/capture_failure.json").exists()
    else:
        result = recorder.record(cfg, unattended=True, preview=True)
        assert result["saved_episodes"] == 1
        assert len(stored) == 6
        assert "saved" in events
    assert len(displayed) == len(stored) > 0
    for frame, images in zip(stored, displayed, strict=True):
        assert set(images) == {c.name for c in cfg.cameras}
        for camera in cfg.cameras:
            assert frame[f"observation.images.{camera.name}"] is images[camera.name].rgb
    assert events[0] == "opened"
    assert "paused" in events
    assert events[-2:] == ["display_closed", "writer_closed"]

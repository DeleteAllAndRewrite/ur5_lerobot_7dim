"""Exercise real Modbus framing and asynchronous control against a socket peer."""
import contextlib
import socket
import struct
import threading
import time
import types

import numpy as np
import pytest

from ur5_capture.config import GripperConfig, load_config
from ur5_capture.gripper import GripperFeedback
from ur5_capture.rg2 import RG2Config, RG2Gripper, RTDEGripper, _ModbusTCP


def eventually(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    assert predicate(), "Condition not reached before deadline"


class ComputeBoxPeer:
    def __init__(self, monkeypatch):
        self.client, self.server = socket.socketpair()
        self.width = 1100
        self.status = 0
        self.contact_width = None
        self.moves = []
        self.stops = 0
        self.reads = 0
        self.reply_mode = None
        self.hold_busy = False
        self.pause = threading.Event()
        self.pause.set()
        self.address = None
        self.errors = []
        monkeypatch.setattr(socket, "create_connection", self.connect)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def connect(self, address, timeout):
        self.address = address
        self.client.settimeout(timeout)
        return self.client

    def receive(self, size):
        data = bytearray()
        while len(data) < size:
            chunk = self.server.recv(size - len(data))
            if not chunk:
                raise EOFError
            data.extend(chunk)
        return bytes(data)

    def run(self):
        try:
            while True:
                tid, protocol, length, unit = struct.unpack(">HHHB", self.receive(7))
                assert protocol == 0 and unit == 65
                request = self.receive(length - 1)
                function = request[0]
                if function == 3:
                    assert struct.unpack(">HH", request[1:]) == (267, 2)
                    self.reads += 1
                    self.pause.wait(2)
                    response = struct.pack(">BBHH", 3, 4, self.width, self.status)
                elif function == 16:
                    assert struct.unpack(">HHB", request[1:6]) == (0, 3, 6)
                    force, width, control = struct.unpack(">HHH", request[6:])
                    assert control == 1 and not self.status & 1
                    self.moves.append((force, width, control))
                    self.width = width if self.contact_width is None else self.contact_width
                    if self.hold_busy:
                        self.status |= 1
                    response = struct.pack(">BHH", 16, 0, 3)
                elif function == 6:
                    assert request[1:] == struct.pack(">HH", 2, 8)
                    self.stops += 1
                    self.status &= ~1
                    response = request
                else:
                    raise AssertionError(f"Unexpected function {function}")
                if self.reply_mode == "exception":
                    response = bytes([function | 0x80, 2])
                elif self.reply_mode == "wrong_tid":
                    tid += 1
                elif self.reply_mode == "wrong_count":
                    response = bytes([3, 2, 0, 0])
                elif self.reply_mode == "bad_ack":
                    response = bytes([function, 0, 9, 0, 9])
                elif self.reply_mode == "close":
                    return
                packet = struct.pack(">HHHB", tid, 0, len(response) + 1, unit) + response
                # TCP may split a frame at ANY byte; do not rely on one recv per response.
                for byte in packet:
                    self.server.sendall(bytes([byte]))
        except (EOFError, OSError):
            pass
        except Exception as exc:
            self.errors.append(exc)
        finally:
            self.server.close()

    def close(self):
        self.pause.set()
        with contextlib.suppress(OSError):
            self.client.shutdown(socket.SHUT_RDWR)
        self.client.close()
        self.thread.join(3)
        assert not self.thread.is_alive()
        assert not self.errors


@pytest.fixture
def peer(monkeypatch):
    server = ComputeBoxPeer(monkeypatch)
    yield server
    server.close()


@pytest.mark.parametrize("kwargs", [
    {"max_width_mm": 111}, {"open_width_mm": 111}, {"force_n": 41}, {"force_n": 2},
    {"close_width_mm": -1}, {"close_width_mm": 110}, {"unit_id": 0}, {"unit_id": 248},
    {"open_width_mm": float("nan")}, {"force_n": float("inf")}, {"max_width_mm": 100},
    {"feedback_timeout_s": 0.01}, {"typo": 1},
])
def test_reject_invalid_rg2_parameters(kwargs):
    with pytest.raises(ValueError):
        RG2Gripper(host="compute-box", **kwargs)


def test_connect_feedback_only_and_signed_width(peer):
    driver = RG2Gripper(host="compute-box")
    try:
        driver.connect()
        assert peer.address == ("compute-box", 502)
        np.testing.assert_allclose(driver.read_qpos(), [0.055, -0.055])
        sampled = driver.sample_host_time
        eventually(lambda: peer.reads >= 3)
        assert driver.sample_host_time == sampled  # timestamp belongs to the previous read
        assert peer.moves == []
    finally:
        driver.disconnect()
    assert peer.stops == 0  # read-only diagnostics do not write to the device


def test_measured_contact_width_is_not_the_closed_command(peer):
    peer.contact_width = 300  # obstruction/object at 30 mm
    cfg = GripperConfig(feedback_source="plugin", feedback_plugin="ur5_capture.rg2:RG2Gripper",
                        feedback_kwargs={"host": "compute-box"}, actuation_enabled=True)
    adapter = GripperFeedback(cfg)
    adapter.connect()
    try:
        first = adapter.read()
        adapter.update_command(1, enabled=False)
        assert peer.moves == []
        adapter.update_command(1, enabled=True)
        eventually(lambda: len(peer.moves) == 1)
        eventually(lambda: np.isclose(adapter.read().qpos[0], 0.015))
        for _ in range(10):
            adapter.update_command(1, enabled=True)
        assert peer.moves == [(200, 0, 1)]
        np.testing.assert_allclose(first.qpos, [.055, -.055])
        np.testing.assert_allclose(adapter.read().qpos, [.015, -.015])
        assert adapter.read().host_time <= time.monotonic()
        adapter.update_command(1, enabled=False)
        assert peer.stops >= 1
        adapter.update_command(0, enabled=True)
        eventually(lambda: len(peer.moves) == 2)
        assert peer.moves[-1] == (200, 1100, 1)
    finally:
        adapter.disconnect()


def test_latest_target_waits_until_idle(peer):
    peer.status = 1
    driver = RG2Gripper(host="compute-box", force_n=3)
    driver.connect()
    try:
        driver.set_grasp(1)
        driver.set_grasp(0)
        eventually(lambda: peer.reads >= 4)
        assert peer.moves == []
        peer.status = 0
        eventually(lambda: len(peer.moves) == 1)
        assert peer.moves == [(30, 1100, 1)]
    finally:
        driver.disconnect()


def test_stop_cancels_queued_move(peer):
    peer.status = 1
    driver = RG2Gripper(host="compute-box")
    driver.connect()
    try:
        driver.set_grasp(1)
        driver.stop()
        eventually(lambda: peer.reads >= 4)
        assert peer.moves == []
        assert peer.stops == 1
    finally:
        driver.disconnect()


def test_busy_movement_timeout_is_reported(peer):
    peer.hold_busy = True
    driver = RG2Gripper(host="compute-box", motion_timeout_s=0.06)
    driver.connect()
    try:
        driver.set_grasp(1)
        eventually(lambda: driver._error is not None)
        with pytest.raises(RuntimeError, match="movement timed out"):
            driver.read_qpos()
    finally:
        with contextlib.suppress(RuntimeError):
            driver.disconnect()
    assert peer.stops >= 1


def test_stale_feedback_is_rejected_without_waiting_for_io(peer):
    driver = RG2Gripper(host="compute-box", feedback_timeout_s=0.04, socket_timeout_s=0.2)
    driver.connect()
    try:
        peer.pause.clear()
        time.sleep(0.07)
        before = time.monotonic()
        with pytest.raises(RuntimeError, match="stale"):
            driver.read_qpos()
        assert time.monotonic() - before < 0.03
    finally:
        peer.pause.set()
        driver.disconnect()


@pytest.mark.parametrize("status", [4, 8, 16, 32, 64])
def test_safety_switch_aborts_connection(peer, status):
    peer.status = status
    driver = RG2Gripper(host="compute-box")
    with pytest.raises(RuntimeError, match="safety status"):
        driver.connect()
    assert driver._client.sock is None
    driver.disconnect()


@pytest.mark.parametrize("mode, message", [
    ("exception", "exception 2"), ("wrong_tid", "MBAP"),
    ("wrong_count", "register count"), ("close", "closed"),
])
def test_invalid_protocol_never_becomes_a_measurement(peer, mode, message):
    peer.reply_mode = mode
    client = _ModbusTCP(RG2Config(host="compute-box"))
    client.connect()
    try:
        with pytest.raises((RuntimeError, ConnectionError), match=message):
            client.read(267, 2)
    finally:
        client.close()


@pytest.mark.parametrize("operation", ["write", "stop"])
def test_write_ack_is_validated(peer, operation):
    peer.reply_mode = "bad_ack"
    client = _ModbusTCP(RG2Config(host="compute-box"))
    client.connect()
    try:
        with pytest.raises(RuntimeError, match="acknowledgement"):
            client.write(0, [200, 0, 1]) if operation == "write" else client.stop()
    finally:
        client.close()


def test_feedback_only_plugin_requires_actuator_api_only_when_enabled(monkeypatch):
    import sys
    module = types.ModuleType("rg2_test_sensor")
    class Sensor:
        def connect(self): pass
        def disconnect(self): pass
        def read_qpos(self): return [.01, -.01]
    module.Sensor = Sensor
    monkeypatch.setitem(sys.modules, module.__name__, module)
    cfg = GripperConfig(feedback_source="plugin", feedback_plugin="rg2_test_sensor:Sensor")
    reader = GripperFeedback(cfg)
    reader.connect()
    reader.update_command(1)
    np.testing.assert_allclose(reader.read().qpos, [.01, -.01])
    reader.disconnect()
    cfg.actuation_enabled = True
    with pytest.raises(TypeError, match="set_grasp"):
        GripperFeedback(cfg).connect()


def test_rg2_config_is_validated_before_connecting():
    cfg = load_config("configs/ur5_three_rgb.yaml")
    assert cfg.gripper.feedback_kwargs["max_width_mm"] == 110
    assert cfg.gripper.actuation_enabled is True
    data = cfg.gripper.model_dump()
    data["feedback_kwargs"]["force_n"] = 50
    with pytest.raises(ValueError):
        GripperConfig.model_validate(data)


class FakeRTDEClient:
    """Deterministic register peer for RTDEGripper worker tests."""

    def __init__(self, error=0):
        self.connected = False
        self.closed = False
        self.error = error
        self.status = (0, 0, 1100, 0, error)
        self.sent = []
        self._lock = threading.Lock()

    def connect(self):
        self.connected = True

    def close(self):
        self.connected = False
        self.closed = True

    def read(self):
        with self._lock:
            return self.status if self.connected else None

    def send(self, width, force, sequence):
        with self._lock:
            self.sent.append((width, force, sequence))
            self.status = (sequence, 2, round(width * 10), 0, 0)


def make_rtde_driver(fake=None, **kwargs):
    driver = RTDEGripper(robot_ip="robot", **kwargs)
    fake = fake or FakeRTDEClient()
    driver._client = fake
    return driver, fake


def test_rtde_connect_reads_status_and_disconnects():
    driver, fake = make_rtde_driver()
    driver.connect()
    try:
        np.testing.assert_allclose(driver.read_qpos(), [0.055, -0.055])
    finally:
        driver.disconnect()
    assert fake.closed
    assert driver._thread is None


def test_rtde_command_is_sent_and_acknowledged():
    driver, fake = make_rtde_driver()
    driver.connect()
    try:
        driver.set_grasp(1)
        eventually(lambda: len(fake.sent) == 1)
        assert fake.sent[0][:2] == (0.0, 20.0)
        eventually(lambda: driver._inflight is None)
    finally:
        driver.disconnect()


def test_rtde_controller_error_fails_connect():
    driver, fake = make_rtde_driver(FakeRTDEClient(error=2))
    with pytest.raises(RuntimeError, match="controller error code: 2"):
        driver.connect()
    assert fake.closed


def test_rtde_duplicate_target_is_suppressed():
    driver, fake = make_rtde_driver()
    driver.connect()
    try:
        driver.set_grasp(1)
        driver.set_grasp(1)
        eventually(lambda: len(fake.sent) == 1)
        time.sleep(0.03)
        assert len(fake.sent) == 1
    finally:
        driver.disconnect()


def test_rtde_stop_allows_resubmitting_same_target():
    driver, fake = make_rtde_driver()
    driver.connect()
    try:
        driver.set_grasp(1)
        eventually(lambda: len(fake.sent) == 1 and driver._inflight is None)
        driver.stop()
        driver.set_grasp(1)
        eventually(lambda: len(fake.sent) == 2)
    finally:
        driver.disconnect()

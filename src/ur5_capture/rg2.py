"""OnRobot RG2 drivers for Compute Box Modbus TCP and UR RTDE registers.

Raw aluminum-finger opening: 0..110 mm, force: 4..40 N for bundled worker. Register map:
OnRobot User Manual for TECHMAN/OMRON TM v1.1.2, Modbus TCP section.
https://onrobot.com/sites/default/files/documents/User_Manual_for_TECHMAN_OMRON_TM_v1.1.2_EN_0.pdf

The RTDE path uses the register worker from the ROS workspace. That worker
reports its accepted target width; it does not expose an RG2 encoder.
"""
import contextlib
import socket
import struct
import threading
import time

from pydantic import Field, model_validator

from .config import StrictModel


class RG2Config(StrictModel):
    host: str = Field(min_length=1)  # Compute Box IP, NOT the UR controller IP
    port: int = Field(502, ge=1, le=65535)
    unit_id: int = Field(65, ge=1, le=247)  # single=65, dual primary=66, secondary=67
    max_width_mm: float = Field(110.0, gt=0, le=110)
    open_width_mm: float = Field(110.0, ge=0, le=110)
    close_width_mm: float = Field(0.0, ge=0, le=110)
    force_n: float = Field(20.0, ge=3, le=40)
    poll_hz: float = Field(50.0, ge=1, le=100)
    socket_timeout_s: float = Field(0.2, gt=0, le=1)
    feedback_timeout_s: float = Field(0.25, gt=0, le=2)
    motion_timeout_s: float = Field(5.0, gt=0, le=30)

    @model_validator(mode="after")
    def validate_ranges(self):
        if not self.close_width_mm < self.open_width_mm <= self.max_width_mm:
            raise ValueError("Require 0 <= close_width_mm < open_width_mm <= max_width_mm <= 110")
        if round(self.close_width_mm * 10) >= round(self.open_width_mm * 10):
            raise ValueError("Open and close widths must differ at RG2's 0.1 mm resolution")
        if self.feedback_timeout_s <= 1 / self.poll_hz:
            raise ValueError("feedback_timeout_s must exceed the polling period")
        return self


class RTDEConfig(StrictModel):
    """UR5 RTDE register map used by the ROS RG2 bridge."""

    robot_ip: str = Field(min_length=1)
    port: int = Field(30004, ge=1, le=65535)
    reg_cmd_seq: int = Field(24, ge=0, le=47)
    reg_target_width: int = Field(25, ge=0, le=47)
    reg_target_force: int = Field(26, ge=0, le=47)
    reg_ack_seq: int = Field(24, ge=0, le=47)
    reg_state: int = Field(25, ge=0, le=47)
    reg_current_width: int = Field(26, ge=0, le=47)
    reg_is_gripped: int = Field(27, ge=0, le=47)
    reg_error_code: int = Field(28, ge=0, le=47)
    max_width_mm: float = Field(110.0, gt=0, le=110)
    open_width_mm: float = Field(110.0, ge=0, le=110)
    close_width_mm: float = Field(0.0, ge=0, le=110)
    # The bundled URScript worker accepts 4..40 N.  RG2 hardware manuals often
    # list 3 N, but sending 3 makes this worker reject every command.
    force_n: float = Field(20.0, ge=4, le=40)
    poll_hz: float = Field(50.0, ge=1, le=125)
    socket_timeout_s: float = Field(0.5, gt=0, le=5)
    feedback_timeout_s: float = Field(0.25, gt=0, le=2)
    motion_timeout_s: float = Field(5.0, gt=0, le=30)

    @model_validator(mode="after")
    def validate_ranges(self):
        if not self.close_width_mm < self.open_width_mm <= self.max_width_mm:
            raise ValueError("Require 0 <= close_width_mm < open_width_mm <= max_width_mm <= 110")
        if round(self.close_width_mm * 10) >= round(self.open_width_mm * 10):
            raise ValueError("Open and close widths must differ at RTDE's 0.1 mm resolution")
        if self.feedback_timeout_s <= 1 / self.poll_hz:
            raise ValueError("feedback_timeout_s must exceed the polling period")
        inputs = (self.reg_cmd_seq, self.reg_target_width, self.reg_target_force)
        outputs = (self.reg_ack_seq, self.reg_state, self.reg_current_width,
                   self.reg_is_gripped, self.reg_error_code)
        if len(set(inputs)) != len(inputs) or len(set(outputs)) != len(outputs):
            raise ValueError("RTDE input and output register addresses must be unique")
        return self


class _ModbusTCP:
    """Single-owner FC3/FC6/FC16 client with strict MBAP and payload validation."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.sock = None
        self.transaction = 0

    def connect(self):
        self.sock = socket.create_connection((self.cfg.host, self.cfg.port), self.cfg.socket_timeout_s)

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def _recv(self, size, deadline):
        data = bytearray()
        while len(data) < size:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("RG2 Modbus response timed out")
            self.sock.settimeout(remaining)
            chunk = self.sock.recv(size - len(data))
            if not chunk:
                raise ConnectionError("RG2 Modbus connection closed")
            data.extend(chunk)
        return bytes(data)

    def _request(self, function, payload):
        if self.sock is None:
            raise RuntimeError("RG2 Modbus not connected")
        self.transaction = (self.transaction + 1) & 0xFFFF
        pdu = bytes([function]) + payload
        deadline = time.monotonic() + self.cfg.socket_timeout_s
        self.sock.settimeout(self.cfg.socket_timeout_s)
        self.sock.sendall(struct.pack(">HHHB", self.transaction, 0, len(pdu) + 1, self.cfg.unit_id) + pdu)
        transaction, protocol, length, unit = struct.unpack(">HHHB", self._recv(7, deadline))
        if (transaction, protocol, unit) != (self.transaction, 0, self.cfg.unit_id) or not 2 <= length <= 254:
            raise RuntimeError("Invalid RG2 Modbus MBAP header")
        response = self._recv(length - 1, deadline)
        if response[0] == (function | 0x80):
            if len(response) != 2:
                raise RuntimeError("Malformed RG2 Modbus exception")
            raise RuntimeError(f"RG2 Modbus exception {response[1]} for function {function}")
        if response[0] != function:
            raise RuntimeError("Unexpected RG2 Modbus function")
        return response[1:]

    def read(self, address, count):
        response = self._request(3, struct.pack(">HH", address, count))
        if len(response) != 1 + 2 * count or response[0] != 2 * count:
            raise RuntimeError("Invalid RG2 Modbus register count")
        return struct.unpack(f">{count}H", response[1:])

    def write(self, address, values):
        payload = struct.pack(">HHB", address, len(values), 2 * len(values))
        payload += struct.pack(f">{len(values)}H", *values)
        if self._request(16, payload) != struct.pack(">HH", address, len(values)):
            raise RuntimeError("Invalid RG2 Modbus write acknowledgement")

    def stop(self):
        payload = struct.pack(">HH", 2, 8)
        if self._request(6, payload) != payload:
            raise RuntimeError("Invalid RG2 Modbus stop acknowledgement")


class RG2Gripper:
    """Feedback plugin plus optional asynchronous set_grasp()/acknowledged stop().

    read_qpos() returns signed *equivalent* coordinates [w/2,-w/2] in meters,
    derived from measured raw width, not separately encoded finger joint angles.
    connect() is read-only. Commands are queued, not claimed as completed moves.
    A busy device finishes its current move before accepting the latest target.
    """

    def __init__(self, **kwargs):
        self.cfg = RG2Config.model_validate(kwargs)
        self._client = _ModbusTCP(self.cfg)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._quit = threading.Event()
        self._thread = None
        self._error = None
        self._width_mm = 0.0
        self._sample_time = 0.0
        self._pending = None
        self._stop_request = None
        self._commanded = False
        self._busy_since = None
        self._desired = None
        self._sample_local = threading.local()

    def connect(self):
        if self._thread is not None:
            raise RuntimeError("RG2 already connected; create a new driver after disconnect")
        try:
            self._client.connect()
            self._poll()  # fail before starting UR control if feedback is unavailable
            self._thread = threading.Thread(target=self._run, name="rg2-modbus", daemon=True)
            self._thread.start()
        except BaseException:
            self._client.close()
            raise

    def _poll(self):
        width_raw, status = self._client.read(267, 2)
        width_mm = width_raw / 10.0
        if not 0 <= width_mm <= 110:
            raise RuntimeError(f"Invalid RG2 measured raw width: {width_mm} mm")
        # Both pushed switches and latched safety errors abort capture.
        if status & 0x7C:
            raise RuntimeError(f"RG2 safety status: 0x{status:04x}")
        with self._lock:
            self._width_mm = width_mm
            self._sample_time = time.monotonic()
        return status

    def _check_locked(self):
        if self._error is not None:
            raise RuntimeError(f"RG2 communication/control failed: {self._error}") from self._error
        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("RG2 not connected")
        if time.monotonic() - self._sample_time > self.cfg.feedback_timeout_s:
            raise RuntimeError("RG2 measured feedback is stale")

    def read_qpos(self):
        with self._lock:
            self._check_locked()
            half_width_m = self._width_mm / 2000.0
            self._sample_local.host_time = self._sample_time
        return [half_width_m, -half_width_m]

    @property
    def sample_host_time(self):
        # Same snapshot as this caller's read_qpos(), even if the worker polls again.
        return self._sample_local.host_time

    def set_grasp(self, grasp):
        if grasp not in (0, 1):
            raise ValueError("RG2 grasp must be 0 (open) or 1 (closed)")
        width = self.cfg.open_width_mm if grasp == 0 else self.cfg.close_width_mm
        with self._lock:
            self._check_locked()
            if self._desired == grasp:
                return
            self._desired = int(grasp)
            self._pending = (width, time.monotonic())
            self._commanded = True
        self._wake.set()

    def stop(self):
        with self._lock:
            self._check_locked()
            self._pending = None
            self._desired = None
            event = self._stop_request = threading.Event()
        self._wake.set()
        if not event.wait(3 * self.cfg.socket_timeout_s + 2 / self.cfg.poll_hz):
            raise TimeoutError("RG2 stop was not acknowledged")
        with self._lock:
            self._check_locked()

    def _run(self):
        try:
            while not self._quit.is_set():
                status = self._poll()
                now = time.monotonic()
                if status & 1 and self._commanded:
                    if self._busy_since is None:
                        self._busy_since = now
                    elif now - self._busy_since > self.cfg.motion_timeout_s:
                        raise TimeoutError("RG2 movement timed out")
                else:
                    self._busy_since = None
                with self._lock:
                    stop = self._stop_request
                    self._stop_request = None
                    pending = self._pending
                    if stop is not None or (pending is not None and not status & 1):
                        self._pending = None
                if stop is not None:
                    self._client.stop()
                    self._busy_since = None
                    stop.set()  # only signal after Modbus acknowledgement
                elif pending is not None:
                    if now - pending[1] > self.cfg.motion_timeout_s:
                        raise TimeoutError("RG2 remained busy; target not accepted")
                    if not status & 1:
                        self._client.write(0, [round(self.cfg.force_n * 10), round(pending[0] * 10), 1])
                self._wake.wait(1 / self.cfg.poll_hz)
                self._wake.clear()
        except Exception as exc:
            # Never reconnect/replay a movement automatically after a protocol error.
            with self._lock:
                self._error = exc
                self._pending = None
            if self._commanded:
                with contextlib.suppress(Exception):
                    self._client.stop()
        finally:
            with self._lock:
                request = self._stop_request
                self._stop_request = None
            if request is not None:
                request.set()
            self._client.close()

    def disconnect(self):
        try:
            if self._commanded and self._thread is not None and self._thread.is_alive():
                self.stop()  # stop movement; never automatically release a held part
        finally:
            self._quit.set()
            self._wake.set()
            if self._thread is not None:
                self._thread.join(4 * self.cfg.socket_timeout_s + 1)
                if self._thread.is_alive():
                    raise RuntimeError("RG2 communication worker failed to exit")
            self._client.close()


class _RTDERegisterClient:
    """Small adapter around Universal Robots' official Python RTDE client."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.client = None
        self.input_data = None
        self.input_names = [
            f"input_int_register_{cfg.reg_target_width}",
            f"input_int_register_{cfg.reg_target_force}",
            f"input_int_register_{cfg.reg_cmd_seq}",
        ]
        self.output_names = [
            f"output_int_register_{cfg.reg_ack_seq}",
            f"output_int_register_{cfg.reg_state}",
            f"output_int_register_{cfg.reg_current_width}",
            f"output_int_register_{cfg.reg_is_gripped}",
            f"output_int_register_{cfg.reg_error_code}",
        ]

    def connect(self):
        if self.client is not None:
            raise RuntimeError("RG2 RTDE already connected")
        self.input_data = None
        try:
            from ._rtde import rtde
        except ImportError as exc:
            raise RuntimeError(
                "RTDE client unavailable; install project package with bundled RTDE support"
            ) from exc
        errors = []
        for version in (rtde.Protocol.VERSION_2, rtde.Protocol.VERSION_1):
            con = None
            try:
                con = rtde.RTDE(self.cfg.robot_ip, self.cfg.port)
                con.connect(protocol_version=version)
                self.input_data = con.send_input_setup(self.input_names, ["INT32"] * 3)
                if self.input_data is None:
                    raise RuntimeError(f"RTDE input register setup failed: {self.input_names}")
                if not con.send_output_setup(
                    self.output_names, ["INT32"] * 5, frequency=self.cfg.poll_hz
                ):
                    raise RuntimeError(f"RTDE output register setup failed: {self.output_names}")
                if not con.send_start():
                    raise RuntimeError("RTDE synchronization failed to start")
                self.client = con
                return
            except Exception as exc:
                errors.append(exc)
                with contextlib.suppress(Exception):
                    if con is not None:
                        con.disconnect()
                self.input_data = None
        message = "; ".join(str(e) for e in errors) or "unknown RTDE connection error"
        raise RuntimeError(message)

    def close(self):
        client = self.client
        self.client = None
        self.input_data = None
        if client is not None:
            with contextlib.suppress(Exception):
                client.send_pause()
            with contextlib.suppress(Exception):
                client.disconnect()

    def send(self, width_mm, force_n, sequence):
        if self.client is None:
            raise RuntimeError("RG2 RTDE not connected")
        setattr(self.input_data, self.input_names[0], int(round(width_mm * 10)))
        setattr(self.input_data, self.input_names[1], int(round(force_n)))
        setattr(self.input_data, self.input_names[2], int(sequence))
        if self.client.send(self.input_data) is False:
            raise RuntimeError("RG2 RTDE command send failed")

    def read(self):
        if self.client is None:
            raise RuntimeError("RG2 RTDE not connected")
        # `receive_buffered()` returns oldest packet first.  Drain queued RTDE
        # samples so a delayed recorder never consumes stale register state.
        data = self.client.receive_buffered()
        if data is None:
            return None
        while True:
            newer = self.client.receive_buffered()
            if newer is None:
                break
            data = newer
        return tuple(int(getattr(data, name)) for name in self.output_names)


class RTDEGripper:
    """RG2 register driver using the ROS workspace's URScript worker.

    The worker reports the last accepted target width.  Legacy RG2 tool I/O
    wiring exposes no encoder through this register protocol, so this is a
    controller-reported command state, not an independent measured width.
    """

    def __init__(self, **kwargs):
        self.cfg = RTDEConfig.model_validate(kwargs)
        self._client = _RTDERegisterClient(self.cfg)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._quit = threading.Event()
        self._thread = None
        self._error = None
        self._reported_width_mm = 0.0
        self._sample_time = 0.0
        self._desired = None
        self._pending = None
        self._sequence = 0
        self._last_ack = 0
        self._controller_error = 0
        self._startup_error = False
        self._inflight = None
        self._commanded = False
        self._sample_local = threading.local()

    def connect(self):
        if self._thread is not None:
            raise RuntimeError("RG2 already connected")
        self._quit.clear()
        self._wake.clear()
        with self._lock:
            self._error = None
            self._desired = None
            self._pending = None
            self._inflight = None
            self._commanded = False
            self._controller_error = 0
            self._startup_error = False
            self._sample_time = 0.0
        try:
            self._client.connect()
            deadline = time.monotonic() + self.cfg.feedback_timeout_s * 4
            while time.monotonic() < deadline:
                if self._poll(allow_startup_error=True) is not None:
                    break
                time.sleep(0.01)
            else:
                raise TimeoutError("RG2 RTDE returned no status")
            self._thread = threading.Thread(target=self._run, name="rg2-rtde", daemon=True)
            self._thread.start()
        except BaseException:
            self._client.close()
            self._thread = None
            raise

    @staticmethod
    def _is_startup_error(ack, state, width_x10, error):
        # Older pendant workers start with last_seq=-1, so default registers
        # (seq=0, force=0) produce this one harmless error before first command.
        return ack == 0 and state == 3 and width_x10 == 0 and error == 2

    def _poll(self, allow_startup_error=False):
        status = self._client.read()
        if status is None:
            return None
        ack, state, width_x10, _gripped, error = status
        width_mm = width_x10 / 10.0
        if not 0 <= width_mm <= self.cfg.max_width_mm:
            raise RuntimeError(f"Invalid RG2 reported width: {width_mm} mm")
        startup_error = bool(error and self._is_startup_error(ack, state, width_x10, error))
        if error and not (allow_startup_error and startup_error):
            raise RuntimeError(f"RG2 controller error code: {error}")
        with self._lock:
            self._last_ack = ack
            self._controller_error = error
            self._startup_error = startup_error
            self._reported_width_mm = width_mm
            self._sample_time = time.monotonic()
        return state

    def _check_locked(self):
        if self._error is not None:
            raise RuntimeError(f"RG2 RTDE communication failed: {self._error}") from self._error
        if self._thread is None or not self._thread.is_alive():
            raise RuntimeError("RG2 RTDE not connected")
        if time.monotonic() - self._sample_time > self.cfg.feedback_timeout_s:
            raise RuntimeError("RG2 reported feedback is stale")

    def read_qpos(self):
        with self._lock:
            self._check_locked()
            half_width_m = self._reported_width_mm / 2000.0
            self._sample_local.host_time = self._sample_time
        return [half_width_m, -half_width_m]

    @property
    def sample_host_time(self):
        return self._sample_local.host_time

    def set_grasp(self, grasp):
        if grasp not in (0, 1):
            raise ValueError("RG2 grasp must be 0 (open) or 1 (closed)")
        with self._lock:
            self._check_locked()
            if self._desired == grasp:
                return
            self._desired = int(grasp)
            self._pending = (self.cfg.open_width_mm if grasp == 0 else self.cfg.close_width_mm,
                             time.monotonic())
            self._commanded = True
        self._wake.set()

    def _run(self):
        try:
            while not self._quit.is_set():
                state = self._poll(allow_startup_error=self._startup_error)
                with self._lock:
                    pending = self._pending
                    inflight = self._inflight
                    ack = self._last_ack
                    controller_error = self._controller_error
                if inflight is not None:
                    sequence, sent_at = inflight
                    if ack == sequence:
                        if state == 3 or controller_error:
                            raise RuntimeError(
                                f"RG2 controller rejected command (error={controller_error})"
                            )
                        with self._lock:
                            self._inflight = None
                            self._startup_error = False
                    elif time.monotonic() - sent_at > self.cfg.motion_timeout_s:
                        raise TimeoutError("RG2 command was not acknowledged")
                if pending is not None and inflight is None:
                    width, queued = pending
                    if time.monotonic() - queued > self.cfg.motion_timeout_s:
                        raise TimeoutError("RG2 target was not acknowledged")
                    self._sequence = (self._sequence % 2_000_000_000) + 1
                    self._client.send(width, self.cfg.force_n, self._sequence)
                    sent_at = time.monotonic()
                    with self._lock:
                        # Do not erase a newer target queued while send() ran.
                        if self._pending is pending:
                            self._pending = None
                        self._inflight = (self._sequence, sent_at)
                self._wake.wait(1 / self.cfg.poll_hz)
                self._wake.clear()
        except Exception as exc:
            with self._lock:
                self._error = exc
                self._pending = None
                self._inflight = None
        finally:
            self._client.close()

    def stop(self):
        with self._lock:
            self._check_locked()
            self._pending = None
            # RTDE worker has no stop register.  Clear desired state so the
            # next enabled command resubmits even if target is unchanged.
            self._desired = None

    def disconnect(self):
        self._quit.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(2)
            if self._thread.is_alive():
                raise RuntimeError("RG2 RTDE worker failed to exit")
        self._client.close()
        self._thread = None

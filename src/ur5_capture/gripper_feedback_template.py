"""可复用的夹爪反馈驱动模板（gripper feedback template）。

本模块实现了 ``src/ur5_capture/gripper.py`` 中 ``GripperFeedback`` 适配器所要求的
驱动契约（connect / read_qpos / disconnect）。采集主循环和适配器都**不用改**，你只需：

1. 直接把本文件里的类用起来（通用 Modbus 驱动只需改 YAML），
   或把本文件复制成 ``src/ur5_capture/my_gripper.py`` 后按你的协议改写 read_qpos()。
2. 在 ``configs/ur5_three_rgb.yaml`` 里填（以通用 Modbus 驱动为例）：

   gripper:
     feedback_source: plugin
     feedback_plugin: "ur5_capture.gripper_feedback_template:ModbusRTUGripperFeedback"
     feedback_kwargs:
       port: /dev/ttyUSB0        # 或 COM3，换成你的夹爪串口
       slave_id: 9               # Robotiq 2F 默认 9
       register: 2004            # 0x07D4 = Position（按你的型号手册核对）
       full_open_m: 0.085        # 2F-85 满开 85 mm；2F-140 填 0.14
       raw_max: 255              # 满开对应的原始值

契约（必须遵守，来自 docs/GRIPPER_FEEDBACK.md）：
- ``read_qpos()`` 返回**恰好 2 个有限实数**，单位米，形如 ``[left, right]``，
  是夹爪左右指的**绝对实测位置**；只测到总开度时按 ``[w/2, -w/2]`` 映射并注明。
- 禁止用开合命令冒充测量值。
- ``connect()`` 连接失败要抛异常，``disconnect()`` 要释放串口等资源。
"""
from __future__ import annotations

import argparse
import time
from typing import Any


class GripperFeedbackBase:
    """驱动契约基类：继承它并实现 connect / read_qpos / disconnect。"""

    def __init__(self, **kwargs: Any) -> None:
        self._kwargs = kwargs

    def connect(self) -> None:
        raise NotImplementedError("connect() must open the hardware and zero/calibrate if needed")

    def read_qpos(self) -> list[float]:
        # 必须返回 [left_m, right_m]，两个有限实数，单位米，是实测位置
        raise NotImplementedError("read_qpos() must return measured [left, right] in meters")

    def disconnect(self) -> None:
        raise NotImplementedError("disconnect() must release the port/hardware")


def _crc16_modbus(data: bytes) -> int:
    """Modbus RTU 的 CRC16（多项式 0xA001，反射）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


class ModbusRTUGripperFeedback(GripperFeedbackBase):
    """通用 Modbus RTU 位置读取驱动（自包含，仅依赖 pyserial）。

    适用：Robotiq 2F、大寰 DH 等通过 Modbus RTU 上报位置寄存器的工业夹爪。
    通过 feedback_kwargs 配置寄存器与标定，无需改代码。

    参数（feedback_kwargs）：
        port: 串口设备，如 /dev/ttyUSB0 或 COM3（必填）
        slave_id: Modbus 从站地址，默认 9（Robotiq 2F 默认）
        baudrate: 波特率，默认 115200
        function: 3=读保持寄存器 / 4=读输入寄存器，默认 4
        register: 起始寄存器地址，默认 2004（0x07D4 Robotiq Position）
        register_count: 寄存器个数，默认 1（16 位；2 表示 32 位大端）
        signed: 原始值是否为有符号数，默认 False
        raw_min / raw_max: 原始值范围，默认 0 / 255
        full_open_m: raw_max 对应的总开度（米），默认 0.085（2F-85）
        reverse: 若 raw_max 表示闭合则为 True，默认 False
        timeout_s: 串口读写超时，默认 0.5
    """

    def __init__(self, port: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.port = port
        self.slave_id: int = kwargs.get("slave_id", 9)
        self.baudrate: int = kwargs.get("baudrate", 115200)
        self.function: int = kwargs.get("function", 4)
        self.register: int = kwargs.get("register", 2004)
        self.register_count: int = kwargs.get("register_count", 1)
        self.signed: bool = kwargs.get("signed", False)
        self.raw_min: float = kwargs.get("raw_min", 0)
        self.raw_max: float = kwargs.get("raw_max", 255)
        self.full_open_m: float = kwargs.get("full_open_m", 0.085)
        self.reverse: bool = kwargs.get("reverse", False)
        self.timeout_s: float = kwargs.get("timeout_s", 0.5)
        self._ser = None

    def connect(self) -> None:
        import serial  # 懒加载：未装 pyserial 时给出清晰报错

        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout_s,
        )
        # 开机自检：读一次寄存器，失败说明地址/接线/从站号有误
        self.read_qpos()

    def _read_register_raw(self) -> int:
        assert self._ser is not None
        req = bytes(
            [
                self.slave_id,
                self.function,
                (self.register >> 8) & 0xFF,
                self.register & 0xFF,
                (self.register_count >> 8) & 0xFF,
                self.register_count & 0xFF,
            ]
        )
        req += _crc16_modbus(req).to_bytes(2, "little")
        self._ser.reset_input_buffer()
        self._ser.write(req)

        # 响应头：从站号 + 功能码（异常时功能码最高位置 1）
        head = self._ser.read(2)
        if len(head) < 2:
            raise TimeoutError(f"No Modbus response from slave {self.slave_id} on {self.port}")
        if head[1] & 0x80:
            exc = self._ser.read(1)
            code = exc[0] if len(exc) == 1 else -1
            raise RuntimeError(f"Modbus exception from slave {self.slave_id}: code {code}")
        if head[1] != self.function:
            raise RuntimeError(f"Unexpected function code {head[1]} (expected {self.function})")

        byte_count = self._ser.read(1)
        if len(byte_count) != 1:
            raise TimeoutError("Truncated Modbus response (no byte count)")
        payload = self._ser.read(byte_count[0] + 2)  # data + CRC16
        if len(payload) != byte_count[0] + 2:
            raise TimeoutError("Truncated Modbus response (incomplete payload)")

        data, crc_recv = payload[:-2], payload[-2:]
        if _crc16_modbus(head + byte_count + payload[:-2]) != int.from_bytes(crc_recv, "little"):
            raise RuntimeError("Modbus CRC mismatch")
        if len(data) < self.register_count * 2:
            raise RuntimeError("Modbus response shorter than requested register count")
        return int.from_bytes(data[: self.register_count * 2], "big")

    def read_qpos(self) -> list[float]:
        raw = self._read_register_raw()
        bits = self.register_count * 16
        if self.signed and raw >= (1 << (bits - 1)):
            raw -= 1 << bits
        span = self.raw_max - self.raw_min
        if span <= 0:
            raise ValueError("raw_max must be greater than raw_min")
        opening = self.full_open_m * (raw - self.raw_min) / span
        if self.reverse:
            opening = self.full_open_m - opening
        opening = max(0.0, min(self.full_open_m, opening))
        # 总开度 w -> 对称映射为左右两指位置（本项目约定 [left,right]=[+w/2,-w/2]）
        return [opening / 2.0, -opening / 2.0]

    def disconnect(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None


class SerialGripperFeedback(GripperFeedbackBase):
    """裸串口骨架：给自定义协议 / 舵机类夹爪填空用。

    在 read_qpos() 里按你的协议下发查询、解析响应、换算成 [left, right]（米）。
    """

    def __init__(self, port: str, baudrate: int = 115200, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.port = port
        self.baudrate = baudrate
        self.timeout_s: float = kwargs.get("timeout_s", 0.5)
        self._ser = None

    def connect(self) -> None:
        import serial

        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        # TODO: 如需校零/回零，在这里完成；失败请 raise

    def read_qpos(self) -> list[float]:
        assert self._ser is not None
        # TODO: 按你的协议查询并解析，示例（注释掉，仅为示意）：
        #   self._ser.write(b"\x01\x03\x00\x00")          # 下发查询帧
        #   resp = self._ser.read(6)                       # 读响应
        #   left_m = ...; right_m = ...
        #   return [left_m, right_m]
        raise NotImplementedError("按你的夹爪协议实现 read_qpos()，返回 [left, right]（米）")

    def disconnect(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None


def _main() -> None:
    p = argparse.ArgumentParser(description="测试夹爪反馈驱动（独立于采集主循环）")
    p.add_argument("--port", required=True, help="串口设备，如 /dev/ttyUSB0")
    p.add_argument("--slave-id", type=int, default=9)
    p.add_argument("--register", type=int, default=2004)
    p.add_argument("--full-open-m", type=float, default=0.085)
    p.add_argument("--raw-max", type=int, default=255)
    p.add_argument("--samples", type=int, default=10)
    args = p.parse_args()

    driver = ModbusRTUGripperFeedback(
        port=args.port,
        slave_id=args.slave_id,
        register=args.register,
        full_open_m=args.full_open_m,
        raw_max=args.raw_max,
    )
    driver.connect()
    print(f"connected on {args.port}; reading {args.samples} samples...")
    try:
        for i in range(args.samples):
            qpos = driver.read_qpos()
            print(f"[{i:02d}] left={qpos[0]:+.5f} m  right={qpos[1]:+.5f} m")
            time.sleep(0.1)
    finally:
        driver.disconnect()


if __name__ == "__main__":
    _main()

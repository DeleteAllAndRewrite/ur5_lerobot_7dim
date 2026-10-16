# OnRobot RG2（110 mm）接入

本驱动适用于 **RG2 + OnRobot Compute Box，经以太网 Modbus TCP 连接电脑**。
UR5 继续由原有 RTDE `servoL` 控制。夹爪驱动不向 UR 控制器发送新的主 URScript 程序，
避免替换正在运行的 RTDE 控制程序。

**连接边界：** 旧款 RG2 仅接 UR 法兰工具数字 I/O、不带 Compute Box 的硬件不能使用本驱动。
RG2-FT、RG6 也不是本次实现的型号。不要把 UR 控制器的 502 端口当作 Compute Box。
Compute Box 应由本驱动单独控制；不要同时通过 URCap、WebLogic 或其他客户端发送夹爪指令。

## 配置

两个实机 YAML 已选用 `ur5_capture.rg2:RG2Gripper`。按实际地址修改：

```yaml
gripper:
  initial_grasp: 0
  toggle_button: null
  feedback_source: plugin
  feedback_plugin: "ur5_capture.rg2:RG2Gripper"
  actuation_enabled: false
  feedback_kwargs:
    host: "192.168.1.1"  # Compute Box IP，独立于 robot.ip
    port: 502
    unit_id: 65
    max_width_mm: 110.0
    open_width_mm: 110.0
    close_width_mm: 0.0
    force_n: 20.0
    poll_hz: 50.0
    socket_timeout_s: 0.2
    feedback_timeout_s: 0.25
    motion_timeout_s: 5.0
```

单夹爪 unit ID 为 65；Dual Quick Changer 主侧为 66，副侧为 67，按实际安装填写。
默认 `actuation_enabled: false` 只测量，不开合；确认安装与参数后改为 `true`。
连接时不发送 `initial_grasp`。录制开始后首次按住 deadman 会提交当前目标，默认张开至 110 mm。
按 `g` 或独立夹爪按键切换目标，目标改变时发送一次；松开 deadman 后停止运动，
再次按住会重新提交当前目标。夹爪仍在运动时保留最新目标，待空闲再执行。

开度关系要求 `0 <= close_width_mm < open_width_mm <= max_width_mm <= 110`。
开度分辨率 0.1 mm；夹持力限制为 3–40 N。
内置驱动使用 Python 标准库 socket，无需额外安装 Modbus 库。

## 测量与寄存器

本次使用**不含指尖偏移的原始开度**作为统一命令和测量口径，单位为铝制手指内侧间距。
若安装定制指尖，实际接触面的间距会不同，应重新确定开闭目标和工具 TCP。
驱动不改写设备指尖偏移、不动态改变 TCP、不自动修改负载。

| 地址（十进制） | 功能 | 本驱动使用方式 |
|---|---|---|
| 0 | 目标夹持力 | 0.1 N；20 N 写 200 |
| 1 | 目标原始开度 | 0.1 mm；110 mm 写 1100 |
| 2 | 控制字 | 1 开始运动（无指尖偏移）；8 停止 |
| 267 | 实测原始开度 | FC3 读取，除以 10 得 mm |
| 268 | 状态 | bit0 忙碌；bit1 夹持检测；bit2–6 安全开关/安全错误 |

力、开度、启动字通过 FC16 一次写入；停止使用 FC6。
每次读取 267–268 两个连续 holding registers，校验 MBAP、事务号、设备地址、功能码和长度。
遇 Modbus 异常、坏帧、断连、安全开关或超时会报告错误，不自动重连或重放动作。

状态后两维始终由测量派生：`left = width_mm / 2000`，`right = -left`，单位米。
例如全开为 `[0.055,-0.055]`，这不是两个独立关节编码器，也不是 RG2 连杆角度。
动作保持 7D、状态保持 8D，LeRobot v2.1/v3 转换接口无需修改。

后台线程串行完成 Modbus 读写，主循环读取有界缓存。
`complementary_info.gripper_host_time` 保存该测量真正完成的时刻，过期阈值由
`feedback_timeout_s` 控制。`set_grasp()` 接受目标进入后台队列，不代表已机械到位；
`action[6]` 始终是请求目标，需结合 deadman 和测量理解执行情况。

松开 deadman、结束 episode 和退出时会取消排队目标、发送停止，不自动张开释放物体。
通信中断时发送停止只能尽力而为，不能保证设备收到；需在实物上验证停止与保持行为。

## 只读检查

在项目根目录执行以下代码，仅连接夹爪、读取测量并断开，不启动 UR RTDE 或相机：

```python
from ur5_capture.config import load_config
from ur5_capture.gripper import GripperFeedback

cfg = load_config("configs/ur5_three_rgb.yaml")
cfg.gripper.actuation_enabled = False
reader = GripperFeedback(cfg.gripper)
try:
    reader.connect()
    print(reader.read())
finally:
    reader.disconnect()
```

## 本次验证

基于仓库提交 `a23f286` 修改。运行：

```bash
python -m pytest tests/test_core.py tests/test_gripper.py tests/test_rg2.py tests/test_terminal_keys.py tests/test_rg2_integration.py -q
python -m ruff check src/ur5_capture tests
```

结果：64 项通过，Ruff 通过。使用 socket 对端模拟设备，验证分片收包、寄存器读写、
110 mm 换算、实测反馈与命令分离、忙碌目标保留、deadman、停止、超时、安全状态、
Windows 按键，以及主循环先观测后动作和编码前停止。

`test_rg2_integration.py` 使用明确的上游基类替身和内存 writer，检查本项目的桥接/采集逻辑。
当前环境缺少 torch、pyarrow、av，未运行官方 LeRobot 写入/读取集成测试及 v2.1/v3 转换。
未连接真实 RG2、UR5、SpaceMouse 或 RealSense；实机通信、夹持力、TCP 和时序需设备联调。

## 协议来源

- [OnRobot RG2 产品参数](https://onrobot.com/en/products/rg2-finger-gripper)：110 mm 行程、3–40 N。
- [OnRobot TECHMAN/OMRON TM User Manual v1.1.2](https://onrobot.com/sites/default/files/documents/User_Manual_for_TECHMAN_OMRON_TM_v1.1.2_EN_0.pdf)：Modbus TCP 的 RG2/6 寄存器及状态定义。
- [OnRobot DOOSAN User Manual v1.05](https://onrobot.com/sites/default/files/robot-comp-download/User_Manual_for_DOOSAN_v1.05_EN.pdf)：Compute Box Modbus 地址及设备 ID。

不同固件应以随设备手册为准；部分官方旧 PDF 链接可能重定向至官网首页。

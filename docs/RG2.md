# OnRobot RG2（110 mm）接入

本驱动适用于 **RG2 通过 UR5 控制器 RTDE 寄存器控制**。
当前实机配置直接由 `ur_rtde` 控制 UR5。示教器必须运行与 `ur_rtde` 匹配的 External Control URCap，
其服务端口必须与 `robot.ur_cap_port` 一致；不要同时启动 ROS `ur_robot_driver`。

RG2 命令和状态经过 UR5 控制器的 RTDE `30004` 端口传输。UR 控制器中的 RG2 程序负责实际工具通信。
不要把 UR 控制器当作 Compute Box 使用，也不要配置 Modbus `502` 端口。
运行录制前，需把 `rg2_worker_template.urscript` 的并行线程放入示教器 External Control 主程序；否则 RTDE recipe
仍可连接，但输出寄存器保持 0，命令不会得到 ack。
录制时由 `RTDEGripper` 独占 RTDE 寄存器 24--28。不要同时启用其他 RTDE register client，否则两个 client 会争抢同一组 input registers。

## 配置

实机 YAML 使用 `ur5_capture.rg2:RTDEGripper`：

```yaml
gripper:
  initial_grasp: 0
  toggle_button: null
  feedback_source: plugin
  feedback_plugin: "ur5_capture.rg2:RTDEGripper"
  actuation_enabled: false
  feedback_kwargs:
    robot_ip: "192.168.100.100"  # UR5 控制器 IP，应与 robot.ip 一致
    port: 30004
    max_width_mm: 110.0
    open_width_mm: 110.0
    close_width_mm: 0.0
    force_n: 20.0
    poll_hz: 50.0
    socket_timeout_s: 0.2
    feedback_timeout_s: 0.25
    motion_timeout_s: 5.0
```

默认 `actuation_enabled: false` 只读 worker 状态，不开合；确认安装与参数后改为 `true`。
连接时不发送 `initial_grasp`。录制开始后首次提交夹爪目标时才执行，默认张开至 110 mm。
按终端 `g` 或 SpaceMouse 按键 0 切换目标，目标改变时发送一次。按键 0 不需要持续按住；
夹爪仍在运动时保留最新目标，待空闲再执行。

开度关系要求 `0 <= close_width_mm < open_width_mm <= max_width_mm <= 110`。
开度分辨率 0.1 mm；本项目 worker 接受夹持力 4–40 N（RG2 手册下限 3 N 不能通过该 worker 校验）。
内置驱动携带 UR RTDE Python 协议实现，无需 ROS 环境；端口为 UR5 RTDE `30004`。

## 状态与 RTDE 寄存器

本次使用**不含指尖偏移的原始开度**作为统一命令和状态口径，单位为铝制手指内侧间距。
若安装定制指尖，实际接触面的间距会不同，应重新确定开闭目标和工具 TCP。
驱动不改写设备指尖偏移、不动态改变 TCP、不自动修改负载。

| 地址（十进制） | 功能 | 本驱动使用方式 |
|---|---|---|
| input 24 | command sequence | 每次目标改变递增 |
| input 25 | target width × 10 | 110 mm 写 1100 |
| input 26 | target force | 单位 N |
| output 24 | acknowledged sequence | 确认命令 |
| output 25 | RG2 state | 控制器状态 |
| output 26 | current width × 10 | worker 回报的目标开度 |
| output 27 | gripped | 夹持检测 |
| output 28 | error code | 非零即报错 |

状态后两维由 worker 回报开度派生：`left = width_mm / 2000`，`right = -left`，单位米。
例如全开为 `[0.055,-0.055]`，这不是两个独立关节编码器，也不是 RG2 连杆角度。
动作保持 7D、状态保持 8D，LeRobot v2.1/v3 转换接口无需修改。

后台线程串行完成 RTDE 寄存器读写，主循环读取有界缓存。
`complementary_info.gripper_host_time` 保存该状态报文到达主机的时刻，过期阈值由
`feedback_timeout_s` 控制。`set_grasp()` 接受目标进入后台队列，不代表已机械到位；
`action[6]` 始终是请求目标，需结合 `actuation_enabled`、ack 和 worker 状态理解执行情况。

结束 episode 和退出时会取消尚未发送的目标，不自动张开释放物体。
RTDE 寄存器协议没有独立停止寄存器；已发送命令由示教器 RG2 worker 完成，需在实物上验证停止与保持行为。

## 只读检查

在项目根目录执行以下代码，仅连接夹爪、读取 worker 状态并断开，不启动 UR RTDE 或相机：

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
110 mm 换算、状态与命令分离、忙碌目标保留、停止、超时、安全状态、
Windows 按键，以及主循环先观测后动作和编码前停止。

`test_rg2_integration.py` 使用明确的上游基类替身和内存 writer，检查本项目的桥接/采集逻辑。
当前环境缺少 torch、pyarrow、av，未运行官方 LeRobot 写入/读取集成测试及 v2.1/v3 转换。
未连接真实 RG2、UR5、SpaceMouse 或 RealSense；实机通信、夹持力、TCP 和时序需设备联调。

## 协议来源

- [OnRobot RG2 产品参数](https://onrobot.com/en/products/rg2-finger-gripper)：110 mm 行程、3–40 N。
- `/home/sumi/ros/catkin_ws/src/universal_robot/ur_control/scripts/rg2_worker_template.urscript`：示教器并行线程与寄存器定义。
- `/home/sumi/ros/catkin_ws/src/universal_robot/ur_control/scripts/rg2_rtde_register_client.py`：ROS 工作区 RTDE recipe 对照实现。

不同固件应以随设备手册为准；部分官方旧 PDF 链接可能重定向至官网首页。

# 实机夹爪反馈：8D Observation 的最后两维

当前 `observation.state` 固定为 `[x,y,z,rx,ry,rz,left_finger_qpos_m,right_finger_qpos_m]`。`x,y,z,rx,ry,rz` 由 UR RTDE 的 `getActualTCPPose()` 提供，基坐标系绝对位姿、旋转向量；`qpos` 由独立夹爪硬件反馈提供，**每个采样时刻的绝对位置**，单位必须是米。

**动作仍是 7D** `[dx,dy,dz,drx,dry,drz,grasp]`；`grasp` 为 0（目标张开）或 1（目标闭合）的命令标签，不会伪造测量，不自动使实体夹爪运动。

## 实机驱动适配协议

在项目的 `src/` 目录创建自己的 Python 文件，例如 `src/my_gripper.py`，实现类似接口：

```python
class MyGripperFeedback:
    def __init__(self, host, **kwargs):
        self.host = host

    def connect(self):
        # TODO: 连接你的实体夹爪硬件，必要时校零；连接失败应抛出异常
        raise NotImplementedError("Implement connection for your gripper")

    def read_qpos(self):
        # TODO: 读取实体夹爪的反馈数据并标定为 [left_m, right_m]
        # 必须返回两个有限实数，单位米；禁止直接使用开合命令作为反馈
        raise NotImplementedError("Implement real measured qpos")

    def disconnect(self):
        # TODO: 释放串口、Modbus 或其他设备资源
        pass
```

将实机配置设置为：

```yaml
gripper:
  initial_grasp: 0
  toggle_button: null
  feedback_source: plugin
  feedback_plugin: "my_gripper:MyGripperFeedback"
  feedback_kwargs:
    host: "192.168.1.20"
```

将 `my_gripper.py` 放在 `src/` 目录，项目通过 `pip install -e .` 后能导入，或将你的硬件驱动单独安装到当前 Conda 环境。只有连接和反馈真实可用时才启用机器人运动。

如果硬件只报告**真实测得的总开口宽度** `w`，在确认对称传动和符号约定后可按 `[w/2,-w/2]` 映射成两个等效关节坐标；必须在文档中标明这是“实测宽度派生值”，不代表独立测量了两个关节。如有左右独立编码器，优先使用两个独立测量值。不同夹爪模型的符号和零点各不相同，不应不经标定复制 Panda 的方向。

运行时校验 `read_qpos()` 恰好返回 2 个有限值，失败则本次采集中止。`complementary_info.gripper_host_time` 记录反馈获取完成时间。反馈与机器人 TCP 状态的采样时间可不同；本项目不实现夹爪设备时钟同步。反馈插件应快速返回最新测量，必要时应独立缓存硬件数据，并监测其新鲜度。

**特别注意：**目前 `update_gripper_command()` 不负责发送实体夹爪的开合指令。真实抓取动作需要针对具体型号额外实现、测试有安全保护的夹爪**执行**驱动；仅加入反馈插件可以采集被外部设备控制的夹爪位置，但不会让 `g` 键驱动机械爪。

## Mock 状态

`feedback_source: mock` 仅用于 `mode: mock`。默认全开宽度 0.08 m，模拟 `qpos=[0.04,-0.04]`，闭合 `qpos=[0,0]`。主循环先采当前反馈，再记录动作；动作会影响**下一帧**的模拟位置，不会倒填本帧。该数据不可标为实机反馈。

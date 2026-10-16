# 夹爪测量与执行接口

`observation.state` 为 `[x,y,z,rx,ry,rz,left_finger_qpos_m,right_finger_qpos_m]`。
前六维是 UR RTDE 的绝对 TCP 位姿，姿态使用旋转向量 rad。
最后两维来自独立夹爪测量，单位米，不能从动作标签推断。

`action` 保持 7D `[dx,dy,dz,drx,dry,drz,grasp]`，`grasp=0` 为目标张开，
`grasp=1` 为目标闭合。它是请求目标，不代表已经到位或检测到物体。
物理执行还要求 `actuation_enabled: true` 和当前 deadman 为真。

## 内置 RG2 驱动

配置 `feedback_plugin: "ur5_capture.rg2:RG2Gripper"`，并提供 Compute Box 的连接参数。
最大原始开度 110 mm；开闭目标和 3–40 N 夹持力均可配置。
完整连接要求、寄存器和参数见 [RG2.md](RG2.md)。

RG2 根据**实测原始总开度** `w` 生成等效坐标 `[w/2,-w/2]`。
110 mm 对应 `[0.055,-0.055]` m；夹持到 30 mm 物体时对应 `[0.015,-0.015]` m，
即使当前命令要求闭到 0 mm，也不会把测量伪造成 `[0,0]`。
这两个坐标不是独立编码器值或 RG2 连杆的实际转角。

## 自定义插件协议

只读插件实现以下接口，原有插件无需添加执行方法：

```python
class MyGripperFeedback:
    def __init__(self, host):
        self.host = host

    def connect(self):
        raise NotImplementedError("Connect hardware and verify measured feedback")

    def read_qpos(self):
        raise NotImplementedError("Return two measured or measurement-derived coordinates in meters")

    def disconnect(self):
        pass
```

```yaml
gripper:
  initial_grasp: 0
  toggle_button: null
  feedback_source: plugin
  feedback_plugin: "my_gripper:MyGripperFeedback"
  actuation_enabled: false
  feedback_kwargs:
    host: "192.168.1.20"
```

需要执行时，插件必须额外实现 `set_grasp(grasp)` 和 `stop()`。
设置 `actuation_enabled: true` 后，连接时会检查这两个方法是否存在。
`connect()` 不应触发开合；首次使能、目标改变或停止后重新使能才提交目标。
`set_grasp()` 应快速返回，耗时通信和等待运动完成应放在后台。
插件应保留最新目标，在夹爪忙碌时延后发送，不能默默丢弃。
`stop()` 应取消未执行目标并停止运动，不自动张开夹爪。

主循环与 LeRobot `send_action()` 均显式传递 deadman；松开后调用 `stop()`。
直接调用适配器 `update_command()` 时默认 `enabled=False`，不会隐式执行。
episode 结束、异常和退出时也清理执行驱动。
只读模式不调用执行方法；mock 模式仍只模拟反馈。

## 测量语义和时间

`read_qpos()` 必须返回恰好两个有限值。若只有总开度测量，可在确认对称与符号后派生
`[w/2,-w/2]`，并在文档中标明来源。不能声称独立测量了两根手指。

后台缓存插件可提供 `sample_host_time` 属性，返回最近一次 `read_qpos()` 对应的硬件读取
完成时刻（主机 monotonic 秒）。适配器将它写入 `complementary_info.gripper_host_time`。
未提供此属性的旧插件保留同步读取完成时间语义。
后台插件必须检测测量过期与通信错误；RG2 已内置检测。

读取当前反馈后才提交动作，因此反馈反映本帧动作之前的状态。
软件采样未实现夹爪、UR 和相机的硬件时钟同步。

## Mock

`feedback_source: mock` 仅用于 `mode: mock`。
默认全开 0.08 m，对应 `[0.04,-0.04]`；闭合对应 `[0,0]`。
若希望演示 RG2 的 110 mm 全开，在 mock YAML 中设置 `mock_open_width_m: 0.11`。
mock 模拟瞬时响应，动作影响下一帧，不代表真实测量或机械响应时间。

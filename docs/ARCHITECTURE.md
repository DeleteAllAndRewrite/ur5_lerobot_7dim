# 设计说明

## 边界与默认配置

本次需求固定为单 UR5、单 SpaceMouse、三路 RGB（2 global + 1 eye-in-hand）。所有设备编号、名称、分辨率、控制频率、轴方向、保存路径和时序阈值集中在 YAML。当前 schema 有意约束相机角色与数量，不宣称已经支持双臂或任意数量相机。新增拓扑需要扩展 schema 与特征定义。

根目录 `lerobot/` 是已核对 API 的官方源码快照。该快照不含 Git 历史、媒体、上游测试及训练示例，包含 src、pyproject、setup、README 和许可证。若你已有自己的 `lerobot/` checkout，请保留原目录，在独立环境比较 commit 和接口后迁移本扩展包，不要不加比较地覆盖自己的源码。

## LeRobot 架构映射

| LeRobot 抽象 | 本项目实现 | 说明 |
|---|---|---|
| RobotConfig | UR5RobotConfig | 官方 dataclass 注册；引用主 YAML |
| Robot | UR5Robot | 继承官方基类，提供观测/动作/连接接口及时间戳扩展 |
| TeleoperatorConfig | SpaceMouseTeleoperatorConfig | 官方 dataclass 注册 |
| Teleoperator | SpaceMouseTeleoperator | 继承官方基类，提供增量动作及 deadman |
| Camera | CameraRig / CameraStream | 使用 RealSense SDK 的独立时间戳采集封装，不冒充官方 Camera 子类 |
| Dataset | 官方 LeRobotDataset | create / add_frame / save_episode / finalize |
| Record script | ur5-capture | 管理时序、命令确认和 episode 生命周期 |

相机没有直接复用官方 RealSenseCamera 的异步图像接口，原因是本需求要保留 SDK 帧号、设备时钟域、内参和有界历史帧选择。三个独立 pipeline 在三个线程中读帧，格式明确为 `rs.format.rgb8`。

## 位姿与动作

令 `p_t`、`R_t` 为记录时实测 TCP 位置与姿态。SpaceMouse 的输入经过 deadzone、轴排列、符号映射和向量模长限制，生成一个记录周期的平移及旋转命令。

base 模式：

\[
p_t^*=p_t+\Delta p_t,\qquad R_t^*=\exp([\Delta r_t]_\times)R_t.
\]

tool 模式：

\[
p_t^*=p_t+R_t\Delta p_t^{tool},\qquad R_t^*=R_t\exp([\Delta r_t^{tool}]_\times).
\]

位置裁剪到 config 中的工作空间后，最终保存动作统一为：

\[
a_t=\left[p_t^*-p_t,\ \operatorname{Log}(R_t^*R_t^\top)^\vee,\ g_t\right],\quad g_t\in\{0,1\}.
\]

其中 $g_t$ 表示锁存的目标 `grasp`（0 张开、1 闭合），非实测夹爪状态，也不会在未接入驱动时执行实体夹爪动作。`action` 为 float32[7]；`observation.state` 为 float32[8]，包括绝对 TCP 位姿(6D)与夹爪左右手指绝对关节位置(2D)。夹爪位置来源于单独的硬件反馈（或明确标注的 mock 模拟），不能使用 grasp 命令伪造。运动使能 `enabled` 仍作为与动作分离的安全控制量传给控制进程并作为 `complementary_info.deadman` 保存。

因此不会对两个轴角向量做直接差分，也不会将相机坐标、工具坐标与 base 坐标混用。±π 附近的旋转差分包含专门测试。

UR5/CB3 默认控制进程 125 Hz，数据保存 30 Hz；二者不要求整除。每个记录周期向控制进程提交目标端点和持续时间，控制进程从接受时的实测 TCP 到目标端点进行平移/旋转插值。目标不会无限积分到实测 TCP 之外；跟踪误差过大时中止。

## 控制与时序

采集主循环的顺序：

1. 从三个相机历史缓冲区中选择尚未使用的、主机时间相近的 RGB 帧。
2. 获取 RTDE 进程最近状态并检查新鲜度。
3. 读取独立夹爪反馈（左右指绝对关节位置），然后读取并排空 SpaceMouse HID 报文队列，检查 stop / deadman / 事件新鲜度；可选第三按键的按下沿或终端 `g` 切换目标 `grasp`。
4. 基于该观测计算目标，提交给 RTDE 进程，等待接收确认。
5. 把 8D 绝对观测、7D 动作（6D 目标增量 + `grasp` 指令状态）、目标和诊断时间戳加入官方写入器。

每帧的记录时间是名义网格时间。实际相机到达、状态采样、命令接收与数据组装时间分别记录，不把后采样的状态冒充动作前观测。相机最晚数据与机器人状态之间仍存在采样偏差，本实现不对位姿做时间插值。

主进程通过 Pipe 提交控制命令，控制进程通过共享数组提供状态。Pipe 每次命令必须确认，没有无限排队或默默丢弃控制命令。主进程写 PNG、编码、保存文件时，控制进程独立运行；主进程长时间无新命令则触发命令超时。机器人端另启用 RTDE watchdog，处理 Python 控制进程停止更新的情形。

非硬实时操作系统、网络抖动、SDK 内部阻塞仍可能影响控制。对于更高速度或严格确定性需求，应进一步改为实时调度或专用控制进程，并完成真实控制器的 watchdog 与 stop 验证。

## Episode 生命周期与故障

- 每条 episode 录制前等待用户 Enter，不自动移动到初始位置。
- 录制期间可提前停止、丢弃或退出。停止运动后才编码或请求下一步输入。
- 任何断帧、严重延迟、旧状态、设备错误、控制拒绝使当前 episode 中止并丢弃。
- 启动不覆盖既有数据目录，也不隐式恢复未完成采集。
- 正常退出始终调用官方 `finalize()`，关闭 Parquet writer。
- Ctrl-C 会走清理；直接杀进程、断电或磁盘故障可能留下未完成文件，不能保证事务级恢复。已有数据应另行备份；没有实现断电续录。
- 实机不自动复位保护停机、恢复机器人运动或再次连接后继续旧动作。

## 版本转换

v2.1 通常按 episode 组织数据与视频，v3 将多 episode 聚合到文件并通过元数据指向片段。两个版本无法通过同一包版本同时导入，故用两个 Python 解释器分别运行同一个独立 worker。

桥接内容为逐帧、无 pickle 的压缩 NPZ 与 manifest。读取数值时访问官方 Dataset 内部的 Arrow-backed 原始表，避免默认 torch 转换造成 float64 精度损失；图像通过公开 `__getitem__` 解码，让官方读取器解析 v3 视频片段偏移。目标统计量、任务表、episode 范围与分块均由目标版本官方写入器重建。

逐帧核验通过后才发布目标目录。其代价是桥接磁盘空间大、按帧解码较慢以及视频重新编码。项目优先保证当前采集格式的语义正确，不将这个工具描述为大规模高速无损迁移器。

固定 v0.4.4 的 `add_frame` 校验器不接受输入字典中的保留字段 `timestamp`。适配层先调用官方 `add_frame`，再在官方 `episode_buffer` 中设置该帧的源时间戳，最后由 `save_episode` 序列化；不修改官方源码。升级源码后需要重新验证这处版本相关接口。

## 后续扩展位置

| 需求 | 扩展点 |
|---|---|
| 实体夹爪 | 已支持 RG2 + Compute Box 的 Modbus TCP 测量/执行驱动，110 mm 上限；按 deadman 与执行配置门控。RG2 等效两指位置由实测总开度派生，详见 `RG2.md`。其他型号使用兼容插件 |
| RGB-D | 深度需要独立无损存储、单位和标定，不能写成普通有损 RGB MP4 |
| 双臂 | 扩展 config、每臂控制进程和统一时钟；修改状态/动作维度及相机所属关系 |
| 四元数状态 | 在记录边界转换，明确 xyzw/wxyz；动作旋转仍用 SO(3) 增量 |
| 只保存 xyz 增量 | 单独改 action schema 为 3 维，并明确是否锁定姿态；不要悄悄丢弃旋转命令 |
| 接入官方 lerobot-record | 增加插件发现、相机观测特征和 processor 映射，再验证原生 CLI 的时序行为 |
| 预览界面 | 增加独立 GUI/视频流，不阻塞 RTDE 进程；当前版本为终端采集器 |

# 验证情况：采集、转换与 Rerun 回放

## 2026-10-19：LeRobot + Rerun 整合

- 将 `/home/sumi/lerobot-rerun` 的提交 `326484d77d2ac90379e84222dfad21fe4acf1f71`
  整合到项目内 `lerobot/` 子模块，软件版本 0.6.2，数据格式仍为 v3.0。
- 更新 v3 环境的 Hub、draccus、构建依赖与 Rerun SDK 0.33.1；v2.1 环境保留原 Hub/draccus 版本。
- 依赖联合解析（`pip install --dry-run -r requirements/v3.txt -e . -e ./lerobot`）和
  当前环境 `pip check` 通过。未另建全新 Conda 环境。
- `python -m pytest tests -q`：**81 passed**，包括官方 Robot/Teleoperator 适配和模拟 v3 写入/读取。
- 新命令 `ur5-replay --help`，以及路径不存在、空数据集、episode 越界、冲突参数的拒绝行为检查通过。
- 浏览器模式使用本地数据集启动，HTTP 访问返回 200；检查后已停止服务。
- `ur5-replay --root datasets/ur5_three_rgb_0003 --episode-index 0 --save` 正常完成：
  1414 帧，导出 `replays/ur5_three_rgb_0003/local_ur5_three_rgb_episode_0.rrd`。
  `rerun rrd verify` 通过；统计确认三路图像、action、state、frame_index 和 timestamp 时间轴齐全，
  时间轴有序，文件大小 342739164 字节。
- 当前实测软件版本见 [verified_environment.json](verified_environment.json)。本次未连接实物设备，
  未重新验证 v2.1 双向转换，也未操作 Rerun 桌面 GUI。

以下为历史验证记录；其旧依赖版本及当时缺少依赖的描述不代表上面的当前环境。

## 2026-10-15：8D state + 7D action + Conda 手动安装版本

更新时间：2026-10-15。本版本已修改 `observation.state` 为 8D，保留 `action` 为 7D，新增夹爪状态反馈适配接口，删除 `tools/setup_env.sh` 并在 README 中提供手动 Conda 命令。

## 本轮实际已完成的检查

- `python -m pytest tests/test_core.py tests/test_gripper.py -q`：**25 passed**，覆盖帧特征形状、绝对 EEF/夹爪状态、动作 7D、真实插件反例、无实测反馈拒绝实机配置、mock 更新、时间戳等。
- 全部修改过的 Python 文件做了源代码编译检查（详见本轮最终执行结果）。
- 原有 `test_dataset_integration.py` 与 `test_lerobot_bridge.py` 需要在 README 创建的 Conda v3 环境（安装 draccus、av、官方 LeRobot 源码及其依赖）中运行。当前执行环境缺少 `draccus`，**尚未**完成本轮官方 v3 writer/reader、v2.1↔v3.0 的完整回归测试。
- 本轮**未验证**两组 Conda 环境全新安装、物理夹爪反馈驱动、UR5/RealSense/SpaceMouse 实机采集或夹爪实体执行。

**历史日志不能视为当前 8D schema 的运行验证。** `docs/reports/` 保留用户原包中上一次测试的历史报告，仅供参考；`docs/verified_environment.json` 已在 2026-10-19 更新。

---

## 历史 6D/7D 版本参考记录（不代表本轮验证）


验证日期：2026-10-15。对象：本代码包与 UPSTREAM.json 固定的官方源码。

## 已执行

| 检查 | 结果 |
|---|---|
| pytest | 18 passed；包括公式、配置、同步、超时停止、官方 Robot/Teleoperator 接口与 v3 读写 |
| Ruff | 项目 src、tests、tools 检查通过 |
| Bash 语法 | setup_env.sh 通过 bash -n |
| 三路虚拟 RGB 采集 | 2 episodes，60 frames，30 FPS，三个视角各 64×48 |
| v3 → v2.1 | 两个解释器分别加载固定版本官方源码；逐帧验证通过 |
| v2.1 → v3 | 完整反向转换；逐帧验证通过 |
| 数值、任务与时间戳 | 两次转换均逐帧精确比较通过；包括 float64 原始时间 |
| episode 边界 | 每个 episode 30 帧；视频片段跨 episode 读取通过 |
| 全量视频解码与动作关系 | 回转 v3 全部 60 帧通过 inspect_dataset.py |
| 硬件 Python API | 导入 ur_rtde 1.6.5 和 RealSense 2.56.5.9235；核对调用签名，无设备连接 |

关键测试：±π 跨界旋转差、tool 坐标旋转复合、低速/高速向量速度限制、按键上升沿、空间裁剪、重复序列号、无效轴映射、NaN、错误相机角色、CB3 频率上限、相机重复/旧帧/skew、跟踪误差和控制命令超时。

## 视频重新编码误差

下表为各次转换目标解码 RGB 与该次输入解码 RGB 之间的平均绝对差，量纲 0–255。它反映本次合成图像，不代表真实场景的误差上限。

| 方向 | agentview_1 | agentview_2 | eye_in_hand |
|---|---:|---:|---:|
| v3 → v2.1 | 1.4979 | 1.4637 | 1.4648 |
| v2.1 → v3 | 1.8698 | 1.8568 | 1.8500 |

## 验证中发现并修复

- v0.4.4 官方 add_frame 的校验器拒绝显式 timestamp 输入：适配层使用 episode_buffer 在序列化前保留源时间戳，重新完成双向回转验证。
- PySpaceMouse 2.0 设备枚举为 get_connected_devices，使用对象 read/close 接口；没有沿用旧版的 list_devices。
- 数据检查工具先确认本地元数据存在并启用离线模式，避免路径写错时尝试 Hub 查询。

## 第三方告警

测试中出现 datasets 对单元素 NumPy 数组标量转换的弃用告警，以及 TorchVision 视频接口弃用告警，没有导致本次测试失败。TorchVision 固定 0.22.1；不要直接升级到移除该接口的版本。原始测试输出在 reports/pytest.txt。

## 未验证及限制

没有物理 UR5、三台 RealSense 或 SpaceMouse 连接。因此网络、USB 带宽、真实帧到达抖动、坐标轴方向、TCP、控制器版本、保护停机、watchdog 实际行为、碰撞风险和长期持续采集性能都仍需实机验收。软件对齐不是曝光级同步。没有测试负载/夹爪，也没有完整机械臂碰撞模型。

只验证当前采集 schema；未声称所有外部 LeRobot 数据集的自定义元数据都可无损转换。视频重新编码，临时桥接数据占用可能很大。安装脚本语法与各安装步骤已验证，未从一台全新物理工作站完整执行系统驱动安装。

精确软件环境见 verified_environment.json。转换环境使用 LeRobot v0.3.3、datasets 3.6.0；v3 环境使用 LeRobot v0.4.4、datasets 4.8.5。

## 历史 7D grasp 更新补充说明（2026-10-15）

新增 LeRobot `action` schema `[dx_m,dy_m,dz_m,drx_rad,dry_rad,drz_rad,grasp]`（float32[7]）。第七维是指令状态 0/1，不是夹爪硬件反馈。模拟输入定期切换 grasp；真实双键 SpaceMouse 可使用终端 `g`，额外可用按键可配置 `gripper.toggle_button`。已同步修改数据检查工具只用 `action[:6]` 对照 TCP 目标，并单独验证 `action[6]` 为 0 或 1。

**注意：**前述历史报告的 18 passed 和双向转换结果对应原始 6D 版本，不构成本次 7D 版本的重新验证结果。本次 7D 修改在现有环境执行 20 项核心 pytest 测试全部通过，另以轻量 LeRobot 接口替身调用实际 UR5 模拟后端，完成 12 次动作和 grasp 切换校验。由于当前环境缺少 draccus 和 av，官方 Robot/Teleoperator 继承测试及真实 v3 官方 writer/reader 双向转换尚未重跑；请在项目安装环境中复测。无实体夹爪驱动，尚无法验证真实抓取动作。

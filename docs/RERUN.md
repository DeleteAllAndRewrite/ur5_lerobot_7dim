# 视频、action 和 state 同步回放

## 多条 episode 回放

在项目目录激活 `ur5-lerobot-v3` 环境后，可直接查看合并数据集：

```bash
# 同一个 Rerun 窗口加载所有 episode，每条作为独立 recording，可在左侧切换
ur5-replay --root dataset/ur5_three_rgb_merged --repo-id local/ur5_three_rgb_merged \
  --episodes all --episode-mode separate

# 将所有 episode 串成同一条连续时间轴
ur5-replay --root dataset/ur5_three_rgb_merged --repo-id local/ur5_three_rgb_merged \
  --episodes all --episode-mode continuous

# 也可以只选几条，支持逗号分隔及闭区间；按所写顺序播放，重复编号自动去重
ur5-replay --root dataset/ur5_three_rgb_merged --repo-id local/ur5_three_rgb_merged \
  --episodes 0,2,5-8 --episode-mode continuous
```

`separate` 是默认模式，每条 recording 的时间从 0 开始。
`continuous` 将后一条的起点设为前一条终点加一个帧周期；全局 `frame_index` 连续递增，
`Episode` 曲线及 `episode_index` 时间轴标识原始 episode 编号。
这是用于浏览的拼接时间，不包含两次采集之间的等待时间；源数据时间戳保持不变。
旧的 `--episode-index 0` 命令仍保持原行为，且不能与 `--episodes` 同时使用。

两种模式都支持 `--save --output-dir replays`。独立模式每条写一个 `.rrd`，
连续模式写一个 `*_continuous_<首编号>_<末编号>.rrd`。相同命名的导出会覆盖原文件，
需要保留不同选择结果时应指定不同 `--output-dir`。
可用一个 Rerun 窗口打开多个已导出的 recording：

```bash
rerun replays/ur5_three_rgb_merged/*.rrd
```

首次加载需要逐帧解码，`all` 会处理全部选中轨迹。
建议先用 `--episodes 0-2` 查看。`--server-memory-limit 4GiB` 控制新模式的服务历史缓存；
超出缓存时，后连接的查看器可能看不到较早的数据。完整留存优先用 `--save` 导出。

新模式也兼容 `--mode distant`。连续模式沿用指定的一个 gRPC/Web 端口；
独立模式为第 n 条 recording 使用 `--grpc-port + n`、`--web-port + n`（n 从 0 开始），
终端会打印各条连接地址。浏览器分别打开对应 Web 端口；桌面 Rerun 可同时打开多个
打印出的 `rerun+http://.../proxy` 地址，在同一窗口切换。服务按 `Ctrl+C` 退出。

本项目通过 `ur5-replay` 调用官方 LeRobot 的 Rerun 可视化接口，在同一时间轴显示三路 RGB、
7 维 action 和 8 维 state。支持播放、暂停和拖动时间轴，回放期间无需连接机械臂、相机或 SpaceMouse。

## 源码与环境

`/home/sumi/lerobot-rerun` 的官方 LeRobot 源码已整合到本项目的 `lerobot/` 子模块中，
固定提交为 `326484d77d2ac90379e84222dfad21fe4acf1f71`（软件版本 0.6.2，数据格式 v3.0）。
安装后使用项目内源码，无需保留外部目录。来源见 [UPSTREAM.json](../UPSTREAM.json)。

已有 `ur5-lerobot-v3` 环境时，在项目根目录执行：

```bash
cd /home/sumi/ur5_lerobot_7dim
conda activate ur5-lerobot-v3
python -m pip install -r requirements/v3.txt
python -m pip install -e ./lerobot --no-deps
python -m pip install -e '.[replay]'
python -m pip check
```

新克隆本项目时先执行 `git submodule update --init --recursive`，然后按
[README 的环境安装步骤](../README.md#21-v3-采集环境)创建 Python 3.12 环境。
`requirements/v3.txt` 已包含固定的 Rerun SDK 0.33.1；v2.1 转换环境继续独立安装。

检查安装来源和命令：

```bash
python -c 'import lerobot, rerun; print(lerobot.__file__); print(rerun.__version__)'
ur5-replay --help
```

`lerobot.__file__` 应位于本项目的 `lerobot/src/lerobot/`。

## 回放本地数据

```bash
ur5-replay \
  --root datasets/ur5_three_rgb_0003 \
  --episode-index 0
```

此命令默认使用 `repo_id=local/ur5_three_rgb`，通过 PyAV 解码视频，并打开 Rerun 桌面查看器。
替换 `--root` 可查看其他数据集；同一数据集有多个 episode 时修改 `--episode-index`，从 0 开始。
其他 repo_id 可用 `--repo-id` 指定。模块形式同样可用：

```bash
python -m ur5_capture.replay --root datasets/ur5_three_rgb_0003 --episode-index 0
```

只选择已经保存过 episode 的数据集。若 `meta/info.json` 中的 `total_episodes` 或
`total_frames` 为 0，命令会提示数据集为空。首次导入 episode 时需要解码和记录数据，长 episode 需要等待。

查看器包含三路相机画面、action 曲线和 state 曲线。用底部时间轴播放、暂停或拖动，
通过图例选择要观察的维度。界面以数据集名义时间戳同步显示，不代表三台相机进行了硬件曝光同步。

## 导出回放文件

无桌面环境或需要重复查看时，先导出 `.rrd`，此过程不会启动图形窗口：

```bash
ur5-replay \
  --root datasets/ur5_three_rgb_0003 \
  --episode-index 0 \
  --save \
  --output-dir replays

rerun replays/ur5_three_rgb_0003/local_ur5_three_rgb_episode_0.rrd
```

输出结构为 `<output-dir>/<数据集目录名>/<repo_id 中的斜线替换为下划线>_episode_<编号>.rrd`。
再次导出相同数据集和 episode 会写入同一路径。`replays/` 已加入 Git 忽略列表。

## 浏览器或远程查看

```bash
ur5-replay \
  --root datasets/ur5_three_rgb_0003 \
  --episode-index 0 \
  --mode distant \
  --web-port 9090 \
  --grpc-port 9876
```

本机浏览器访问 `http://localhost:9090`。远程桌面查看器可执行：

```bash
rerun rerun+http://<采集电脑IP>:9876/proxy
```

服务保持运行，按 `Ctrl+C` 退出；`--save` 与 `--mode distant` 不能同时使用。

## 数据含义

| 显示内容 | 维度 | 含义 |
|---|---|---|
| `observation.images.*` | 三路 RGB | 两个全局视角与一个腕部视角 |
| `action` | 7 | `[dx,dy,dz,drx,dry,drz,grasp]`，下发的 TCP 增量与夹爪目标 |
| `state` | 8 | `[x,y,z,rx,ry,rz,left_finger_qpos_m,right_finger_qpos_m]`，绝对 TCP 位姿与夹爪状态 |

平移单位米，旋转单位 rad，旋转向量不是欧拉角。action 中的 TCP 增量不等于真实相邻帧运动，
`grasp` 也不代表抓取成功。当前 RG2 RTDE 的手指位置由控制器回报的目标开度派生，详见
[RG2 接入说明](RG2.md)。

`ur5-replay` 只读取本地数据并进行可视化，内部启用 Hugging Face 离线模式。
上游 `lerobot-replay` 用于向机器人执行数据集动作，与这里的视频/曲线回放用途不同。

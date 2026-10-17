# 启动SpaceMouse遥操作采集数据程序

1. 终端启动程序
```
conda activate ur5-lerobot-v3
ur5-capture record --config configs/ur5_three_rgb.yaml
```

2. UR5示教器上启动外部控制程序

运行程序 -> 左上角“文件”按钮 -> 加载程序 -> “external_contral_rg2.urp” -> 打开 -> 底部的三角形按钮（点击之后会变成暂停按键）

3. 终端中按回车开始录制

每个 episode 默认录制 `300` 秒（[ur5_three_rgb.yaml](../configs/ur5_three_rgb.yaml) 中的
`dataset.episode_seconds`）。时间到后会自动停止机器人并进入保存/丢弃确认，不需要按 `s`。


# 遥操作方式

1. 终端中按回车开始
    - ‘g’可以控制夹爪开合
    - ‘s’ 代表结束
    - ‘d’代表丢弃这条数据
    - ‘q’代表丢弃数据并结束程序
2. SpaceMouse 遥操作
    - 移动键帽可以遥操作机械臂
    - 左键同样控制夹爪开合
    - 右键切换高速档、低速挡

如果需要更长 episode，修改 `dataset.episode_seconds`，例如改为 `120.0`。




# 合并采集数据集

在项目目录运行（使用已安装本地 LeRobot v3 的环境）：

```bash
/home/sumi/anaconda3/envs/ur5-lerobot-v3/bin/python tools/merge_datasets.py
```

默认扫描 `dataset/ur5_three_rgb_0000` 这样的四位编号目录，按编号合并到
`dataset/ur5_three_rgb_merged`，repo ID 为 `local/ur5_three_rgb_merged`。
空数据集自动跳过；有错误记录的采集批次中，已经保存的 episode 仍然纳入。
原始目录保持不变，输出目录已存在时拒绝覆盖。

```bash
# 仅查看输入和预计总数
/home/sumi/anaconda3/envs/ur5-lerobot-v3/bin/python tools/merge_datasets.py --dry-run

# 下次新增采集后，重新生成到新目录
/home/sumi/anaconda3/envs/ur5-lerobot-v3/bin/python tools/merge_datasets.py \
  --output dataset/ur5_three_rgb_merged_v2 --repo-id local/ur5_three_rgb_merged_v2
```

使用官方 `aggregate_datasets` 重排 episode、全局 frame 和 task 索引，更新三路视频引用，
并合并统计信息。视频直接复制，不重新编码。只纳入保存后的数据，不包含已丢弃的片段。
`meta/merge_manifest.json` 记录来源目录、各来源在合并结果中的 episode 编号区间
（左闭右开）、原始数量及是否有采集错误记录；原始相机配置仍可在来源目录查询。

查看合并结果：

```bash
ur5-replay --root dataset/ur5_three_rgb_merged --repo-id local/ur5_three_rgb_merged --episode-index 0
```

## 生成 V2.1 版本

对于实际采集的大数据集，转换入口提供直接拆分选项，避免把全部 RGB 导出为 NPZ：

```bash
/home/sumi/anaconda3/envs/ur5-lerobot-v3/bin/python -m ur5_capture.convert \
  --source dataset/ur5_three_rgb_merged \
  --destination dataset/ur5_three_rgb_merged_v21 \
  --python-v21 /home/sumi/anaconda3/envs/ur5-lerobot-v21/bin/python \
  --python-v3 /home/sumi/anaconda3/envs/ur5-lerobot-v3/bin/python \
  --repo-id local/ur5_three_rgb_merged --direct-v3-to-v21
```

直接模式适用于本项目 V3 RGB 视频数据：数值 parquet 按 episode 原值拆分，
视频按 V3 元数据中的时间偏移准确切割，使用 H.264 CRF 0 无损编码保存解码后的像素。
V2.1 元数据由隔离环境中的官方 LeRobot 0.3.3 写入，保留来源的 episode 统计量。
转换检查所有视频帧数、数值写入前后一致性，以及每条 episode 首尾三路图像的可读取性。
验证结果写入 `meta/conversion_validation.json`，全部成功后才生成正式输出目录。
输出使用每 episode 一个 parquet、每视角每 episode 一个 MP4 的 V2.1 布局。
未指定此选项时仍使用原来的 NPZ 桥接转换流程。

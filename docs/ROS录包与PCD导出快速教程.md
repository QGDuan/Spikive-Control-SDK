# ROS1 录包与 PCD 导出快速教程

## 准备

雷达和飞控通电后，按现场流程启动驱动、SLAM/LIO 和 SDK。通过 SSH 在机载机开两个终端，两边都执行 `source ~/driver_ws/devel/setup.bash`，使用同一个 ROS master。下面以 Drone ID `2` 为例，其他机型配置请替换为实际配准点云话题。

## 接口速查

| Action | `/goal` 消息类型 | `cmd=1` | `cmd=2` |
| --- | --- | --- | --- |
| `/nav/map/record` | `spikive_control_sdk/RecordActionGoal` | 开始录包 | 停止并保存 bag |
| `/nav/map/post_process` | `spikive_control_sdk/PostProcessActionGoal` | 从 bag 导出 PCD | 停止当前导出 |

在对应的 `/result` 中按 `status.goal_id.id` 找到本次请求。`result.success: true` 和 `result.error_code: 0` 表示成功。`timeout` 是单次指令的期限，单位为秒，`0` 表示不限时。

## 1. 确认有真实点云

```bash
CLOUD_TOPIC=/drone_2_cloud_registered
rostopic type "$CLOUD_TOPIC"
rostopic hz "$CLOUD_TOPIC"
```

类型应为 `sensor_msgs/PointCloud2`，并持续显示发布频率。按 Ctrl+C 后，用 `rostopic echo -n 1 /nav/map/record/output_directory` 查看机载保存目录。

## 2. 录包并检查 bag

监控终端先运行 `rostopic echo /nav/map/record/result`。指令终端依次执行以下两条 Goal，保留同一个终端中的 `BAG_NAME` 和 `CLOUD_TOPIC`。

```bash
BAG_NAME="record-test-$(date +%s)"
rostopic pub -1 /nav/map/record/goal spikive_control_sdk/RecordActionGoal \
  "{goal_id: {id: '$BAG_NAME-start'}, goal: {cmd: 1, bag_name: '$BAG_NAME', timeout: 10, topics: ['$CLOUD_TOPIC']}}"
```

等 `-start` 请求成功后继续采集至少 10 秒，再发送停止请求。

```bash
rostopic pub -1 /nav/map/record/goal spikive_control_sdk/RecordActionGoal \
  "{goal_id: {id: '$BAG_NAME-stop'}, goal: {cmd: 2, timeout: 0}}"
```

等 `-stop` 请求成功，从结果中的 `Bag saved:` 复制完整路径填入 `BAG_PATH`。`rosbag info` 应列出配准点云话题，且消息数大于 0。

```bash
# 使用停止结果返回的路径。下行为默认目录示例。
BAG_PATH="/home/colman/data/$BAG_NAME.bag"
rosbag info "$BAG_PATH"
```

启用分段时，另开终端运行 `rostopic echo /nav/map/record/feedback`。录制达到至少两段后停止，在 `state=2` 的反馈中查看 `completed_parts` 和 `total_parts`。

## 3. 导出并检查 PCD

录包保存后，按现场流程停止 SLAM/LIO，保持 ROS master 和 SDK 后处理节点运行。默认检查的 `/laserMapping` 和 `/slam_to_uav_transform` 应从 `rosnode list` 消失。

监控终端切换到 `rostopic echo /nav/map/post_process/result`。指令终端发送导出请求。

```bash
BAG_DIR=$(dirname "$BAG_PATH")
rostopic pub -1 /nav/map/post_process/goal spikive_control_sdk/PostProcessActionGoal \
  "{goal_id: {id: '$BAG_NAME-pcd'}, goal: {cmd: 1, bag_path: '$BAG_DIR', bag_name: '$BAG_NAME', timeout: 0}}"
```

等 `-pcd` 请求成功，从结果复制 PCD 路径填入 `PCD_PATH`。文件中的 `POINTS` 应大于 0，并与结果中的点数一致。

```bash
# 使用导出结果返回的路径。下行为默认目录示例。
PCD_PATH="$HOME/spikive-output/$BAG_NAME.pcd"
LC_ALL=C grep -a -m 1 '^POINTS ' "$PCD_PATH"
```

## 常用配置

修改 YAML 后重启对应 SDK 节点。路径均指机载机。 [control_sdk.yaml](../config/control_sdk.yaml)

| 配置 | 默认行为 |
| --- | --- |
| `record.output_directory` | 保存到 `/home/colman/data`。录包 `bag_path` 可覆盖 |
| `record.topics` | 录包 Goal 留空 `topics` 时使用这份列表 |
| `record.split_enabled` / `record.split_size_mib` | 默认单包。启用分段后默认每段 500 MiB |
| `post_process.pointcloud_topic` | 默认使用 `/drone_{drone_id}_cloud_registered` |
| `post_process.input_directory` | 留空沿用录包目录。导出 `bag_path` 可覆盖 |
| `post_process.output_directory` | 留空使用 SDK 运行用户的 `~/spikive-output`。`map_path` 可覆盖 |

完整字段和错误码见接口说明。 [录包](README_录包.md) · [PCD 导出](README_点云后处理.md)

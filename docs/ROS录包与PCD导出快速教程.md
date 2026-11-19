# ROS1 录包与 PCD 导出快速教程

## 准备

雷达和飞控通电后，按现场流程启动驱动、SLAM/LIO 和 SDK。通过 SSH 在机载机开两个终端，两边都执行 `source ~/driver_ws/devel/setup.bash`，使用同一个 ROS master。下面以 Drone ID `2` 为例，其他机型配置请替换为实际配准点云话题。

## 接口速查

| Action | `/goal` 消息类型 | `cmd=1` | `cmd=2` |
| --- | --- | --- | --- |
| `/nav/map/record` | `spikive_control_sdk/RecordActionGoal` | 开始录包 | 停止并保存 bag |
| `/nav/map/post_process` | `spikive_control_sdk/PostProcessActionGoal` | 从 bag 导出 PCD | 停止当前导出 |

### 如何读回包

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `status.goal_id.id` | `string` | 对应请求的 `goal_id.id`，在 `/feedback` 和 `/result` 中都用它匹配本次请求 |
| `status.status` | `uint8` | ROS actionlib 状态。`1` 已接受并执行，`2` 已取消，`3` 已成功，`4` 执行中止，`5` 请求被拒绝 |
| `result.success` | `bool` | `true` 表示该条指令成功，`false` 时查看 `error_code` 和 `message` |
| `result.error_code` | `uint8` | SDK 结果码，含义见下表 |
| `result.message` | `string` | 结果详情。保存成功时给出文件路径，失败时给出原因 |

`feedback.state` 是 SDK 的进度状态，`status.status` 是 actionlib 的请求状态。两个 Action 都在 `/feedback` 报进度，在 `/result` 给最终结果。请求被拒绝时直接查看 `/result`。

`timeout` 是单条指令的期限，单位为秒，`0` 表示不限时。发送 `cmd=2` 时，录包的 `bag_path`、`bag_name`、`topics` 留空，导出的 `bag_path`、`bag_name`、`map_path`、`map_name` 留空。

### 录包反馈与结果

反馈话题为 `/nav/map/record/feedback`，类型为 `spikive_control_sdk/RecordActionFeedback`。

| `feedback.state`（`uint8`） | 含义 |
| --- | --- |
| `0` | 等待录包进程打开文件 |
| `1` | 录制已开始，或正在等待录包进程关闭文件 |
| `2` | 正在处理分段并生成最终 bag |
| `3` | 停止请求完成，最终 bag 已保存 |
| `4` | 本次请求执行失败或超时，详情见 `/result` |

合并阶段，`feedback.completed_parts` 和 `feedback.total_parts` 均为 `uint32`，分别表示已处理段数和总段数。总数为 `0` 表示尚未统计或未启用分段。段数处理完毕后，仍需等待停止请求的最终结果确认落盘。

结果话题为 `/nav/map/record/result`，类型为 `spikive_control_sdk/RecordActionResult`。`cmd=1` 成功表示文件已打开，录制继续。`cmd=2` 成功表示最终 bag 已保存，`message` 为 `Bag saved: <绝对路径>`。录包 Goal 超时或取消时，SDK 结束该请求的跟踪。若开始请求失败后录包进程仍在运行，用 `cmd=2` 停止。已发出的停止操作会继续。

### PCD 导出反馈与结果

反馈话题为 `/nav/map/post_process/feedback`，类型为 `spikive_control_sdk/PostProcessActionFeedback`。

| `feedback.state`（`uint8`） | 含义 |
| --- | --- |
| `0` | 定义为空闲，接口只对正在处理的 Goal 发布反馈 |
| `1` | 正在导出，或正在等待当前导出任务结束 |
| `3` | 本条指令完成，导出或停止结果以 `/result` 为准 |
| `4` | 本条指令失败，或原导出请求已取消，详情见 `/result` |

结果话题为 `/nav/map/post_process/result`，类型为 `spikive_control_sdk/PostProcessActionResult`。`cmd=1` 成功时，`message` 给出 `PCD saved: <绝对路径>`、处理的点云消息数 `messages`、总点数 `points`，以及有值时的 `frame`。

`cmd=2` 成功表示任务已停止，或任务在停止生效前已经完成。两种情况由停止结果的 `message` 区分。若停止生效，原 `cmd=1` 请求返回 `success=false`、`error_code=3`。若停止超时，取消请求仍然有效。

### SDK 结果码

| `error_code` | 录包 | PCD 导出 |
| --- | --- | --- |
| `0` | 开始录制或停止保存成功 | 导出或停止请求成功 |
| `2` | 开始或停止命令超时 | 导出或停止命令超时 |
| `3` | 当前 Action 请求已取消 | 原导出请求已取消 |
| `12` | ROS 时间无效，或当前没有可停止的录制 | LIO 仍在运行、无法确认 LIO 状态、bag 缺失或未关闭、没有可导出的点，或当前没有可停止的导出 |
| `19` | 命令、超时值、路径、文件名或话题参数无效 | 参数无效、输出重名、点云格式或坐标系不符、点数超限 |
| `21` | 正在录制、处理另一条指令或合并分段 | 已有导出任务或停止请求在执行 |
| `99` | 录包进程、文件保存或分段合并失败 | 无法读取 bag、写入 PCD 或处理任务发生内部错误 |

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

# ROS1 录包与 PCD 导出快速教程

## 准备

雷达和飞控通电后，按现场流程启动驱动、SLAM/LIO 和 SDK。通过 SSH 在机载机开两个终端，两边都执行 `source ~/driver_ws/devel/setup.bash`，使用同一个 ROS master。下面以 Drone ID `2` 为例，其他机型配置请替换为实际配准点云话题。

## Action 消息字段

### 录包 `/nav/map/record`

#### Req（`RecordGoal`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `cmd` | `uint32` | `1` 开始录包，`2` 停止并保存当前录包。停止时只填写 `cmd` 和所需的 `timeout`。 |
| `bag_path` | `string` | 机载 bag 保存目录，使用绝对路径。留空使用 `record.output_directory`。 |
| `bag_name` | `string` | 开始时必填。填写文件名，省略目录和 `.bag` 后缀。 |
| `timeout` | `float32` | 本次开始或停止指令的超时时间，单位为秒。`0` 表示不限时。 |
| `topics` | `string[]` | 开始时录制的话题列表，使用绝对话题名。空列表使用 `record.topics`。 |

#### Feedback（`RecordFeedback`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `state` | `uint8` | `0` 等待打开文件，`1` 已开始录制或正在关闭文件，`2` 合并分段，`3` 最终 bag 已保存，`4` 本次指令失败或超时。 |
| `completed_parts` | `uint32` | 已处理的分段数。 |
| `total_parts` | `uint32` | 本次待处理的总分段数。`0` 表示尚未统计或使用单包模式。 |

#### Result（`RecordResult`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `success` | `bool` | `true` 表示指令成功。`cmd=1` 表示文件已打开并开始录制，`cmd=2` 表示最终 bag 已保存。`false` 表示指令失败、超时或取消。 |
| `error_code` | `uint8` | `0` 成功，`2` 超时，`3` 取消，`12` ROS 时间无效或当前没有可停止的录制，`19` 参数无效，`21` 正在录制或已有指令在处理，`99` 录包进程、写盘或合并失败。 |
| `message` | `string` | 结果说明。停止成功时为 `Bag saved: <绝对路径>`，失败时说明原因。 |

### PCD 导出 `/nav/map/post_process`

#### Req（`PostProcessGoal`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `cmd` | `uint32` | `1` 从 bag 导出 PCD，`2` 停止当前导出。停止时只填写 `cmd` 和所需的 `timeout`。 |
| `map_path` | `string` | 机载 PCD 输出目录，使用绝对路径。留空使用 `post_process.output_directory`。 |
| `map_name` | `string` | PCD 文件名，省略目录和 `.pcd` 后缀。留空沿用 bag 文件名。 |
| `timeout` | `float32` | 本次导出或停止指令的超时时间，单位为秒。`0` 表示不限时。 |
| `bag_path` | `string` | 机载输入 bag 目录，使用绝对路径。留空使用 `post_process.input_directory`。 |
| `bag_name` | `string` | 输入 bag 文件名，省略目录和 `.bag` 后缀。留空使用输入目录内最新的已关闭 bag。 |

#### Feedback（`PostProcessFeedback`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `state` | `uint8` | `0` 空闲，`1` 正在导出或等待停止完成，`3` 本次指令完成，`4` 本次指令失败或原导出被取消。 |

#### Result（`PostProcessResult`）

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `success` | `bool` | `true` 表示指令成功。`cmd=1` 表示 PCD 已保存，`cmd=2` 表示导出已停止或已先完成。`false` 表示指令失败、超时或取消。 |
| `error_code` | `uint8` | `0` 成功，`2` 超时，`3` 导出取消，`12` 前置条件未满足（例如 LIO 仍在运行、bag 缺失或未关闭、无可导出点），`19` 参数或点云格式无效，`21` 已有任务在处理，`99` 读取 bag、写入 PCD 或内部执行失败。 |
| `message` | `string` | 结果说明。导出成功时包含 PCD 绝对路径、点云消息数 `messages`、点数 `points`，以及有值时的坐标系 `frame`。停止或失败时给出相应原因。 |

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

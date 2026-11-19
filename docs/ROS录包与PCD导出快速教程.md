# ROS1 录包与 PCD 导出快速教程

| 版本 | 日期 | 变更 |
| --- | --- | --- |
| V1.0 | 2026-11-19 | SDK 录包、分段合并、目录配置和 PCD 导出 |
| V1.1 | 2026-11-20 | 补充基于真实雷达点云的 ROS1 Action 验收流程 |

本文按 V1.4 接口规范的结构组织，字段和默认值以本仓库为准。 [ROS 接口规范 V1.4](specs/ROS接口规范V1.4.pdf)

## 1. 总体约定

- 机载 SDK 写入 `.bag` 和 `.pcd`。路径均指机载机，调用方需要用相同的 ROS master 或连接其 Foxglove Bridge。
- 地面站 Record 使用打开确认弹窗时的地面站本地时间命名，格式为 `项目名-YYYYMMDDHHmm.bag`。无项目时使用 `unknown`。CLI 调用可自行给出 `bag_name`。
- 每条指令发送一次。监控反馈和结果时按 `goal_id` 匹配。`success=true`、`error_code=0` 表示该条指令完成。录包开始的成功结果表示录制已开始，停止的成功结果才表示最终文件已保存。
- 使用工作区的 `devel/setup.bash`。整套启动用 `roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=2`。

## 2. 接口清单

| 接口 | ROS1 类型 | 方向 | 用途 |
| --- | --- | --- | --- |
| `/nav/map/record` | `spikive_control_sdk/RecordAction` | 双向 | 开始录包、停止并保存、分段合并进度 |
| `/nav/map/record/output_directory` | `std_msgs/String`（latched） | SDK → 调用方 | 当前默认 bag 保存目录 |
| `/nav/map/post_process` | `spikive_control_sdk/PostProcessAction` | 双向 | 将已关闭 bag 中的配准点云导出为 PCD |

Action 话题按标准 ROS1 actionlib 命名，例如 `/nav/map/record/goal`、`/feedback`、`/result`。直接发布 `/goal` 时使用相应的 `RecordActionGoal` 或 `PostProcessActionGoal`，并为每条请求生成唯一 ID。

## 3. 接口定义

### 3.1 录包与保存目录

| Goal 字段 | 类型 | 含义 |
| --- | --- | --- |
| `cmd` | `uint32` | `1` 开始，`2` 停止并保存 |
| `bag_path` | `string` | 开始时指定机载绝对目录，留空使用 `record.output_directory` |
| `bag_name` | `string` | 开始时必填，文件名去掉 `.bag` |
| `timeout` | `float32` | 本条指令的期限，单位秒，`0` 禁用期限 |
| `topics` | `string[]` | 开始时可覆盖配置中的绝对话题列表，留空使用 `record.topics` |

停止时只需 `cmd` 和所需的 `timeout`。反馈 `state` 为 `0` 启动中、`1` 录制或关闭中、`2` 合并中、`3` 完成、`4` 失败。合并进度由 `completed_parts` 和 `total_parts` 返回。保存成功后，结果 `message` 含最终 `.bag` 的绝对路径。

### 3.2 点云导出

| Goal 字段 | 类型 | 含义 |
| --- | --- | --- |
| `cmd` | `uint32` | `1` 开始导出，`2` 请求停止当前导出 |
| `map_path` | `string` | PCD 输出目录，留空使用 `post_process.output_directory` |
| `map_name` | `string` | 文件名去掉 `.pcd`，留空沿用 bag 名 |
| `timeout` | `float32` | 单次导出期限，单位秒，`0` 禁用期限 |
| `bag_path` | `string` | 输入 bag 目录，留空使用 `post_process.input_directory` |
| `bag_name` | `string` | 文件名去掉 `.bag`，留空取目录内最新的已关闭 bag |

反馈 `state` 为 `0` 空闲、`1` 处理中、`3` 完成、`4` 失败。导出前先完成录包并停止 `post_process.lio_nodes` 中列出的 SLAM/LIO 节点。SDK 按 bag 顺序拼接同坐标系、同字段布局的 `sensor_msgs/PointCloud2`，写出二进制 PCD。已有的同名输出文件会返回错误。 [点云后处理详细说明](README_点云后处理.md)

### 3.3 常见结果码

| `error_code` | 含义 |
| --- | --- |
| `0` | 成功 |
| `2` | 指令超时 |
| `3` | 指令取消 |
| `12` | 前置条件缺失，例如 LIO 仍在运行、bag 未关闭或其中没有可导出的点 |
| `19` | 参数、路径、重名或点云格式校验失败 |
| `21` | SDK 正在执行相关任务，或录包已经开始 |
| `99` | 读写、录包子进程或合并失败，详情见 `message` |

## 4. 配置

编辑 `config/control_sdk.yaml`，然后重启对应 SDK 节点。以下是与本流程有关的默认值节选。

```yaml
record:
  output_directory: /home/colman/data
  split_enabled: false
  split_size_mib: 500
post_process:
  pointcloud_topic: ""    # 留空使用 /drone_{drone_id}_cloud_registered
  input_directory: ""     # 留空沿用 record.output_directory
  output_directory: ""    # 留空使用 $HOME/spikive-output
```

`record.topics` 保存在同一 YAML。要导出 PCD，录包话题中应包含导出所用的 PointCloud2 话题。启用分段后，SDK 在停止录包后合并分段并提交最终文件。

## 5. 实机接口验收

在雷达和飞控均已通电、已连接的设备上执行。按交付环境的流程启动驱动、SLAM/LIO 和 SDK。验收在地面完成，全部命令通过 SSH 或在机载机本地终端执行。

打开三个终端，分别用于**发送指令**、**查看结果**和**查看反馈**。三个终端均执行 `source ~/driver_ws/devel/setup.bash`，并连接同一个机载 ROS master。示例使用 Drone ID 2。如果现场的 ID 或 `post_process.pointcloud_topic` 不同，将示例中的点云话题替换为实际话题。

### 5.1 确认真实点云和 Action 已就绪

在指令终端执行以下命令。

```bash
CLOUD_TOPIC=/drone_2_cloud_registered
rostopic type "$CLOUD_TOPIC"
rostopic hz "$CLOUD_TOPIC"
```

类型应为 `sensor_msgs/PointCloud2`，并连续数秒看到 `average rate`。按 Ctrl+C 退出频率监控，然后检查两个 `/goal` 都有 SDK 订阅者，记录 SDK 发布的 bag 保存目录。

```bash
rostopic info /nav/map/record/goal
rostopic info /nav/map/post_process/goal
rostopic echo -n 1 /nav/map/record/output_directory
```

在结果终端运行 `rostopic echo /nav/map/record/result`，在反馈终端运行 `rostopic echo /nav/map/record/feedback`。保持两条命令运行。

### 5.2 开始录制真实点云

继续在指令终端发送开始 Goal。

```bash
BAG_NAME="record-test-$(date +%Y%m%d-%H%M%S)"
START_ID="$BAG_NAME-start"
echo "$START_ID"

rostopic pub -1 /nav/map/record/goal spikive_control_sdk/RecordActionGoal \
  "{goal_id: {id: '$START_ID'}, goal: {cmd: 1, bag_path: '', bag_name: '$BAG_NAME', timeout: 10, topics: ['$CLOUD_TOPIC']}}"
```

此请求显式录制正在发布的配准点云，`bag_path` 留空使用 SDK 的配置目录。在结果终端找到 `status.goal_id.id` 等于 `START_ID` 的消息。确认 `result.success: true`、`result.error_code: 0` 后，保持雷达和 SLAM/LIO 运行至少 10 秒，再发送停止 Goal。

### 5.3 停止录制并检查 bag

```bash
STOP_ID="$BAG_NAME-stop"
echo "$STOP_ID"

rostopic pub -1 /nav/map/record/goal spikive_control_sdk/RecordActionGoal \
  "{goal_id: {id: '$STOP_ID'}, goal: {cmd: 2, timeout: 0}}"
```

等待 `status.goal_id.id` 等于 `STOP_ID` 的结果。保存成功时 `result.success: true`、`result.error_code: 0`，`result.message` 中的 `Bag saved:` 后面是最终文件的绝对路径。将它填入 `BAG_PATH`。

```bash
# 示例使用默认目录。若结果返回其他目录，用实际路径替换。
BAG_PATH="/home/colman/data/$BAG_NAME.bag"
rosbag info "$BAG_PATH"
```

通过条件是 `rosbag info` 能读取文件，且 `topics` 列出 `CLOUD_TOPIC`，消息数大于 0。

如需验收分段合并，启用 `record.split_enabled` 并重启录包节点。重新录制，使数据至少生成两个分段。停止后按 `STOP_ID` 查看 `state=2`、`completed_parts` 和 `total_parts`，再用停止结果和 `rosbag info` 确认合并后的最终文件可读。

### 5.4 停止 SLAM/LIO 并导出 PCD

确认录包保存完成后，按现场流程停止 SLAM/LIO，保持机载 ROS master 和 SDK 的 `spikive_post_process_node` 运行。如果现场由 Manager 启动 SLAM/LIO，使用地面站的 Manager 停止按钮。随后核对节点和订阅者。

```bash
rosnode list | grep -Ex '/laserMapping|/slam_to_uav_transform'
rostopic info /nav/map/post_process/goal
```

第一条命令预期没有输出。现场修改过 `post_process.lio_nodes` 时，按实际节点名核对。第二条命令仍应显示后处理节点订阅 `/goal`。

在两个监控终端分别按 Ctrl+C 退出 Record 监控。在结果终端运行 `rostopic echo /nav/map/post_process/result`，在反馈终端运行 `rostopic echo /nav/map/post_process/feedback`。

回到指令终端，发送导出请求。输入使用刚验收的 bag，输出目录由 SDK 配置决定。

```bash
BAG_DIR=$(dirname "$BAG_PATH")
EXPORT_ID="$BAG_NAME-pcd"
echo "$EXPORT_ID"

rostopic pub -1 /nav/map/post_process/goal spikive_control_sdk/PostProcessActionGoal \
  "{goal_id: {id: '$EXPORT_ID'}, goal: {cmd: 1, map_path: '', map_name: '', timeout: 0, bag_path: '$BAG_DIR', bag_name: '$BAG_NAME'}}"
```

等待 `status.goal_id.id` 等于 `EXPORT_ID` 的结果。通过时 `result.success: true`、`result.error_code: 0`，`result.message` 给出 PCD 路径、处理的消息数和点数。将它给出的路径填入 `PCD_PATH`。

```bash
# 示例假定当前用户与 SDK 运行用户相同，且使用默认输出目录。
PCD_PATH="$HOME/spikive-output/$BAG_NAME.pcd"
ls -lh "$PCD_PATH"
LC_ALL=C grep -a -m 1 '^POINTS ' "$PCD_PATH"
LC_ALL=C grep -a -m 1 '^DATA ' "$PCD_PATH"
```

通过条件是 PCD 文件存在，头部 `POINTS` 大于 0 且与 Action 结果中的点数一致，`DATA` 为 `binary`。若导出返回 `error_code=12`，检查 LIO 节点是否仍在运行，或 bag 是否已关闭并包含配置的点云消息。

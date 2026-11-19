# ROS1 录包与 PCD 导出快速教程

| 版本 | 日期 | 变更 |
| --- | --- | --- |
| V1.0 | 2026-11-19 | SDK 录包、分段合并、目录配置和 PCD 导出 |

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

## 5. 无硬件端到端测试

在同一台 ROS Noetic 主机开两个终端。本示例使用独立 master、本地临时文件和合成点云。

### 5.1 终端 A 启动录包和导出

```bash
source ~/driver_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11419
test_dir=$(mktemp -d /tmp/spikive-map-demo.XXXXXX)
cat > "$test_dir/sdk.yaml" <<EOF
record:
  output_directory: $test_dir/bags
  split_enabled: false
  topics: [/demo/cloud]
post_process:
  pointcloud_topic: /demo/cloud
  output_directory: $test_dir/pcd
EOF
roslaunch -p 11419 spikive_control_sdk spikive_control_sdk.launch \
  drone_id:=2 config_file:="$test_dir/sdk.yaml" \
  enable_flight:=false enable_localization:=false enable_data_mapping:=false
```

### 5.2 终端 B 录制合成点云并导出

```bash
source ~/driver_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11419
python3 - <<'PY'
from pathlib import Path
from uuid import uuid4

import actionlib
import rosbag
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from spikive_control_sdk.msg import PostProcessAction, PostProcessGoal, RecordAction, RecordGoal
from std_msgs.msg import Header, String

rospy.init_node('spikive_map_demo', anonymous=True)
directory = Path(rospy.wait_for_message('/nav/map/record/output_directory', String, timeout=10).data)
stem = 'demo-' + uuid4().hex[:8]
recorder = actionlib.SimpleActionClient('/nav/map/record', RecordAction)
exporter = actionlib.SimpleActionClient('/nav/map/post_process', PostProcessAction)
publisher = rospy.Publisher('/demo/cloud', PointCloud2, queue_size=2)
for client in (recorder, exporter):
    assert client.wait_for_server(rospy.Duration(10)), 'Action server unavailable'

def command(client, goal):
    client.send_goal(goal)
    assert client.wait_for_result(rospy.Duration(40)), 'Action result timeout'
    result = client.get_result()
    assert result is not None and result.success and result.error_code == 0, result
    print(result.message)
    return result

command(recorder, RecordGoal(cmd=1, bag_name=stem, timeout=10))
try:
    deadline = rospy.Time.now() + rospy.Duration(5)
    while publisher.get_num_connections() == 0 and rospy.Time.now() < deadline:
        rospy.sleep(0.05)
    assert publisher.get_num_connections() > 0, 'Recorder did not subscribe to /demo/cloud'
    for index in range(10):
        header = Header(stamp=rospy.Time.now(), frame_id='world')
        publisher.publish(point_cloud2.create_cloud_xyz32(header, [(float(index), 2.0, 3.0)]))
        rospy.sleep(0.1)
finally:
    command(recorder, RecordGoal(cmd=2, timeout=30))

bag_path = directory / (stem + '.bag')
with rosbag.Bag(str(bag_path)) as bag:
    assert bag.get_message_count('/demo/cloud') == 10
command(exporter, PostProcessGoal(cmd=1, bag_name=stem, timeout=30))
pcd = directory.parent / 'pcd' / (stem + '.pcd')
header = pcd.read_bytes().split(b'DATA binary\n', 1)[0].splitlines()
assert b'POINTS 10' in header, header
print('Verified', bag_path, 'and', pcd)
PY
```

预期得到一个包含 10 条点云消息的 bag，以及一个包含 10 个点的二进制 PCD。测试结束后在终端 A 按 Ctrl+C。分段及合并、默认目录和常见错误分支还可用仓库的自动测试覆盖。

```bash
cd ~/driver_ws
catkin_make run_tests_spikive_control_sdk -j2 -l2
catkin_test_results build/test_results
```

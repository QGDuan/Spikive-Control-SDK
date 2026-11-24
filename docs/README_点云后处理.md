# 点云后处理与导出（PostProcess）

`/nav/map/post_process` 把一次已关闭 bag 中的 `/drone_{drone_id}_cloud_registered` 点云按 bag 时间顺序全量拼接，导出为一个二进制 PCD。它不是 SLAM 建图算法，不做回环、ICP、降采样、去重或坐标变换；输入点云必须已经在同一坐标系下。

## 1. 前置条件

1. 飞行与录包已结束，输入 bag 已关闭（不能有同名 `.bag.active`）。
2. LIO/SLAM 已停止：默认不允许 `/laserMapping` 或 `/slam_to_uav_transform` 仍在 ROS master 中；存在时直接返回 `error_code=12`，不等待。
3. 输入 bag 中存在配置的点云 topic；默认 `/drone_{drone_id}_cloud_registered`。
4. 输出目录可写，且目标 `.pcd` 不存在。

LIO 节点名可在 `post_process/lio_nodes` 中配置。当前默认对应 `lsdc_slam mapping_livox.launch` 的 `/laserMapping` 和 `/slam_to_uav_transform`。

## 2. Action

```text
uint32 cmd              # 1 开始后处理，2 停止当前后处理
string map_path         # PCD 输出目录；空 => post_process/output_directory
string map_name         # PCD 文件名，不含路径/扩展名；空 => bag stem
float32 timeout         # 本次后处理期限 s；0 不限时
string bag_path         # 可选：输入 bag 绝对目录；空 => post_process/input_directory
string bag_name         # 可选：输入 bag 名，不含路径/扩展名；空 => 该目录最新 .bag
---
bool success
uint8 error_code
string message
---
uint8 state             # 0 IDLE 1 PROCESSING 3 COMPLETED 4 FAILED
```

开始示例（显式指定 record 输出目录和 bag 名）：

```bash
rostopic pub -1 /nav/map/post_process/goal spikive_control_sdk/PostProcessActionGoal \
"{goal_id: {id: 'pp-$(date +%s%N)'}, goal: {cmd: 1, map_path: '', map_name: '', timeout: 0, bag_path: '/home/colman/data', bag_name: 'record_test'}}"
```

不指定 bag 时，节点使用 `post_process/input_directory` 下最新的已关闭 `.bag`：

```bash
rostopic pub -1 /nav/map/post_process/goal spikive_control_sdk/PostProcessActionGoal \
"{goal_id: {id: 'pp-$(date +%s%N)'}, goal: {cmd: 1, map_path: '', map_name: '', timeout: 0, bag_path: '', bag_name: ''}}"
```

默认输出：

```text
$HOME/spikive-output/<map_name>.pcd
```

`map_name` 为空时取 bag 文件名去掉 `.bag`。例如 `/home/colman/data/record_test.bag` 默认输出 `$HOME/spikive-output/record_test.pcd`。

停止：

```bash
rostopic pub -1 /nav/map/post_process/goal spikive_control_sdk/PostProcessActionGoal \
"{goal_id: {id: 'pp-stop-$(date +%s%N)'}, goal: {cmd: 2, map_path: '', map_name: '', timeout: 30, bag_path: '', bag_name: ''}}"
```

`cmd=2` 或 Action cancel 只结束当前后处理；原 `cmd=1` goal 返回取消，停止请求 goal 返回停止结果。

## 3. 点云拼接规则

- 只处理 `sensor_msgs/PointCloud2`；类型不符返回 `19`。
- 必须存在 `x/y/z`，且为 `FLOAT32`、`count=1`；`intensity/ring/time` 等字段原样保留。
- 所有消息的字段名、offset、datatype、count、`point_step` 必须一致；不一致返回 `19`，不静默丢字段。
- 所有消息的 `frame_id` 必须一致；不做 TF。空 frame 只允许全部为空。
- 只支持 little-endian；大端点云返回 `19`。
- 按 `width * height` 取点，丢弃 `row_step` 的行间 padding；输出 `height=1`、`width=总点数`。
- 空点云跳过；全部为空返回 `12`。
- 不按 NaN/Inf 过滤，不做 voxel、半径、去重或排序。
- `post_process/max_points` 默认 `0`（不限制）；正数超限返回 `19`，用于防止内存失控。
- 使用 `pcl::PCDWriter::writeBinary` 输出二进制 PCD，保留字段布局。

## 4. 文件与失败处理

- 临时文件：`<output_dir>/.<map_name>.pcd.tmp.<pid>.<seq>`。
- 成功后先写临时文件，再硬链接到最终 `.pcd`；最终文件已存在时绝不覆盖。
- 失败、超时或取消时保留 `<map_name>.pcd.partial[.<pid>]`，result.message 给出路径；不会发布最终 `.pcd`。
- 输出目录不存在时自动创建；不可写返回 `19`。
- bag 仍在录制（`.bag.active`）返回 `12`；没有匹配 topic 或没有点返回 `12`。

## 5. 错误码

| 场景 | error_code |
| --- | --- |
| 成功 | 0 |
| 后处理超时 | 2 |
| 用户取消 / `cmd=2` | 3 |
| LIO 仍在运行、bag 缺失/仍 active、topic 无消息、无点 | 12 |
| 路径/名称非法、字段/frame 不一致、大端、输出已存在、点数超限 | 19 |
| 已有后处理任务 | 21 |
| bag 读取、PCL 写入、原子链接失败 | 99 |

## 6. 配置

```yaml
post_process:
  action: /nav/map/post_process
  pointcloud_topic: ""          # 空 => /drone_{drone_id}_cloud_registered
  input_directory: ""           # 空 => record.output_directory
  output_directory: ""          # 空 => $HOME/spikive-output
  lio_nodes:
    - /laserMapping
    - /slam_to_uav_transform
  max_points: 0
  feedback_period: 0.2
```

`post_process.input_directory` 留空时，后处理从 `record.output_directory` 读取 bag，默认是 `~/bags`。可用 `post_process.input_directory` 指定其他来源，也可在请求里传 `bag_path`。PCD 导出目录由 `post_process.output_directory` 管理，留空时使用 `$HOME/spikive-output`。修改 YAML 后重启使用该配置的节点。

## 7. 测试

```bash
catkin_make --only-pkg-with-deps spikive_control_sdk -j2 -l2
catkin_make run_tests_spikive_control_sdk -j2 -l2
catkin_test_results build/test_results
```

2026-11-19 在 Orin（`colman@192.168.31.93`）用真实 bag 切片验证：读取 55 条 `/drone_2_cloud_registered`，输出 `~/spikive-output/pp_slice.pcd`，248601 点、7.6 MB，frame 为 `world`；C++ 单元测试 6/6、rostest 4/4 通过。

录包本身见 [独立录包说明](README_录包.md)。此接口只导出 PCD，不生成 PCD 之外的 SLAM 地图后处理结果。

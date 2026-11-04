# 数据映射与可视化点云

`spikive_data_mapping_node` 已整合原 `visual_pointcloud_node` 的抽稀逻辑，无需依赖或启动原包。

| 输入（默认） | 输出 | 处理 |
| --- | --- | --- |
| `/drone_{id}_visual_slam/odom` | `/nav/data/odom` | 原消息完整转发 |
| `/drone_{id}_cloud_registered` | `/nav/data/pointcloud` | 原消息完整转发 |
| 同上，可单独配置 | `/nav/data/visual_pointcloud` | 均匀保留 10% 完整点记录 |

## 配置

编辑 [control_sdk.yaml](../config/control_sdk.yaml)，重启节点生效：

```yaml
visual_pointcloud:
  enabled: true
  input_topic: ""  # 留空复用 data/pointcloud_source，按 drone_id 选择
  output_topic: /nav/data/visual_pointcloud
  keep_ratio: 0.1
```

`keep_ratio` 范围 `(0, 1]`，0.5 保留 50%，1 原样转发；关闭使用 `enabled: false`。启用时非法比例会报错退出。源话题和输出必须不同，各输出不得重名；配置检查包含 ROS remap 后的名称。

非空云保留 `max(1, floor(输入点数 × keep_ratio))` 个点，沿原始点序均匀选取。每个点的全部字节（含自定义字段、NaN）保留；`fields`、`point_step`、字节序、`is_dense`、Header（包括 seq/时间戳/frame_id）保持原值。源为 world 时仍为 world，不做坐标变换。

抽稀输出为紧凑单行：`height=1`，更新 `width`、`row_step` 和 `data` 长度；支持源点云的多行及行尾 padding。比例 1 或空云原样转发。异常布局仅丢弃可视化输出，完整点云映射沿用原行为。比例控制每帧点数，不控制帧率；无可视化订阅者时跳过抽稀。

同源时共用完整点云订阅（队列 2），自定义可视化源另建订阅（队列 1）；可视化发布队列为 1。不修改 Planner/SLAM 的输入或原独立包，SDK 内的可视化发布保留源 seq。

## 启动与检查

只启动数据节点：

```bash
roslaunch spikive_control_sdk data_mapping.launch drone_id:=2
```

总启动 `spikive_control_sdk.launch` 已包含此节点。自定义 YAML 可传 `config_file:=/绝对路径/control_sdk.yaml`。避免同时运行原独立可视化节点向同一输出发布。

```bash
rostopic echo -n 1 /nav/data/visual_pointcloud/header
rostopic echo -n 1 /nav/data/visual_pointcloud/width
rostopic hz /nav/data/visual_pointcloud
```

RViz：Fixed Frame=`world`，添加 PointCloud2，Topic=`/nav/data/visual_pointcloud`。

验证：ROS Noetic Docker 编译/安装通过；5 个抽稀单元用例、4 组数据节点 ROS 场景及原 10 组 SDK 集成场景通过。未测 Orin 实际吞吐。

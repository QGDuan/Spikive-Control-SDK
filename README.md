# Spikive-Control-SDK

ROS1 Noetic C++ SDK，负责已有命令转发、状态监视和数据整理。当前版本 **1.4.0**，按 [V1.4 规范](docs/specs/ROS接口规范V1.4.pdf) 和 [阶段附件](docs/specs/接口优先级V1.1_20261102.pdf) 提供一阶段接口、二阶段 1～3 项，并保留 Land。

| 节点 | 功能 | 对应后端 |
| --- | --- | --- |
| `spikive_flight_node` | Takeoff / Goto / Return / Land、Emergency、FlightState | Planner、FlightManager、px4ctrl、MAVROS 状态 |
| `spikive_localization_node` | LoadMap、Relocalize、定位状态 | SLAM global_match |
| `spikive_map_record_node` | Record 启停、确认、bag 保存路径整理 | SLAM save_result |
| `spikive_data_mapping_node` | Odometry / PointCloud 原样映射、可视化点云抽稀 | SLAM fusion_repub |

```bash
roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=2
```

`drone_id` 默认 1，须与现有后端一致。各节点也可分别用 `flight.launch`、`localization.launch`、`map_record.launch`、`data_mapping.launch` 启动；总 launch 支持 `enable_<模块>:=false`。后端仍由原系统启动。

- [最简命令与 rostopic 监控](docs/README_命令行测试.md)
- [V1.4 范围、接口差异与验证](docs/README_V1.4_实现与验证.md)
- [中文开发逻辑与模块对应](docs/开发逻辑.md)
- [定位流程与坐标定义](docs/README_定位与点云加载.md)
- [数据映射与可视化点云配置](docs/README_数据映射.md)
- [开发 Skill](skills/spikive-control-sdk/SKILL.md)；[V1.2 历史评审](docs/ROS接口规范V1.2_第3章评审.md)

**控制边界**：按用户确认，只修改 SDK。`target_altitude=0` 使用原生起飞高度；非零高度、正 `speed`、`yaw_enable=true` 返回 `20 ERR_NOT_SUPPORTED`。`timeout>0` 已生效，`0` 不限时。超时和取消仅结束跟踪，Emergency 才显式转发位置保持/恢复。

**坐标与数据**：初值/Goto 使用原生 world，导航里程计保持 world/body；既有匹配结果 map→world 保留。数据映射保留 Header（含 seq）、协方差、twist、点字段及源采样时间；不补算、不换轴。上报随源消息触发，负载过高时仍受 ROS 队列约束。

`data_mapping` 已整合原 `visual_pointcloud_node` 逻辑，额外发布 `/nav/data/visual_pointcloud`，默认均匀保留 10% 的完整点记录。比例和开关见 `config/control_sdk.yaml` 的 `visual_pointcloud`；原 `/nav/data/pointcloud` 仍保留完整点云。

**升级**：V1.4 修改四个飞行 Action 的字段和阶段编号，旧客户端必须重新编译。C++ 定位库独立为 `spikive_localization_sdk`；原单节点拆为上表四节点，私有参数路径随节点名改变。LoadMap/Relocalize 字段保持不变。二阶段后处理、三阶段配置不在本轮范围。

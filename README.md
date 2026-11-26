# Spikive-Control-SDK

ROS1 Noetic SDK，负责已有命令转发、状态监视和数据整理。当前版本 **1.7.1**，包含 Goto/Return 规划速度上限与 Goto 终点 yaw 控制；飞行消息沿用 [V1.4 规范](docs/specs/ROS接口规范V1.4.pdf) 和 [阶段附件](docs/specs/接口优先级V1.1_20261102.pdf)，提供一阶段接口、二阶段 1～4 项，并保留 Land。

| 节点 | 功能 | 对应后端 |
| --- | --- | --- |
| `node_manager.py` | 整套与单项启停、依赖就绪检查、进程回收和状态上报 | 机型 YAML 中配置的启动项 |
| `spikive_flight_node` | Takeoff / Goto / Return / Land、Emergency、FlightState | Planner、FlightManager、px4ctrl、MAVROS 状态 |
| `spikive_localization_node` | LoadMap、Relocalize、定位状态 | SLAM global_match |
| `spikive_map_record_node` | 独立 rosbag 录制；命令行指定 topics，停止后保存 | SDK 管理的 ROS 原生 Recorder |
| `spikive_post_process_node` | 读取已关闭 bag 的 registered cloud，全量拼接导出 PCD | SDK 原生 rosbag + PCL |
| `spikive_data_mapping_node` | Odometry / PointCloud 原样映射、可视化点云抽稀 | SLAM fusion_repub |

```bash
roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=2
```

`drone_id` 默认 1，须与现有后端一致。各节点也可分别用 `flight.launch`、`localization.launch`、`map_record.launch`、`post_process.launch`、`data_mapping.launch` 启动；总 launch 支持 `enable_<模块>:=false`。Manager 默认关闭。原机载入口切换到 SDK Manager 时，使用 `enable_manager:=true` 并移除原 `astro_manager` 启动命令。

### Manager 启停接口

独立启动 Manager 可执行 `roslaunch spikive_control_sdk manager.launch drone_id:=2`。默认使用 MID360 配置，传入 `config:=<YAML绝对路径>` 可选择 Airy 或 S10 Ultra。统一 launch 对应参数为 `manager_config`。三个机型文件位于 [config/manager](config/manager)。

地面站通过同步服务 `/drone_{id}_manage_launch` 调用 `All`，START=1、STOP=2。服务返回后检查 success 和 message。兼容的命令和状态 topic 为 `/drone_{id}_command_topic` 和 `/drone_{id}_auto_manager_status`，消息类型为 `spikive_control_sdk/Command` 和 `spikive_control_sdk/AutoManager`。单项调用使用同步服务 `/drone_{id}_manage_launch`，类型为 `spikive_control_sdk/ManageLaunch`。

```bash
# operation=1 启动 Planner，operation=2 只关闭 Planner
rosservice call /drone_2_manage_launch "{launch_name: 'All', operation: 1}"
rosservice call /drone_2_manage_launch "{launch_name: 'All', operation: 2}"
rosservice call /drone_2_manage_launch "{launch_name: 'Planner', operation: 1}"
rosservice call /drone_2_manage_launch "{launch_name: 'Planner', operation: 2}"
```

- `launch_name` 取当前 YAML 中的启动项名，区分大小写。启动前检查全部直接依赖；依赖未就绪时返回 `success=false` 和 `unready_dependencies`。启动成功表示目标节点已注册并经过配置的就绪延时。
- 关闭只清理目标进程组，返回前检查目标节点退出。上游和下游继续运行。对本 Manager 已启动的就绪项重复启动或对已停止项重复关闭，可返回成功。其他进程持有同名节点时返回失败及原因。
- 单项与整套操作串行。忙时返回拒绝原因。新鲜的 `/mavros/state` 为 armed 时拒绝启停；状态缺失或超过 3 秒时只允许启动。单独停止 MavRos 后，要恢复 MAVROS 状态才能再次执行关闭请求。
- 部分运行时 `mode=partial`、`is_active=false`。整套启动可补齐缺项，整套停止会按逆序回收 Manager 持有的启动项。

- [v1.6 同事交接入口](docs/交接说明_V1.6.md)
- [v1.6 开发记录](docs/开发记录_V1.6.md)
- [地面站前后端交互合同](docs/地面站交互_V1.6.md)
- [录包与 PCD 导出快速教程](docs/ROS录包与PCD导出快速教程.md)
- [独立录包：话题列表与命令行测试](docs/README_录包.md)
- [点云后处理与 PCD 导出](docs/README_点云后处理.md)
- [v1.5 历史安装包说明](docs/README_V1.5_安装.md)
- [最简命令与 rostopic 监控](docs/README_命令行测试.md)
- [V1.4 范围、接口差异与验证](docs/README_V1.4_实现与验证.md)
- [中文开发逻辑与模块对应](docs/开发逻辑.md)
- [定位流程与坐标定义](docs/README_定位与点云加载.md)
- [数据映射与可视化点云配置](docs/README_数据映射.md)
- [新版 Planner 速度与终点航向接入](docs/README_速度与航向.md)
- [开发 Skill](skills/spikive-control-sdk/SKILL.md)；[V1.2 历史评审](docs/ROS接口规范V1.2_第3章评审.md)

**控制边界**：按用户确认，只修改 SDK。`target_altitude=0` 使用原生起飞高度，正的非零高度仍返回 20。新版后端支持 Goto/Return 的正 `speed`（规划上限，持续生效）及 Goto `yaw_enable=true`（到点后一次航向调整）；接口缺失返回 20。`speed<=0` 沿用当前上限。`timeout>0` 生效，0 不限时；超时/取消只结束跟踪，不撤回速度或 yaw 控制。

**坐标与数据**：初值/Goto 使用原生 world，导航里程计保持 world/body；既有匹配结果 map→world 保留。数据映射保留 Header（含 seq）、协方差、twist、点字段及源采样时间；不补算、不换轴。上报随源消息触发，负载过高时仍受 ROS 队列约束。

`data_mapping` 已整合原 `visual_pointcloud_node` 逻辑，额外发布 `/nav/data/visual_pointcloud`，默认均匀保留 10% 的完整点记录。比例和开关见 `config/control_sdk.yaml` 的 `visual_pointcloud`；原 `/nav/data/pointcloud` 仍保留完整点云。

**升级**：V1.4 修改四个飞行 Action 的字段和阶段编号，旧客户端必须重新编译。C++ 定位库独立为 `spikive_localization_sdk`；原单节点拆为上表五节点，私有参数路径随节点名改变。LoadMap/Relocalize 字段保持不变。二阶段后处理中的 `post_process` 已作为独立节点提供，三阶段配置仍不在本轮范围。

1.5.0 正式包含 1.4.1 接入的速度/yaw 功能，与 1.4.0/1.4.1 的消息定义一致；重编译 SDK 节点并部署带 max_vel/yaw_override 接口的新后端即可。SDK 不替换 Planner 源码，也不接管 MAVROS setpoint。

1.6.0 按新要求将 Record 改为 SDK 自行管理 `rosbag::Recorder`，不再调用 SLAM 录包后端。`Record.action` 新增 `string[] topics`，开始命令可以显式覆盖录制话题，留空时使用 YAML 配置的默认清单。停止只操作本 SDK 的当前录制。**Record 消息 MD5 已变化，SDK 和录包客户端必须一起重新编译、重新 source 工作区。旧 v1.5 安装包不支持此字段。** 四个飞行 Action 和定位消息未变，使用见 [录包说明](docs/README_录包.md)。

1.7.0 增加 YAML 默认录包话题和保存目录、可选分段与合并进度，以及独立的 PCD 导出 Action。

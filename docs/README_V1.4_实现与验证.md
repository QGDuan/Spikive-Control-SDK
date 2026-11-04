# V1.4 实现与验证

交付日期：2026-11-04。依据 [接口 V1.4](specs/ROS接口规范V1.4.pdf) 与 [阶段附件 V1.1](specs/接口优先级V1.1_20261102.pdf)。用户确认仅修改 Control SDK；后端没有的控制输入明确报错。SLAM 参考 `main@c86c818`，Planner/WayPoint 沿用本工作区。

## 1. 范围逐项对应

| 阶段 / 序号 | 接口 | 实现与边界 |
| --- | --- | --- |
| 一阶段 1 | `/nav/localization/load_map` | PCD/MD5 校验，转发地图，等待新回显和 waiting_initialpose |
| 一阶段 2 | `/nav/localization/relocalize` | world 初值；本轮 matching→localized 加新修正，回报 fitness/threshold/attempt_count；不重复触发 ICP |
| 一阶段 3 | `/nav/flight/takeoff` | cmd=1；新 tookoff 成功；非零 target_altitude 报 20，0 沿用后端高度 |
| 一阶段 4 | `/nav/flight/goto` | Continue→0.2 s→原生 GoalSet；控制器 reached 加 SDK 距离/稳定观察；指定速度/yaw 报 20 |
| 一阶段 5 | `/nav/safety/emergency` | HOVER/STOP 均为 cmd=5 位置保持；RESUME 为 cmd=4 当前点继续；准入后结束已有飞行 Action |
| 一阶段 6 | `/nav/flight/return_home` | cmd=3；观察新目标及 returned false→true；返航距离用新目标减导航位置，缺少目标时 NaN；正 speed 报 20 |
| 一阶段 7 | `/nav/status/flight_state` | 2 Hz 已知状态快照；读取控制器/MAVROS并观察外部 /control、GoalSet；无依据不伪造状态 |
| 二阶段 1 | `/nav/data/pointcloud` | `/drone_{id}_cloud_registered` 原样映射，包含原 Header.seq 和二进制布局 |
| 二阶段 2 | `/nav/data/odom` | `/drone_{id}_visual_slam/odom` 原样映射；协方差及 twist 取实际源版本，不拼接 MAVROS |
| 二阶段 3 | `/nav/map/record` | 对接 SLAM bag_record_start/stop/status；request_id 确认，停止后保存已关闭 bag |
| 保留接口 | `/nav/flight/land` | 同步 V1.4 字段；新 landed 与 MAVROS ON_GROUND 共同确认；不主动上锁 |

附件一阶段没有单列 Land；本轮保留并迁移已有能力。二阶段第 4 项 `/nav/map/post_process` 及三阶段接口未加入。

## 2. 飞行协议与迁移

ROS 类型仍沿用本包既有小写 `takeoffAction/gotoAction/return_homeAction/landAction`，字段类型、顺序按 V1.4；新增 `RecordAction`、`EmergencyCmd`、`FlightState`。

| 接口 | V1.4 变化 | 当前反馈编号 |
| --- | --- | --- |
| Takeoff | 删除 climb_speed、progress | 1 PRECHECK、2 CLIMBING、3 DONE |
| Goto | 增加 yaw/yaw_enable；到达阈值/stable_time 移至 YAML，删除 final_distance | 1 NAVIGATING、2 HOLDING、3 DONE |
| Return | 增加 speed；distance_to_home 替换 progress | 1 RETURNING、2 ARRIVED |
| Land | 删除 descend_speed、Result.disarmed | 1 DESCENDING、2 TOUCHDOWN、3 DISARMED |

`success/error_code/message` 保留。Land 着地时若也观察到新的 `!armed`，反馈 DISARMED；否则 TOUCHDOWN，随后 Action 结束，不等待上锁。需要持续看上锁状态可订阅 `/mavros/state`。

`timeout`：有限非负秒数；0 禁用，正值从接受请求开始计时。使用单调时钟，不受 `/clock` 暂停或回跳影响；到期返回 `success=false,error_code=2`，停止未发送的 Goto 定时目标，释放跟踪任务。取消返回 3。已经转发的命令不撤回。

四个飞行 Action 共用一个任务槽，重复请求返回 21。显式 Emergency 成功转发或观察到更新的外部控制命令切换时，当前 Action 结束跟踪。互斥不约束前端/WayPoint 等其他发布者；已有协议无请求 ID/执行 ACK，时间戳和新目标只能排除旧缓存，不能证明严格归属。

Emergency 准入：连接有效、MAVROS IN_AIR、有效导航位置，并已观察到本机 GoalSet 和 Planner 轨迹。这与现有 cmd=5 的轨迹分支一致；单纯起飞且尚无轨迹时不冒充支持位置保持。`success=true` 表示已转发，不表示物理静止；恢复不续接旧航线。

## 3. FlightState 的现有来源

| state | 依据 |
| --- | --- |
| 0 IDLE | 已连接、已知地面状态；SDK 尚未观察过空中或降落命令 |
| 1 PRECHECK | 本节点 Takeoff 已接受，尚处于准入后跟踪阶段 |
| 2 TAKING_OFF | MAVROS TAKEOFF，或起飞命令/任务尚未完成 |
| 3 HOLDING | 控制器 tookoff/reached/returned，或已观察到有效轨迹分支的位置保持命令；是保持模式，不是速度为零证明 |
| 4 NAVIGATING | 有目标和 Planner 轨迹，未观察到到达 |
| 5 AVOIDING | 后端无独立可靠信号，本版不生成 |
| 6 RETURNING | 返航命令/任务，尚未 returned |
| 7 LANDING | MAVROS LANDING 或空中降落命令/任务 |
| 8 LANDED | 已知地面状态，且曾观察到空中/降落命令 |

优先处理起降过渡态；本节点活动任务使用其实际跟踪阶段，避免旧 reached 覆盖新的 Goto 距离判据。原枚举无 UNKNOWN，缺连接/基础状态时停止发布，不伪造 IDLE。Header.stamp 保留所用控制器/ExtendedState 中较早的采样时间；2 Hz 是快照频率，不表示两路数据每次都更新。没有新增数据年龄策略；消费者可检查源时间。订阅已开始的外部任务，仍受非 latched 命令不可追溯的限制。

## 4. 采集命令和保存行为

Record 的两个 Goal 是独立命令：cmd=1 在收到本次启动确认后返回 success、feedback=1 RECORDING，采集继续；cmd=2 在停止确认和保存完成后返回 success、feedback=3 COMPLETED。失败 feedback=4。timeout 控制本次命令确认/导出耗时，不是总采集时长。

| SDK 内容 | SLAM 对应 |
| --- | --- |
| cmd=1 | `/drone_{id}_bag_record_start`：JSON 数字 schema_version=1、唯一 request_id |
| cmd=2 | `/drone_{id}_bag_record_stop`：独立 request_id |
| 确认 | `/drone_{id}_bag_record_status` 的相同 request_id、record_start/record_stop、ok、state |
| bag 数据 | save_result 原有 cloud_registered_body、Odometry_trans、visual_slam/odom、IMU；本节点不另选录制字段 |
| bag_path / bag_name | cmd=1 给出本机绝对目录和无路径/扩展名的名称；cmd=2 留空沿用，或同时提供新目标用于恢复 |

SLAM 暂存目录仍由其原启动配置决定。停止后 SDK 将状态中的已关闭 bag 整理为 `bag_path/bag_name.bag`：同盘硬链接，跨盘后台复制后原子发布，保留 SLAM 源文件，拒绝覆盖同名文件。只有最终文件完整落地才返回停止成功；不生成 PCD，不做地图后处理。SDK 与后端须可访问同一文件系统。

取消/超时只结束跟踪和正在进行的文件导出，不自动发送 stop。用户须用 cmd=2 明确停止后端；晚到确认更新缓存，不恢复已结束 Action。已原子保存的完整目标文件不会因随后到达的取消/超时被删除。已收到关闭确认但导出失败，可再次 cmd=2 并给新路径重试。SDK 重启后若后端仍在采集，停止请求同时提供路径与名称。建议采集命令由单一业务入口管理。

## 5. 构建与验证

在 ROS Noetic / MAVROS 环境中构建本包及 controller_msgs、quadrotor_msgs 依赖。四个节点、三个库及消息均由 catkin 安装。

```bash
catkin_make --only-pkg-with-deps spikive_control_sdk -j2 -l2
catkin_make run_tests_spikive_control_sdk -j2 -l2
catkin_test_results build/test_results
```

2026-11-04 验证记录：使用 `spikive-slam:master-build` 的 ROS Noetic / MAVROS 环境，在无外部网络的独立 Docker 容器中运行模拟后端。

| 检查 | 结果 |
| --- | --- |
| 9 个 Action/Service/消息定义 | 字段、类型、顺序与 V1.4 一致 |
| catkin 构建与安装 | 四节点、三个库及消息生成通过；安装后链接依赖完整 |
| C++ 单元测试 | 18 个用例通过：定位文件、坐标约定、录制文件与截止时间 |
| ROS 集成测试 | 10 个场景通过：原消息完整透传、参数拒绝、飞行闭环、超时/取消、Emergency、状态映射、录制确认与保存、单次重定位 |
| 命令行示例 | 12 条请求使用安装后的消息类型完成序列化检查，未向实际飞机发送 |
| 改动范围与文档 | 13 个 SDK 外源码/文件目录校验值不变；文档链接、XML、Skill 格式通过 |

以上使用模拟飞行和定位回报，未验证 Orin/真机飞行或真实 ICP；录制跨文件系统复制分支未做独立运行测试。

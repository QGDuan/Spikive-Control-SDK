# SDK 命令行测试

直接使用 `roslaunch`、`rostopic pub` 和 `rostopic echo`，无需编写客户端。地图加载与重定位命令另见 [定位测试 README](README_定位与点云加载.md)，可在飞行测试前完成。

支持 Takeoff → Goto → Return → Land。每一步等待结果成功后再执行下一步；四个飞行 Action 共用一个任务槽。Goto 使用项目原生局部 `[x,y,z]`，不是 NED，也不是相对位移；Return 使用 FlightManager 已配置的返航点。

## 1. 启动

以下命令使用默认配置、`drone_id=1`。每个终端先 source 同一个 catkin 工作区的 `devel/setup.bash` 或 `install/setup.bash`。MAVROS、Planner、FlightManager、px4ctrl 和 `/drone_1_state` 转发节点按原流程启动；这些命令会转发真实飞行指令。

```bash
roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=1
```

Orin 已有后端编号为 2 时，上面的 launch 改用 `drone_id:=2`，下文所有 `/drone_1_*` 对应改成 `/drone_2_*`；Action 接口名称不变。地图和初值必须在起飞前完成一次匹配，成功后后端关闭地图 ICP（`continuous_match=false`），飞行期间继续使用 LIO 位姿及固定修正变换。

可先查看后端状态，每条 echo 单独运行，按 Ctrl+C 退出该监视窗口：

```bash
rostopic echo /mavros/state
rostopic echo /mavros/extended_state
rostopic echo /drone_1_state
```

| 观察字段 | 含义 |
| --- | --- |
| `State.connected` | true 表示 MAVROS 报告已连接 |
| `State.armed` | true 已解锁；false 已上锁 |
| `ExtendedState.landed_state` | 0 未知、1 在地面、2 在空中、3 起飞中、4 降落中 |
| `DroneState.tookoff` | 现有控制器报告起飞完成 |
| `DroneState.reached` | 控制器现有 XY 到达标志；Goto 还会检查目标距离和稳定时长 |
| `DroneState.returned` | 控制器返航完成标志；Return 需观察命令后的 false→true，并结合新 MAVROS IN_AIR |
| `DroneState.landed` | 控制器地面标志；需要结合 MAVROS ON_GROUND 判断真实着地 |

## 2. 起飞测试

起飞前需有有效里程计、控制器状态、MAVROS 连接和 ON_GROUND 状态，且 `/control` 有订阅者。先开两个监视终端，再发送请求，避免错过一次性的结果。

终端 A：看阶段与高度。

```bash
rostopic echo /nav/flight/takeoff/feedback
```

终端 B：看最终结果。

```bash
rostopic echo /nav/flight/takeoff/result
```

终端 C：发送一次起飞请求。

```bash
rostopic pub -1 /nav/flight/takeoff/goal spikive_control_sdk/takeoffActionGoal '{goal: {target_altitude: 0, climb_speed: 0, timeout: 0}}'
```

三个 Goal 参数目前均为预留，填 0 不代表目标高度是 0；实际起飞高度和速度由原控制器决定。没有填写的 `goal_id` 由当前 C++ ActionServer 自动生成，可以重复使用这条命令，但已有活动任务时会拒绝新请求。

| 检查步骤 | 预期 |
| --- | --- |
| 请求被接受 | `status.status=1`；收到反馈；SDK 向 `/control` 转发一次 `cmd=1` |
| 起飞过程中 | `feedback.phase` 为 1 或 2；`current_altitude` 为相对本次起点的高度；`progress=0` |
| 控制器确认完成 | 最终反馈 `phase=4, progress=1`；结果 `status.status=3, result.success=true, result.error_code=0` |
| 请求被拒绝 | `status.status=5, result.success=false`；按 `error_code` 和 `message` 查看原因 |

阶段允许从 1 直接到 4，本版没有阶段 3。无效高度样本输出 NaN；没有超时策略，未收到完成证据时持续跟踪。

## 3. Goto 测试

等起飞 Action 成功、MAVROS 为 `IN_AIR` 后执行。Planner 需订阅 `/goal_with_id`，`/drone_1_visual_slam/odom` 有有效 xyz、非零时间戳，frame 为 world，并与目标使用同一原生地图坐标。先看当前位置和 header：

```bash
rostopic echo -n 1 /drone_1_visual_slam/odom
```

终端 A、B 分别监控反馈和结果：

```bash
rostopic echo /nav/flight/goto/feedback
rostopic echo /nav/flight/goto/result
```

终端 C 发送目标。示例为原生坐标 `x=1, y=0, z=1.5`（m）；按实际可飞区域和当前坐标修改数值，SDK 原样转发，不交换轴或反转 z。

```bash
rostopic pub -1 /nav/flight/goto/goal spikive_control_sdk/gotoActionGoal '{goal: {position: [1, 0, 1.5]}}'
```

未填写字段默认 0：到达三维距离、水平距离、稳定时长采用配置中的 `0.30 m、0.25 m、1.0 s`；`speed`、`timeout` 保留但不执行。目标 z 是原生局部坐标值，不是相对本次起飞点的高度。

| 检查步骤 | 预期 |
| --- | --- |
| 请求接受 | `status.status=1`；转发 `cmd=4`，约 0.2 s 后向 `/goal_with_id` 发布 `{drone_id: 1, goal: [1, 0, 1.5]}` |
| 飞向目标 | `feedback.state=1`；`distance_total`、`distance_horizontal` 为到请求目标的距离；`current_speed` 为 MAVROS 当前速度模长 |
| 满足到达条件 | 新 `reached=true`、两个距离满足阈值，反馈 `state=3`；继续观测稳定时长 |
| 持续满足 | 最终 `state=4`；结果 `status.status=3, success=true, error_code=0`，`final_distance` 为结束时距离 |
| 目标尚未发出时条件变化 | `status.status=4`（ABORTED）、`error_code=12`；Continue 可能已经发出 |

本版不输出 `state=2 AVOIDING`。`state=3` 是到达稳定检查，不是另发悬停命令。没有超时；地形跟随改写目标 z、仅变 z 的目标或没有持续更新的状态，可能导致一直等待，详见 [Goto 对应表](../README.md#goto-v11-对应关系)。

## 4. Return 返航测试

等 Goto 完成、MAVROS 为 `IN_AIR` 且 `DroneState.returned=false` 后执行。FlightManager 需已启动，Planner 需订阅 `/goal_with_id`。返航点使用后端 `init_x/init_y/init_z`（已有 autoloc 可更新 XY），不是 SDK 自动记录的起飞点，也不会随 Relocalize 初值自动改变；先确认其与当前原生坐标一致。

终端 A、B 分别看反馈和结果：

```bash
rostopic echo /nav/flight/return_home/feedback
rostopic echo /nav/flight/return_home/result
```

终端 C 发送一次返航请求：

```bash
rostopic pub -1 /nav/flight/return_home/goal spikive_control_sdk/return_homeActionGoal '{goal: {timeout: 0}}'
```

`timeout` 预留，不设置超时。该 Action 名称来自 PDF，字段为 SDK 补充定义。

| 检查步骤 | 预期 |
| --- | --- |
| 请求接受 | `status.status=1`；SDK 只向 `/control` 发布一次 `cmd=3`；FlightManager 在命令切换到 3 时发布返航点 |
| 跟踪返航 | 初始 `feedback.phase=1`；收到命令后 `returned=false` 转为 `phase=2, progress=0`，此阶段不是后端命令 ACK |
| 控制器报告完成 | 之后收到时间更晚的 `returned=true && !landed`，连接有效且新 MAVROS 为 IN_AIR；最终 `phase=4, progress=1`，结果 `status.status=3, success=true, error_code=0` |
| 当前已报告返航完成 | `status.status=5, success=false, error_code=12`；不发送命令，不复用已有 true |

`returned` 沿用控制器 XY 到达和已有约 3 s 的状态持续判断，不表示三维距离和航向均完成。若一直没有命令后的 false→true，Action 持续等待，可取消跟踪。连续 `cmd=3` 不会让 FlightManager 重新发布返航点，因此取消后再次请求不保证重新派发。**成功后不会自动降落或上锁，下一步需单独调用 Land。**

## 5. 降落测试

等 Return、Goto 或起飞 Action 完成，或取消其跟踪后，再发降落请求。先分别打开下面两个监视窗口：

```bash
rostopic echo /nav/flight/land/feedback
rostopic echo /nav/flight/land/result
```

在另一个终端发送一次：

```bash
rostopic pub -1 /nav/flight/land/goal spikive_control_sdk/landActionGoal '{goal: {descend_speed: 0, timeout: 0}}'
```

`descend_speed` 和 `timeout` 均预留。反馈的 `height` 是下方净空，无有效测量时为 NaN；不是本地坐标 z。新的控制器 `landed=true` 与 MAVROS ON_GROUND 共同确认后返回 `success=true, error_code=0`。`disarmed` 单独报告是否确认已上锁，成功不要求 `disarmed=true`。

## 6. 取消与监控

取消对应接口的当前任务：

```bash
rostopic pub -1 /nav/flight/takeoff/cancel actionlib_msgs/GoalID '{}'
rostopic pub -1 /nav/flight/land/cancel actionlib_msgs/GoalID '{}'
rostopic pub -1 /nav/flight/goto/cancel actionlib_msgs/GoalID '{}'
rostopic pub -1 /nav/flight/return_home/cancel actionlib_msgs/GoalID '{}'
```

按需要只运行其中一条。空 GoalID 表示取消该 Action 接口的所有活动目标；本节点最多跟踪一个任务。返回 `result.success=false, result.error_code=3`，已接受任务通常为 `status.status=2`。**取消和退出 echo 都不会停止飞机；取消只结束 SDK 跟踪。** Goto 在 0.2 s 等待期被取消时不再发送目标，已发出的 Continue 或目标不撤回。

| 想看什么 | 命令 |
| --- | --- |
| 起飞 Action 生命周期 | `rostopic echo /nav/flight/takeoff/status` |
| Goto Action 生命周期 | `rostopic echo /nav/flight/goto/status` |
| Return Action 生命周期 | `rostopic echo /nav/flight/return_home/status` |
| 目标坐标原样转发 | `rostopic echo /goal_with_id` |
| 降落 Action 生命周期 | `rostopic echo /nav/flight/land/status` |
| 实际转发的控制消息 | `rostopic echo /control` |
| 控制器完成标志 | `rostopic echo /drone_1_state` |
| 上锁状态 | `rostopic echo /mavros/state` |

`status` 中可能同时保留多个近期任务，按 `goal_id.id` 区分。不要混淆 Action 的 `status.status=3`（成功）与 `result.error_code=3`（业务取消）。`/control` 也可能收到前端等其他发布者的消息。

## 7. 常见结果

| `result.error_code` | 本版含义 |
| --- | --- |
| 0 | 已观察到完成状态 |
| 3 | 已取消跟踪 |
| 4 | 起飞缺少有限 z，或 Goto 导航里程计缺失、frame 不符、xyz 非有限或时间戳为零 |
| 6 | 降落时已在地面，或 Goto / Return 时不是 IN_AIR |
| 7 | 起飞请求时已有空中或起降过渡状态 |
| 12 | 状态、连接、订阅者或 ROS 时间条件不满足；Return 请求时已有 returned=true |
| 19 | Goto 请求含 NaN 或 Inf |
| 20 | Goto 原生目标 z < -0.1，现有 Planner 不支持 |
| 21 | 已有 Takeoff、Land、Goto 或 Return Action 正在跟踪 |

本说明中的案例用于用户运行测试。上述 8 条 pub 命令已通过 Noetic 离线 YAML 填充和 ROS 消息序列化校验；未向运行中的飞机发布测试命令。离线校验和编译不代表已经完成飞行验证。

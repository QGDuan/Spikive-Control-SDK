# V1.4 最简命令行测试

每个终端先 source 同一个工作区的 `devel/setup.bash`。后端按原系统启动，示例编号 2，须与实际飞机一致。下面飞行请求会转发真实控制命令；先完成地图匹配，再逐步执行并等待 success。

## 1. 启动与监控

```bash
roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=2
```

每条监控命令独立终端运行：

```bash
rostopic echo /nav/status/flight_state
rostopic echo /nav/status/localization
rostopic echo /nav/flight/takeoff/feedback
rostopic echo /nav/flight/takeoff/result
```

其他 Action 将 `takeoff` 换为 `goto`、`return_home` 或 `land`。先开监控，再发请求；结果是一次性消息。

## 2. 加载地图与给入初始位置

```bash
rosservice call /nav/localization/load_map "{map_path: '/home/colman/data/waypoint.pcd', map_md5: ''}"
```

初值是已有地图 world 中的机体位姿；下面为示例原点，按现场位置填写：

```bash
rostopic pub -1 /nav/localization/relocalize/goal spikive_control_sdk/relocalizeActionGoal '{goal: {initial_pose: {header: {frame_id: world}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {w: 1}}}}}}'
rostopic echo /nav/localization/relocalize/result
```

应在另一个终端预先运行最后一条 echo，等待 `success=true` 后再发送地图坐标中的目标。一次匹配成功不自动证明 MAVROS 本地估计器已完成原点/航向对齐。

## 3. Takeoff / Goto / Return / Land

每次只执行一个请求，等结果成功后再执行下一个。timeout=30 为跟踪时限；到期仅返回错误，不自动停机。

```bash
rostopic pub -1 /nav/flight/takeoff/goal spikive_control_sdk/takeoffActionGoal '{goal: {target_altitude: 0, timeout: 30}}'
```

`target_altitude=0` 沿用 px4ctrl 启动高度，并非飞到 0 m；其他高度当前返回 20。

```bash
rostopic pub -1 /nav/flight/goto/goal spikive_control_sdk/gotoActionGoal '{goal: {position: [1, 0, 1], speed: 0, yaw: 0, yaw_enable: false, timeout: 30}}'
```

目标为原生 world 绝对坐标。speed<=0 用后端速度；正 speed 或 yaw_enable=true 返回 20。

```bash
rostopic pub -1 /nav/flight/return_home/goal spikive_control_sdk/return_homeActionGoal '{goal: {speed: 0, timeout: 30}}'
```

返航点来自 FlightManager 配置，返航成功不自动降落。

```bash
rostopic pub -1 /nav/flight/land/goal spikive_control_sdk/landActionGoal '{goal: {timeout: 30}}'
```

| 接口 | 完成反馈 | 成功结果 |
| --- | --- | --- |
| Takeoff | phase=3 | success=true、error_code=0 |
| Goto | state=3 | 同上 |
| Return | phase=2 | 同上 |
| Land | phase=2 或 3 | 同上；3 表示同时观察到已上锁 |

`error_code=2` 超时，3 取消，19 参数非法，20 后端不支持，21 忙。拒绝请求不会发对应控制命令。完整阶段/状态来源见 [实现说明](README_V1.4_实现与验证.md)。

## 4. 位置保持、恢复与取消

已有轨迹控制在运行、飞机在空中时，HOVER/STOP 均复用位置保持；未观察到目标/轨迹时返回不支持。

```bash
rosservice call /nav/safety/emergency '{cmd: 1}'
rosservice call /nav/safety/emergency '{cmd: 2}'
rosservice call /nav/safety/emergency '{cmd: 3}'
```

分别为保持、保持、当前位置继续。成功响应只确认转发；前两项不代表停桨，第三项不恢复旧航线。

取消当前 Goto 跟踪（也可换成其他 Action 路径）：

```bash
rostopic pub -1 /nav/flight/goto/cancel actionlib_msgs/GoalID '{}'
```

## 5. 数据映射与采集

```bash
rostopic echo -n 1 /nav/data/odom
rostopic echo -n 1 /nav/data/pointcloud/header
rostopic hz /nav/data/pointcloud
rostopic echo -n 1 /nav/data/visual_pointcloud/header
rostopic echo -n 1 /nav/data/visual_pointcloud/width
rostopic hz /nav/data/visual_pointcloud
```

源为 `/drone_2_visual_slam/odom`、`/drone_2_cloud_registered`，frame、协方差、点结构和采样时间原样保留。

`visual_pointcloud` 为同源可视化支路，默认保留 10% 点，frame 仍为源 world；RViz 添加 PointCloud2 选择该话题。修改比例或输入见 [数据映射配置](README_数据映射.md)。此时不再同时启动原独立 `visual_pointcloud_node` 向同一话题发布。

先开采集监控：

```bash
rostopic echo /nav/map/record/feedback
rostopic echo /nav/map/record/result
```

启动采集（要求 SLAM save_result 已运行）：

```bash
rostopic pub -1 /nav/map/record/goal spikive_control_sdk/RecordActionGoal '{goal: {cmd: 1, bag_path: /home/colman/data, bag_name: record_test, timeout: 10}}'
```

收到 success 后采集继续。完成采集后明确发送停止：

```bash
rostopic pub -1 /nav/map/record/goal spikive_control_sdk/RecordActionGoal '{goal: {cmd: 2, bag_path: "", bag_name: "", timeout: 120}}'
```

停止成功后得到 `/home/colman/data/record_test.bag`，可用 `rosbag info /home/colman/data/record_test.bag` 检查。再次采集请换 bag_name；同名文件不会覆盖。采集取消或超时不自动停止 SLAM，仍需 cmd=2。此接口产出 bag，不能直接交给 LoadMap；PCD 后处理属于二阶段第 4 项。

# Spikive-Control-SDK v1.5.0 安装说明

本包为 ROS1 Noetic C++ 源码安装包，可在已配置 Noetic 的 Orin ARM64 或 x86_64 主机上编译。包含 SDK、原样复制的 controller_msgs / quadrotor_msgs 消息依赖、配置、测试和中文文档；不包含 Planner/SLAM 算法、系统库或预编译可执行文件。

## 本版功能

| 功能 | 行为 |
| --- | --- |
| Goto / Return 的 speed>0 | 向 `/drone_{id}_planning/max_vel` 转发规划速度上限，单位 m/s；持续生效 |
| speed<=0 | 沿用当前速度上限，不恢复启动值 |
| Goto 的 yaw_enable=true | 位置稳定到达后发送 yaw；等待新的 pending→turning→done 才成功 |
| yaw 坐标 | 原生 world 绝对航向，单位 rad；0 朝 +X、π/2 朝 +Y，沿用既有航向对齐前提 |

保留 Takeoff、Land、Return、定位、录制和数据映射。消息定义与 1.4.0/1.4.1 相同；v1.5 是包含速度/yaw 功能的正式 SDK 版本，不代表客户接口 PDF 升版。取消/超时仅结束跟踪，不撤回已发的速度或 yaw。

## 安装

目标环境须预装 ROS Noetic、catkin、MAVROS、PCL、OpenSSL、Boost、Eigen3 开发依赖和 C++ 编译器。安装脚本不联网装系统依赖、不使用 sudo、不启动节点。

把压缩包放到主目录，在新终端执行：

```bash
cd ~
unzip Spikive-Control-SDK-v1.5.0.zip
cd Spikive-Control-SDK-v1.5.0
sha256sum -c SHA256SUMS
bash install.sh
source devel/setup.bash
rosversion spikive_control_sdk
```

最后应输出 `1.5.0`。这是独立 catkin 工作区，默认以 2 个编译任务生成本目录的 build/devel，不覆盖原工作区；内存紧张可使用 `SDK_BUILD_JOBS=1 bash install.sh`。不要把整个安装包放进已有工作区的 src，以免与已有两个消息包重名。

后续每个使用终端先执行：

```bash
source ~/Spikive-Control-SDK-v1.5.0/devel/setup.bash
```

需要 C++ 库的使用方可将此 devel 工作区作为 underlay；保留解压后的目录，移动目录后重新编译。安装后的完整说明见 `src/Spikive-Control-SDK/README.md`，飞行命令见 `src/Spikive-Control-SDK/docs/README_命令行测试.md`。

## 启动与后端要求

原系统启动 SLAM、Planner、FlightManager、px4ctrl、MAVROS 后，退出旧 SDK 节点，再启动：

```bash
roslaunch spikive_control_sdk spikive_control_sdk.launch drone_id:=2
```

编号按现场设置。速度和 yaw 需要新版后端的以下接口；本安装包不会替换后端：

| 话题 | 类型 | 后端方向 |
| --- | --- | --- |
| `/drone_{id}_planning/max_vel` | std_msgs/Float64 | Planner 订阅 |
| `/drone_{id}_control/yaw_override` | std_msgs/Float64 | px4ctrl 订阅 |
| `/drone_{id}_control/yaw_override/status` | std_msgs/String | px4ctrl 发布 |

对应配置位于 `src/Spikive-Control-SDK/config/control_sdk.yaml`。后端缺少接口时，指定正速度或启用 yaw 返回 error_code=20；未启用这些能力的请求沿用原有链路。

验证范围：Noetic/MAVROS 隔离容器中的源码构建与测试。既有速度/yaw 隔离联调记录见 `src/Spikive-Control-SDK/docs/README_速度与航向.md`；不等同于 Orin 实机飞行验证。

#pragma once

#include <spikive_control_sdk/config.h>
#include <controller_msgs/DroneState.h>
#include <controller_msgs/cmd.h>
#include <quadrotor_msgs/GoalSet.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/Altitude.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cstdint>
#include <functional>
#include <limits>

namespace spikive_control_sdk
{

// Goto 的位置必须是带时间戳、坐标系匹配且有限的 Planner 同源里程计。
bool validNavigationOdometry(const nav_msgs::Odometry& odom, const std::string& world_frame);

// 消息转发与状态缓存；使用单线程 ros::spin()。
// 直接调用 takeoff()/land()/returnHome()/resume()/goTo() 只发消息，Action 准入、互斥、完成判定由节点负责。
class ControlSdk
{
public:
  using StateCallback = std::function<void(const controller_msgs::DroneState&)>;
  using UpdateCallback = std::function<void()>;

  ControlSdk(ros::NodeHandle& nh, int drone_id);
  ControlSdk(ros::NodeHandle& nh, const Config& config);
  void setStateCallback(StateCallback callback);
  void setUpdateCallback(UpdateCallback callback);

  bool hasDroneState() const { return have_drone_state_; }
  const controller_msgs::DroneState& droneState() const { return drone_state_; }
  bool connected() const { return mavros_state_.connected; }
  const mavros_msgs::State& mavrosState() const { return mavros_state_; }
  bool hasExtendedState() const { return have_extended_state_; }
  const mavros_msgs::ExtendedState& extendedState() const { return extended_state_; }
  bool hasOdometry() const { return have_odometry_; }
  bool hasPosition() const { return have_position_; }
  const nav_msgs::Odometry& odometry() const { return odometry_; }
  bool hasNavigationOdometry() const { return have_navigation_odometry_; }
  const nav_msgs::Odometry& navigationOdometry() const { return navigation_odometry_; }
  double altitudeZ() const { return altitude_z_; }
  float groundClearance() const { return ground_clearance_; }
  // 本类也观察命令/目标，扣除自身连接；自己的订阅不能证明后端在线。
  bool hasControlSubscriber() const { return control_pub_.getNumSubscribers() > 1; }
  bool hasGoalSubscriber() const { return goal_pub_.getNumSubscribers() > 1; }
  bool hasTrajectory() const { return have_trajectory_; }
  std::uint64_t goalSequence() const { return goal_sequence_; }
  const quadrotor_msgs::GoalSet& observedGoal() const { return observed_goal_; }
  const controller_msgs::cmd& observedCommand() const { return observed_command_; }
  bool hasSpeedSubscriber() const { return speed_pub_.getNumSubscribers() > 0; }
  bool hasYawBackend() const
  { return yaw_pub_.getNumSubscribers() > 0 && yaw_status_sub_.getNumPublishers() > 0 && yaw_status_sequence_ > 0; }
  std::uint64_t yawStatusSequence() const { return yaw_status_sequence_; }
  const std::string& yawStatus() const { return yaw_status_; }

  // 各发布一次既有命令，返回发送时间戳；不是控制器 ACK。
  ros::Time takeoff();
  ros::Time land();
  ros::Time returnHome();  // cmd=3，返航点由现有 FlightManager 决定。
  ros::Time resume();  // cmd=4，沿用 WayPoint 的 Continue 协议。
  ros::Time hold();  // cmd=5，沿用已有轨迹分支的位置保持。
  ros::Time goTo(float x, float y, float z);  // 原生局部坐标直接转发给 GoalSet。
  void setPlanningSpeed(double speed);  // 新 Planner 的持续速度上限，不写参数服务器。
  void setArrivalYaw(double yaw);  // 原生世界轴绝对弧度；仅转发一次，不 latch。

private:
  ros::Time sendCommand(std::uint8_t command);
  void notifyUpdate();
  void stateCallback(const controller_msgs::DroneState::ConstPtr& msg);
  void mavrosCallback(const mavros_msgs::State::ConstPtr& msg);
  void extendedCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
  void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void navigationOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Publisher control_pub_;
  ros::Publisher goal_pub_;
  ros::Publisher speed_pub_, yaw_pub_;
  ros::Subscriber yaw_status_sub_;
  std::string yaw_status_;
  std::uint64_t yaw_status_sequence_{0};
  ros::Subscriber state_sub_;
  ros::Subscriber mavros_sub_;
  ros::Subscriber extended_sub_;
  ros::Subscriber altitude_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber navigation_odometry_sub_;
  ros::Subscriber command_observer_, goal_observer_, trajectory_sub_;
  controller_msgs::cmd observed_command_;
  quadrotor_msgs::GoalSet observed_goal_;
  std::uint64_t goal_sequence_{0};
  bool have_trajectory_{false};
  controller_msgs::DroneState drone_state_;
  mavros_msgs::State mavros_state_;
  mavros_msgs::ExtendedState extended_state_;
  nav_msgs::Odometry odometry_;
  nav_msgs::Odometry navigation_odometry_;
  StateCallback state_callback_;
  UpdateCallback update_callback_;
  bool have_drone_state_{false};
  bool have_odometry_{false};
  bool have_position_{false};
  bool have_navigation_odometry_{false};
  bool have_extended_state_{false};
  double altitude_z_{0.0};
  float ground_clearance_{std::numeric_limits<float>::quiet_NaN()};
  int drone_id_;
  std::string world_frame_;
};

}  // namespace spikive_control_sdk

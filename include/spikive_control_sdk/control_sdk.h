#pragma once

#include <spikive_control_sdk/config.h>
#include <controller_msgs/DroneState.h>
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
  double altitudeZ() const { return altitude_z_; }
  float groundClearance() const { return ground_clearance_; }
  bool hasControlSubscriber() const { return control_pub_.getNumSubscribers() > 0; }
  bool hasGoalSubscriber() const { return goal_pub_.getNumSubscribers() > 0; }

  // 各发布一次既有命令，返回发送时间戳；不是控制器 ACK。
  ros::Time takeoff();
  ros::Time land();
  ros::Time returnHome();  // cmd=3，返航点由现有 FlightManager 决定。
  ros::Time resume();  // cmd=4，沿用 WayPoint 的 Continue 协议。
  ros::Time goTo(float x, float y, float z);  // 原生局部坐标直接转发给 GoalSet。

private:
  ros::Time sendCommand(std::uint8_t command);
  void notifyUpdate();
  void stateCallback(const controller_msgs::DroneState::ConstPtr& msg);
  void mavrosCallback(const mavros_msgs::State::ConstPtr& msg);
  void extendedCallback(const mavros_msgs::ExtendedState::ConstPtr& msg);
  void altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg);
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

  ros::Publisher control_pub_;
  ros::Publisher goal_pub_;
  ros::Subscriber state_sub_;
  ros::Subscriber mavros_sub_;
  ros::Subscriber extended_sub_;
  ros::Subscriber altitude_sub_;
  ros::Subscriber odometry_sub_;
  controller_msgs::DroneState drone_state_;
  mavros_msgs::State mavros_state_;
  mavros_msgs::ExtendedState extended_state_;
  nav_msgs::Odometry odometry_;
  StateCallback state_callback_;
  UpdateCallback update_callback_;
  bool have_drone_state_{false};
  bool have_odometry_{false};
  bool have_position_{false};
  bool have_extended_state_{false};
  double altitude_z_{0.0};
  float ground_clearance_{std::numeric_limits<float>::quiet_NaN()};
  int drone_id_;
};

}  // namespace spikive_control_sdk

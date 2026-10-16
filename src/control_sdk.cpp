#include <spikive_control_sdk/control_sdk.h>

#include <controller_msgs/cmd.h>
#include <quadrotor_msgs/GoalSet.h>

#include <cmath>
#include <string>
#include <utility>

namespace spikive_control_sdk
{

ControlSdk::ControlSdk(ros::NodeHandle& nh, int drone_id)
  : ControlSdk(nh, Config::forDrone(drone_id))
{
}

ControlSdk::ControlSdk(ros::NodeHandle& nh, const Config& config)
  : drone_id_(config.drone_id)
{
  config.validate();
  control_pub_ = nh.advertise<controller_msgs::cmd>(config.control_topic, 1, false);
  goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>(config.goal_topic, 1, false);
  state_sub_ = nh.subscribe(config.drone_state_topic, 10, &ControlSdk::stateCallback, this);
  mavros_sub_ = nh.subscribe(config.mavros_state_topic, 10, &ControlSdk::mavrosCallback, this);
  extended_sub_ = nh.subscribe(config.extended_state_topic, 10, &ControlSdk::extendedCallback, this);
  altitude_sub_ = nh.subscribe(config.altitude_topic, 10, &ControlSdk::altitudeCallback, this);
  odometry_sub_ = nh.subscribe(config.odometry_topic, 10, &ControlSdk::odometryCallback, this);
}

void ControlSdk::setStateCallback(StateCallback callback)
{
  state_callback_ = std::move(callback);
}

ros::Time ControlSdk::takeoff()
{
  return sendCommand(1);
}

ros::Time ControlSdk::land()
{
  return sendCommand(2);
}

ros::Time ControlSdk::resume()
{
  return sendCommand(4);
}

ros::Time ControlSdk::goTo(float x, float y, float z)
{
  quadrotor_msgs::GoalSet goal;
  goal.drone_id = static_cast<std::int16_t>(drone_id_);
  goal.goal = {{x, y, z}};
  // GoalSet 没有 Header；本地记录发送时刻，仅用于过滤请求前的状态。
  const auto stamp = ros::Time::now();
  goal_pub_.publish(goal);
  return stamp;
}

ros::Time ControlSdk::sendCommand(std::uint8_t value)
{
  controller_msgs::cmd command;
  command.header.stamp = ros::Time::now();
  command.cmd = value;
  control_pub_.publish(command);
  return command.header.stamp;
}

void ControlSdk::setUpdateCallback(UpdateCallback callback)
{
  update_callback_ = std::move(callback);
}

void ControlSdk::notifyUpdate()
{
  if (update_callback_)
  {
    update_callback_();
  }
}

void ControlSdk::stateCallback(const controller_msgs::DroneState::ConstPtr& msg)
{
  drone_state_ = *msg;
  have_drone_state_ = true;
  if (state_callback_)
  {
    state_callback_(drone_state_);
  }
  notifyUpdate();
}

void ControlSdk::mavrosCallback(const mavros_msgs::State::ConstPtr& msg)
{
  mavros_state_ = *msg;
  notifyUpdate();
}

void ControlSdk::extendedCallback(const mavros_msgs::ExtendedState::ConstPtr& msg)
{
  extended_state_ = *msg;
  have_extended_state_ = true;
  notifyUpdate();
}

void ControlSdk::altitudeCallback(const mavros_msgs::Altitude::ConstPtr& msg)
{
  // MAVLink ALTITUDE.bottom_clearance 为机体下方净空；负值表示没有测量。
  ground_clearance_ = std::isfinite(msg->bottom_clearance) && msg->bottom_clearance >= 0.0f
                       ? msg->bottom_clearance : std::numeric_limits<float>::quiet_NaN();
}

void ControlSdk::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry_ = *msg;
  have_odometry_ = std::isfinite(msg->pose.pose.position.z);
  have_position_ = have_odometry_ && std::isfinite(msg->pose.pose.position.x) &&
                   std::isfinite(msg->pose.pose.position.y);
  if (have_odometry_)
  {
    altitude_z_ = msg->pose.pose.position.z;
  }
  notifyUpdate();
}

}  // namespace spikive_control_sdk

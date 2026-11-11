#include <spikive_control_sdk/control_sdk.h>

#include <controller_msgs/cmd.h>
#include <quadrotor_msgs/GoalSet.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <cmath>
#include <string>
#include <utility>
#include <set>
#include <stdexcept>

namespace spikive_control_sdk
{

bool validNavigationOdometry(const nav_msgs::Odometry& odom, const std::string& world_frame)
{
  const auto& p = odom.pose.pose.position;
  return !world_frame.empty() && odom.header.frame_id == world_frame && !odom.header.stamp.isZero() &&
         std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

ControlSdk::ControlSdk(ros::NodeHandle& nh, int drone_id)
  : ControlSdk(nh, Config::forDrone(drone_id))
{
}

ControlSdk::ControlSdk(ros::NodeHandle& nh, const Config& config)
  : drone_id_(config.drone_id), world_frame_(config.world_frame)
{
  config.validate();
  if (std::set<std::string>{nh.resolveName(config.planning_speed_topic), nh.resolveName(config.yaw_override_topic),
                            nh.resolveName(config.yaw_status_topic)}.size() != 3)
    throw std::invalid_argument("Planning speed/yaw topics resolve to the same name");
  control_pub_ = nh.advertise<controller_msgs::cmd>(config.control_topic, 1, false);
  goal_pub_ = nh.advertise<quadrotor_msgs::GoalSet>(config.goal_topic, 1, false);
  speed_pub_ = nh.advertise<std_msgs::Float64>(config.planning_speed_topic, 1, false);
  yaw_pub_ = nh.advertise<std_msgs::Float64>(config.yaw_override_topic, 1, false);
  yaw_status_sub_ = nh.subscribe<std_msgs::String>(config.yaw_status_topic, 20,
    [this](const std_msgs::String::ConstPtr& msg)
    {
      if (msg->data != "idle" && msg->data != "pending" && msg->data != "turning" && msg->data != "done") return;
      yaw_status_ = msg->data;
      ++yaw_status_sequence_;
      notifyUpdate();
    });
  state_sub_ = nh.subscribe(config.drone_state_topic, 10, &ControlSdk::stateCallback, this);
  mavros_sub_ = nh.subscribe(config.mavros_state_topic, 10, &ControlSdk::mavrosCallback, this);
  extended_sub_ = nh.subscribe(config.extended_state_topic, 10, &ControlSdk::extendedCallback, this);
  altitude_sub_ = nh.subscribe(config.altitude_topic, 10, &ControlSdk::altitudeCallback, this);
  odometry_sub_ = nh.subscribe(config.odometry_topic, 10, &ControlSdk::odometryCallback, this);
  navigation_odometry_sub_ = nh.subscribe(config.navigation_odometry_topic, 10,
                                         &ControlSdk::navigationOdometryCallback, this);
  command_observer_ = nh.subscribe<controller_msgs::cmd>(config.control_topic, 10,
    [this](const controller_msgs::cmd::ConstPtr& msg)
    {
      observed_command_ = *msg;
      notifyUpdate();
    });
  goal_observer_ = nh.subscribe<quadrotor_msgs::GoalSet>(config.goal_topic, 10,
    [this](const quadrotor_msgs::GoalSet::ConstPtr& msg)
    {
      if (msg->drone_id != drone_id_ || msg->goal[2] < -0.1) return;
      for (float v : msg->goal) if (!std::isfinite(v)) return;
      observed_goal_ = *msg;
      ++goal_sequence_;
      notifyUpdate();
    });
  trajectory_sub_ = nh.subscribe<quadrotor_msgs::PositionCommand>(config.trajectory_topic, 10,
    [this](const quadrotor_msgs::PositionCommand::ConstPtr& msg)
    {
      have_trajectory_ = std::isfinite(msg->position.x) && std::isfinite(msg->position.y) &&
                         std::isfinite(msg->position.z);
    });
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

ros::Time ControlSdk::returnHome()
{
  return sendCommand(3);
}

ros::Time ControlSdk::resume()
{
  return sendCommand(4);
}

ros::Time ControlSdk::hold()
{
  return sendCommand(5);
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

void ControlSdk::setPlanningSpeed(double speed)
{
  if (!std::isfinite(speed) || speed <= 0) throw std::invalid_argument("Planning speed must be finite and positive");
  std_msgs::Float64 msg;
  msg.data = speed;
  speed_pub_.publish(msg);
}

void ControlSdk::setArrivalYaw(double yaw)
{
  if (!std::isfinite(yaw)) throw std::invalid_argument("Arrival yaw must be finite");
  std_msgs::Float64 msg;
  msg.data = yaw;
  yaw_pub_.publish(msg);
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

void ControlSdk::navigationOdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  navigation_odometry_ = *msg;
  have_navigation_odometry_ = validNavigationOdometry(*msg, world_frame_);
  notifyUpdate();
}

}  // namespace spikive_control_sdk

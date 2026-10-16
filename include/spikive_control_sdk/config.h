#pragma once

#include <ros/ros.h>

#include <string>

namespace spikive_control_sdk
{

// 配置通信、反馈和到达状态判据；不修改飞控高度、速度或超时策略。
struct Config
{
  int drone_id{1};
  double feedback_rate{10.0};
  std::string control_topic{"/control"};
  std::string goal_topic{"/goal_with_id"};
  std::string drone_state_topic{"/drone_1_state"};
  std::string mavros_state_topic{"/mavros/state"};
  std::string extended_state_topic{"/mavros/extended_state"};
  std::string odometry_topic{"/mavros/local_position/odom"};
  std::string altitude_topic{"/mavros/altitude"};
  std::string takeoff_action{"/nav/flight/takeoff"};
  std::string land_action{"/nav/flight/land"};
  std::string goto_action{"/nav/flight/goto"};
  std::string return_home_action{"/nav/flight/return_home"};
  double arrival_total_dist{0.30};
  double arrival_horiz_dist{0.25};
  double stable_time{1.0};

  std::string load_map_service{"/nav/localization/load_map"};
  std::string relocalize_action{"/nav/localization/relocalize"};
  std::string localization_status_topic{"/nav/status/localization"};
  std::string localization_map_topic{"/drone_1_localization_pcl"};
  std::string initial_pose_topic{"/drone_1_initialpose"};
  std::string match_status_topic{"/drone_1_localization_match_status"};
  std::string match_transform_topic{"/drone_1_diff_odom"};
  std::string map_echo_topic{"/map"};
  double map_ack_timeout{5.0};  // 仅 LoadMap Service 的确认等待，不是飞行任务超时。
  double localization_status_rate{10.0};

  static Config forDrone(int drone_id);
  static Config load(const ros::NodeHandle& private_nh);
  void validate() const;
};

}  // namespace spikive_control_sdk

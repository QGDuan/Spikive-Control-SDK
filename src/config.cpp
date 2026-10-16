#include <spikive_control_sdk/config.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <set>
#include <stdexcept>

namespace spikive_control_sdk
{

Config Config::forDrone(int drone_id)
{
  Config config;
  config.drone_id = drone_id;
  config.drone_state_topic = "/drone_" + std::to_string(drone_id) + "_state";
  const auto prefix = "/drone_" + std::to_string(drone_id);
  config.navigation_odometry_topic = prefix + "_visual_slam/odom";
  config.localization_map_topic = prefix + "_localization_pcl";
  config.initial_pose_topic = prefix + "_initialpose";
  config.match_status_topic = prefix + "_localization_match_status";
  config.match_transform_topic = prefix + "_diff_odom";
  return config;
}

Config Config::load(const ros::NodeHandle& private_nh)
{
  int drone_id = 1;
  private_nh.param("drone_id", drone_id, drone_id);
  Config config = forDrone(drone_id);
  private_nh.param("feedback_rate", config.feedback_rate, config.feedback_rate);
  private_nh.param("topics/control", config.control_topic, config.control_topic);
  private_nh.param("topics/goal", config.goal_topic, config.goal_topic);
  private_nh.param("topics/drone_state", config.drone_state_topic, config.drone_state_topic);
  if (config.drone_state_topic.empty())
  {
    config.drone_state_topic = forDrone(drone_id).drone_state_topic;
  }
  private_nh.param("topics/mavros_state", config.mavros_state_topic, config.mavros_state_topic);
  private_nh.param("topics/extended_state", config.extended_state_topic, config.extended_state_topic);
  private_nh.param("topics/odometry", config.odometry_topic, config.odometry_topic);
  private_nh.param("frames/world", config.world_frame, config.world_frame);
  private_nh.param("topics/altitude", config.altitude_topic, config.altitude_topic);
  private_nh.param("actions/takeoff", config.takeoff_action, config.takeoff_action);
  private_nh.param("actions/land", config.land_action, config.land_action);
  private_nh.param("actions/goto", config.goto_action, config.goto_action);
  private_nh.param("actions/return_home", config.return_home_action, config.return_home_action);
  private_nh.param("goto/arrival_total_dist", config.arrival_total_dist, config.arrival_total_dist);
  private_nh.param("goto/arrival_horiz_dist", config.arrival_horiz_dist, config.arrival_horiz_dist);
  private_nh.param("goto/stable_time", config.stable_time, config.stable_time);
  const auto defaults = forDrone(drone_id);
  const auto load_topic = [&private_nh](const std::string& key, std::string& value, const std::string& fallback)
  {
    private_nh.param(key, value, value);
    if (value.empty()) value = fallback;
  };
  load_topic("localization/map_topic", config.localization_map_topic, defaults.localization_map_topic);
  load_topic("topics/navigation_odometry", config.navigation_odometry_topic, defaults.navigation_odometry_topic);
  load_topic("localization/initial_pose_topic", config.initial_pose_topic, defaults.initial_pose_topic);
  load_topic("localization/match_status_topic", config.match_status_topic, defaults.match_status_topic);
  load_topic("localization/match_transform_topic", config.match_transform_topic, defaults.match_transform_topic);
  private_nh.param("localization/map_echo_topic", config.map_echo_topic, config.map_echo_topic);
  private_nh.param("localization/load_map_service", config.load_map_service, config.load_map_service);
  private_nh.param("localization/relocalize_action", config.relocalize_action, config.relocalize_action);
  private_nh.param("localization/status_topic", config.localization_status_topic, config.localization_status_topic);
  private_nh.param("localization/map_ack_timeout", config.map_ack_timeout, config.map_ack_timeout);
  private_nh.param("localization/status_rate", config.localization_status_rate, config.localization_status_rate);
  config.validate();
  return config;
}

void Config::validate() const
{
  if (drone_id < 0 || drone_id > std::numeric_limits<std::int16_t>::max())
  {
    throw std::invalid_argument("drone_id must fit GoalSet.int16: 0..32767");
  }
  for (double value : {feedback_rate, arrival_total_dist, arrival_horiz_dist, stable_time, map_ack_timeout, localization_status_rate})
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("Rates, arrival defaults and map acknowledgement timeout must be finite and positive");
    }
  }
  for (const auto* name : {&control_topic, &goal_topic, &drone_state_topic, &mavros_state_topic,
                           &extended_state_topic, &odometry_topic, &navigation_odometry_topic, &altitude_topic,
                           &takeoff_action, &land_action, &goto_action, &return_home_action,
                           &load_map_service, &relocalize_action,
                           &localization_status_topic, &localization_map_topic, &initial_pose_topic,
                           &match_status_topic, &match_transform_topic, &map_echo_topic})
  {
    if (name->empty())
    {
      throw std::invalid_argument("Topic and action names must not be empty");
    }
  }
  if (world_frame.empty())
    throw std::invalid_argument("frames/world must not be empty; use the native WayPoint/Planner frame");
  if (std::set<std::string>{takeoff_action, land_action, goto_action, return_home_action, relocalize_action}.size() != 5)
  {
    throw std::invalid_argument("Actions must use different names");
  }
  if (localization_map_topic == map_echo_topic || match_status_topic == localization_status_topic)
    throw std::invalid_argument("Localization input and output topics must differ");
}

}  // namespace spikive_control_sdk

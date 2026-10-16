#include <spikive_control_sdk/config.h>

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace spikive_control_sdk
{

Config Config::forDrone(int drone_id)
{
  Config config;
  config.drone_id = drone_id;
  config.drone_state_topic = "/drone_" + std::to_string(drone_id) + "_state";
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
  private_nh.param("topics/altitude", config.altitude_topic, config.altitude_topic);
  private_nh.param("actions/takeoff", config.takeoff_action, config.takeoff_action);
  private_nh.param("actions/land", config.land_action, config.land_action);
  private_nh.param("actions/goto", config.goto_action, config.goto_action);
  private_nh.param("goto/arrival_total_dist", config.arrival_total_dist, config.arrival_total_dist);
  private_nh.param("goto/arrival_horiz_dist", config.arrival_horiz_dist, config.arrival_horiz_dist);
  private_nh.param("goto/stable_time", config.stable_time, config.stable_time);
  config.validate();
  return config;
}

void Config::validate() const
{
  if (drone_id < 0 || drone_id > std::numeric_limits<std::int16_t>::max())
  {
    throw std::invalid_argument("drone_id must fit GoalSet.int16: 0..32767");
  }
  for (double value : {feedback_rate, arrival_total_dist, arrival_horiz_dist, stable_time})
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument("Feedback rate and goto arrival defaults must be finite and positive");
    }
  }
  for (const auto* name : {&control_topic, &goal_topic, &drone_state_topic, &mavros_state_topic,
                           &extended_state_topic, &odometry_topic, &altitude_topic,
                           &takeoff_action, &land_action, &goto_action})
  {
    if (name->empty())
    {
      throw std::invalid_argument("Topic and action names must not be empty");
    }
  }
  if (takeoff_action == land_action || takeoff_action == goto_action || land_action == goto_action)
  {
    throw std::invalid_argument("Flight actions must use different names");
  }
}

}  // namespace spikive_control_sdk

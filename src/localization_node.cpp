#include <spikive_control_sdk/config.h>
#include <spikive_control_sdk/localization_sdk.h>
#include <exception>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spikive_localization_node");
  ros::NodeHandle nh, pnh("~");
  try
  {
    const auto config = spikive_control_sdk::Config::load(pnh);
    spikive_control_sdk::LocalizationSdk node(nh, config);
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("Localization SDK: " << e.what());
    return 1;
  }
  return 0;
}

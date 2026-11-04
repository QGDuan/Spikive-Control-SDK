#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include "visual_pointcloud.h"
#include <cmath>
#include <set>
#include <stdexcept>
#include <vector>

// 原生数据映射 + 可视化抽稀支路；不修改源消息或 Planner/定位输入。
class DataMappingNode
{
public:
  DataMappingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    int id = 1;
    pnh.param("drone_id", id, id);
    if (id < 0 || id > 32767) throw std::invalid_argument("drone_id out of range");
    const auto prefix = "/drone_" + std::to_string(id);
    std::string odom_in, cloud_in, odom_out, cloud_out, visual_in, visual_out;
    pnh.param<std::string>("topics/navigation_odometry", odom_in, "");
    pnh.param<std::string>("data/pointcloud_source", cloud_in, "");
    if (odom_in.empty()) odom_in = prefix + "_visual_slam/odom";
    if (cloud_in.empty()) cloud_in = prefix + "_cloud_registered";
    pnh.param<std::string>("data/odometry", odom_out, "/nav/data/odom");
    pnh.param<std::string>("data/pointcloud", cloud_out, "/nav/data/pointcloud");
    pnh.param("visual_pointcloud/enabled", visual_enabled_, true);
    pnh.param("visual_pointcloud/keep_ratio", keep_ratio_, 0.1);
    pnh.param<std::string>("visual_pointcloud/input_topic", visual_in, "");
    pnh.param<std::string>("visual_pointcloud/output_topic", visual_out, "/nav/data/visual_pointcloud");
    if (visual_in.empty()) visual_in = cloud_in;
    if (visual_enabled_ && (!std::isfinite(keep_ratio_) || keep_ratio_ <= 0.0 || keep_ratio_ > 1.0))
      throw std::invalid_argument("visual_pointcloud/keep_ratio must be in (0, 1]");
    std::set<std::string> sources{nh.resolveName(odom_in), nh.resolveName(cloud_in)};
    if (visual_enabled_) sources.insert(nh.resolveName(visual_in));
    std::set<std::string> outputs;
    std::vector<std::string> output_topics{odom_out, cloud_out};
    if (visual_enabled_) output_topics.push_back(visual_out);
    for (const auto& topic : output_topics)
    {
      if (topic.empty() || sources.count(nh.resolveName(topic)))
        throw std::invalid_argument("Data output topics must be nonempty and distinct from inputs");
      outputs.insert(nh.resolveName(topic));
    }
    if (outputs.size() != (visual_enabled_ ? 3u : 2u))
      throw std::invalid_argument("Data output topics must be distinct from each other");
    ros::AdvertiseOptions odom_options, cloud_options;
    odom_options.init<nav_msgs::Odometry>(odom_out, 10);
    cloud_options.init<sensor_msgs::PointCloud2>(cloud_out, 2);
    // roscpp 默认会重写 Header.seq；映射支路禁用该行为以保留源序号。
    odom_options.has_header = cloud_options.has_header = false;
    odom_pub_ = nh.advertise(odom_options);
    cloud_pub_ = nh.advertise(cloud_options);
    if (visual_enabled_)
    {
      ros::AdvertiseOptions options;
      options.init<sensor_msgs::PointCloud2>(visual_out, 1);
      options.has_header = false;
      visual_pub_ = nh.advertise(options);
      ROS_INFO_STREAM("Visual pointcloud: " << nh.resolveName(visual_in) << " -> "
                      << nh.resolveName(visual_out) << ", keep_ratio=" << keep_ratio_);
    }
    const bool shared_cloud = nh.resolveName(visual_in) == nh.resolveName(cloud_in);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_in, 10,
      [this](const nav_msgs::Odometry::ConstPtr& m) { odom_pub_.publish(m); });
    cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(cloud_in, 2,
      [this, shared_cloud](const sensor_msgs::PointCloud2::ConstPtr& m)
      {
        cloud_pub_.publish(m);
        if (visual_enabled_ && shared_cloud) publishVisual(m);
      });
    if (visual_enabled_ && !shared_cloud)
      visual_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(visual_in, 1, &DataMappingNode::publishVisual, this);
    ROS_INFO_STREAM("Native data mapping: " << odom_in << " -> " << odom_out << "; " << cloud_in << " -> " << cloud_out);
  }
private:
  void publishVisual(const sensor_msgs::PointCloud2::ConstPtr& cloud)
  {
    if (!visual_pub_.getNumSubscribers()) return;
    try { visual_pub_.publish(spikive_control_sdk::visualPointCloud(cloud, keep_ratio_)); }
    catch (const std::exception& e) { ROS_WARN_STREAM_THROTTLE(5, "Visual pointcloud dropped: " << e.what()); }
  }

  bool visual_enabled_{true};
  double keep_ratio_{0.1};
  ros::Publisher odom_pub_, cloud_pub_, visual_pub_;
  ros::Subscriber odom_sub_, cloud_sub_, visual_sub_;
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "spikive_data_mapping_node");
  ros::NodeHandle nh, pnh("~");
  try { DataMappingNode node(nh, pnh); ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}

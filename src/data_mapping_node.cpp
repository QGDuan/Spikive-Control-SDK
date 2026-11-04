#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdexcept>

// 只改话题名：消息对象、Header、frame、协方差、点布局及采样时间原样转发。
class DataMappingNode
{
public:
  DataMappingNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  {
    int id = 1;
    pnh.param("drone_id", id, id);
    if (id < 0 || id > 32767) throw std::invalid_argument("drone_id out of range");
    const auto prefix = "/drone_" + std::to_string(id);
    std::string odom_in, cloud_in, odom_out, cloud_out;
    pnh.param<std::string>("topics/navigation_odometry", odom_in, "");
    pnh.param<std::string>("data/pointcloud_source", cloud_in, "");
    if (odom_in.empty()) odom_in = prefix + "_visual_slam/odom";
    if (cloud_in.empty()) cloud_in = prefix + "_cloud_registered";
    pnh.param<std::string>("data/odometry", odom_out, "/nav/data/odom");
    pnh.param<std::string>("data/pointcloud", cloud_out, "/nav/data/pointcloud");
    if (odom_out.empty() || cloud_out.empty() || nh.resolveName(odom_in) == nh.resolveName(odom_out) ||
        nh.resolveName(cloud_in) == nh.resolveName(cloud_out) || nh.resolveName(odom_out) == nh.resolveName(cloud_out))
      throw std::invalid_argument("Data output topics must be nonempty and distinct from inputs/each other");
    ros::AdvertiseOptions odom_options, cloud_options;
    odom_options.init<nav_msgs::Odometry>(odom_out, 10);
    cloud_options.init<sensor_msgs::PointCloud2>(cloud_out, 2);
    // roscpp 默认会重写 Header.seq；映射支路禁用该行为以保留源序号。
    odom_options.has_header = cloud_options.has_header = false;
    odom_pub_ = nh.advertise(odom_options);
    cloud_pub_ = nh.advertise(cloud_options);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_in, 10,
      [this](const nav_msgs::Odometry::ConstPtr& m) { odom_pub_.publish(m); });
    cloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(cloud_in, 2,
      [this](const sensor_msgs::PointCloud2::ConstPtr& m) { cloud_pub_.publish(m); });
    ROS_INFO_STREAM("Native data mapping: " << odom_in << " -> " << odom_out << "; " << cloud_in << " -> " << cloud_out);
  }
private:
  ros::Publisher odom_pub_, cloud_pub_;
  ros::Subscriber odom_sub_, cloud_sub_;
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "spikive_data_mapping_node");
  ros::NodeHandle nh, pnh("~");
  try { DataMappingNode node(nh, pnh); ros::spin(); }
  catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
  return 0;
}

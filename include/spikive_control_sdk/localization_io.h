#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <cstdint>
#include <string>

namespace spikive_control_sdk
{
struct LoadedMap
{
  sensor_msgs::PointCloud2 cloud;
  std::string md5;
};

struct MatchStatus
{
  std::string state;
  std::string message;
  double fitness_score{-1.0};
  double threshold{0.0};
  double elapsed_time{0.0};
  std::uint32_t attempt_count{0};
};

// 本地文件校验及后端协议解析；不触碰 ROS master 或飞行控制。
LoadedMap readLocalizationMap(const std::string& path, const std::string& expected_md5);
MatchStatus parseMatchStatus(const std::string& json, int drone_id);
}  // namespace spikive_control_sdk

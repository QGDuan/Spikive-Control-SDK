#pragma once

#include <sensor_msgs/PointCloud2.h>

namespace spikive_control_sdk
{
// 沿用 visual_pointcloud_node 的按点序均匀抽稀；复制完整点记录，不转换字段或坐标。
// 节点内部工具：非法比例/布局抛出异常；比例 1 或空云返回原消息。
sensor_msgs::PointCloud2ConstPtr visualPointCloud(const sensor_msgs::PointCloud2ConstPtr& input,
                                               double keep_ratio);
}  // namespace spikive_control_sdk

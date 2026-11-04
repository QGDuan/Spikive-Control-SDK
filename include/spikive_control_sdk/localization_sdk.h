#pragma once

#include <spikive_control_sdk/config.h>
#include <memory>
#include <ros/ros.h>

namespace spikive_control_sdk
{
// 独立定位节点中的适配器。匹配计算仍在 Spikive-SLAM global_match 中。
// 普通回调沿用单线程；仅 LoadMap Service 用独立队列，避免读文件/等待确认阻塞定位状态回调。
class LocalizationSdk
{
public:
  LocalizationSdk(ros::NodeHandle& nh, const Config& config);
  ~LocalizationSdk();
  LocalizationSdk(const LocalizationSdk&) = delete;
  LocalizationSdk& operator=(const LocalizationSdk&) = delete;
private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace spikive_control_sdk

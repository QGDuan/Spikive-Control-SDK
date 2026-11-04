#pragma once
#include <ros/ros.h>
#include <cmath>
#include <cstdint>
#include <string>

namespace spikive_control_sdk
{
enum ErrorCode : std::uint8_t
{
  OK = 0, ERR_UNKNOWN = 1, ERR_TIMEOUT = 2, ERR_CANCELLED = 3,
  ERR_POSE_INVALID = 4, ERR_NOT_TAKEOFF = 6, ERR_ALREADY_FLYING = 7,
  ERR_PRECHECK_FAILED = 12, ERR_PARAM_INVALID = 19, ERR_NOT_SUPPORTED = 20,
  ERR_BUSY = 21, ERR_INTERNAL = 99
};

template<class Result>
Result makeResult(bool success, ErrorCode code, const std::string& message)
{
  Result result;
  result.success = success;
  result.error_code = code;
  result.message = message;
  return result;
}

// 不依赖 /clock：仿真暂停、ROS 时间回跳也不会阻止业务超时。
class TaskDeadline
{
public:
  static bool valid(double seconds) { return std::isfinite(seconds) && seconds >= 0.0; }
  void start(double seconds) { seconds_ = seconds; start_ = ros::SteadyTime::now(); }
  bool expired() const { return seconds_ > 0 && (ros::SteadyTime::now() - start_).toSec() >= seconds_; }
private:
  double seconds_{0};
  ros::SteadyTime start_;
};
}  // namespace spikive_control_sdk

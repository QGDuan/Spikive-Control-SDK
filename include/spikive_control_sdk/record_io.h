#pragma once
#include <atomic>
#include <string>

namespace spikive_control_sdk
{
struct RecordStatus
{
  std::string request_id, phase, state, bag_path, error;
  bool ok{false};
};
RecordStatus parseRecordStatus(const std::string& json, int drone_id);
std::string recordCommand(const std::string& request_id);
std::string recordingDestination(const std::string& directory, const std::string& name);
// SLAM 已关闭的 bag → 请求路径。保留源文件；目标已存在则拒绝，取消不留半成品。
void exportRecordedBag(const std::string& source, const std::string& destination,
                       const std::atomic_bool& cancelled);
}  // namespace spikive_control_sdk

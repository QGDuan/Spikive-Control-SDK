#pragma once
#include <string>
#include <vector>

namespace spikive_control_sdk
{
// 只接受明确的绝对话题，不解析 shell、正则、remap 或隐式 drone_id。
void validateRecordingTopics(const std::vector<std::string>& topics);
std::string recordingDestination(const std::string& directory, const std::string& name);
// 同盘私有暂存目录中的已关闭 bag → 最终路径；原子发布，绝不覆盖。
void publishRecordedBag(const std::string& source, const std::string& destination);
}  // namespace spikive_control_sdk

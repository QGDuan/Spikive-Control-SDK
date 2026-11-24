#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

namespace spikive_control_sdk
{
constexpr char DEFAULT_RECORD_OUTPUT_DIRECTORY[] = "~/bags";

// 只接受明确的绝对话题，不解析 shell、正则、remap 或隐式 drone_id。
void validateRecordingTopics(const std::vector<std::string>& topics);
std::string recordingDestination(const std::string& directory, const std::string& name);
// 已关闭的同盘 bag → 最终路径；原子发布，绝不覆盖。
void publishRecordedBag(const std::string& source, const std::string& destination);

struct RecordingMergeProgress
{
  std::atomic<std::uint32_t> completed{0}, total{0};
  std::atomic<bool> cancelled{false};
};
void mergeRecordedParts(const std::string& recording_prefix, const std::string& staging,
                        const std::string& destination,
                        RecordingMergeProgress& progress);
}  // namespace spikive_control_sdk

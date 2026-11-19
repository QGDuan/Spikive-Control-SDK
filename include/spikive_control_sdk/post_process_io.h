#pragma once

#include <spikive_control_sdk/action_support.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace spikive_control_sdk
{
// Post-process input/output errors are mapped to the SDK action error codes.
class PostProcessError : public std::runtime_error
{
public:
  PostProcessError(ErrorCode code, const std::string& message)
    : std::runtime_error(message), code_(code)
  {
  }

  ErrorCode code() const noexcept { return code_; }

private:
  ErrorCode code_;
};

struct PostProcessRunOptions
{
  std::string pointcloud_topic;
  std::uint64_t max_points{0};
};

struct PostProcessRunRequest
{
  std::string bag_path;
  std::string output_path;
  PostProcessRunOptions options;
};

struct PostProcessControl
{
  const std::atomic<bool>* cancelled{nullptr};
  std::function<bool()> expired;
};

struct PostProcessOutcome
{
  bool success{false};
  ErrorCode code{ERR_INTERNAL};
  std::string message;
  std::string output_path;
  std::string partial_path;
  std::uint64_t messages{0};
  std::uint64_t points{0};
  std::string frame_id;
};

// Expand a leading "~" from $HOME. Other paths are returned unchanged.
std::string expandHome(const std::string& path);

// Resolve the input bag from an optional directory/name pair. With no name the
// newest regular *.bag in the directory is selected. Rejects ambiguous or
// still-active recordings and returns the full absolute bag path.
std::string resolveInputBag(const std::string& bag_path, const std::string& bag_name,
                            const std::string& input_directory);

// Resolve the final *.pcd path. Empty map_path uses default_directory and an
// empty map_name uses bag_stem. Existing outputs and malformed names are
// rejected; the parent directory is created by ensureOutputDirectory().
std::string resolveOutputPath(const std::string& map_path, const std::string& map_name,
                              const std::string& default_directory,
                              const std::string& bag_stem);

void ensureOutputDirectory(const std::string& directory);

// Exact node-name match after adding a leading slash; used for the LIO gate.
bool hasForbiddenNode(const std::vector<std::string>& nodes,
                      const std::vector<std::string>& forbidden);

// Read every PointCloud2 on the configured topic, append all points while
// preserving the original field layout, and publish one binary PCD.
// Throws PostProcessError with the SDK error code on failure.
PostProcessOutcome runPostProcess(const PostProcessRunRequest& request,
                                  const PostProcessControl& control);

}  // namespace spikive_control_sdk

#include <spikive_control_sdk/record_io.h>
#include <boost/filesystem.hpp>
#include <ros/names.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <set>
#include <stdexcept>
#include <unistd.h>

namespace spikive_control_sdk
{
void validateRecordingTopics(const std::vector<std::string>& topics)
{
  if (topics.empty()) throw std::invalid_argument("cmd=1 requires a nonempty topics list");
  std::set<std::string> seen;
  for (const auto& topic : topics)
  {
    std::string reason;
    if (topic.empty() || topic.front() != '/' || topic == "/" ||
        !ros::names::validate(topic, reason) || ros::names::clean(topic) != topic ||
        !seen.insert(topic).second)
      throw std::invalid_argument("topics must be unique absolute ROS names (no regex/remap): " + topic);
  }
}

std::string recordingDestination(const std::string& directory, const std::string& name)
{
  namespace fs = boost::filesystem;
  const fs::path dir(directory), stem(name);
  if (!dir.is_absolute() || name.empty() || stem.has_parent_path() || stem.has_extension() ||
      name == "." || name == ".." || name.find('\0') != std::string::npos || directory.find('\0') != std::string::npos)
    throw std::invalid_argument("bag_path must be an absolute directory; bag_name must have no path/extension");
  fs::create_directories(dir);
  if (!fs::is_directory(dir)) throw std::invalid_argument("bag_path is not a directory");
  const auto destination = (fs::canonical(dir) / (name + ".bag")).string();
  if (fs::exists(destination) || fs::is_symlink(destination))
    throw std::invalid_argument("Destination bag already exists; choose a new name");
  return destination;
}

void publishRecordedBag(const std::string& source, const std::string& destination)
{
  namespace fs = boost::filesystem;
  if (!fs::path(source).is_absolute() || fs::path(source).extension() != ".bag" ||
      !fs::is_regular_file(source) || fs::is_symlink(source))
    throw std::runtime_error("Recorder did not close its bag: " + source);
  std::ifstream input(source, std::ios::binary);
  char magic[13];
  if (!input.read(magic, sizeof(magic)) || std::string(magic, sizeof(magic)) != "#ROSBAG V2.0\n")
    throw std::runtime_error("Recorder output is not a ROS bag v2");
  // 暂存目录就在目标目录内；硬链接为原子、不覆盖的发布操作，无需复制大文件。
  if (::link(source.c_str(), destination.c_str()) != 0)
    throw std::runtime_error("Cannot publish bag without overwriting: " + std::string(std::strerror(errno)));
}
}  // namespace spikive_control_sdk

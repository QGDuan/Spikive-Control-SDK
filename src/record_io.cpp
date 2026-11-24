#include <spikive_control_sdk/record_io.h>
#include <boost/filesystem.hpp>
#include <rosbag/view.h>
#include <ros/names.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <map>
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
  // 同盘硬链接为原子、不覆盖的发布操作，无需复制大文件。
  if (::link(source.c_str(), destination.c_str()) != 0)
    throw std::runtime_error("Cannot publish bag without overwriting: " + std::string(std::strerror(errno)));
}

void mergeRecordedParts(const std::string& recording_prefix, const std::string& staging,
                        const std::string& destination,
                        RecordingMergeProgress& progress)
{
  namespace fs = boost::filesystem;
  const auto prefix = fs::path(recording_prefix).filename().string() + "_";
  std::map<unsigned long, fs::path> parts;
  for (const auto& entry : fs::directory_iterator(fs::path(recording_prefix).parent_path()))
  {
    const auto name = entry.path().filename().string();
    if (name.compare(0, prefix.size(), prefix) != 0) continue;
    if (entry.path().extension() == ".active")
      throw std::runtime_error("Recorder left an unclosed part: " + name);
    if (entry.path().extension() != ".bag") continue;
    const auto index = entry.path().stem().string().substr(prefix.size());
    if (index.empty() || index.find_first_not_of("0123456789") != std::string::npos ||
        !fs::is_regular_file(entry.path()) || fs::is_symlink(entry.path()) ||
        !parts.emplace(std::stoul(index), entry.path()).second)
      throw std::runtime_error("Invalid recording part: " + name);
  }
  if (parts.empty()) throw std::runtime_error("No closed recording parts");
  unsigned long expected_index = 0;
  for (const auto& part : parts)
    if (part.first != expected_index++) throw std::runtime_error("A recording part is missing");
  progress.total = static_cast<std::uint32_t>(parts.size());

  // Keep incomplete merge output apart from source segments for offline recovery.
  const auto merge_dir = fs::path(staging) / ".merge";
  const auto merged = (merge_dir / fs::path(destination).filename()).string();
  if (parts.size() == 1)
  {
    const auto path = parts.begin()->second.string();
    rosbag::Bag input(path, rosbag::bagmode::Read);
    input.close();
    if (progress.cancelled) throw std::runtime_error("Merge interrupted");
    publishRecordedBag(path, destination);
    progress.completed = 1;
  }
  else
  {
    fs::create_directory(merge_dir);
    rosbag::Bag output(merged, rosbag::bagmode::Write);
    std::uint64_t written = 0;
    for (const auto& part : parts)
    {
      if (progress.cancelled) throw std::runtime_error("Merge interrupted");
      rosbag::Bag input(part.second.string(), rosbag::bagmode::Read);
      for (const auto& message : rosbag::View(input))
      {
        if (progress.cancelled) throw std::runtime_error("Merge interrupted");
        output.write(message.getTopic(), message.getTime(), message, message.getConnectionHeader());
        ++written;
      }
      ++progress.completed;
    }
    output.close();
    rosbag::Bag check(merged, rosbag::bagmode::Read);
    if (rosbag::View(check).size() != written)
      throw std::runtime_error("Merged bag message count mismatch");
    check.close();
    if (progress.cancelled) throw std::runtime_error("Merge interrupted");
    publishRecordedBag(merged, destination);
  }
  boost::system::error_code ignored;
  for (const auto& part : parts) fs::remove(part.second, ignored);
  fs::remove(merged, ignored);
  fs::remove(merge_dir, ignored);
  fs::remove(staging, ignored);
}
}  // namespace spikive_control_sdk

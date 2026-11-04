#include <spikive_control_sdk/record_io.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/filesystem.hpp>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace spikive_control_sdk
{
RecordStatus parseRecordStatus(const std::string& json, int drone_id)
{
  boost::property_tree::ptree tree;
  std::istringstream input(json);
  boost::property_tree::read_json(input, tree);
  if (tree.get<int>("schema_version") != 1 || tree.get<std::string>("drone_id") != std::to_string(drone_id))
    throw std::invalid_argument("Recording status schema/drone_id mismatch");
  RecordStatus out;
  out.request_id = tree.get<std::string>("request_id");
  out.phase = tree.get<std::string>("phase");
  out.state = tree.get<std::string>("state");
  out.ok = tree.get<bool>("ok");
  out.bag_path = tree.get<std::string>("bag_path");
  out.error = tree.get<std::string>("error.message", "Backend recording failed");
  if (out.state != "idle" && out.state != "recording")
    throw std::invalid_argument("Unknown backend recording state");
  return out;
}

std::string recordCommand(const std::string& request_id)
{
  // IDs 仅由 SDK 生成；仍使用 JSON 序列化处理转义。
  boost::property_tree::ptree tree;
  tree.put("schema_version", 1);
  tree.put("request_id", request_id);
  std::ostringstream out;
  boost::property_tree::write_json(out, tree, false);
  // property_tree 把数字写成字符串，而 save_result 要求 schema_version 为数字。
  auto json = out.str();
  const auto key = json.find("\"schema_version\":\"1\"");
  if (key == std::string::npos) throw std::runtime_error("Cannot encode recorder schema");
  json.replace(key, std::string("\"schema_version\":\"1\"").size(), "\"schema_version\":1");
  return json;
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

void exportRecordedBag(const std::string& source, const std::string& destination,
                       const std::atomic_bool& cancelled)
{
  namespace fs = boost::filesystem;
  if (!fs::path(source).is_absolute() || fs::path(source).extension() != ".bag" ||
      (source.size() >= 11 && source.substr(source.size() - 11) == ".active.bag") || !fs::is_regular_file(source))
    throw std::runtime_error("Backend did not provide a closed local bag: " + source);
  const int input = ::open(source.c_str(), O_RDONLY);
  if (input < 0) throw std::runtime_error("Cannot open backend bag: " + std::string(std::strerror(errno)));
  int output = -1;
  std::string temporary;
  try
  {
    char magic[13];
    if (::read(input, magic, sizeof(magic)) != sizeof(magic) || std::string(magic, sizeof(magic)) != "#ROSBAG V2.0\n")
      throw std::runtime_error("Backend file is not a ROS bag v2");
    if (source == destination) return static_cast<void>(::close(input));
    if (cancelled) throw std::runtime_error("Bag export cancelled; backend file retained");
    // 同盘硬链接原子发布，不重复复制大 bag；EEXIST 时始终拒绝覆盖。
    if (::link(source.c_str(), destination.c_str()) == 0) return static_cast<void>(::close(input));
    if (errno == EEXIST) throw std::runtime_error("Destination bag already exists");
    std::vector<char> pattern(destination.begin(), destination.end());
    const std::string suffix = ".part.XXXXXX";
    pattern.insert(pattern.end(), suffix.begin(), suffix.end()); pattern.push_back('\0');
    output = ::mkstemp(pattern.data());
    if (output < 0) throw std::runtime_error("Cannot create destination temporary bag");
    temporary = pattern.data();
    if (::lseek(input, 0, SEEK_SET) < 0) throw std::runtime_error("Cannot seek backend bag");
    std::vector<char> buffer(1024 * 1024);
    for (;;)
    {
      if (cancelled) throw std::runtime_error("Bag export cancelled; backend file retained");
      const auto count = ::read(input, buffer.data(), buffer.size());
      if (count < 0) { if (errno == EINTR) continue; throw std::runtime_error("Reading backend bag failed"); }
      if (count == 0) break;
      ssize_t written = 0;
      while (written < count)
      {
        const auto n = ::write(output, buffer.data() + written, count - written);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) throw std::runtime_error("Writing destination bag failed");
        written += n;
      }
    }
    if (::fsync(output) != 0) throw std::runtime_error("Syncing destination bag failed");
    if (cancelled) throw std::runtime_error("Bag export cancelled; backend file retained");
    if (::link(temporary.c_str(), destination.c_str()) != 0)
      throw std::runtime_error("Cannot publish bag without overwriting an existing file");
    ::close(output); output = -1;
    ::unlink(temporary.c_str());
    ::close(input);
  }
  catch (...)
  {
    ::close(input);
    if (output >= 0) ::close(output);
    if (!temporary.empty()) ::unlink(temporary.c_str());
    throw;
  }
}
}  // namespace spikive_control_sdk

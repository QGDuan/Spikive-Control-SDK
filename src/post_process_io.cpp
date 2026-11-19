#include <spikive_control_sdk/post_process_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <limits>
#include <set>
#include <sstream>
#include <unistd.h>

namespace spikive_control_sdk
{
namespace
{
namespace fs = boost::filesystem;

constexpr std::size_t MAX_MAP_NAME_BYTES = 200;

std::string homeDirectory()
{
  const char* home = std::getenv("HOME");
  if (home == nullptr || *home == '\0')
    throw PostProcessError(ERR_PARAM_INVALID, "HOME is not set; cannot resolve spikive-output");
  return home;
}

std::string defaultOutputDirectory()
{
  const fs::path home(homeDirectory());
  return (home / "spikive-output").string();
}

std::string normalizeNodeName(std::string name)
{
  if (!name.empty() && name.front() != '/') name.insert(name.begin(), '/');
  return name;
}

bool isRegularNonSymlink(const fs::path& path)
{
  boost::system::error_code error;
  return fs::is_regular_file(path, error) && !error && !fs::is_symlink(path, error);
}

void validateBagStem(const std::string& name)
{
  if (name.empty() || name == "." || name == ".." || name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos || fs::path(name).has_parent_path() ||
      fs::path(name).has_extension())
    throw PostProcessError(
      ERR_PARAM_INVALID,
      "bag_name must be a plain name without path or .bag extension");
}

void validateMapStem(const std::string& name)
{
  if (name.empty() || name == "." || name == ".." || name.find('/') != std::string::npos ||
      name.find('\\') != std::string::npos || fs::path(name).has_parent_path() ||
      fs::path(name).has_extension())
    throw PostProcessError(
      ERR_PARAM_INVALID,
      "map_name must be a plain name without path or .pcd extension");
  if (name.size() > MAX_MAP_NAME_BYTES)
    throw PostProcessError(ERR_PARAM_INVALID, "map_name is too long");
}

std::string requireAbsoluteDirectory(const std::string& value, const std::string& label,
                                     bool must_exist)
{
  const std::string expanded = expandHome(value);
  if (expanded.empty() || expanded.front() != '/')
    throw PostProcessError(ERR_PARAM_INVALID, label + " must be an absolute directory");
  const fs::path path(expanded);
  boost::system::error_code error;
  if (must_exist && (!fs::is_directory(path, error) || error))
    throw PostProcessError(ERR_PRECHECK_FAILED, label + " does not exist or is not a directory: " + expanded);
  if (!must_exist && fs::exists(path, error) && !fs::is_directory(path, error))
    throw PostProcessError(ERR_PARAM_INVALID, label + " is not a directory: " + expanded);
  return expanded;
}

std::string makePartialPath(const std::string& output)
{
  std::string partial = output + ".partial";
  if (fs::exists(partial) || fs::is_symlink(partial))
    partial += "." + std::to_string(::getpid());
  return partial;
}

void moveToPartial(const std::string& temp, const std::string& output, PostProcessOutcome& outcome)
{
  if (temp.empty() || !fs::exists(temp)) return;
  const std::string partial = makePartialPath(output);
  boost::system::error_code error;
  fs::rename(temp, partial, error);
  outcome.partial_path = error ? temp : partial;
}

void checkControl(const PostProcessControl& control)
{
  if (control.cancelled != nullptr && control.cancelled->load())
    throw PostProcessError(ERR_CANCELLED, "Post-process cancelled");
  if (control.expired && control.expired())
    throw PostProcessError(ERR_TIMEOUT, "Post-process timed out");
}

std::uint32_t fieldSize(std::uint8_t datatype)
{
  switch (datatype)
  {
    case pcl::PCLPointField::INT8:
    case pcl::PCLPointField::UINT8:
      return 1;
    case pcl::PCLPointField::INT16:
    case pcl::PCLPointField::UINT16:
      return 2;
    case pcl::PCLPointField::INT32:
    case pcl::PCLPointField::UINT32:
    case pcl::PCLPointField::FLOAT32:
      return 4;
    case pcl::PCLPointField::FLOAT64:
      return 8;
    default:
      throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 contains an unsupported field type");
  }
}

void validateFields(const pcl::PCLPointCloud2& cloud)
{
  if (cloud.point_step == 0)
    throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 point_step is zero");
  std::set<std::string> names;
  bool has_x = false;
  bool has_y = false;
  bool has_z = false;
  for (const auto& field : cloud.fields)
  {
    if (field.name.empty() || !names.insert(field.name).second)
      throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 has empty or duplicate field names");
    if (field.count == 0)
      throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 field has zero count: " + field.name);
    const std::uint64_t bytes = static_cast<std::uint64_t>(fieldSize(field.datatype)) * field.count;
    if (static_cast<std::uint64_t>(field.offset) + bytes > cloud.point_step)
      throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 field exceeds point_step: " + field.name);
    if (field.name == "x") has_x = true;
    if (field.name == "y") has_y = true;
    if (field.name == "z") has_z = true;
    if ((field.name == "x" || field.name == "y" || field.name == "z") &&
        (field.datatype != pcl::PCLPointField::FLOAT32 || field.count != 1))
      throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 x/y/z must be float32 with count 1");
  }
  if (!has_x || !has_y || !has_z)
    throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 requires x/y/z fields");
}

bool sameFields(const std::vector<pcl::PCLPointField>& lhs,
                const std::vector<pcl::PCLPointField>& rhs)
{
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t i = 0; i < lhs.size(); ++i)
  {
    if (lhs[i].name != rhs[i].name || lhs[i].offset != rhs[i].offset ||
        lhs[i].datatype != rhs[i].datatype || lhs[i].count != rhs[i].count)
      return false;
  }
  return true;
}

void appendCloud(const sensor_msgs::PointCloud2& message, pcl::PCLPointCloud2& merged,
                 bool& first, std::string& frame_id, std::uint64_t max_points,
                 PostProcessOutcome& outcome)
{
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(message, cloud);
  validateFields(cloud);
  if (cloud.is_bigendian)
    throw PostProcessError(ERR_PARAM_INVALID, "Big-endian PointCloud2 is not supported");
  const std::uint64_t width = cloud.width;
  const std::uint64_t height = cloud.height;
  const std::uint64_t count = width * height;
  const std::uint64_t packed_row = width * cloud.point_step;
  const std::uint64_t required = static_cast<std::uint64_t>(cloud.row_step) * height;
  if (height > 0 && cloud.row_step < packed_row)
    throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 row_step is smaller than width * point_step");
  if (cloud.data.size() < required)
    throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 data is shorter than row_step * height");

  if (first)
  {
    merged.fields = cloud.fields;
    merged.point_step = cloud.point_step;
    merged.is_bigendian = false;
    merged.is_dense = cloud.is_dense;
    merged.header = cloud.header;
    frame_id = message.header.frame_id;
    first = false;
  }
  else
  {
    if (cloud.point_step != merged.point_step || !sameFields(cloud.fields, merged.fields))
      throw PostProcessError(ERR_PARAM_INVALID, "PointCloud2 field layout differs between bag messages");
    if (message.header.frame_id != frame_id)
      throw PostProcessError(ERR_PARAM_INVALID,
                             "PointCloud2 frame_id differs between bag messages: '" + frame_id +
                               "' vs '" + message.header.frame_id + "'");
    merged.is_dense = merged.is_dense && cloud.is_dense;
  }

  if (max_points > 0 && outcome.points + count > max_points)
    throw PostProcessError(ERR_PARAM_INVALID, "Point count exceeds post_process/max_points");
  if (outcome.points + count > std::numeric_limits<std::uint32_t>::max())
    throw PostProcessError(ERR_PARAM_INVALID, "Merged point cloud exceeds PCD uint32 point count");

  if (count > 0)
  {
    merged.data.reserve(merged.data.size() + static_cast<std::size_t>(count * cloud.point_step));
    for (std::uint64_t row = 0; row < height; ++row)
    {
      const auto* source = cloud.data.data() + row * cloud.row_step;
      merged.data.insert(merged.data.end(), source,
                         source + static_cast<std::size_t>(packed_row));
    }
  }
  outcome.messages++;
  outcome.points += count;
}

}  // namespace

std::string expandHome(const std::string& path)
{
  if (path.empty() || (path != "~" && path.rfind("~/", 0) != 0)) return path;
  return homeDirectory() + path.substr(1);
}

std::string resolveInputBag(const std::string& bag_path, const std::string& bag_name,
                            const std::string& input_directory)
{
  const std::string directory_value =
    bag_path.empty() ? (input_directory.empty() ? defaultOutputDirectory() : input_directory) : bag_path;
  const std::string directory = requireAbsoluteDirectory(directory_value, "bag_path", true);

  if (!bag_name.empty())
  {
    validateBagStem(bag_name);
    const fs::path candidate = fs::path(directory) / (bag_name + ".bag");
    if (!isRegularNonSymlink(candidate))
      throw PostProcessError(ERR_PRECHECK_FAILED, "Input bag does not exist: " + candidate.string());
    if (fs::exists(candidate.string() + ".active") || fs::is_symlink(candidate.string() + ".active"))
      throw PostProcessError(ERR_PRECHECK_FAILED, "Input bag is still being recorded: " + candidate.string());
    return candidate.string();
  }

  fs::path newest;
  std::time_t newest_time = 0;
  boost::system::error_code error;
  for (fs::directory_iterator it(directory, error), end; !error && it != end; it.increment(error))
  {
    const fs::path candidate = it->path();
    if (candidate.extension() != ".bag" || !isRegularNonSymlink(candidate)) continue;
    if (fs::exists(candidate.string() + ".active") || fs::is_symlink(candidate.string() + ".active")) continue;
    const std::time_t modified = fs::last_write_time(candidate, error);
    if (error)
    {
      error.clear();
      continue;
    }
    if (newest.empty() || modified > newest_time ||
        (modified == newest_time && candidate.filename().string() > newest.filename().string()))
    {
      newest = candidate;
      newest_time = modified;
    }
  }
  if (newest.empty())
    throw PostProcessError(ERR_PRECHECK_FAILED, "No closed .bag found in " + directory);
  return newest.string();
}

std::string resolveOutputPath(const std::string& map_path, const std::string& map_name,
                              const std::string& default_directory,
                              const std::string& bag_stem)
{
  const std::string directory_value =
    map_path.empty() ? (default_directory.empty() ? defaultOutputDirectory() : default_directory) : map_path;
  const std::string directory = requireAbsoluteDirectory(directory_value, "map_path", false);
  const std::string stem = map_name.empty() ? bag_stem : map_name;
  validateMapStem(stem);
  const fs::path output = fs::path(directory) / (stem + ".pcd");
  boost::system::error_code error;
  if (fs::exists(output, error) || fs::is_symlink(output, error))
    throw PostProcessError(ERR_PARAM_INVALID, "Output PCD already exists; refusing to overwrite: " + output.string());
  return output.string();
}

void ensureOutputDirectory(const std::string& directory)
{
  if (directory.empty() || directory.front() != '/')
    throw PostProcessError(ERR_PARAM_INVALID, "Output directory must be absolute");
  boost::system::error_code error;
  fs::create_directories(directory, error);
  if (error || !fs::is_directory(directory, error))
    throw PostProcessError(ERR_PARAM_INVALID, "Cannot create output directory: " + directory);
}

bool hasForbiddenNode(const std::vector<std::string>& nodes,
                      const std::vector<std::string>& forbidden)
{
  std::set<std::string> names;
  for (const auto& node : nodes) names.insert(normalizeNodeName(node));
  for (const auto& node : forbidden)
    if (names.count(normalizeNodeName(node)) != 0) return true;
  return false;
}

PostProcessOutcome runPostProcess(const PostProcessRunRequest& request,
                                  const PostProcessControl& control)
{
  PostProcessOutcome outcome;
  outcome.output_path = request.output_path;
  std::string temp_path;

  try
  {
    ensureOutputDirectory(fs::path(request.output_path).parent_path().string());
    if (fs::exists(request.output_path) || fs::is_symlink(request.output_path))
      throw PostProcessError(ERR_PARAM_INVALID, "Output PCD already exists; refusing to overwrite");
    checkControl(control);

    rosbag::Bag bag;
    try
    {
      bag.open(request.bag_path, rosbag::bagmode::Read);
    }
    catch (const std::exception& error)
    {
      throw PostProcessError(ERR_INTERNAL, std::string("Cannot open input bag: ") + error.what());
    }

    pcl::PCLPointCloud2 merged;
    bool first = true;
    {
      rosbag::View view(bag, rosbag::TopicQuery(request.options.pointcloud_topic));
      if (view.size() == 0)
        throw PostProcessError(ERR_PRECHECK_FAILED,
                               "Input bag has no messages on " + request.options.pointcloud_topic);

      for (const auto& instance : view)
      {
        checkControl(control);
        const auto cloud = instance.instantiate<sensor_msgs::PointCloud2>();
        if (!cloud)
          throw PostProcessError(ERR_PARAM_INVALID,
                                 "Bag message on " + request.options.pointcloud_topic +
                                   " is not sensor_msgs/PointCloud2");
        appendCloud(*cloud, merged, first, outcome.frame_id, request.options.max_points, outcome);
      }
    }
    bag.close();
    checkControl(control);
    if (outcome.points == 0)
      throw PostProcessError(ERR_PRECHECK_FAILED, "Input bag contains no points on the selected topic");
    if (outcome.points > std::numeric_limits<std::uint32_t>::max())
      throw PostProcessError(ERR_PARAM_INVALID, "Merged point cloud exceeds PCD uint32 point count");

    merged.width = static_cast<std::uint32_t>(outcome.points);
    merged.height = 1;
    const std::uint64_t row_step = static_cast<std::uint64_t>(merged.width) * merged.point_step;
    if (row_step > std::numeric_limits<std::uint32_t>::max())
      throw PostProcessError(ERR_PARAM_INVALID, "Merged point cloud row_step exceeds uint32");
    merged.row_step = static_cast<std::uint32_t>(row_step);
    merged.data.resize(static_cast<std::size_t>(row_step));

    const fs::path output(request.output_path);
    temp_path = (output.parent_path() /
                 ("." + output.stem().string() + ".pcd.tmp." + std::to_string(::getpid()) + "." +
                  std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())))
                  .string();
    pcl::PCDWriter writer;
    if (writer.writeBinary(temp_path, merged) != 0)
      throw PostProcessError(ERR_INTERNAL, "PCL failed to write PCD: " + temp_path);
    checkControl(control);
    if (!isRegularNonSymlink(temp_path) || fs::file_size(temp_path) == 0)
      throw PostProcessError(ERR_INTERNAL, "PCD writer produced no output: " + temp_path);
    if (fs::exists(request.output_path) || fs::is_symlink(request.output_path))
      throw PostProcessError(ERR_PARAM_INVALID, "Output PCD appeared during post-process; refusing to overwrite");
    if (::link(temp_path.c_str(), request.output_path.c_str()) != 0)
      throw PostProcessError(ERR_INTERNAL,
                             "Cannot publish PCD atomically: " + std::string(std::strerror(errno)));
    boost::system::error_code remove_error;
    fs::remove(temp_path, remove_error);
    temp_path.clear();

    outcome.success = true;
    outcome.code = OK;
    outcome.message = "PCD saved: " + request.output_path;
  }
  catch (const PostProcessError& error)
  {
    outcome.message = error.what();
    outcome.code = error.code();
    moveToPartial(temp_path, request.output_path, outcome);
  }
  catch (const std::exception& error)
  {
    outcome.message = std::string("Post-process failed: ") + error.what();
    outcome.code = ERR_INTERNAL;
    moveToPartial(temp_path, request.output_path, outcome);
  }

  if (!outcome.partial_path.empty())
    outcome.message += "; partial PCD retained: " + outcome.partial_path;
  return outcome;
}

}  // namespace spikive_control_sdk

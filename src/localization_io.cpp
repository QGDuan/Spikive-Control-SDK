#include <spikive_control_sdk/localization_io.h>

#include <boost/property_tree/json_parser.hpp>
#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>

namespace spikive_control_sdk
{
namespace
{
std::string fileMd5(const std::string& path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read map file: " + path);
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_md5(), nullptr) != 1)
    throw std::runtime_error("Cannot initialize MD5");
  std::array<char, 65536> buffer;
  while (stream)
  {
    stream.read(buffer.data(), buffer.size());
    if (EVP_DigestUpdate(ctx.get(), buffer.data(), stream.gcount()) != 1)
      throw std::runtime_error("Cannot calculate map MD5");
  }
  if (!stream.eof()) throw std::runtime_error("Map file read failed");
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest, &length) != 1)
    throw std::runtime_error("Cannot finish map MD5");
  std::ostringstream out;
  for (unsigned int i = 0; i < length; ++i)
    out << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned int>(digest[i]);
  return out.str();
}
}  // namespace

LoadedMap readLocalizationMap(const std::string& path, const std::string& expected_md5)
{
  struct stat info{};
  if (path.empty() || path.front() != '/' || stat(path.c_str(), &info) != 0 || !S_ISREG(info.st_mode))
    throw std::invalid_argument("map_path must name an existing absolute regular PCD file");
  std::string expected = expected_md5;
  if (!expected.empty() && (expected.size() != 32 ||
      !std::all_of(expected.begin(), expected.end(), [](unsigned char c) { return std::isxdigit(c); })))
    throw std::invalid_argument("map_md5 must be empty or 32 hexadecimal characters");
  std::transform(expected.begin(), expected.end(), expected.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  LoadedMap result;
  result.md5 = fileMd5(path);
  if (!expected.empty() && result.md5 != expected)
    throw std::domain_error("Map MD5 mismatch");

  pcl::PCLPointCloud2 raw;
  if (pcl::io::loadPCDFile(path, raw) < 0) throw std::domain_error("Cannot parse PCD file");
  for (const std::string name : {"x", "y", "z"})
  {
    const auto it = std::find_if(raw.fields.begin(), raw.fields.end(),
                                [&name](const pcl::PCLPointField& field) { return field.name == name; });
    if (it == raw.fields.end() || it->datatype != pcl::PCLPointField::FLOAT32 || it->count != 1)
      throw std::domain_error("PCD requires float32 x/y/z fields, as produced by Spikive-SLAM");
  }
  pcl::PointCloud<pcl::PointXYZ> input, finite;
  pcl::fromPCLPointCloud2(raw, input);
  finite.reserve(input.size());
  for (const auto& p : input)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) finite.push_back(p);
  if (finite.size() < 5) throw std::domain_error("Map needs at least five finite XYZ points");
  // 文件读两遍：发送前确认 MD5 对应的文件在解析期间没有改变。
  if (fileMd5(path) != result.md5) throw std::domain_error("Map file changed during loading; retry with a stable file");
  pcl::toROSMsg(finite, result.cloud);
  result.cloud.header.frame_id = "map";
  return result;
}

MatchStatus parseMatchStatus(const std::string& json, int drone_id)
{
  boost::property_tree::ptree tree;
  std::istringstream stream(json);
  boost::property_tree::read_json(stream, tree);
  if (tree.get<int>("schema_version") != 1 || tree.get<std::string>("drone_id") != std::to_string(drone_id))
    throw std::invalid_argument("Unexpected localization status schema or drone_id");
  MatchStatus result;
  result.state = tree.get<std::string>("state");
  if (result.state != "idle" && result.state != "waiting_initialpose" && result.state != "matching" &&
      result.state != "localized" && result.state != "failed")
    throw std::invalid_argument("Unknown localization state");
  result.message = tree.get<std::string>("message");
  result.fitness_score = tree.get<double>("fitness_score");
  result.threshold = tree.get<double>("threshold");
  const auto attempts = tree.get<std::int64_t>("attempt_count");
  const auto elapsed_ms = tree.get<double>("elapsed_ms");
  if (attempts < 0 || attempts > UINT32_MAX || !std::isfinite(result.fitness_score) ||
      !std::isfinite(result.threshold) || result.threshold <= 0 || !std::isfinite(elapsed_ms) || elapsed_ms < 0)
    throw std::invalid_argument("Invalid localization match metrics");
  result.attempt_count = static_cast<std::uint32_t>(attempts);
  result.elapsed_time = elapsed_ms / 1000.0;
  // 后端 ok=true 也包含 idle/matching，绝不能将其解释为匹配成功。
  return result;
}
}  // namespace spikive_control_sdk

#include "visual_pointcloud.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace spikive_control_sdk
{
sensor_msgs::PointCloud2ConstPtr visualPointCloud(const sensor_msgs::PointCloud2ConstPtr& input,
                                               double keep_ratio)
{
  if (!std::isfinite(keep_ratio) || keep_ratio <= 0.0 || keep_ratio > 1.0)
    throw std::invalid_argument("visual_pointcloud/keep_ratio must be in (0, 1]");
  if (!input) throw std::invalid_argument("Null PointCloud2");
  const std::uint64_t count = static_cast<std::uint64_t>(input->width) * input->height;
  const std::uint64_t packed_row = static_cast<std::uint64_t>(input->width) * input->point_step;
  const std::uint64_t size = static_cast<std::uint64_t>(input->row_step) * input->height;
  if ((count > 0 && input->point_step == 0) || input->row_step < packed_row || input->data.size() < size)
    throw std::invalid_argument("Malformed PointCloud2: invalid point/row stride or data size");
  if (keep_ratio == 1.0 || count == 0) return input;

  const auto kept = std::max<std::uint64_t>(1, static_cast<std::uint64_t>(count * keep_ratio));
  if (kept > std::numeric_limits<std::uint32_t>::max() / input->point_step)
    throw std::overflow_error("Downsampled PointCloud2 exceeds uint32 row_step");
  sensor_msgs::PointCloud2Ptr output(new sensor_msgs::PointCloud2);
  output->header = input->header;
  output->fields = input->fields;
  output->is_bigendian = input->is_bigendian;
  output->is_dense = input->is_dense;
  output->point_step = input->point_step;
  output->height = 1;
  output->width = static_cast<std::uint32_t>(kept);
  output->row_step = output->width * output->point_step;
  output->data.resize(output->row_step);

  const double stride = static_cast<double>(count) / kept;
  for (std::uint64_t i = 0; i < kept; ++i)
  {
    const auto index = std::min(count - 1, static_cast<std::uint64_t>(i * stride));
    const auto offset = (index / input->width) * input->row_step + (index % input->width) * input->point_step;
    std::memcpy(output->data.data() + i * output->point_step, input->data.data() + offset, output->point_step);
  }
  return output;
}
}  // namespace spikive_control_sdk

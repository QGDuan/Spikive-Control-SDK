#include "visual_pointcloud.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using spikive_control_sdk::visualPointCloud;

namespace
{
sensor_msgs::PointCloud2Ptr cloud(bool big_endian = false)
{
  sensor_msgs::PointCloud2Ptr c(new sensor_msgs::PointCloud2);
  c->header.seq = 73; c->header.stamp = ros::Time(32, 12); c->header.frame_id = "world";
  c->height = 5; c->width = 4; c->point_step = 20; c->row_step = 85;
  c->is_bigendian = big_endian; c->is_dense = false;
  sensor_msgs::PointField f;
  f.name = "x"; f.offset = 0; f.count = 1; f.datatype = sensor_msgs::PointField::FLOAT32;
  c->fields.push_back(f);
  f.name = "custom"; f.offset = 4; f.count = 4; f.datatype = sensor_msgs::PointField::UINT32;
  c->fields.push_back(f);
  c->data.resize(c->row_step * c->height, 0xee);
  for (unsigned i = 0; i < 20; ++i)
  {
    const auto offset = (i / 4) * c->row_step + (i % 4) * c->point_step;
    for (unsigned j = 0; j < c->point_step; ++j) c->data[offset + j] = (i * 20 + j) % 256;
  }
  // 第一个 x 为 NaN；完整记录复制仍保留它，不做无效点过滤。
  const std::vector<std::uint8_t> nan = big_endian ? std::vector<std::uint8_t>{0x7f, 0xc0, 0, 0}
                                                 : std::vector<std::uint8_t>{0, 0, 0xc0, 0x7f};
  std::copy(nan.begin(), nan.end(), c->data.begin());
  return c;
}
}

TEST(VisualPointCloud, PreservesRecordsMetadataAndPaddingForBothByteOrders)
{
  for (bool big : {false, true})
  {
    const auto input = cloud(big);
    const auto original = *input;
    const auto out = visualPointCloud(input, 0.3);
    EXPECT_EQ(out->header, input->header);
    EXPECT_EQ(out->header.frame_id, "world");
    EXPECT_EQ(out->fields, input->fields);
    EXPECT_EQ(out->is_bigendian, big); EXPECT_EQ(out->is_dense, input->is_dense);
    EXPECT_EQ(out->point_step, input->point_step);
    ASSERT_EQ(out->height, 1u); ASSERT_EQ(out->width, 6u);
    EXPECT_EQ(out->row_step, 120u); ASSERT_EQ(out->data.size(), 120u);
    const std::vector<unsigned> indices{0, 3, 6, 10, 13, 16};
    for (unsigned i = 0; i < indices.size(); ++i)
    {
      const auto n = indices[i]; const auto offset = (n / 4) * input->row_step + (n % 4) * input->point_step;
      EXPECT_TRUE(std::equal(input->data.begin() + offset, input->data.begin() + offset + 20,
                             out->data.begin() + i * 20));
    }
    EXPECT_EQ(*input, original);
  }
}

TEST(VisualPointCloud, KeepsConfiguredCountAndAtLeastOnePoint)
{
  EXPECT_EQ(visualPointCloud(cloud(), 0.1)->width, 2u);
  EXPECT_EQ(visualPointCloud(cloud(), 0.5)->width, 10u);
  EXPECT_EQ(visualPointCloud(cloud(), 1e-12)->width, 1u);
}

TEST(VisualPointCloud, PassesThroughFullRatioAndEmptyCloud)
{
  const auto input = cloud();
  EXPECT_EQ(visualPointCloud(input, 1.0).get(), input.get());
  sensor_msgs::PointCloud2Ptr empty(new sensor_msgs::PointCloud2);
  empty->header.frame_id = "world";
  EXPECT_EQ(visualPointCloud(empty, 0.1).get(), empty.get());
}

TEST(VisualPointCloud, RejectsMalformedLayouts)
{
  auto input = cloud(); input->data.pop_back();
  EXPECT_THROW(visualPointCloud(input, 0.1), std::invalid_argument);
  input = cloud(); input->row_step = 79;
  EXPECT_THROW(visualPointCloud(input, 1.0), std::invalid_argument);
  input = cloud(); input->point_step = 0;
  EXPECT_THROW(visualPointCloud(input, 0.1), std::invalid_argument);
  EXPECT_THROW(visualPointCloud({}, 0.1), std::invalid_argument);
}

TEST(VisualPointCloud, RejectsInvalidRatios)
{
  for (double ratio : {0., -0.1, 1.01, std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()})
    EXPECT_THROW(visualPointCloud(cloud(), ratio), std::invalid_argument);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

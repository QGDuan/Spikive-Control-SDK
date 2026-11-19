#include <spikive_control_sdk/post_process_io.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <boost/filesystem.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>

namespace
{
namespace fs = boost::filesystem;
using spikive_control_sdk::PostProcessControl;
using spikive_control_sdk::PostProcessError;
using spikive_control_sdk::PostProcessRunRequest;
using spikive_control_sdk::PostProcessOutcome;

class PostProcessIO : public ::testing::Test
{
protected:
  fs::path root;
  void SetUp() override
  {
    root = fs::temp_directory_path() / fs::unique_path("sdk-post-process-%%%%-%%%%");
    fs::create_directories(root);
  }
  void TearDown() override { fs::remove_all(root); }

  sensor_msgs::PointCloud2 makeCloud(const std::vector<float>& xyz, const std::string& frame,
                                     bool intensity)
  {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(xyz.size() / 3);
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.fields.resize(intensity ? 4u : 3u);
    for (std::size_t i = 0; i < 3; ++i)
    {
      cloud.fields[i].name = std::string(1, "xyz"[i]);
      cloud.fields[i].offset = static_cast<std::uint32_t>(i * 4);
      cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
      cloud.fields[i].count = 1;
    }
    if (intensity)
    {
      cloud.fields[3].name = "intensity";
      cloud.fields[3].offset = 12;
      cloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
      cloud.fields[3].count = 1;
    }
    cloud.point_step = intensity ? 16 : 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step);
    std::memcpy(cloud.data.data(), xyz.data(), xyz.size() * sizeof(float));
    return cloud;
  }

  std::string writeBag(const std::string& name, const std::vector<sensor_msgs::PointCloud2>& clouds,
                       const std::string& topic = "/test/cloud")
  {
    const std::string path = (root / (name + ".bag")).string();
    rosbag::Bag bag;
    bag.open(path, rosbag::bagmode::Write);
    for (std::size_t i = 0; i < clouds.size(); ++i)
      bag.write(topic, ros::Time(100, static_cast<std::uint32_t>(i)), clouds[i]);
    bag.close();
    return path;
  }

  PostProcessOutcome run(const std::string& bag, const std::string& output,
                         std::uint64_t max_points = 0)
  {
    PostProcessRunRequest request;
    request.bag_path = bag;
    request.output_path = output;
    request.options.pointcloud_topic = "/test/cloud";
    request.options.max_points = max_points;
    PostProcessControl control;
    return spikive_control_sdk::runPostProcess(request, control);
  }
};

TEST_F(PostProcessIO, ConcatenatesAllCloudsAndPreservesFields)
{
  const auto bag = writeBag("clouds",
    {makeCloud({1, 2, 3, 4, 5, 6}, "world", true),
     makeCloud({7, 8, 9, 10, 11, 12, 13, 14, 15}, "world", true)});
  const auto output = (root / "out" / "map.pcd").string();
  const auto outcome = run(bag, output);
  ASSERT_TRUE(outcome.success) << outcome.message;
  EXPECT_EQ(outcome.messages, 2u);
  EXPECT_EQ(outcome.points, 5u);
  EXPECT_EQ(outcome.frame_id, "world");
  ASSERT_TRUE(fs::is_regular_file(output));

  pcl::PCLPointCloud2 pcd;
  ASSERT_EQ(pcl::io::loadPCDFile(output, pcd), 0);
  EXPECT_EQ(pcd.width, 5u);
  EXPECT_EQ(pcd.height, 1u);
  EXPECT_EQ(pcd.fields.size(), 4u);
  EXPECT_EQ(pcd.point_step, 16u);
  EXPECT_EQ(pcd.data.size(), 5u * 16u);
}

TEST_F(PostProcessIO, RejectsDifferentFieldLayoutsAndFrames)
{
  const auto fields_bag = writeBag("fields",
    {makeCloud({1, 2, 3}, "world", true), makeCloud({4, 5, 6}, "world", false)});
  const auto fields = run(fields_bag, (root / "fields.pcd").string());
  EXPECT_FALSE(fields.success);
  EXPECT_EQ(fields.code, spikive_control_sdk::ERR_PARAM_INVALID);
  EXPECT_FALSE(fs::exists(root / "fields.pcd"));

  const auto frame_bag = writeBag("frames",
    {makeCloud({1, 2, 3}, "world", true), makeCloud({4, 5, 6}, "map", true)});
  const auto frames = run(frame_bag, (root / "frames.pcd").string());
  EXPECT_FALSE(frames.success);
  EXPECT_EQ(frames.code, spikive_control_sdk::ERR_PARAM_INVALID);
}

TEST_F(PostProcessIO, RejectsMissingTopicEmptyTopicAndExistingOutput)
{
  const auto empty = writeBag("empty", {}, "/other");
  const auto no_topic = run(empty, (root / "no_topic.pcd").string());
  EXPECT_FALSE(no_topic.success);
  EXPECT_EQ(no_topic.code, spikive_control_sdk::ERR_PRECHECK_FAILED);

  const auto bag = writeBag("clouds2", {makeCloud({1, 2, 3}, "world", false)});
  const auto output = (root / "exists.pcd").string();
  std::ofstream(output) << "existing";
  const auto existing = run(bag, output);
  EXPECT_FALSE(existing.success);
  EXPECT_EQ(existing.code, spikive_control_sdk::ERR_PARAM_INVALID);
}

TEST_F(PostProcessIO, RejectsCancelTimeoutAndPointLimit)
{
  const auto bag = writeBag("limit", {makeCloud({1, 2, 3, 4, 5, 6}, "world", false)});
  std::atomic<bool> cancelled{true};
  PostProcessRunRequest request;
  request.bag_path = bag;
  request.output_path = (root / "cancel.pcd").string();
  request.options.pointcloud_topic = "/test/cloud";
  PostProcessControl cancel_control;
  cancel_control.cancelled = &cancelled;
  const auto cancelled_result = spikive_control_sdk::runPostProcess(request, cancel_control);
  EXPECT_FALSE(cancelled_result.success);
  EXPECT_EQ(cancelled_result.code, spikive_control_sdk::ERR_CANCELLED);

  PostProcessControl timeout_control;
  timeout_control.expired = [] { return true; };
  const auto timeout_result = spikive_control_sdk::runPostProcess(request, timeout_control);
  EXPECT_FALSE(timeout_result.success);
  EXPECT_EQ(timeout_result.code, spikive_control_sdk::ERR_TIMEOUT);

  const auto limited = run(bag, (root / "limited.pcd").string(), 1);
  EXPECT_FALSE(limited.success);
  EXPECT_EQ(limited.code, spikive_control_sdk::ERR_PARAM_INVALID);
}

TEST_F(PostProcessIO, ResolvesInputBagAndOutputWithoutOverwrite)
{
  const auto bag = writeBag("resolve", {makeCloud({1, 2, 3}, "world", false)});
  EXPECT_EQ(spikive_control_sdk::resolveInputBag(root.string(), "resolve", ""),
            (root / "resolve.bag").string());
  EXPECT_EQ(spikive_control_sdk::resolveInputBag("", "", root.string()),
            (root / "resolve.bag").string());
  EXPECT_THROW(spikive_control_sdk::resolveInputBag(root.string(), "missing", ""),
               PostProcessError);

  const auto output = spikive_control_sdk::resolveOutputPath(
    root.string(), "map", "", fs::path(bag).stem().string());
  EXPECT_EQ(output, (root / "map.pcd").string());
  std::ofstream(output) << "occupied";
  EXPECT_THROW(spikive_control_sdk::resolveOutputPath(
                 root.string(), "map", "", fs::path(bag).stem().string()),
               PostProcessError);
}

TEST(PostProcessNodes, MatchesConfiguredLioNodesExactly)
{
  EXPECT_TRUE(spikive_control_sdk::hasForbiddenNode(
    {"/camera", "/laserMapping"}, {"/laserMapping", "/slam_to_uav_transform"}));
  EXPECT_TRUE(spikive_control_sdk::hasForbiddenNode(
    {"laserMapping"}, {"/laserMapping"}));
  EXPECT_FALSE(spikive_control_sdk::hasForbiddenNode(
    {"/camera", "/other"}, {"/laserMapping", "/slam_to_uav_transform"}));
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

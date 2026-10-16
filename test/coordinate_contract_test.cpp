#include <spikive_control_sdk/control_sdk.h>
#include <spikive_control_sdk/localization_io.h>
#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

using namespace spikive_control_sdk;

TEST(CoordinateContract, DroneTwoUsesBackendTopics)
{
  auto config = Config::forDrone(2);
  EXPECT_EQ(config.drone_state_topic, "/drone_2_state");
  EXPECT_EQ(config.navigation_odometry_topic, "/drone_2_visual_slam/odom");
  EXPECT_EQ(config.localization_map_topic, "/drone_2_localization_pcl");
  EXPECT_EQ(config.initial_pose_topic, "/drone_2_initialpose");
  EXPECT_EQ(config.match_status_topic, "/drone_2_localization_match_status");
  EXPECT_EQ(config.match_transform_topic, "/drone_2_diff_odom");
  EXPECT_EQ(config.odometry_topic, "/mavros/local_position/odom");
  EXPECT_EQ(config.world_frame, "world");
  EXPECT_NO_THROW(config.validate());
  config.world_frame.clear();
  EXPECT_THROW(config.validate(), std::invalid_argument);
}

TEST(CoordinateContract, NavigationRequiresMatchingFrameAndStampedFinitePosition)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time(10);
  odom.header.frame_id = "world";
  EXPECT_TRUE(validNavigationOdometry(odom, "world"));
  odom.header.frame_id = "map";
  EXPECT_FALSE(validNavigationOdometry(odom, "world"));
  odom.header.frame_id.clear();
  EXPECT_FALSE(validNavigationOdometry(odom, "world"));
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time();
  EXPECT_FALSE(validNavigationOdometry(odom, "world"));
  odom.header.stamp = ros::Time(10);
  odom.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validNavigationOdometry(odom, "world"));
  odom.pose.pose.position.x = 0;
  odom.pose.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validNavigationOdometry(odom, "world"));
}

TEST(CoordinateContract, InitialPosePreservesPositionAndCovariance)
{
  geometry_msgs::PoseWithCovarianceStamped input;
  input.header.frame_id = "world";
  input.pose.pose.position.x = 3;
  input.pose.pose.position.y = -4;
  input.pose.pose.position.z = 2;
  input.pose.pose.orientation.z = 2;
  input.pose.pose.orientation.w = 2;
  input.pose.covariance[0] = 0.25;
  const auto pose = prepareInitialPose(input, "world");
  EXPECT_EQ(pose.header.frame_id, "world");
  EXPECT_EQ(pose.pose.pose.position, input.pose.pose.position);
  EXPECT_EQ(pose.pose.covariance, input.pose.covariance);
  EXPECT_NEAR(pose.pose.pose.orientation.z, 0.7071067811865475, 1e-12);
  EXPECT_NEAR(pose.pose.pose.orientation.w, 0.7071067811865475, 1e-12);
  EXPECT_DOUBLE_EQ(input.pose.pose.orientation.w, 2);
}

TEST(CoordinateContract, InitialPoseUsesConfiguredFrameAndRejectsOtherFrames)
{
  geometry_msgs::PoseWithCovarianceStamped input;
  input.pose.pose.orientation.w = 1;
  EXPECT_EQ(prepareInitialPose(input, "world").header.frame_id, "world");
  EXPECT_EQ(prepareInitialPose(input, "site_world").header.frame_id, "site_world");
  input.header.frame_id = "map";
  EXPECT_THROW(prepareInitialPose(input, "world"), std::invalid_argument);
  EXPECT_EQ(prepareInitialPose(input, "map").header.frame_id, "map");
  EXPECT_THROW(prepareInitialPose(input, ""), std::invalid_argument);
}

TEST(CoordinateContract, InitialPoseRejectsInvalidValues)
{
  geometry_msgs::PoseWithCovarianceStamped input;
  EXPECT_THROW(prepareInitialPose(input, "world"), std::domain_error);
  input.pose.pose.orientation.w = 1;
  input.pose.pose.orientation.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(prepareInitialPose(input, "world"), std::domain_error);
  input.pose.pose.orientation.x = 0;
  input.pose.pose.position.y = std::numeric_limits<double>::infinity();
  EXPECT_THROW(prepareInitialPose(input, "world"), std::domain_error);
  input.pose.pose.position.y = 0;
  input.pose.covariance[35] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(prepareInitialPose(input, "world"), std::domain_error);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

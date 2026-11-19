#include <spikive_control_sdk/record_io.h>
#include <spikive_control_sdk/action_support.h>
#include <boost/filesystem.hpp>
#include <gtest/gtest.h>
#include <fstream>
#include <thread>
#include <limits>

namespace fs = boost::filesystem;
using namespace spikive_control_sdk;
class RecordIO : public ::testing::Test
{
protected:
  fs::path root;
  void SetUp() override { root = fs::temp_directory_path() / fs::unique_path("sdk-record-%%%%-%%%%"); fs::create_directories(root); }
  void TearDown() override { fs::remove_all(root); }
  std::string bag(const std::string& name) { auto p=(root/name).string(); std::ofstream(p) << "#ROSBAG V2.0\nfixture"; return p; }
};
TEST_F(RecordIO, DestinationRejectsPathsExtensionsAndOverwrite)
{
  EXPECT_THROW(recordingDestination("relative", "name"), std::invalid_argument);
  EXPECT_THROW(recordingDestination(root.string(), "../name"), std::invalid_argument);
  EXPECT_THROW(recordingDestination(root.string(), "name.bag"), std::invalid_argument);
  bag("existing.bag");
  EXPECT_THROW(recordingDestination(root.string(), "existing"), std::invalid_argument);
  fs::create_symlink(root/"missing", root/"symlink.bag");
  EXPECT_THROW(recordingDestination(root.string(), "symlink"), std::invalid_argument);
  EXPECT_EQ(recordingDestination((root/"sub").string(), "valid"), (root/"sub/valid.bag").string());
}
TEST_F(RecordIO, AtomicPublishNeverOverwritesIncludingConcurrentDestination)
{
  const auto source = bag("source.bag");
  const auto target = recordingDestination(root.string(), "target");
  EXPECT_NO_THROW(publishRecordedBag(source, target));
  EXPECT_TRUE(fs::exists(source));
  EXPECT_EQ(fs::file_size(source), fs::file_size(target));
  EXPECT_THROW(publishRecordedBag(source, target), std::runtime_error);
  const auto collision = recordingDestination(root.string(), "race");
  std::ofstream(collision) << "another writer";
  EXPECT_THROW(publishRecordedBag(source, collision), std::runtime_error);
  std::ifstream in(collision); std::string content; std::getline(in, content);
  EXPECT_EQ(content, "another writer");
}
TEST_F(RecordIO, InvalidOrActiveSourceLeavesNoOutput)
{
  const auto target = recordingDestination(root.string(), "target");
  EXPECT_THROW(publishRecordedBag(bag("record.bag.active"), target), std::runtime_error);
  std::ofstream((root/"bad.bag").string()) << "invalid";
  EXPECT_THROW(publishRecordedBag((root/"bad.bag").string(), target), std::runtime_error);
  EXPECT_FALSE(fs::exists(target));
}
TEST(RecordTopics, OnlyExplicitUniqueAbsoluteTopics)
{
  EXPECT_NO_THROW(validateRecordingTopics({"/nav/data/odom", "/livox/imu"}));
  for (const auto& topics : std::vector<std::vector<std::string>>{
       {}, {""}, {"relative"}, {"/"}, {"/a", "/a"}, {"/a//b"}, {"/a/"},
       {"/foo.*"}, {"-a"}, {"/a:=/b"}, {"/a;touch /tmp/file"}, {std::string("/a\0b", 4)}})
    EXPECT_THROW(validateRecordingTopics(topics), std::invalid_argument);
}
TEST(Deadline, DisabledInvalidAndSteadyExpiry)
{
  EXPECT_TRUE(TaskDeadline::valid(0));
  EXPECT_FALSE(TaskDeadline::valid(-1));
  EXPECT_FALSE(TaskDeadline::valid(std::numeric_limits<double>::quiet_NaN()));
  TaskDeadline deadline; deadline.start(0); EXPECT_FALSE(deadline.expired());
  deadline.start(0.01); std::this_thread::sleep_for(std::chrono::milliseconds(20)); EXPECT_TRUE(deadline.expired());
}
int main(int argc, char** argv) { ::testing::InitGoogleTest(&argc, argv); return RUN_ALL_TESTS(); }

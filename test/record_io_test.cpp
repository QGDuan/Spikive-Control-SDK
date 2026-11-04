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
  EXPECT_EQ(recordingDestination((root/"sub").string(), "valid"), (root/"sub/valid.bag").string());
}
TEST_F(RecordIO, ExportPreservesSourceAndNeverOverwrites)
{
  const auto source = bag("source.bag");
  const auto target = recordingDestination(root.string(), "target");
  std::atomic_bool cancelled(false);
  EXPECT_NO_THROW(exportRecordedBag(source, target, cancelled));
  EXPECT_TRUE(fs::exists(source));
  EXPECT_EQ(fs::file_size(source), fs::file_size(target));
  EXPECT_THROW(exportRecordedBag(source, target, cancelled), std::runtime_error);
}
TEST_F(RecordIO, CancelAndInvalidSourceLeaveNoPartialFile)
{
  const auto target = recordingDestination(root.string(), "target");
  const auto source = bag("source.bag");
  std::atomic_bool cancelled(true);
  EXPECT_THROW(exportRecordedBag(source, target, cancelled), std::runtime_error);
  EXPECT_FALSE(fs::exists(target));
  cancelled = false;
  EXPECT_THROW(exportRecordedBag(bag("open.active.bag"), target, cancelled), std::runtime_error);
  std::ofstream((root/"bad.bag").string()) << "invalid";
  EXPECT_THROW(exportRecordedBag((root/"bad.bag").string(), target, cancelled), std::runtime_error);
  EXPECT_FALSE(fs::exists(target));
}
TEST(RecordProtocol, JsonSchemaDroneAndStateAreValidated)
{
  const auto payload = recordCommand("a\"b");
  EXPECT_NE(payload.find("\"schema_version\":1"), std::string::npos);
  const std::string status = R"({"schema_version":1,"request_id":"r1","drone_id":"2","phase":"record_start","ok":true,"state":"recording","bag_path":"/tmp/x.bag","error":null})";
  const auto parsed = parseRecordStatus(status, 2);
  EXPECT_EQ(parsed.request_id, "r1"); EXPECT_TRUE(parsed.ok);
  EXPECT_THROW(parseRecordStatus(status, 1), std::invalid_argument);
  EXPECT_THROW(parseRecordStatus("{}", 2), std::exception);
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

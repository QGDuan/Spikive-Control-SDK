#include <spikive_control_sdk/localization_io.h>
#include <gtest/gtest.h>
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

using namespace spikive_control_sdk;

namespace
{
class MapFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    char pattern[] = "/tmp/spikive-map-test-XXXXXX";
    const int fd = mkstemp(pattern);
    ASSERT_GE(fd, 0);
    close(fd);
    path = pattern;
    write("0 0 0\n1 0 0\n0 1 0\n0 0 1\n1 1 1\n2 0 1\n");
  }
  void TearDown() override { std::remove(path.c_str()); }
  void write(const std::string& points)
  {
    std::ofstream file(path);
    file << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
         << "WIDTH 6\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 6\nDATA ascii\n" << points;
  }
  std::string path;
};

TEST_F(MapFixture, ReadsSlamXyzWithoutChangingCoordinates)
{
  const auto map = readLocalizationMap(path, "");
  EXPECT_EQ(map.cloud.header.frame_id, "map");
  EXPECT_EQ(map.cloud.width * map.cloud.height, 6u);
  EXPECT_EQ(map.md5.size(), 32u);
  // 第 2 个点保持 x=1，不进行 NED 转换或交换轴。
  float x = 0, y = 0;
  std::memcpy(&x, map.cloud.data.data() + map.cloud.point_step, sizeof(float));
  std::memcpy(&y, map.cloud.data.data() + map.cloud.point_step + 4, sizeof(float));
  EXPECT_FLOAT_EQ(x, 1); EXPECT_FLOAT_EQ(y, 0);
}

TEST_F(MapFixture, Md5MatchesFileAndAllowsUppercase)
{
  auto map = readLocalizationMap(path, "");
  EXPECT_EQ(map.md5, "101c8ab259c4e453c6021e119851757a");
  std::string digest = map.md5;
  std::transform(digest.begin(), digest.end(), digest.begin(), [](unsigned char c) { return std::toupper(c); });
  EXPECT_NO_THROW(readLocalizationMap(path, digest));
  EXPECT_THROW(readLocalizationMap(path, "00000000000000000000000000000000"), std::domain_error);
  EXPECT_THROW(readLocalizationMap(path, "not-md5"), std::invalid_argument);
}

TEST_F(MapFixture, InvalidInputDoesNotBecomeAnEmptyMap)
{
  EXPECT_THROW(readLocalizationMap("relative.pcd", ""), std::invalid_argument);
  EXPECT_THROW(readLocalizationMap("/tmp", ""), std::invalid_argument);
  EXPECT_THROW(readLocalizationMap(path + ".missing", ""), std::invalid_argument);
  write("nan nan nan\nnan nan nan\n0 0 0\n1 0 0\n0 1 0\n0 0 1\n");
  EXPECT_THROW(readLocalizationMap(path, ""), std::domain_error);
}

TEST_F(MapFixture, RejectsMissingXyzFields)
{
  std::ofstream(path) << "VERSION .7\nFIELDS intensity\nSIZE 4\nTYPE F\nCOUNT 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1\n";
  EXPECT_THROW(readLocalizationMap(path, ""), std::domain_error);
}

TEST_F(MapFixture, RemovesNonfinitePoints)
{
  write("nan nan nan\n1 0 0\n0 1 0\n0 0 1\n1 1 1\n2 0 1\n");
  EXPECT_EQ(readLocalizationMap(path, "").cloud.width, 5u);
}

std::string status(const std::string& state = "matching", int drone = 2, int schema = 1,
                   const std::string& attempts = "0")
{
  return "{\"state\":\"" + state + "\",\"drone_id\":\"" + std::to_string(drone) +
    "\",\"schema_version\":" + std::to_string(schema) + ",\"attempt_count\":" + attempts +
    ",\"elapsed_ms\":1200,\"fitness_score\":0.12,\"threshold\":0.3,\"ok\":true,\"message\":\"backend status\"}";
}

TEST(MatchProtocol, OkDoesNotMeanLocalized)
{
  auto s = parseMatchStatus(status(), 2);
  EXPECT_EQ(s.state, "matching");
  EXPECT_DOUBLE_EQ(s.elapsed_time, 1.2);
  EXPECT_DOUBLE_EQ(s.fitness_score, 0.12);
  EXPECT_EQ(parseMatchStatus(status("failed"), 2).state, "failed");
  EXPECT_EQ(parseMatchStatus(status("localized"), 2).state, "localized");
}

TEST(MatchProtocol, RejectsOtherDroneSchemaAndMalformedMetrics)
{
  EXPECT_THROW(parseMatchStatus(status(), 1), std::exception);
  EXPECT_THROW(parseMatchStatus(status("matching", 2, 2), 2), std::exception);
  EXPECT_THROW(parseMatchStatus(status("unknown"), 2), std::exception);
  EXPECT_THROW(parseMatchStatus(status("matching", 2, 1, "-1"), 2), std::exception);
  EXPECT_THROW(parseMatchStatus(status("matching", 2, 1, "4294967296"), 2), std::exception);
  EXPECT_THROW(parseMatchStatus("{\"ok\":true}", 2), std::exception);
  EXPECT_THROW(parseMatchStatus("not json", 2), std::exception);
}
}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

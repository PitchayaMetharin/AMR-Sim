// Acceptance tests for the front + rear LiDAR merge core. Written before the
// implementation; the implementer must not edit this file.
#include <cmath>
#include <limits>
#include <vector>

#include "amr_perception/scan_merge.hpp"
#include "gtest/gtest.h"

namespace {

using amr_perception::MergedScanLayout;
using amr_perception::ScanSource;
using amr_perception::SensorPose2D;
using amr_perception::merge_scans;
using amr_perception::stamps_usable;

constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Real mount poses from amr_description/urdf/amr.urdf.xacro, in base_footprint.
const SensorPose2D kFrontPose{0.30805, -0.153, 0.0};
const SensorPose2D kRearPose{-0.308, 0.153, -3.1416};

MergedScanLayout layout() {
  MergedScanLayout result;
  result.angle_min = -kPi;
  result.angle_increment = 2.0 * kPi / 720.0;
  result.bin_count = 720;
  result.range_min = 0.2;
  result.range_max = 20.5;
  return result;
}

// Same shape as the simulated gpu_lidar: 720 rays over +/-2.4 rad, 0.2-20 m.
sensor_msgs::msg::LaserScan real_shaped_scan(float range) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "front_lidar_link";
  scan.angle_min = -2.4f;
  scan.angle_max = 2.4f;
  scan.angle_increment = 0.006675938609987497f;
  scan.range_min = 0.2f;
  scan.range_max = 20.0f;
  scan.ranges.assign(720, range);
  return scan;
}

// Three rays at -0.1, 0.0, +0.1 rad; the middle one is index 1.
sensor_msgs::msg::LaserScan three_ray_scan(float left, float middle, float right) {
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "test_lidar_link";
  scan.angle_min = -0.1f;
  scan.angle_max = 0.1f;
  scan.angle_increment = 0.1f;
  scan.range_min = 0.2f;
  scan.range_max = 20.0f;
  scan.ranges = {left, middle, right};
  return scan;
}

std::size_t bin_of(const MergedScanLayout & grid, double angle) {
  double offset = std::fmod(angle - grid.angle_min, 2.0 * kPi);
  if (offset < 0.0) offset += 2.0 * kPi;
  return static_cast<std::size_t>(std::floor(offset / grid.angle_increment)) %
         grid.bin_count;
}

std::size_t finite_count(const sensor_msgs::msg::LaserScan & scan) {
  std::size_t count = 0;
  for (const float range : scan.ranges) {
    if (std::isfinite(range)) ++count;
  }
  return count;
}

const float kNoReturn = std::numeric_limits<float>::infinity();

}  // namespace

TEST(ScanMerge, WritesOutputGeometryAndMarksEmptyBinsAsNoReturn) {
  const auto grid = layout();
  sensor_msgs::msg::LaserScan out;
  out.header.frame_id = "caller_owned";
  ASSERT_TRUE(merge_scans(
    {ScanSource{three_ray_scan(kNoReturn, 5.0f, kNoReturn), {0.0, 0.0, 0.0}}},
    grid, out));
  EXPECT_EQ(out.header.frame_id, "caller_owned");
  EXPECT_NEAR(out.angle_min, grid.angle_min, 1e-6);
  EXPECT_NEAR(out.angle_increment, grid.angle_increment, 1e-6);
  EXPECT_NEAR(
    out.angle_max, grid.angle_min + (grid.bin_count - 1) * grid.angle_increment, 1e-5);
  EXPECT_NEAR(out.range_min, grid.range_min, 1e-6);
  EXPECT_NEAR(out.range_max, grid.range_max, 1e-6);
  ASSERT_EQ(out.ranges.size(), grid.bin_count);
  EXPECT_TRUE(out.intensities.empty());
  EXPECT_EQ(finite_count(out), 1u);
  EXPECT_NEAR(out.ranges[bin_of(grid, 0.0)], 5.0, 1e-4);
  EXPECT_TRUE(std::isinf(out.ranges[bin_of(grid, kPi / 2.0)]));
  EXPECT_GT(out.ranges[bin_of(grid, kPi / 2.0)], 0.0f);
}

TEST(ScanMerge, RearOnlyObstacleLandsBehindTheRobotInBaseFrame) {
  const auto grid = layout();
  sensor_msgs::msg::LaserScan out;
  ASSERT_TRUE(merge_scans(
    {ScanSource{three_ray_scan(kNoReturn, 3.0f, kNoReturn), kRearPose}}, grid, out));
  // The rear sensor faces backwards: its 0 rad ray at 3 m hits base-frame
  // point (-0.308 - 3, 0.153 + ~0) behind the robot.
  const double x = kRearPose.x + 3.0 * std::cos(kRearPose.yaw);
  const double y = kRearPose.y + 3.0 * std::sin(kRearPose.yaw);
  EXPECT_LT(x, -3.0);
  const std::size_t bin = bin_of(grid, std::atan2(y, x));
  ASSERT_TRUE(std::isfinite(out.ranges[bin]));
  EXPECT_NEAR(out.ranges[bin], std::hypot(x, y), 1e-3);
  EXPECT_EQ(finite_count(out), 1u);
  EXPECT_TRUE(std::isinf(out.ranges[bin_of(grid, 0.0)]));
}

TEST(ScanMerge, OffsetFrontSensorProjectsFromItsMountPoint) {
  const auto grid = layout();
  sensor_msgs::msg::LaserScan out;
  ASSERT_TRUE(merge_scans(
    {ScanSource{three_ray_scan(kNoReturn, kNoReturn, 2.0f), kFrontPose}}, grid, out));
  const double x = kFrontPose.x + 2.0 * std::cos(0.1);
  const double y = kFrontPose.y + 2.0 * std::sin(0.1);
  const std::size_t bin = bin_of(grid, std::atan2(y, x));
  ASSERT_TRUE(std::isfinite(out.ranges[bin]));
  EXPECT_NEAR(out.ranges[bin], std::hypot(x, y), 1e-3);
}

TEST(ScanMerge, NearestReturnWinsWhenSourcesShareABin) {
  const auto grid = layout();
  sensor_msgs::msg::LaserScan out;
  ASSERT_TRUE(merge_scans(
    {
      ScanSource{three_ray_scan(kNoReturn, 4.0f, kNoReturn), {0.0, 0.0, 0.0}},
      ScanSource{three_ray_scan(kNoReturn, 2.0f, kNoReturn), {0.0, 0.0, 0.0}},
    },
    grid, out));
  EXPECT_NEAR(out.ranges[bin_of(grid, 0.0)], 2.0, 1e-4);
  EXPECT_EQ(finite_count(out), 1u);
}

TEST(ScanMerge, IgnoresUnusableSourceReturns) {
  const auto grid = layout();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const float bad : {nan, kNoReturn, -kNoReturn, 0.1f, 25.0f}) {
    sensor_msgs::msg::LaserScan out;
    ASSERT_TRUE(merge_scans(
      {ScanSource{three_ray_scan(bad, bad, bad), {0.0, 0.0, 0.0}}}, grid, out));
    EXPECT_EQ(finite_count(out), 0u) << "source range " << bad;
  }
}

TEST(ScanMerge, DropsProjectedReturnsOutsideTheMergedRangeLimits) {
  auto grid = layout();
  grid.range_min = 1.0;
  grid.range_max = 4.0;
  sensor_msgs::msg::LaserScan out;
  ASSERT_TRUE(merge_scans(
    {ScanSource{three_ray_scan(0.5f, 3.0f, 10.0f), {0.0, 0.0, 0.0}}}, grid, out));
  EXPECT_EQ(finite_count(out), 1u);
  EXPECT_NEAR(out.ranges[bin_of(grid, 0.0)], 3.0, 1e-4);
}

TEST(ScanMerge, FrontAndRearTogetherCoverEveryDirection) {
  const auto grid = layout();
  sensor_msgs::msg::LaserScan front_only;
  ASSERT_TRUE(merge_scans({ScanSource{real_shaped_scan(5.0f), kFrontPose}}, grid, front_only));
  // Front alone leaves the rear blind cone empty.
  EXPECT_TRUE(std::isinf(front_only.ranges[bin_of(grid, kPi)]));
  EXPECT_LT(finite_count(front_only), grid.bin_count);

  sensor_msgs::msg::LaserScan merged;
  ASSERT_TRUE(merge_scans(
    {
      ScanSource{real_shaped_scan(5.0f), kFrontPose},
      ScanSource{real_shaped_scan(5.0f), kRearPose},
    },
    grid, merged));
  EXPECT_EQ(finite_count(merged), grid.bin_count);
  EXPECT_TRUE(std::isfinite(merged.ranges[bin_of(grid, kPi)]));
}

TEST(ScanMerge, RejectsInvalidInputWithoutTouchingOutput) {
  const auto valid = ScanSource{three_ray_scan(1.0f, 1.0f, 1.0f), {0.0, 0.0, 0.0}};
  sensor_msgs::msg::LaserScan sentinel;
  sentinel.ranges = {42.0f};

  auto check_rejected = [&](const std::vector<ScanSource> & sources,
                            const MergedScanLayout & grid, const char * label) {
      auto out = sentinel;
      EXPECT_FALSE(merge_scans(sources, grid, out)) << label;
      ASSERT_EQ(out.ranges.size(), 1u) << label;
      EXPECT_EQ(out.ranges[0], 42.0f) << label;
    };

  check_rejected({}, layout(), "no sources");

  auto grid = layout();
  grid.bin_count = 0;
  check_rejected({valid}, grid, "zero bins");
  grid = layout();
  grid.angle_increment = 0.0;
  check_rejected({valid}, grid, "zero increment");
  grid = layout();
  grid.angle_increment = std::nan("");
  check_rejected({valid}, grid, "nan increment");
  grid = layout();
  grid.range_max = grid.range_min;
  check_rejected({valid}, grid, "empty range window");

  auto mismatched = valid;
  mismatched.scan.ranges.push_back(1.0f);
  check_rejected({mismatched}, layout(), "ranges size disagrees with angles");

  auto bad_increment = valid;
  bad_increment.scan.angle_increment = 0.0f;
  check_rejected({bad_increment}, layout(), "source zero increment");

  auto bad_pose = valid;
  bad_pose.pose.yaw = std::nan("");
  check_rejected({bad_pose}, layout(), "non-finite pose");

  check_rejected({valid, mismatched}, layout(), "one bad source rejects all");
}

TEST(ScanMergeStamps, AcceptsFreshAlignedStamps) {
  EXPECT_TRUE(stamps_usable({100.00, 100.05}, 100.10, 0.5, 0.2));
  EXPECT_TRUE(stamps_usable({100.10, 100.10}, 100.10, 0.5, 0.2));
}

TEST(ScanMergeStamps, RejectsStaleFutureSkewedMissingOrZeroStamps) {
  EXPECT_FALSE(stamps_usable({99.0, 100.05}, 100.10, 0.5, 0.2)) << "one stale";
  EXPECT_FALSE(stamps_usable({100.00, 100.50}, 100.10, 0.5, 0.2)) << "future";
  EXPECT_FALSE(stamps_usable({99.75, 100.05}, 100.10, 0.5, 0.2)) << "skew";
  EXPECT_FALSE(stamps_usable({}, 100.10, 0.5, 0.2)) << "empty";
  EXPECT_FALSE(stamps_usable({0.0, 100.05}, 100.10, 0.5, 0.2)) << "zero stamp";
}

TEST(ScanMergeStamps, RejectsInvalidLimits) {
  EXPECT_FALSE(stamps_usable({100.0, 100.0}, 100.0, 0.0, 0.2));
  EXPECT_FALSE(stamps_usable({100.0, 100.0}, 100.0, 0.5, -0.1));
  EXPECT_FALSE(stamps_usable({100.0, 100.0}, 100.0, kInf, 0.2));
  EXPECT_FALSE(stamps_usable({100.0, 100.0}, std::nan(""), 0.5, 0.2));
}

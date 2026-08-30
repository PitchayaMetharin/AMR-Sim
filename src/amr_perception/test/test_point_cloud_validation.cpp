#include "amr_perception/point_cloud_validation.hpp"
#include "gtest/gtest.h"

namespace {

sensor_msgs::msg::PointCloud2 valid_cloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = "front_lidar_link";
  cloud.header.stamp.sec = 1;
  cloud.width = 1;
  cloud.height = 1;
  cloud.point_step = 12;
  cloud.row_step = 12;
  cloud.data.resize(12);
  uint32_t offset = 0;
  for (const char * name : {"x", "y", "z"}) {
    sensor_msgs::msg::PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
    offset += 4;
  }
  return cloud;
}

}  // namespace

TEST(PointCloudValidation, AcceptsWellFormedXyzCloud) {
  EXPECT_TRUE(amr_perception::has_valid_layout(valid_cloud()));
}

TEST(PointCloudValidation, RejectsMissingFrameOrCoordinates) {
  auto cloud = valid_cloud();
  cloud.header.frame_id.clear();
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));
  cloud = valid_cloud();
  cloud.fields.pop_back();
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));
}

TEST(PointCloudValidation, RejectsInconsistentDataLayout) {
  auto cloud = valid_cloud();
  cloud.data.clear();
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));
}

TEST(PointCloudValidation, RejectsDuplicateOrOverlappingCoordinates) {
  auto cloud = valid_cloud();
  cloud.fields.push_back(cloud.fields.front());
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));

  cloud = valid_cloud();
  cloud.fields[2].offset = cloud.fields[1].offset;
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));
}

TEST(PointCloudValidation, RejectsInvalidCoordinateMetadata) {
  auto cloud = valid_cloud();
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::UINT8;
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));

  cloud = valid_cloud();
  cloud.fields[1].count = 2;
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));

  cloud = valid_cloud();
  cloud.fields[2].offset = cloud.point_step;
  EXPECT_FALSE(amr_perception::has_valid_layout(cloud));
}

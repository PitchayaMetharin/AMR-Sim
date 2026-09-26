#include <chrono>
#include <string>

#include <gtest/gtest.h>

#include "../src/portable_robot_inserter.cpp"

namespace
{

using portable_robot_inserter::SpawnArguments;

SpawnArguments sample_arguments()
{
  SpawnArguments arguments;
  arguments.world = "test_world";
  arguments.name = "amr";
  arguments.x = 1.0;
  arguments.y = 2.0;
  arguments.z = 0.12;
  arguments.yaw = 1.5707963267948966;
  return arguments;
}

std::string isolated_service(const std::string &suffix)
{
  return "/portable_robot_inserter_test/" + suffix;
}

}  // namespace

TEST(PortableRobotInserterTest, BuildsWorldRelativeNoRenamePoseRequest)
{
  EXPECT_EQ(portable_robot_inserter::kRequestTimeoutMs, 30000U);
  const auto request = portable_robot_inserter::build_request(
    sample_arguments(), "<robot name='amr'/>");

  EXPECT_EQ(request.name(), "amr");
  EXPECT_FALSE(request.allow_renaming());
  EXPECT_EQ(request.sdf(), "<robot name='amr'/>");
  EXPECT_EQ(request.relative_to(), "world");
  ASSERT_TRUE(request.has_pose());
  EXPECT_DOUBLE_EQ(request.pose().position().x(), 1.0);
  EXPECT_DOUBLE_EQ(request.pose().position().y(), 2.0);
  EXPECT_DOUBLE_EQ(request.pose().position().z(), 0.12);
  EXPECT_NEAR(request.pose().orientation().z(), 0.7071067811865475, 1e-12);
  EXPECT_NEAR(request.pose().orientation().w(), 0.7071067811865475, 1e-12);
}

TEST(PortableRobotInserterTest, QualifiesBlockingServiceAndRejectsUnsafeWorldNames)
{
  EXPECT_EQ(
    portable_robot_inserter::spawn_service_name("aws_warehouse_world"),
    "/world/aws_warehouse_world/create/blocking");
  EXPECT_THROW(portable_robot_inserter::spawn_service_name(""), std::invalid_argument);
  EXPECT_THROW(portable_robot_inserter::spawn_service_name("bad/name"), std::invalid_argument);
  EXPECT_THROW(portable_robot_inserter::spawn_service_name("bad\\name"), std::invalid_argument);
}

TEST(PortableRobotInserterTest, ParsesRequiredFiniteArguments)
{
  SpawnArguments parsed;
  std::string error;
  ASSERT_TRUE(portable_robot_inserter::parse_arguments(
    {"--world", "aws_warehouse_world", "--name", "amr", "--x", "1", "--y", "2",
      "--z", "0.12", "--yaw", "0.5"}, parsed, error)) << error;
  EXPECT_EQ(parsed.world, "aws_warehouse_world");
  EXPECT_EQ(parsed.name, "amr");
  EXPECT_DOUBLE_EQ(parsed.x, 1.0);
  EXPECT_DOUBLE_EQ(parsed.z, 0.12);
  EXPECT_DOUBLE_EQ(parsed.yaw, 0.5);
}

TEST(PortableRobotInserterTest, RejectsMalformedPoseArguments)
{
  SpawnArguments parsed;
  std::string error;
  EXPECT_FALSE(portable_robot_inserter::parse_arguments(
    {"--world", "world", "--name", "amr", "--x", "nan", "--y", "0", "--z", "0",
      "--yaw", "0"}, parsed, error));
  EXPECT_FALSE(error.empty());
}

TEST(PortableRobotInserterTransportTest, PositiveBlockingServiceResponseIsAccepted)
{
  gz::transport::Node server;
  gz::transport::Node client;
  const auto service = isolated_service("positive");
  std::function<bool(const gz::msgs::EntityFactory &, gz::msgs::Boolean &)> callback =
    [](const gz::msgs::EntityFactory &, gz::msgs::Boolean &reply) {
      reply.set_data(true);
      return true;
    };
  const bool advertised = server.Advertise<gz::msgs::EntityFactory, gz::msgs::Boolean>(
    service, callback);
  ASSERT_TRUE(advertised);
  ASSERT_TRUE(gz::transport::waitForService(client, service, std::chrono::seconds(2)));

  gz::msgs::Boolean reply;
  bool service_result = false;
  const bool request_completed = client.Request(
    service, portable_robot_inserter::build_request(sample_arguments(), "<robot/>"),
    1000, reply, service_result);
  EXPECT_TRUE(portable_robot_inserter::insertion_succeeded(
    request_completed, service_result, reply));
}

TEST(PortableRobotInserterTransportTest, NegativeBlockingServiceResponseIsRejected)
{
  gz::transport::Node server;
  gz::transport::Node client;
  const auto service = isolated_service("negative");
  std::function<bool(const gz::msgs::EntityFactory &, gz::msgs::Boolean &)> callback =
    [](const gz::msgs::EntityFactory &, gz::msgs::Boolean &reply) {
      reply.set_data(false);
      return true;
    };
  const bool advertised = server.Advertise<gz::msgs::EntityFactory, gz::msgs::Boolean>(
    service, callback);
  ASSERT_TRUE(advertised);
  ASSERT_TRUE(gz::transport::waitForService(client, service, std::chrono::seconds(2)));

  gz::msgs::Boolean reply;
  bool service_result = false;
  const bool request_completed = client.Request(
    service, portable_robot_inserter::build_request(sample_arguments(), "<robot/>"),
    1000, reply, service_result);
  EXPECT_FALSE(portable_robot_inserter::insertion_succeeded(
    request_completed, service_result, reply));
}

TEST(PortableRobotInserterTransportTest, UndiscoveredServiceFailsClosed)
{
  gz::transport::Node client;
  const auto service = isolated_service("never_advertised");
  EXPECT_FALSE(gz::transport::waitForService(client, service, std::chrono::milliseconds(50)));

  gz::msgs::Boolean reply;
  EXPECT_FALSE(portable_robot_inserter::insertion_succeeded(false, false, reply));
}

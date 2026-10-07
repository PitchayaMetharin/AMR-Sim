#include <cmath>
#include <limits>
#include <memory>
#include "gtest/gtest.h"
#include "amr_mpc_controller/heading_latched_rpp.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"

using Upstream = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
class InspectableLatch : public amr_mpc_controller::HeadingLatchedRPP
{
public:
  nav_msgs::msg::Path plan() const {return global_plan_;}
};

class HeadingLatchTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("heading_latch_test");
    node->declare_parameter("controller_frequency", 20.0);
    node->declare_parameter("goal.xy_goal_tolerance", 0.07);
    node->declare_parameter("goal.yaw_goal_tolerance", 0.15);
    for (const auto & name : {"latch", "upstream"}) {
      node->declare_parameter(std::string(name) + ".max_angular_accel", 0.4);
      node->declare_parameter(std::string(name) + ".rotate_to_heading_angular_vel", 0.64);
      node->declare_parameter(std::string(name) + ".rotate_to_heading_min_angle", 0.785);
      node->declare_parameter(std::string(name) + ".transform_tolerance", 0.0);
    }
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    // All fixture transforms are supplied synchronously, not by a listener.
    // Match upstream's test buffer contract without introducing a timed wait.
    tf->setUsingDedicatedThread(true);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("heading_latch_costmap");
    costmap->set_parameters({rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base"),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("width", 10), rclcpp::Parameter("height", 10),
      rclcpp::Parameter("origin_x", -5.0), rclcpp::Parameter("origin_y", -5.0),
      rclcpp::Parameter("resolution", 0.05)});
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    costmap->getCostmap()->resetMap(0, 0, 200, 200);
    latch.configure(node, "latch", tf, costmap);
    upstream.configure(node, "upstream", tf, costmap);
    goal.initialize(node, "goal", costmap);
    latch.activate();
    upstream.activate();
    pose.header.frame_id = "map";
    heading(0.0);
  }
  void TearDown() override
  {
    upstream.deactivate(); latch.deactivate();
    upstream.cleanup(); latch.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  }
  void heading(double yaw)
  {
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base";
    transform.transform.rotation = pose.pose.orientation;
    ASSERT_TRUE(tf->setTransform(transform, "fixture", true));
  }
  nav_msgs::msg::Path path(double angle, double distance = 2.0)
  {
    nav_msgs::msg::Path result;
    result.header.frame_id = "map";
    for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0}) {
      auto p = pose;
      p.pose.position.x = fraction * distance * std::cos(angle);
      p.pose.position.y = fraction * distance * std::sin(angle);
      p.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(angle);
      result.poses.push_back(p);
    }
    return result;
  }
  geometry_msgs::msg::TwistStamped command(double linear = 0.0, double angular = 0.0)
  {
    geometry_msgs::msg::Twist speed;
    speed.linear.x = linear; speed.angular.z = angular;
    return latch.computeVelocityCommands(pose, speed, &goal);
  }
  void zero(const geometry_msgs::msg::TwistStamped & cmd)
  {
    EXPECT_DOUBLE_EQ(cmd.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(cmd.twist.angular.z, 0.0);
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  InspectableLatch latch;
  Upstream upstream;
  nav2_controller::SimpleGoalChecker goal;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(HeadingLatchTest, UpstreamTranslatesBelowItsThresholdAfterLargeTurn)
{
  upstream.setPlan(path(2.0));
  geometry_msgs::msg::Twist speed;
  EXPECT_EQ(upstream.computeVelocityCommands(pose, speed, &goal).twist.linear.x, 0.0);
  heading(1.7);  // remaining error 0.30 rad: outside goal yaw tolerance
  EXPECT_GT(upstream.computeVelocityCommands(pose, speed, &goal).twist.linear.x, 0.0);
}

TEST_F(HeadingLatchTest, HoldsBothTurnDirectionsUntilMeasuredStopAndSettle)
{
  for (double sign : {-1.0, 1.0}) {
    heading(0.0);
    latch.setPlan(path(sign * 2.0));
    zero(command(0.2, 0.1));  // mandatory entry stop
    zero(command(0.02));  // still translating: no rotation
    EXPECT_GT(sign * command().twist.angular.z, 0.0);
    heading(sign * 1.7);
    const auto hold = command(0.0, sign * 0.1);
    EXPECT_DOUBLE_EQ(hold.twist.linear.x, 0.0);
    EXPECT_GT(sign * hold.twist.angular.z, 0.0);  // latch below .785
    heading(sign * 1.9);
    zero(command(0.0, sign * 0.04));  // angular velocity has not settled
    zero(command(0.02, 0.0));  // linear velocity has not settled
    EXPECT_GT(command(0.01, 0.03).twist.linear.x, 0.0);
  }
}

TEST_F(HeadingLatchTest, DelegatesAtOrBelowUpstreamRotateThreshold)
{
  for (double angle : {0.3, 0.785}) {
    heading(0.0);
    const auto plan = path(angle);
    latch.setPlan(plan); upstream.setPlan(plan);
    geometry_msgs::msg::Twist speed;
    const auto expected = upstream.computeVelocityCommands(pose, speed, &goal);
    const auto actual = command();
    EXPECT_DOUBLE_EQ(actual.twist.linear.x, expected.twist.linear.x);
    EXPECT_DOUBLE_EQ(actual.twist.angular.z, expected.twist.angular.z);
    EXPECT_GT(actual.twist.linear.x, 0.0);
  }
}

TEST_F(HeadingLatchTest, JustAboveUpstreamThresholdStartsWithCompleteStopBothDirections)
{
  for (double sign : {-1.0, 1.0}) {
    heading(0.0);
    latch.setPlan(path(sign * 0.785001));
    zero(command(0.0, sign * 0.323));
  }
}

TEST_F(HeadingLatchTest, ModerateTurnsDoNotTranslateUntilMeasuredSettleBothDirections)
{
  for (double sign : {-1.0, 1.0}) {
    heading(0.0);
    latch.setPlan(path(sign * 0.816));
    zero(command(0.2, sign * 0.323));  // mandatory entry stop

    heading(sign * 0.033);  // bearing crosses below the upstream 0.785 threshold
    const auto crossing = command(0.0, sign * 0.323);
    EXPECT_DOUBLE_EQ(crossing.twist.linear.x, 0.0);
    EXPECT_GT(sign * crossing.twist.angular.z, 0.0);

    heading(sign * 0.716);  // bearing is inside the active 0.15 yaw tolerance
    zero(command(0.0, sign * 0.323));  // angular velocity is still moving
    zero(command(0.02, 0.0));  // linear velocity is still moving
    EXPECT_GT(command(0.01, 0.03).twist.linear.x, 0.0);
  }
}

TEST_F(HeadingLatchTest, ModerateAndNinetyDegreeAnglesEnterLatch)
{
  for (double sign : {-1.0, 1.0}) {
    for (double angle : {1.0, 1.57079632679489661923}) {
      heading(0.0);
      latch.setPlan(path(sign * angle));
      zero(command());  // even an initially stopped base gets an entry stop
      const auto rotating = command();
      EXPECT_DOUBLE_EQ(rotating.twist.linear.x, 0.0);
      EXPECT_GT(sign * rotating.twist.angular.z, 0.0);
    }
  }
}

TEST_F(HeadingLatchTest, NewPlanResetsLatchAndTerminalHeadingDelegates)
{
  latch.setPlan(path(2.0)); zero(command());
  latch.setPlan(path(0.3)); EXPECT_GT(command().twist.linear.x, 0.0);
  latch.setPlan(path(2.0)); zero(command());
  const auto terminal = path(1.0, 0.02);
  latch.setPlan(terminal); upstream.setPlan(terminal);
  geometry_msgs::msg::Twist speed;
  const auto expected = upstream.computeVelocityCommands(pose, speed, &goal);
  const auto actual = command();
  EXPECT_DOUBLE_EQ(actual.twist.linear.x, expected.twist.linear.x);
  EXPECT_DOUBLE_EQ(actual.twist.angular.z, expected.twist.angular.z);
}

TEST_F(HeadingLatchTest, SettlingHeadingGrowthResumesRotation)
{
  latch.setPlan(path(2.0)); zero(command()); command();
  heading(1.9); zero(command(0.0, 0.1));
  heading(1.7);
  const auto resumed = command();
  EXPECT_DOUBLE_EQ(resumed.twist.linear.x, 0.0);
  EXPECT_GT(resumed.twist.angular.z, 0.0);
}

TEST_F(HeadingLatchTest, CollisionStopsHoldWithUpstreamException)
{
  latch.setPlan(path(2.0));
  auto map = costmap->getCostmap();
  for (unsigned int x = 95; x <= 105; ++x) {
    for (unsigned int y = 95; y <= 105; ++y) {map->setCost(x, y, 254);}
  }
  try {command(); FAIL() << "collision must throw";}
  catch (const nav2_core::PlannerException & error) {
    EXPECT_STREQ(error.what(), "RegulatedPurePursuitController detected collision ahead!");
  }
}

TEST_F(HeadingLatchTest, BadInputsAndMissingTransformsFailClosed)
{
  latch.setPlan(path(2.0));
  pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(command(), nav2_core::PlannerException);
  pose.pose.position.x = 0.0;
  EXPECT_THROW(command(std::numeric_limits<double>::infinity()), nav2_core::PlannerException);
  auto missing = path(2.0); missing.header.frame_id = "missing";
  latch.setPlan(missing);
  EXPECT_THROW(command(), nav2_core::PlannerException);
  latch.setPlan(nav_msgs::msg::Path{});
  EXPECT_THROW(command(), nav2_core::PlannerException);
}

TEST_F(HeadingLatchTest, HoldInspectionRestoresLoopPrefixOnEveryCall)
{
  auto loop = path(2.0);
  auto earlier = loop.poses.front(); earlier.pose.position.x = -0.2;
  loop.poses.insert(loop.poses.begin(), earlier);
  // A later coincident segment must not become the next nearest-search prefix.
  const auto initial = loop.poses;
  loop.poses.insert(loop.poses.end(), initial.begin(), initial.end());
  latch.setPlan(loop);
  zero(command());
  for (int i = 0; i < 5; ++i) {
    EXPECT_DOUBLE_EQ(command().twist.linear.x, 0.0);
    EXPECT_EQ(latch.plan(), loop);
  }
}

TEST_F(HeadingLatchTest, UnsupportedDynamicBatchesRejectBeforeUpstreamMutation)
{
  latch.setPlan(path(2.0)); zero(command());
  for (const auto & invalid : {
      rclcpp::Parameter("latch.use_rotate_to_heading", false),
      rclcpp::Parameter("latch.allow_reversing", true)})
  {
    const auto result = node->set_parameters_atomically({
      rclcpp::Parameter("latch.rotate_to_heading_angular_vel", 0.001), invalid});
    ASSERT_FALSE(result.successful);
    EXPECT_DOUBLE_EQ(node->get_parameter("latch.rotate_to_heading_angular_vel").as_double(), 0.64);
    EXPECT_TRUE(node->get_parameter("latch.use_rotate_to_heading").as_bool());
    EXPECT_FALSE(node->get_parameter("latch.allow_reversing").as_bool());
    // Prove the inherited cap was not changed before the batch was rejected.
    EXPECT_NEAR(command().twist.angular.z, 0.02, 1e-12);
  }
}

TEST_F(HeadingLatchTest, UnsupportedConfigurationRejectsAndLifecycleResetsLatch)
{
  node->declare_parameter("invalid.allow_reversing", true);
  amr_mpc_controller::HeadingLatchedRPP invalid;
  EXPECT_THROW(invalid.configure(node, "invalid", tf, costmap), nav2_core::PlannerException);
  invalid.cleanup();
  latch.setPlan(path(2.0)); zero(command()); command();
  latch.deactivate(); latch.activate();
  heading(1.7);
  EXPECT_GT(command().twist.linear.x, 0.0);
}

TEST_F(HeadingLatchTest, InstalledPluginCanLoadThroughNav2Interface)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto plugin = loader.createSharedInstance("amr_mpc_controller::HeadingLatchedRPP");
  ASSERT_NE(plugin, nullptr);
  plugin->configure(node, "loaded", tf, costmap);
  plugin->activate(); plugin->setPlan(path(2.0));
  geometry_msgs::msg::Twist speed;
  zero(plugin->computeVelocityCommands(pose, speed, &goal));
  plugin->deactivate(); plugin->cleanup();
}

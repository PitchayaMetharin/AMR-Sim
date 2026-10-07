#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <type_traits>

#include "gtest/gtest.h"
#include "amr_interfaces/final_placement_stance.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "amr_mpc_controller/final_position_rpp.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "rcl/time.h"
#include "pluginlib/class_loader.hpp"

using RPP = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

class FinalPositionTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("final_position_test");
    node->declare_parameter("controller_frequency", 20.0);
    node->declare_parameter("normal_goal.xy_goal_tolerance", 0.07);
    node->declare_parameter("normal_goal.yaw_goal_tolerance", 0.15);
    node->declare_parameter("precise_goal.xy_goal_tolerance", 0.01);
    node->declare_parameter("precise_goal.yaw_goal_tolerance", 0.15);
    node->declare_parameter("precise_goal.stateful", false);
    for (const auto & name : {"normal", "precision"}) {
      const std::string prefix(name);
      const bool precise = prefix == "precision";
      node->declare_parameter(prefix + ".desired_linear_vel", precise ? 0.1 : 0.5);
      node->declare_parameter(prefix + ".min_approach_linear_velocity", precise ? 0.02 : 0.05);
      node->declare_parameter(prefix + ".approach_velocity_scaling_dist", 0.60);
      node->declare_parameter(prefix + ".allow_reversing", precise);
      node->declare_parameter(prefix + ".use_rotate_to_heading", !precise);
      node->declare_parameter(prefix + ".max_angular_accel", 0.4);
      node->declare_parameter(prefix + ".rotate_to_heading_angular_vel", 0.64);
      node->declare_parameter(prefix + ".rotate_to_heading_min_angle", 0.785);
      node->declare_parameter(prefix + ".regulated_linear_scaling_min_radius", 0.90);
      node->declare_parameter(prefix + ".regulated_linear_scaling_min_speed", 0.15);
      node->declare_parameter(prefix + ".cost_scaling_dist", 0.55);
      node->declare_parameter(prefix + ".inflation_cost_scaling_factor", 3.0);
      node->declare_parameter(prefix + ".transform_tolerance", 0.0);
    }
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf->setUsingDedicatedThread(true);
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("final_position_costmap");
    costmap->set_parameters({rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base"),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("width", 12), rclcpp::Parameter("height", 10),
      rclcpp::Parameter("origin_x", -6.0), rclcpp::Parameter("origin_y", -5.0),
      rclcpp::Parameter("resolution", 0.05),
      rclcpp::Parameter("footprint", "[[0.6,0.4],[0.6,-0.4],[-0.6,-0.4],[-0.6,0.4]]"),
      rclcpp::Parameter("footprint_padding", 0.01)});
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
    auto * map = costmap->getCostmap();
    for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
      for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) map->setCost(x,y,0);
    }
    normal.configure(node,"normal",tf,costmap);
    precision.configure(node,"precision",tf,costmap);
    normal_goal.initialize(node,"normal_goal",costmap);
    precise_goal.initialize(node,"precise_goal",costmap);
    normal.activate(); precision.activate();
    at_distance(1.0);
  }
  void TearDown() override
  {
    precision.deactivate(); normal.deactivate();
    precision.cleanup(); normal.cleanup();
    costmap->on_cleanup(rclcpp_lifecycle::State());
  }
  void at_distance(double distance)
  {
    const auto stance = amr_interfaces::placement::final_placement_stance(
      101, {-3.4,0.0,M_PI}, {-4.1,0.5,0.075});
    pose.header.frame_id = "map";
    pose.pose.position.x = stance.physical[0] + distance;
    pose.pose.position.y = stance.physical[1];
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(M_PI);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base";
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.rotation = pose.pose.orientation;
    ASSERT_TRUE(tf->setTransform(transform,"fixture",true));
  }
  nav_msgs::msg::Path path(double distance)
  {
    nav_msgs::msg::Path result;
    result.header = pose.header;
    for (double fraction : {0.0,0.25,0.5,0.75,1.0}) {
      auto p = pose;
      p.pose.position.x -= fraction*distance;
      result.poses.push_back(p);
    }
    return result;
  }
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  RPP normal, precision;
  nav2_controller::SimpleGoalChecker normal_goal, precise_goal;
  geometry_msgs::msg::PoseStamped pose;
};

TEST_F(FinalPositionTest, GenuinePublicBaselineUsesIntermediateEndpointScaling)
{
  normal.setPlan(path(2.0));
  const auto travel = normal.computeVelocityCommands(pose,geometry_msgs::msg::Twist{},&normal_goal);
  EXPECT_NEAR(travel.twist.linear.x,0.50,1e-12);
  EXPECT_NEAR(travel.twist.angular.z,0.0,1e-12);
  at_distance(0.30);
  precision.setPlan(path(0.08));
  const auto approach = precision.computeVelocityCommands(pose,geometry_msgs::msg::Twist{},&precise_goal);
  EXPECT_GT(approach.twist.linear.x,0.0);
  EXPECT_LT(approach.twist.linear.x,0.10);
  EXPECT_NEAR(approach.twist.linear.x,0.02,1e-12);
  std::cout << "Genuine public baseline: d=1m normal=" << travel.twist.linear.x
            << "; d=.30m intermediate=.08m precision=" << approach.twist.linear.x << " m/s\n";
}

template<class Controller>
class InspectProfile : public Controller
{
public:
  using ReceiveDecision = typename Controller::ReceiveDecision;
  using ReceiveSnapshot = typename Controller::ReceiveSnapshot;
  using FailureSnapshot = typename Controller::FailureSnapshot;
  static_assert(std::is_trivially_copyable<ReceiveSnapshot>::value &&
    std::is_standard_layout<ReceiveSnapshot>::value, "Receive snapshot must remain fixed-size POD");
  static_assert(std::is_trivially_copyable<FailureSnapshot>::value &&
    std::is_standard_layout<FailureSnapshot>::value, "Failure snapshot must remain fixed-size POD");
  double cap() {std::lock_guard<std::mutex> lock(this->mutex_); return this->desired_linear_vel_;}
  double nominal() {std::lock_guard<std::mutex> lock(this->mutex_); return this->base_desired_linear_vel_;}
  double angular() {std::lock_guard<std::mutex> lock(this->mutex_); return this->rotate_to_heading_angular_vel_;}
  uint64_t callbacks() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->callbacks_;}
  bool eligible() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->eligible_;}
  auto receive_snapshot() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->last_receive_;}
  auto failure_snapshot() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->last_failure_;}
  auto cached() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->sample_;}
  auto receipt() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->receipt_;}
  auto high_water() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->high_water_;}
  auto pending_receipt() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->pending_receipt_;}
  auto pending_start() {std::lock_guard<std::mutex> lock(this->sample_mutex_); return this->pending_episode_start_;}
  void sampling(bool enabled) {std::lock_guard<std::mutex> lock(this->sample_mutex_); this->sampling_ = enabled;}
  void expire_receipt() {std::lock_guard<std::mutex> lock(this->sample_mutex_);
    this->receipt_ -= std::chrono::seconds(1);}
};

class ProfileTest : public FinalPositionTest
{
protected:
  void SetUp() override
  {
    FinalPositionTest::SetUp();
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    clock_at(100000000000LL);
    node->declare_parameter("final_position_profiles.dispatch_dock", std::vector<double>{-3.4,0.0,M_PI});
    node->declare_parameter("final_position_profiles.slot_a", std::vector<double>{-4.1,0.5,0.075});
    node->declare_parameter("final_position_profiles.slot_b", std::vector<double>{-4.1,0.0,0.075});
    for (const auto & prefix : names) {
      node->declare_parameter(prefix + ".max_angular_accel", 0.4);
      node->declare_parameter(prefix + ".rotate_to_heading_angular_vel", 0.64);
      node->declare_parameter(prefix + ".rotate_to_heading_min_angle", 0.785);
      node->declare_parameter(prefix + ".regulated_linear_scaling_min_radius", 0.90);
      node->declare_parameter(prefix + ".regulated_linear_scaling_min_speed", 0.15);
      node->declare_parameter(prefix + ".cost_scaling_dist", 0.55);
      node->declare_parameter(prefix + ".inflation_cost_scaling_factor", 3.0);
      node->declare_parameter(prefix + ".transform_tolerance", 0.0);
    }
    // The first configure installs its guard before the other three Base declares.
    a.configure(node,names[0],tf,costmap);
    b.configure(node,names[1],tf,costmap);
    pa.configure(node,names[2],tf,costmap);
    pb.configure(node,names[3],tf,costmap);
    source = std::make_shared<rclcpp::Node>("physical_fixture_source");
    publisher = source->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/amr/simulation/ground_truth/pose", amr_interfaces::qos::sensor());
    executor.add_node(node->get_node_base_interface());
    executor.add_node(source);
    ASSERT_TRUE(until([this] {return publisher->get_subscription_count() == 4;}));
  }
  void TearDown() override
  {
    pb.deactivate(); pa.deactivate(); b.deactivate(); a.deactivate();
    pb.cleanup(); pa.cleanup(); b.cleanup(); a.cleanup();
    executor.remove_node(node->get_node_base_interface()); executor.remove_node(source);
    publisher.reset(); source.reset();
    FinalPositionTest::TearDown();
  }
  template<class Predicate> bool until(Predicate predicate)
  {
    const auto deadline = std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return predicate();
  }
  void clock_at(int64_t ns)
  {
    ros_now = ns;
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), ns), RCL_RET_OK);
  }
  geometry_msgs::msg::PoseStamped observation()
  {
    auto message = pose;
    message.header.frame_id = "factory_world";
    message.header.stamp = rclcpp::Time(ros_now-1000000, RCL_ROS_TIME);
    return message;
  }
  void send(const geometry_msgs::msg::PoseStamped & message)
  {
    const auto ca=a.callbacks(), cb=b.callbacks(), cpa=pa.callbacks(), cpb=pb.callbacks();
    publisher->publish(message);
    ASSERT_TRUE(until([&] {return a.callbacks()>ca && b.callbacks()>cb &&
      pa.callbacks()>cpa && pb.callbacks()>cpb;}));
  }
  void fresh(double distance)
  {
    at_distance(distance); clock_at(ros_now+1000000); send(observation());
  }
  void activate_all() {a.activate(); b.activate(); pa.activate(); pb.activate();}
  template<class Controller> double command(Controller & controller, double length=2.0)
  {
    controller.setPlan(path(length));
    return controller.computeVelocityCommands(pose,geometry_msgs::msg::Twist{},&precise_goal).twist.linear.x;
  }
  template<class Controller> void reject_observation(Controller & controller)
  {
    const auto sequence = controller.failure_snapshot().sequence;
    try {
      command(controller);
      FAIL() << "Expected unchanged physical-observation guard exception";
    } catch (const nav2_core::PlannerException & error) {
      EXPECT_STREQ(error.what(), "Final-position physical observation missing, stale or inactive");
    }
    const auto failure = controller.failure_snapshot();
    EXPECT_EQ(failure.sequence, sequence+1);
    EXPECT_EQ(failure.callbacks, controller.callbacks());
    EXPECT_EQ(failure.receive.sequence, controller.receive_snapshot().sequence);
    EXPECT_TRUE(failure.configured);
    EXPECT_EQ(failure.eligible, controller.eligible());
    EXPECT_EQ(failure.high_water, controller.high_water());
    EXPECT_EQ(failure.sec, controller.cached().header.stamp.sec);
    EXPECT_EQ(failure.nsec, controller.cached().header.stamp.nanosec);
    EXPECT_DOUBLE_EQ(failure.cached_pose[0], controller.cached().pose.position.x);
    EXPECT_EQ(failure.receipt_ns, std::chrono::duration_cast<std::chrono::nanoseconds>(
      controller.receipt().time_since_epoch()).count());
  }
  template<class Controller> void zero_command(Controller & controller)
  {
    controller.setPlan(path(2.0));
    const auto result = controller.computeVelocityCommands(
      pose, geometry_msgs::msg::Twist{}, &precise_goal);
    EXPECT_EQ(result.header.frame_id, costmap->getBaseFrameID());
    EXPECT_EQ(rclcpp::Time(result.header.stamp).nanoseconds(), ros_now);
    EXPECT_DOUBLE_EQ(result.twist.linear.x, 0.0); EXPECT_DOUBLE_EQ(result.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(result.twist.linear.z, 0.0); EXPECT_DOUBLE_EQ(result.twist.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(result.twist.angular.y, 0.0); EXPECT_DOUBLE_EQ(result.twist.angular.z, 0.0);
  }
  auto future_observation(int64_t ahead = 3333333)
  {
    auto message = observation();
    message.header.stamp = rclcpp::Time(ros_now+ahead, RCL_ROS_TIME);
    return message;
  }
  const std::array<std::string,4> names{"FinalPositionFollowPathA", "FinalPositionFollowPathB",
    "FinalPositionPlacementFollowPathA", "FinalPositionPlacementFollowPathB"};
  InspectProfile<amr_mpc_controller::FinalPositionRPP> a,b;
  InspectProfile<amr_mpc_controller::FinalPositionPlacementRPP> pa,pb;
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::Node::SharedPtr source;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
  int64_t ros_now{100000000000LL};
};

TEST_F(ProfileTest, PublicClockAdmissionHoldsZeroThenAdmitsNewPhysicalPose)
{
  activate_all(); fresh(1.0);
  const auto accepted = a.cached();
  const auto accepted_receipt = a.receipt();
  const auto accepted_high = a.high_water();
  auto future = observation();
  future.header.stamp = rclcpp::Time(ros_now+3333333, RCL_ROS_TIME);
  future.pose.position.x -= 0.40;  // New physical distance .60m, not old 1m.
  send(future);
  EXPECT_FALSE(a.eligible());
  EXPECT_EQ(a.cached(), accepted); EXPECT_EQ(a.receipt(), accepted_receipt);
  EXPECT_EQ(a.high_water(), accepted_high);
  const auto zero = [&](auto & controller) {
      controller.setPlan(path(2.0));
      geometry_msgs::msg::TwistStamped command;
      ASSERT_NO_THROW(command = controller.computeVelocityCommands(
        pose, geometry_msgs::msg::Twist{}, &precise_goal));
      EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
      EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
      EXPECT_DOUBLE_EQ(command.twist.linear.z, 0.0);
      EXPECT_DOUBLE_EQ(command.twist.angular.x, 0.0);
      EXPECT_DOUBLE_EQ(command.twist.angular.y, 0.0);
      EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
      EXPECT_EQ(command.header.frame_id, costmap->getBaseFrameID());
      EXPECT_EQ(rclcpp::Time(command.header.stamp).nanoseconds(), ros_now);
    };
  zero(a); zero(pa);
  clock_at(rclcpp::Time(future.header.stamp).nanoseconds());
  EXPECT_NEAR(command(a), 0.30, 1e-12);
  EXPECT_NEAR(command(pa), 0.10, 1e-12);
  EXPECT_EQ(a.cached(), future);
  EXPECT_EQ(a.high_water(), rclcpp::Time(future.header.stamp).nanoseconds());
  EXPECT_NE(a.cached(), accepted);
}

TEST_F(ProfileTest, ClockBeforePoseAdmitsImmediatelyAndPoseBeforeClockKeepsCallbackReceipt)
{
  activate_all(); fresh(1.0);
  auto message = future_observation(); message.pose.position.x -= 0.40;
  clock_at(rclcpp::Time(message.header.stamp).nanoseconds());
  send(message);
  EXPECT_TRUE(a.eligible()); EXPECT_EQ(a.cached(), message);
  EXPECT_NEAR(command(a), 0.30, 1e-12);
  auto pending = future_observation(); pending.pose.position.x -= 0.60;
  send(pending);
  const auto receipt = a.pending_receipt();
  EXPECT_NE(a.cached(), pending); zero_command(a); zero_command(pa);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  clock_at(rclcpp::Time(pending.header.stamp).nanoseconds());
  EXPECT_NEAR(command(a), 0.20, 1e-12); EXPECT_NEAR(command(pa), 0.10, 1e-12);
  EXPECT_EQ(a.cached(), pending); EXPECT_EQ(a.receipt(), receipt);
  EXPECT_LT(a.receipt(), std::chrono::steady_clock::now()-std::chrono::milliseconds(15));
}

TEST_F(ProfileTest, PendingDuplicateAndReplacementCannotRefreshFirstEpisode)
{
  activate_all(); fresh(1.0);
  const auto cache = a.cached(); const auto accepted_receipt = a.receipt();
  const auto high = a.high_water();
  auto first = future_observation(10000000); send(first);
  const auto start = a.pending_start(); const auto receipt = a.pending_receipt();
  first.pose.position.x += 100.0; // Duplicate payload must not replace pose or receipt.
  send(first);
  EXPECT_EQ(a.pending_start(), start); EXPECT_EQ(a.pending_receipt(), receipt);
  EXPECT_EQ(a.receive_snapshot().decision, decltype(a)::ReceiveDecision::Duplicate);
  zero_command(a); zero_command(pa);
  auto replacement = future_observation(11000000); replacement.pose.position.x -= 0.40;
  send(replacement);
  const auto replacement_receipt = a.pending_receipt();
  EXPECT_EQ(a.pending_start(), start); EXPECT_GT(replacement_receipt, receipt);
  EXPECT_EQ(a.cached(), cache); EXPECT_EQ(a.receipt(), accepted_receipt); EXPECT_EQ(a.high_water(), high);
  clock_at(rclcpp::Time(replacement.header.stamp).nanoseconds());
  EXPECT_NEAR(command(a), 0.30, 1e-12);
  EXPECT_EQ(a.cached(), replacement); EXPECT_EQ(a.receipt(), replacement_receipt);
}

TEST_F(ProfileTest, DuplicatePendingAdmitsOriginalPayloadOnly)
{
  activate_all(); fresh(1.0);
  auto first = future_observation(); first.pose.position.x -= 0.40; send(first);
  const auto receipt = a.pending_receipt();
  auto duplicate = first; duplicate.pose.position.x += 100.0; send(duplicate);
  clock_at(rclcpp::Time(first.header.stamp).nanoseconds());
  EXPECT_NEAR(command(a), 0.30, 1e-12); EXPECT_EQ(a.cached(), first);
  EXPECT_EQ(a.receipt(), receipt);
}

TEST_F(ProfileTest, FrozenClockExpiryAndLateFutureCannotRestartWithoutValidReceive)
{
  activate_all(); fresh(1.0); send(future_observation());
  const auto start = a.pending_start(); zero_command(a); zero_command(pa);
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  send(future_observation(5000000)); // Callback must detect expiry before first command.
  EXPECT_EQ(a.pending_start(), start);
  reject_observation(a); reject_observation(pa);
  send(future_observation(6000000));
  reject_observation(a); reject_observation(pa);
  fresh(1.0); EXPECT_NEAR(command(a), 0.50, 1e-12);
}

TEST_F(ProfileTest, ContinuousFutureStreamAndPlansCannotExtendFirstDeadline)
{
  activate_all(); fresh(1.0); send(future_observation());
  const auto start = a.pending_start();
  int64_t ahead = 3333333;
  while (std::chrono::steady_clock::now()-start < std::chrono::milliseconds(210)) {
    send(future_observation(++ahead));
    a.setPlan(path(2.0)); pa.setPlan(path(2.0));
  }
  EXPECT_EQ(a.pending_start(), start);
  EXPECT_GT(a.callbacks(), 3u);
  reject_observation(a); reject_observation(pa);
  send(future_observation(++ahead)); reject_observation(a);
  fresh(1.0); EXPECT_NEAR(command(a), 0.50, 1e-12);
}

TEST_F(ProfileTest, PendingReorderAndMalformedFailureRequireIndependentValidNonfuture)
{
  activate_all(); fresh(1.0);
  for (bool malformed : {false, true}) {
    send(future_observation(10000000));
    const auto high = a.high_water();
    auto bad = future_observation(9000000);
    if (malformed) {bad.header.frame_id = "map";}
    send(bad); reject_observation(a); reject_observation(pa);
    send(future_observation(11000000)); reject_observation(a); reject_observation(pa);
    EXPECT_EQ(a.high_water(), high);
    send(observation()); reject_observation(a); // Accepted-cache duplicate cannot recover.
    fresh(1.0); // Valid new acquisition is below discarded future, above accepted high-water.
    EXPECT_GT(a.high_water(), high); EXPECT_NEAR(command(a), 0.50, 1e-12);
  }
}

TEST_F(ProfileTest, PendingCatchupStillChecksAcquisitionAndOriginalReceiptFreshness)
{
  activate_all(); fresh(1.0); auto message = future_observation(); send(message);
  clock_at(rclcpp::Time(message.header.stamp).nanoseconds()+200000001);
  reject_observation(a); reject_observation(pa);
  fresh(1.0); message = future_observation(); send(message);
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  clock_at(rclcpp::Time(message.header.stamp).nanoseconds());
  reject_observation(a); reject_observation(pa);
}

TEST_F(ProfileTest, PendingCannotMaskExternalInvalidityRollbackOrInactiveLifecycle)
{
  activate_all(); fresh(1.0); auto message = future_observation(); send(message);
  for (double limit : {-1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    a.setSpeedLimit(limit, false); pa.setSpeedLimit(limit, false);
    EXPECT_THROW(command(a), nav2_core::PlannerException);
    EXPECT_THROW(command(pa), nav2_core::PlannerException);
  }
  a.setSpeedLimit(0.04, false); pa.setSpeedLimit(0.04, false);
  zero_command(a); zero_command(pa);
  clock_at(ros_now-1000000); reject_observation(a); reject_observation(pa);
  clock_at(rclcpp::Time(message.header.stamp).nanoseconds());
  reject_observation(a); reject_observation(pa); // Catch-up cannot recover rollback cancellation.
  fresh(1.0); send(future_observation());
  a.deactivate(); pa.deactivate(); reject_observation(a); reject_observation(pa);
  a.activate(); pa.activate(); reject_observation(a); reject_observation(pa);
  send(future_observation(4000000)); reject_observation(a); // Future alone cannot reopen canceled episode.
  fresh(1.0); EXPECT_NEAR(command(a), 0.04, 1e-12); EXPECT_NEAR(command(pa), 0.04, 1e-12);
}

TEST_F(ProfileTest, CleanupReconfigureRejectsObsoletePublicSubscriptionCallbacks)
{
  activate_all(); fresh(1.0); send(future_observation());
  std::vector<rclcpp::SubscriptionBase::SharedPtr> old_subscriptions;
  node->get_node_base_interface()->for_each_callback_group(
    [&](rclcpp::CallbackGroup::SharedPtr group) {
      group->collect_all_ptrs([&](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
        if (std::string(subscription->get_topic_name()) == "/amr/simulation/ground_truth/pose") {
          old_subscriptions.push_back(subscription);
        }
      }, [](const auto &) {}, [](const auto &) {}, [](const auto &) {}, [](const auto &) {});
    });
  ASSERT_EQ(old_subscriptions.size(), 4u);
  pb.deactivate(); pa.deactivate(); b.deactivate(); a.deactivate();
  pb.cleanup(); pa.cleanup(); b.cleanup(); a.cleanup();
  a.configure(node, names[0], tf, costmap); b.configure(node, names[1], tf, costmap);
  pa.configure(node, names[2], tf, costmap); pb.configure(node, names[3], tf, costmap);
  activate_all();
  const auto seq = a.receive_snapshot().sequence;
  const auto callbacks = a.callbacks();
  std::shared_ptr<void> message = std::make_shared<geometry_msgs::msg::PoseStamped>(observation());
  for (const auto & subscription : old_subscriptions) {
    subscription->handle_message(message, rclcpp::MessageInfo{}); // Actual installed callback dispatch.
  }
  EXPECT_EQ(a.receive_snapshot().sequence, seq); EXPECT_EQ(a.callbacks(), callbacks);
  EXPECT_FALSE(a.eligible()); reject_observation(a); reject_observation(pa);
  old_subscriptions.clear(); // Release handles; no fixture filesystem deletion.
  fresh(1.0); EXPECT_NEAR(command(a), 0.50, 1e-12); EXPECT_NEAR(command(pa), 0.10, 1e-12);
}

TEST_F(ProfileTest, ObservationReceiveDecisionsPreserveCacheReceiptAndFlags)
{
  using Decision = decltype(a)::ReceiveDecision;
  activate_all(); fresh(1.0);
  const auto accepted = a.receive_snapshot();
  EXPECT_EQ(accepted.decision, Decision::Accepted);
  EXPECT_EQ(accepted.callbacks, a.callbacks());
  EXPECT_EQ(accepted.now, ros_now);
  EXPECT_EQ(accepted.acquired, ros_now-1000000);
  EXPECT_EQ(accepted.high_water_after, a.high_water());
  EXPECT_TRUE(accepted.sampling); EXPECT_TRUE(accepted.eligible_after);
  EXPECT_FALSE(accepted.rollback);
  const auto cache = a.cached(); const auto receipt = a.receipt(); const auto high = a.high_water();
  send(observation());
  EXPECT_EQ(a.receive_snapshot().decision, Decision::Duplicate);
  EXPECT_EQ(a.receive_snapshot().sequence, accepted.sequence+1);
  EXPECT_EQ(a.cached(), cache); EXPECT_EQ(a.receipt(), receipt);
  EXPECT_EQ(a.high_water(), high); EXPECT_TRUE(a.eligible());
  auto reordered = observation();
  reordered.header.stamp = rclcpp::Time(ros_now-2000000, RCL_ROS_TIME);
  send(reordered);
  EXPECT_EQ(a.receive_snapshot().decision, Decision::Reordered);
  EXPECT_TRUE(a.receive_snapshot().eligible_before);
  EXPECT_FALSE(a.receive_snapshot().eligible_after);
  EXPECT_EQ(a.cached(), cache); EXPECT_EQ(a.receipt(), receipt); EXPECT_EQ(a.high_water(), high);
  reject_observation(a); EXPECT_TRUE(a.failure_snapshot().ineligible);
  send(observation());
  EXPECT_EQ(a.receive_snapshot().decision, Decision::Duplicate);
  EXPECT_FALSE(a.eligible()); EXPECT_EQ(a.receipt(), receipt); // Cannot recover or refresh.
  fresh(1.0); EXPECT_EQ(a.receive_snapshot().decision, Decision::Accepted);
  EXPECT_TRUE(a.eligible());
  // Exercise the existing !sampling return via the real subscription without
  // changing its callbacks_ counter or touching production callback visibility.
  const auto before = a.receive_snapshot(); const auto stopped_cache = a.cached();
  const auto stopped_receipt = a.receipt(); const auto stopped_high = a.high_water();
  a.sampling(false); publisher->publish(observation());
  ASSERT_TRUE(until([&] {return a.receive_snapshot().sequence > before.sequence;}));
  const auto stopped = a.receive_snapshot();
  EXPECT_EQ(stopped.decision, Decision::NotSampling); EXPECT_FALSE(stopped.sampling);
  EXPECT_EQ(stopped.now, -1); EXPECT_EQ(stopped.callbacks, before.callbacks);
  EXPECT_TRUE(a.eligible()); EXPECT_EQ(a.cached(), stopped_cache);
  EXPECT_EQ(a.receipt(), stopped_receipt); EXPECT_EQ(a.high_water(), stopped_high);
  a.sampling(true);
}

TEST_F(ProfileTest, ObservationInvalidReceiveCapturesEveryPredicateWithoutCacheMutation)
{
  using Decision = decltype(a)::ReceiveDecision;
  activate_all();
  for (int defect=0; defect<9; ++defect) {
    SCOPED_TRACE(defect);
    fresh(1.0);
    const auto cache=a.cached(); const auto receipt=a.receipt(); const auto high=a.high_water();
    auto message=observation();
    if (defect==0) {message.header.frame_id="map";}
    if (defect==1) {message.pose.position.x=std::numeric_limits<double>::quiet_NaN();}
    if (defect==2) {
      message.pose.orientation.x=0.0; message.pose.orientation.y=0.0;
      message.pose.orientation.z=0.0; message.pose.orientation.w=0.0;}
    if (defect==3) {message.header.stamp.sec=-1; message.header.stamp.nanosec=0;}
    if (defect==4) {message.header.stamp.nanosec=1000000000;}
    if (defect==5) {message.header.stamp={};}
    if (defect==6) {message.header.stamp=rclcpp::Time(ros_now+1,RCL_ROS_TIME);}
    if (defect==7) {message.header.stamp=rclcpp::Time(ros_now-200000001,RCL_ROS_TIME);}
    if (defect==8) {message.header.frame_id="map";
      message.pose.orientation.x=0.0; message.pose.orientation.y=0.0;
      message.pose.orientation.z=0.0; message.pose.orientation.w=0.0;
      message.header.stamp=rclcpp::Time(ros_now+1,RCL_ROS_TIME);}
    send(message);
    const auto check_receive = [&](const auto & r) {
      EXPECT_EQ(static_cast<unsigned int>(r.decision),
        static_cast<unsigned int>(defect==6 ? Decision::Pending : Decision::Invalid));
      EXPECT_EQ(r.bad_frame, defect==0 || defect==8);
      EXPECT_EQ(r.bad_pose, defect==1 || defect==2 || defect==8);
      EXPECT_EQ(r.bad_sec, defect==3); EXPECT_EQ(r.bad_nsec, defect==4);
      EXPECT_EQ(r.nonpositive, defect==3 || defect==5);
      EXPECT_EQ(r.future, defect==4 || defect==6 || defect==8);
      EXPECT_EQ(r.stale, defect==3 || defect==5 || defect==7);
      EXPECT_TRUE(r.eligible_before); EXPECT_FALSE(r.eligible_after);
      EXPECT_EQ(r.high_water_before, high); EXPECT_EQ(r.high_water_after, high);
      EXPECT_EQ(r.sec, message.header.stamp.sec); EXPECT_EQ(r.nsec, message.header.stamp.nanosec);
      EXPECT_EQ(r.now, ros_now); EXPECT_GT(r.steady_ns, 0);
      };
    check_receive(a.receive_snapshot()); check_receive(pa.receive_snapshot());
    EXPECT_EQ(a.cached(), cache); EXPECT_EQ(a.receipt(), receipt); EXPECT_EQ(a.high_water(), high);
    EXPECT_FALSE(a.eligible());
    if (defect==6) {
      zero_command(a); zero_command(pa);  // Authorized pure-future hold only.
    } else {
      reject_observation(a); reject_observation(pa);
      EXPECT_TRUE(a.failure_snapshot().ineligible);
      EXPECT_FALSE(a.failure_snapshot().future); // Rejected future stamp never enters cache.
    }
  }
}

TEST_F(ProfileTest, ObservationGuardCapturesAllFailedPredicatesAndClockOrdering)
{
  activate_all();
  reject_observation(a);
  auto missing=a.failure_snapshot();
  EXPECT_TRUE(missing.ineligible); EXPECT_TRUE(missing.acquisition_stale);
  EXPECT_TRUE(missing.receipt_stale); EXPECT_EQ(missing.receive.sequence, 0u);
  EXPECT_FALSE(missing.inactive); EXPECT_FALSE(missing.rollback); EXPECT_FALSE(missing.future);
  fresh(1.0); a.deactivate(); reject_observation(a);
  auto inactive=a.failure_snapshot();
  EXPECT_TRUE(inactive.inactive); EXPECT_FALSE(inactive.ineligible);
  EXPECT_FALSE(inactive.rollback); EXPECT_FALSE(inactive.future);
  EXPECT_FALSE(inactive.acquisition_stale); EXPECT_FALSE(inactive.receipt_stale);
  a.activate();
  const auto prior=ros_now; const auto high=a.high_water(); const auto receipt=a.receipt();
  const auto cache=a.cached();
  clock_at(prior-1000000000);
  reject_observation(a); reject_observation(pa);
  const auto check_rollback = [&](const auto & f) {
    EXPECT_TRUE(f.rollback); EXPECT_TRUE(f.ineligible); EXPECT_TRUE(f.future);
    EXPECT_FALSE(f.inactive); EXPECT_FALSE(f.acquisition_stale); EXPECT_FALSE(f.receipt_stale);
    EXPECT_EQ(f.previous_clock, prior); EXPECT_EQ(f.now, ros_now);
    EXPECT_EQ(f.high_water_before, high); EXPECT_EQ(f.high_water, 0);
    EXPECT_EQ(static_cast<unsigned int>(f.receive.decision),
      static_cast<unsigned int>(decltype(a)::ReceiveDecision::Accepted));
    EXPECT_FALSE(f.receive.rollback); // Guard-first rollback, latest receive was earlier.
    };
  check_rollback(a.failure_snapshot()); check_rollback(pa.failure_snapshot());
  EXPECT_EQ(a.receipt(), receipt); EXPECT_EQ(a.cached(), cache);
  // Callback-first rollback records the old clock/highwater, then the unchanged
  // receive path accepts a valid new epoch and refreshes only that accepted receipt.
  clock_at(prior); fresh(1.0);
  const auto old=a.receive_snapshot(); clock_at(ros_now-1000000000); send(observation());
  const auto epoch=a.receive_snapshot();
  EXPECT_TRUE(epoch.rollback); EXPECT_EQ(epoch.previous_clock, old.now);
  EXPECT_EQ(epoch.high_water_before, old.high_water_after);
  EXPECT_EQ(epoch.decision, decltype(a)::ReceiveDecision::Accepted); EXPECT_TRUE(a.eligible());
  EXPECT_NEAR(command(a),0.50,1e-12);
  clock_at(ros_now+200000001);
  reject_observation(a); reject_observation(pa);
  auto acquisition=a.failure_snapshot();
  EXPECT_TRUE(acquisition.acquisition_stale); EXPECT_FALSE(acquisition.receipt_stale);
  EXPECT_FALSE(acquisition.inactive); EXPECT_FALSE(acquisition.rollback);
  EXPECT_FALSE(acquisition.ineligible); EXPECT_FALSE(acquisition.future);
  fresh(1.0); a.expire_receipt(); pa.expire_receipt();
  const auto expired=a.receipt(); send(observation());
  reject_observation(a); reject_observation(pa);
  auto steady=a.failure_snapshot();
  EXPECT_TRUE(steady.receipt_stale); EXPECT_FALSE(steady.acquisition_stale);
  EXPECT_FALSE(steady.inactive); EXPECT_FALSE(steady.rollback);
  EXPECT_FALSE(steady.ineligible); EXPECT_FALSE(steady.future);
  EXPECT_EQ(steady.receive.decision, decltype(a)::ReceiveDecision::Duplicate);
  EXPECT_EQ(a.receipt(), expired); EXPECT_GT(steady.receipt_age, 0.200);
}

TEST_F(ProfileTest, IndependentNumericEnvelopeWithEndpointAndPrecisionBounds)
{
  activate_all();
  fresh(0.60);
  normal.setPlan(path(2.0));
  EXPECT_NEAR(normal.computeVelocityCommands(
    pose,geometry_msgs::msg::Twist{},&normal_goal).twist.linear.x,0.50,1e-12);
  EXPECT_NEAR(command(a),0.30,1e-12);
  fresh(0.30);
  precision.setPlan(path(0.08));
  EXPECT_NEAR(precision.computeVelocityCommands(
    pose,geometry_msgs::msg::Twist{},&precise_goal).twist.linear.x,0.02,1e-12);
  EXPECT_NEAR(command(pa,0.08),0.02,1e-12);
  EXPECT_NEAR(command(pa,0.18),0.15*0.18/0.60,1e-12);
  for (const auto & pair : std::vector<std::pair<double,double>>{
      {0.0,0.02},{0.04,0.02},{0.10,0.05},{0.15,0.075},{0.30,0.15},
      {0.60,0.30},{0.65,0.325},{0.999,0.4995},{1.0,0.50},{1.001,0.50},{2.0,0.50}})
  {
    fresh(pair.first); EXPECT_NEAR(command(pa),std::min(0.10,pair.second),1e-12);
    EXPECT_NEAR(command(a),std::max(0.05,pair.second),1e-12);
    auto physical=observation();
    physical.pose.position.x=-3.352+pair.first;
    physical.pose.position.y=0.100;
    clock_at(ros_now+1000000);
    physical.header.stamp=rclcpp::Time(ros_now-1000000,RCL_ROS_TIME);
    send(physical);
    EXPECT_NEAR(command(pb),std::min(0.10,pair.second),1e-12);
    EXPECT_NEAR(command(b),std::max(0.05,pair.second),1e-12);
  }
  fresh(0.0); EXPECT_NEAR(command(a),0.05,1e-12);
  EXPECT_NEAR(a.nominal(),0.50,1e-12); EXPECT_NEAR(pa.nominal(),0.50,1e-12);
  // Independent B oracle: the center-slot reference is exactly (-3.352,.100).
  at_distance(1.0);
  auto physical=observation(); physical.pose.position.x=-3.352+1.0;
  physical.pose.position.y=0.100; clock_at(ros_now+1000000);
  physical.header.stamp=rclcpp::Time(ros_now-1000000,RCL_ROS_TIME); send(physical);
  EXPECT_NEAR(command(b),0.50,1e-12);
}

TEST_F(ProfileTest, IntermediateEndpointDecelerationAndPrecisionModeBound)
{
  activate_all(); fresh(2.0);
  const auto normal_short = command(a,0.08);
  const auto precise_short = command(pa,0.08);
  const auto normal_long = command(a,2.0);
  const auto precise_long = command(pa,2.0);
  std::cout << "Endpoint regression physical d>=1: normal .08m=" << normal_short
            << "; precision .08m=" << precise_short << "; normal long=" << normal_long
            << "; precision long=" << precise_long << " m/s" << std::endl;
  EXPECT_NEAR(normal_short,0.50*0.08/0.60,1e-12);
  EXPECT_NEAR(precise_short,0.02,1e-12);
  EXPECT_NEAR(normal_long,0.50,1e-12);
  EXPECT_NEAR(precise_long,0.10,1e-12);
  EXPECT_NEAR(command(b,0.08),0.50*0.08/0.60,1e-12);
  EXPECT_NEAR(command(pb,0.08),0.02,1e-12);
  EXPECT_NEAR(command(b),0.50,1e-12);
  EXPECT_NEAR(command(pb),0.10,1e-12);
  a.setSpeedLimit(0.04,false); b.setSpeedLimit(0.04,false);
  pa.setSpeedLimit(0.04,false); pb.setSpeedLimit(0.04,false);
  EXPECT_NEAR(command(a),0.04,1e-12); EXPECT_NEAR(command(b),0.04,1e-12);
  EXPECT_NEAR(command(pa),0.04,1e-12); EXPECT_NEAR(command(pb),0.04,1e-12);
}

TEST_F(ProfileTest, ExternalBoundsSurvivePlansAndResetUsesNominalReference)
{
  activate_all(); fresh(1.0);
  pa.setSpeedLimit(0.08,false);
  for(int i=0;i<20;++i) EXPECT_NEAR(command(pa),0.08,1e-12);
  pa.setSpeedLimit(20.0,true); EXPECT_NEAR(command(pa),0.10,1e-12);
  pa.setSpeedLimit(10.0,true); EXPECT_NEAR(command(pa),0.05,1e-12);
  pa.setSpeedLimit(0.0,true); EXPECT_NEAR(command(pa),0.10,1e-12);
  pa.setSpeedLimit(200.0,true); EXPECT_NEAR(command(pa),0.10,1e-12);
  pa.setSpeedLimit(0.0,false); fresh(0.30); EXPECT_NEAR(command(pa),0.10,1e-12);
  for (double invalid : {-1.0,std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    pa.setSpeedLimit(invalid,false);
    EXPECT_THROW(command(pa),nav2_core::PlannerException);
    pa.setSpeedLimit(0.04,false); EXPECT_NEAR(command(pa),0.04,1e-12);
  }
  pa.deactivate(); pa.activate(); EXPECT_NEAR(command(pa),0.04,1e-12);
}

TEST_F(ProfileTest, RealSubscriptionFreshCacheAcrossPlansAndLifecycle)
{
  a.activate(); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0); EXPECT_NEAR(command(a),0.50,1e-12); EXPECT_NEAR(command(a,1.8),0.50,1e-12);
  a.deactivate(); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0); a.activate(); EXPECT_NEAR(command(a),0.50,1e-12);
  clock_at(ros_now+200000001); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0); EXPECT_NEAR(command(a),0.50,1e-12);
  // Paused ROS time: receipt bound alone expires, despite acquisition still fresh.
  const auto duplicate=observation();
  std::this_thread::sleep_for(std::chrono::milliseconds(210));
  send(duplicate); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0); EXPECT_NEAR(command(a),0.50,1e-12);
  a.deactivate(); a.cleanup(); EXPECT_FALSE(a.eligible());
  EXPECT_THROW(command(a),nav2_core::PlannerException);
}

TEST_F(ProfileTest, InvalidMessagesAndEpochCannotResurrectCache)
{
  activate_all(); fresh(1.0);
  auto invalid=observation(); invalid.header.frame_id="map";
  send(invalid); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0);
  invalid=observation(); invalid.pose.position.x=std::numeric_limits<double>::quiet_NaN();
  send(invalid); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0);
  invalid=observation(); invalid.pose.orientation.w=0.0; invalid.pose.orientation.z=0.0;
  send(invalid); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0);
  invalid=observation(); invalid.header.stamp=rclcpp::Time(ros_now+1000000,RCL_ROS_TIME);
  send(invalid); zero_command(a);  // Authorized pure-future hold; old cache remains invalid.
  fresh(1.0);  // Invalid future stamp did not poison the high-water mark.
  invalid=observation(); invalid.header.stamp=builtin_interfaces::msg::Time{};
  send(invalid); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0);
  invalid=observation(); invalid.header.stamp.nanosec=1000000000;
  send(invalid); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0);
  invalid=observation(); invalid.header.stamp=rclcpp::Time(ros_now-2000000,RCL_ROS_TIME);
  send(invalid); EXPECT_THROW(command(a),nav2_core::PlannerException);
  send(observation()); EXPECT_THROW(command(a),nav2_core::PlannerException); // Duplicate cannot recover.
  fresh(1.0); EXPECT_NEAR(command(a),0.50,1e-12);
  const auto prior=ros_now;
  clock_at(prior-1000000000); EXPECT_THROW(command(a),nav2_core::PlannerException);
  clock_at(prior); EXPECT_THROW(command(a),nav2_core::PlannerException);
  fresh(1.0); EXPECT_NEAR(command(a),0.50,1e-12);
  clock_at(ros_now-1000000000); send(observation());
  EXPECT_NEAR(command(a),0.50,1e-12); // Callback-first rollback accepts only new epoch message.
}

TEST_F(ProfileTest, ActualAtomicBatchesProtectBothBasesInBothActivationOrders)
{
  for (bool reversed : {false,true}) {
    if (reversed) {b.activate(); a.activate();} else {a.activate(); b.activate();}
    fresh(2.0); a.setSpeedLimit(0.10,false); b.setSpeedLimit(0.10,false);
    EXPECT_NEAR(command(a),0.10,1e-12); EXPECT_NEAR(command(b),0.10,1e-12);
    const auto a_angle=a.angular(), b_angle=b.angular();
    std::vector<rclcpp::Parameter> forbidden{
      {names[0]+".desired_linear_vel",0.50}, {names[0]+".desired_linear_vel",0.30},
      {names[0]+".min_approach_linear_velocity",0.01},
      {names[0]+".allow_reversing",true}, {names[0]+".use_rotate_to_heading",false},
      {names[0]+".approach_velocity_scaling_dist",0.0},
      {names[2]+".approach_velocity_scaling_dist",0.0},
      {names[3]+".approach_velocity_scaling_dist",0.0},
      {names[0]+".min_approach_linear_velocity",std::string("wrong type")},
      {names[0]+".final_position_reference",std::vector<double>{0.0,0.0,0.0}},
      {"final_position_profiles.slot_a",std::vector<double>{0.0,0.0,0.0}},
      {names[1]+".desired_linear_vel",0.50}};
    for (const auto & bad : forbidden) {
      const std::string other = bad.get_name().find(names[1]) == 0 ? names[0] : names[1];
      const auto other_before=node->get_parameter(other+".rotate_to_heading_angular_vel");
      const auto result=node->set_parameters_atomically(
        {bad,rclcpp::Parameter(other+".rotate_to_heading_angular_vel",0.40)});
      EXPECT_FALSE(result.successful) << bad.get_name();
      EXPECT_EQ(node->get_parameter(other+".rotate_to_heading_angular_vel"),other_before);
      EXPECT_NEAR(a.cap(),0.10,1e-12); EXPECT_NEAR(b.cap(),0.10,1e-12);
      EXPECT_NEAR(a.nominal(),0.50,1e-12); EXPECT_NEAR(b.nominal(),0.50,1e-12);
      EXPECT_NEAR(a.angular(),a_angle,1e-12); EXPECT_NEAR(b.angular(),b_angle,1e-12);
      EXPECT_DOUBLE_EQ(node->get_parameter(names[0]+".desired_linear_vel").as_double(),0.50);
      EXPECT_DOUBLE_EQ(node->get_parameter(names[1]+".desired_linear_vel").as_double(),0.50);
    }
    EXPECT_TRUE(node->set_parameters_atomically({
      rclcpp::Parameter(names[0]+".min_approach_linear_velocity",0.05),
      rclcpp::Parameter(names[1]+".approach_velocity_scaling_dist",0.60),
      rclcpp::Parameter(names[2]+".approach_velocity_scaling_dist",0.60),
      rclcpp::Parameter(names[3]+".approach_velocity_scaling_dist",0.60)}).successful);
    a.deactivate(); b.deactivate();
    EXPECT_FALSE(node->set_parameters_atomically({rclcpp::Parameter(names[1]+".desired_linear_vel",0.50)}).successful);
  }
}

TEST_F(ProfileTest, ReverseAndTurnAndCollisionRemainInherited)
{
  activate_all(); fresh(0.30);
  EXPECT_NEAR(command(pa,-0.18),-0.15*0.18/0.60,1e-12);
  fresh(1.0);
  auto turn=path(0.001);
  for(auto & p:turn.poses) p.pose.orientation=nav2_util::geometry_utils::orientationAroundZAxis(M_PI-1.0);
  normal.setPlan(turn); a.setPlan(turn);
  const auto public_turn=normal.computeVelocityCommands(pose,geometry_msgs::msg::Twist{},&normal_goal);
  const auto private_turn=a.computeVelocityCommands(pose,geometry_msgs::msg::Twist{},&normal_goal);
  EXPECT_NEAR(private_turn.twist.linear.x,0.0,1e-12);
  EXPECT_NEAR(private_turn.twist.angular.z,public_turn.twist.angular.z,1e-12);
  auto * map = costmap->getCostmap();
  const auto footprint = costmap->getRobotFootprint();
  std::vector<geometry_msgs::msg::Point> oriented;
  nav2_costmap_2d::transformFootprint(
    pose.pose.position.x, pose.pose.position.y, M_PI, footprint, oriented);
  ASSERT_EQ(oriented.size(), 4u);
  unsigned int x, y;
  ASSERT_TRUE(map->worldToMap(oriented[0].x, oriented[0].y, x, y));
  map->setCost(x, y, nav2_costmap_2d::LETHAL_OBSTACLE);
  unsigned int lethal_cells = 0;
  for (unsigned int row = 0; row < map->getSizeInCellsY(); ++row) {
    for (unsigned int column = 0; column < map->getSizeInCellsX(); ++column) {
      if (map->getCost(column, row) == nav2_costmap_2d::LETHAL_OBSTACLE) {++lethal_cells;}
    }
  }
  ASSERT_EQ(lethal_cells, 1u);
  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(map);
  ASSERT_EQ(checker.footprintCostAtPose(
    pose.pose.position.x, pose.pose.position.y, M_PI, footprint),
    static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
  const std::string expected_collision = "RegulatedPurePursuitController detected collision ahead!";
  try {
    normal.setPlan(path(2.0));
    (void)normal.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, &normal_goal);
    FAIL() << "public controller did not throw on exact footprint vertex collision";
  } catch (const nav2_core::PlannerException & error) {
    ASSERT_EQ(error.what(), expected_collision);
  } catch (const std::exception & error) {
    FAIL() << "public controller threw unexpected exception: " << error.what();
  }
  try {
    (void)command(pa);
    FAIL() << "private controller did not throw on exact footprint vertex collision";
  } catch (const nav2_core::PlannerException & error) {
    ASSERT_EQ(error.what(), expected_collision);
  } catch (const std::exception & error) {
    FAIL() << "private controller threw unexpected exception: " << error.what();
  }
}

TEST_F(FinalPositionTest, StartupContractsAndInstalledPlugins)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core","nav2_core::Controller");
  EXPECT_NE(loader.createSharedInstance("amr_mpc_controller::FinalPositionRPP"),nullptr);
  EXPECT_NE(loader.createSharedInstance("amr_mpc_controller::FinalPositionPlacementRPP"),nullptr);
  for(bool wrong_type : {false,true}) {
    auto bad=std::make_shared<rclcpp_lifecycle::LifecycleNode>(wrong_type ? "bad_type" : "bad_value");
    bad->declare_parameter("final_position_profiles.dispatch_dock",std::vector<double>{-3.4,0.0,M_PI});
    bad->declare_parameter("final_position_profiles.slot_a",std::vector<double>{-4.1,0.5,0.075});
    bad->declare_parameter("final_position_profiles.slot_b",std::vector<double>{-4.1,0.0,0.075});
    if(wrong_type) bad->declare_parameter("FinalPositionFollowPathB.desired_linear_vel",std::string("bad"));
    else bad->declare_parameter("FinalPositionFollowPathB.desired_linear_vel",0.10);
    amr_mpc_controller::FinalPositionRPP candidate;
    EXPECT_THROW(candidate.configure(bad,"FinalPositionFollowPathA",tf,costmap),nav2_core::PlannerException);
  }
  auto missing=std::make_shared<rclcpp_lifecycle::LifecycleNode>("missing_geometry");
  amr_mpc_controller::FinalPositionRPP candidate;
  EXPECT_THROW(candidate.configure(missing,"FinalPositionFollowPathA",tf,costmap),nav2_core::PlannerException);
}

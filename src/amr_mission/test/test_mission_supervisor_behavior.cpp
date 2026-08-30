#include <atomic>
#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#define main mission_supervisor_main
#include "../src/mission_supervisor_node.cpp"
#undef main

using namespace std::chrono_literals;

template<typename Predicate>
bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor,
  Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    if (predicate()) return true;
    std::this_thread::sleep_for(10ms);
  }
  executor.spin_some();
  return predicate();
}

class MissionBehaviorContext {
 public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using Compute = nav2_msgs::action::ComputePathToPose;
  using Smooth = nav2_msgs::action::SmoothPath;
  using Follow = nav2_msgs::action::FollowPath;

  explicit MissionBehaviorContext(const std::string & suffix)
  : peer(std::make_shared<rclcpp::Node>("mission_behavior_peer_" + suffix)),
    tf_broadcaster(peer), supervisor(std::make_shared<amr_mission::MissionSupervisorNode>())
  {
    planner = rclcpp_action::create_server<Compute>(
      peer, "/amr/compute_path_to_pose",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Compute::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Compute>> goal) {
        planner_cancel_requested = true;
        planner_cancel_goal = goal;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Compute>> goal) {
        planner_goals.push_back(goal);
        if (planner_result_mode == ResultMode::SUCCEED) {
          auto result = std::make_shared<Compute::Result>();
          result->path = successful_path();
          goal->succeed(result);
        }
      });
    smoother = rclcpp_action::create_server<Smooth>(
      peer, "/amr/smooth_path",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Smooth::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Smooth>> goal) {
        smoother_cancel_requested = true;
        smoother_cancel_goal = goal;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Smooth>> goal) {
        smoother_goals.push_back(goal);
        auto result = std::make_shared<Smooth::Result>();
        result->path = goal->get_goal()->path;
        result->was_completed = true;
        goal->succeed(result);
      });
    controller = rclcpp_action::create_server<Follow>(
      peer, "/amr/follow_path",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Follow::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> goal) {
        controller_cancel_requested = true;
        controller_cancel_goal = goal;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> goal) {
        controller_goals.push_back(goal);
        controller_goal_checker_ids.push_back(goal->get_goal()->goal_checker_id);
        controller_ids.push_back(goal->get_goal()->controller_id);
        if (controller_result_mode == ResultMode::SUCCEED) {
          goal->succeed(std::make_shared<Follow::Result>());
        } else if (controller_result_mode == ResultMode::ABORT) {
          goal->abort(std::make_shared<Follow::Result>());
        }
      });

    if (supervisor->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE).id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
      supervisor->trigger_transition(
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE).id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      throw std::runtime_error("mission supervisor lifecycle setup failed");
    }
    client = rclcpp_action::create_client<Navigate>(
      peer, "/amr/mission/navigate_to_pose");
    precise_client = rclcpp_action::create_client<Navigate>(
      peer, "/amr/mission/navigate_to_pose_precise");
    retreat_client = rclcpp_action::create_client<Navigate>(
      peer, "/amr/mission/navigate_to_pose_retreat");
    planner_probe = rclcpp_action::create_client<Compute>(peer, "/amr/compute_path_to_pose");
    smoother_probe = rclcpp_action::create_client<Smooth>(peer, "/amr/smooth_path");
    controller_probe = rclcpp_action::create_client<Follow>(peer, "/amr/follow_path");

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_footprint";
    transform.transform.rotation.w = 1.0;
    tf_broadcaster.sendTransform(transform);
  }

  enum class ResultMode { HOLD, SUCCEED, ABORT };

  static nav_msgs::msg::Path successful_path() {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    return path;
  }

  void finish_planner_cancel() {
    if (planner_cancel_goal) {
      planner_cancel_goal->canceled(std::make_shared<Compute::Result>());
    }
  }

  void finish_controller_cancel() {
    if (controller_cancel_goal) {
      controller_cancel_goal->canceled(std::make_shared<Follow::Result>());
    }
  }

  void finish_smoother_cancel() {
    if (smoother_cancel_goal) {
      smoother_cancel_goal->canceled(std::make_shared<Smooth::Result>());
    }
  }

  std::shared_ptr<rclcpp::Node> peer;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
  std::shared_ptr<amr_mission::MissionSupervisorNode> supervisor;
  rclcpp_action::Server<Compute>::SharedPtr planner;
  rclcpp_action::Server<Smooth>::SharedPtr smoother;
  rclcpp_action::Server<Follow>::SharedPtr controller;
  rclcpp_action::Client<Navigate>::SharedPtr client;
  rclcpp_action::Client<Navigate>::SharedPtr precise_client;
  rclcpp_action::Client<Navigate>::SharedPtr retreat_client;
  rclcpp_action::Client<Compute>::SharedPtr planner_probe;
  rclcpp_action::Client<Smooth>::SharedPtr smoother_probe;
  rclcpp_action::Client<Follow>::SharedPtr controller_probe;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Compute>>> planner_goals;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Smooth>>> smoother_goals;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>>> controller_goals;
  std::vector<std::string> controller_goal_checker_ids;
  std::vector<std::string> controller_ids;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Compute>> planner_cancel_goal;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Smooth>> smoother_cancel_goal;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> controller_cancel_goal;
  std::atomic_bool planner_cancel_requested{false};
  std::atomic_bool smoother_cancel_requested{false};
  std::atomic_bool controller_cancel_requested{false};
  ResultMode planner_result_mode{ResultMode::HOLD};
  ResultMode controller_result_mode{ResultMode::HOLD};
};

static nav2_msgs::action::NavigateToPose::Goal valid_goal() {
  nav2_msgs::action::NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.pose.orientation.w = 1.0;
  return goal;
}

TEST(MissionSupervisorBehavior, CancellationDuringPlanningCompletesOnce) {
  MissionBehaviorContext context("planning_cancel");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));

  auto goal_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(goal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto mission_goal = goal_future.get();
  ASSERT_NE(mission_goal, nullptr);
  ASSERT_TRUE(spin_until(executor, [&]() { return !context.planner_goals.empty(); }, 2s));

  auto cancel_future = context.client->async_cancel_goal(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(cancel_future.get()->goals_canceling.empty());
  ASSERT_TRUE(context.planner_cancel_requested.load());
  context.finish_planner_cancel();

  auto result_future = context.client->async_get_result(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(result_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, PreciseEndpointUsesPrivateCheckerAndSharesReservation) {
  MissionBehaviorContext context("precise_endpoint");
  context.planner_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  context.controller_result_mode = MissionBehaviorContext::ResultMode::HOLD;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.precise_client->action_server_is_ready() &&
      context.retreat_client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));

  auto precise_future = context.precise_client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(precise_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto precise_goal = precise_future.get();
  ASSERT_NE(precise_goal, nullptr);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return !context.controller_goal_checker_ids.empty();
  }, 2s));
  EXPECT_EQ(context.controller_goal_checker_ids.back(), "placement_goal_checker");
  ASSERT_FALSE(context.controller_ids.empty());
  EXPECT_EQ(context.controller_ids.back(), "PlacementFollowPath");

  auto normal_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(normal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(normal_future.get(), nullptr);

  auto cancel_future = context.precise_client->async_cancel_goal(precise_goal);
  ASSERT_EQ(executor.spin_until_future_complete(cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(cancel_future.get()->goals_canceling.empty());
  ASSERT_TRUE(context.controller_cancel_requested.load());
  context.finish_controller_cancel();
  auto result_future = context.precise_client->async_get_result(precise_goal);
  ASSERT_EQ(executor.spin_until_future_complete(result_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);

  auto retreat_future = context.retreat_client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(retreat_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto retreat_goal = retreat_future.get();
  ASSERT_NE(retreat_goal, nullptr);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.controller_goal_checker_ids.size() >= 2;
  }, 2s));
  EXPECT_EQ(context.controller_goal_checker_ids.back(), "retreat_goal_checker");
  EXPECT_EQ(context.controller_ids.back(), "PlacementFollowPath");
  auto retreat_cancel_future = context.retreat_client->async_cancel_goal(retreat_goal);
  ASSERT_EQ(executor.spin_until_future_complete(retreat_cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(retreat_cancel_future.get()->goals_canceling.empty());
  ASSERT_TRUE(context.controller_cancel_requested.load());
  context.finish_controller_cancel();
  auto retreat_result_future = context.retreat_client->async_get_result(retreat_goal);
  ASSERT_EQ(executor.spin_until_future_complete(retreat_result_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(retreat_result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, PlannerSuccessThenCancellationDuringFollowing) {
  MissionBehaviorContext context("following_cancel");
  context.planner_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));
  auto goal_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(goal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto mission_goal = goal_future.get();
  ASSERT_NE(mission_goal, nullptr);
  ASSERT_TRUE(spin_until(executor, [&]() { return !context.controller_goals.empty(); }, 2s));

  auto cancel_future = context.client->async_cancel_goal(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_FALSE(cancel_future.get()->goals_canceling.empty());
  ASSERT_TRUE(context.controller_cancel_requested.load());
  context.finish_controller_cancel();
  auto result_future = context.client->async_get_result(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(result_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::CANCELED);
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, SequentialSuccessAndAbortReleaseMissionIdentity) {
  MissionBehaviorContext context("sequential");
  context.planner_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  context.controller_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));
  for (int i = 0; i < 2; ++i) {
    auto goal_future = context.client->async_send_goal(valid_goal());
    ASSERT_EQ(executor.spin_until_future_complete(goal_future, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    auto mission_goal = goal_future.get();
    ASSERT_NE(mission_goal, nullptr);
    auto result_future = context.client->async_get_result(mission_goal);
    ASSERT_EQ(executor.spin_until_future_complete(result_future, 2s),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  }
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, ControllerAbortAllowsNewGoalAndDeactivationStopsPlanning) {
  MissionBehaviorContext context("abort_deactivate");
  context.planner_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  context.controller_result_mode = MissionBehaviorContext::ResultMode::ABORT;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));
  auto first_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(first_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto first_goal = first_future.get();
  ASSERT_NE(first_goal, nullptr);
  auto first_result = context.client->async_get_result(first_goal);
  ASSERT_EQ(executor.spin_until_future_complete(first_result, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::ABORTED);

  context.planner_result_mode = MissionBehaviorContext::ResultMode::HOLD;
  context.controller_result_mode = MissionBehaviorContext::ResultMode::HOLD;
  auto second_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(second_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto second_goal = second_future.get();
  ASSERT_NE(second_goal, nullptr);
  ASSERT_EQ(
    context.supervisor->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.planner_cancel_requested.load();
  }, 2s));
  context.finish_planner_cancel();
  auto second_result = context.client->async_get_result(second_goal);
  ASSERT_EQ(executor.spin_until_future_complete(second_result, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(second_result.get().code, rclcpp_action::ResultCode::ABORTED);

  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, DeactivationDuringFollowingAbortsAfterDownstreamStops) {
  MissionBehaviorContext context("deactivate_following");
  context.planner_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  context.controller_result_mode = MissionBehaviorContext::ResultMode::HOLD;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));
  auto goal_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(goal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto mission_goal = goal_future.get();
  ASSERT_NE(mission_goal, nullptr);
  ASSERT_TRUE(spin_until(executor, [&]() { return !context.controller_goals.empty(); }, 2s));
  ASSERT_EQ(
    context.supervisor->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE).id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.controller_cancel_requested.load();
  }, 2s));
  context.finish_controller_cancel();
  auto result_future = context.client->async_get_result(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(result_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(result_future.get().code, rclcpp_action::ResultCode::ABORTED);
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, MalformedGoalRejectedWithoutProcessException) {
  MissionBehaviorContext context("malformed");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() { return context.client->action_server_is_ready(); }, 2s));
  auto malformed = valid_goal();
  malformed.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  auto goal_future = context.client->async_send_goal(malformed);
  ASSERT_EQ(executor.spin_until_future_complete(goal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(goal_future.get(), nullptr);
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

TEST(MissionSupervisorBehavior, CancellationRacingControllerResultHasOneTerminalOutcome) {
  MissionBehaviorContext context("cancel_result_race");
  context.planner_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  context.controller_result_mode = MissionBehaviorContext::ResultMode::SUCCEED;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(context.supervisor->get_node_base_interface());
  executor.add_node(context.peer);
  ASSERT_TRUE(spin_until(executor, [&]() {
    return context.client->action_server_is_ready() &&
      context.planner_probe->action_server_is_ready() &&
      context.smoother_probe->action_server_is_ready() &&
      context.controller_probe->action_server_is_ready();
  }, 2s));
  auto goal_future = context.client->async_send_goal(valid_goal());
  ASSERT_EQ(executor.spin_until_future_complete(goal_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto mission_goal = goal_future.get();
  ASSERT_NE(mission_goal, nullptr);
  auto cancel_future = context.client->async_cancel_goal(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(cancel_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  auto result_future = context.client->async_get_result(mission_goal);
  ASSERT_EQ(executor.spin_until_future_complete(result_future, 2s),
    rclcpp::FutureReturnCode::SUCCESS);
  const auto result = result_future.get();
  EXPECT_TRUE(result.code == rclcpp_action::ResultCode::SUCCEEDED ||
    result.code == rclcpp_action::ResultCode::CANCELED);
  executor.remove_node(context.peer);
  executor.remove_node(context.supervisor->get_node_base_interface());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

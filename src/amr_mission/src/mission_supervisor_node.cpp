#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "amr_mission/goal_validation.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace amr_mission {

// Exposes the normal, bounded-retreat, and bounded-precision mission actions.
// All endpoints share one lifecycle and mission identity, sequence Nav2
// planning, smoothing, and following, and never publish velocity, so
// controller output still passes the motion gate.
class MissionSupervisorNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using ComputePath = nav2_msgs::action::ComputePathToPose;
  using FollowPath = nav2_msgs::action::FollowPath;
  using MissionGoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;
  using ComputeGoalHandle = rclcpp_action::ClientGoalHandle<ComputePath>;
  using SmootherGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::SmoothPath>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<FollowPath>;

  enum class MissionState {
    IDLE,
    PLANNER_PENDING,
    PLANNER_ACTIVE,
    SMOOTHER_PENDING,
    SMOOTHER_ACTIVE,
    CONTROLLER_PENDING,
    CONTROLLER_ACTIVE,
    CANCELING,
  };

  enum class Completion {
    SUCCEEDED,
    CANCELED,
    ABORTED,
  };

  MissionSupervisorNode()
  : LifecycleNode("mission_supervisor_node"), tf_buffer_(get_clock())
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Bind TF subscriptions to this lifecycle node. The explicit node and
    // non-dedicated-thread form avoids an implicit hidden ROS node while
    // keeping TF callbacks in the owning executor.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      tf_buffer_, shared_from_this(), false);
    planner_client_ = rclcpp_action::create_client<ComputePath>(
      this, "/amr/compute_path_to_pose");
    controller_client_ = rclcpp_action::create_client<FollowPath>(
      this, "/amr/follow_path");
    smoother_client_ = rclcpp_action::create_client<nav2_msgs::action::SmoothPath>(
      this, "/amr/smooth_path");
    server_ = create_mission_server(
      "/amr/mission/navigate_to_pose", "goal_checker", "FollowPath");
    precise_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_precise", "placement_goal_checker",
      "PlacementFollowPath");
    retreat_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_retreat", "retreat_goal_checker",
      "PlacementFollowPath");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    // Deactivation is a fail-closed abort, but downstream cancellation still
    // follows the same identity and terminal-result rules as a public cancel.
    std::shared_ptr<MissionGoalHandle> mission;
    std::shared_ptr<ComputeGoalHandle> planner_goal;
    std::shared_ptr<SmootherGoalHandle> smoother_goal;
    std::shared_ptr<FollowGoalHandle> controller_goal;
    bool pending_acceptance = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mission = mission_goal_;
      if (mission) {
        pending_acceptance = state_ == MissionState::PLANNER_PENDING ||
          state_ == MissionState::SMOOTHER_PENDING ||
          state_ == MissionState::CONTROLLER_PENDING;
        cancel_requested_ = true;
        abort_on_stop_ = true;
        state_ = MissionState::CANCELING;
        planner_goal = planner_goal_;
        smoother_goal = smoother_goal_;
        controller_goal = controller_goal_;
        planner_goal_.reset();
        smoother_goal_.reset();
        controller_goal_.reset();
      }
    }
    cancel_downstream(mission, planner_goal, smoother_goal, controller_goal);
    if (mission && !pending_acceptance && !planner_goal && !smoother_goal &&
      !controller_goal) {
      complete_after_stop(mission);
    }
    return CallbackReturn::SUCCESS;
  }

 private:
  struct CancelTargets {
    std::shared_ptr<MissionGoalHandle> mission;
    std::shared_ptr<ComputeGoalHandle> planner;
    std::shared_ptr<SmootherGoalHandle> smoother;
    std::shared_ptr<FollowGoalHandle> controller;
    bool pending_acceptance{false};
  };

  rclcpp_action::Server<NavigateToPose>::SharedPtr create_mission_server(
    const std::string & endpoint,
    const std::string & goal_checker_id,
    const std::string & controller_id)
  {
    return rclcpp_action::create_server<NavigateToPose>(
      this, endpoint,
      [this, goal_checker_id, controller_id](const rclcpp_action::GoalUUID &,
             std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handle_goal(*goal, goal_checker_id, controller_id);
      },
      [this](const std::shared_ptr<MissionGoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this, goal_checker_id, controller_id](
        const std::shared_ptr<MissionGoalHandle> goal_handle) {
        start_planning(goal_handle, goal_checker_id, controller_id);
      });
  }

  rclcpp_action::GoalResponse handle_goal(
    const NavigateToPose::Goal & goal,
    const std::string & goal_checker_id,
    const std::string & controller_id)
  {
    // Reject before reserving work unless lifecycle, frame, planar geometry,
    // behavior-tree policy, and single-mission rule all hold.
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE ||
        goal.pose.header.frame_id != "map" ||
        !valid_planar_pose(goal.pose.pose) ||
        !goal.behavior_tree.empty()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_reserved_ || state_ != MissionState::IDLE || mission_goal_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    goal_reserved_ = true;
    reserved_goal_checker_id_ = goal_checker_id;
    reserved_controller_id_ = controller_id;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<MissionGoalHandle> & goal_handle)
  {
    const auto targets = mark_cancel(goal_handle, false);
    if (!targets.mission) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_downstream(
      targets.mission, targets.planner, targets.smoother, targets.controller);
    if (!targets.pending_acceptance && !targets.planner && !targets.smoother &&
      !targets.controller) {
      complete_after_stop(targets.mission);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  CancelTargets mark_cancel(
    const std::shared_ptr<MissionGoalHandle> & mission,
    bool abort_on_stop)
  {
    CancelTargets targets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!mission || mission != mission_goal_ || terminal_reported_ ||
        !mission->is_active())
      {
        return targets;
      }
      targets.mission = mission;
      targets.pending_acceptance = state_ == MissionState::PLANNER_PENDING ||
        state_ == MissionState::SMOOTHER_PENDING ||
        state_ == MissionState::CONTROLLER_PENDING;
      cancel_requested_ = true;
      abort_on_stop_ = abort_on_stop;
      state_ = MissionState::CANCELING;
      // Copy accepted handles while locked, then clear them before any
      // downstream call. A handle exists only in its corresponding ACTIVE
      // state, and late response callbacks are handled by identity checks.
      targets.planner = planner_goal_;
      targets.smoother = smoother_goal_;
      targets.controller = controller_goal_;
      planner_goal_.reset();
      smoother_goal_.reset();
      controller_goal_.reset();
    }
    return targets;
  }

  void start_planning(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const std::string & goal_checker_id,
    const std::string & controller_id)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!mission || !goal_reserved_ || mission_goal_ ||
        reserved_goal_checker_id_ != goal_checker_id ||
        reserved_controller_id_ != controller_id)
      {
        return;
      }
      mission_goal_ = mission;
      goal_reserved_ = false;
      mission_goal_checker_id_ = goal_checker_id;
      mission_controller_id_ = controller_id;
      cancel_requested_ = false;
      abort_on_stop_ = false;
      terminal_reported_ = false;
      state_ = MissionState::PLANNER_PENDING;
      mission_start_ = get_clock()->now();
      last_ros_time_ = mission_start_;
    }

    // Fail closed when either required downstream server is unavailable.
    if (!planner_client_->action_server_is_ready() ||
        !smoother_client_->action_server_is_ready() ||
        !controller_client_->action_server_is_ready()) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission == mission_goal_ && state_ == MissionState::PLANNER_PENDING)
          state_ = MissionState::IDLE;
      }
      abort(mission, "planner or controller action is unavailable");
      return;
    }

    ComputePath::Goal goal;
    goal.goal = mission->get_goal()->pose;
    goal.planner_id = "GridBased";
    goal.use_start = false;
    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, mission](ComputeGoalHandle::SharedPtr planner_goal) {
        bool cancel_late = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!planner_goal) {
            // A rejection is terminal only for the still-current pending
            // request. A cancellation race is completed as canceled.
            if (mission != mission_goal_ || terminal_reported_) return;
            if (cancel_requested_) {
              state_ = MissionState::CANCELING;
            } else {
              state_ = MissionState::IDLE;
            }
          } else if (mission == mission_goal_ && !terminal_reported_ &&
            state_ == MissionState::PLANNER_PENDING && !cancel_requested_)
          {
            planner_goal_ = planner_goal;
            state_ = MissionState::PLANNER_ACTIVE;
          } else {
            cancel_late = true;
          }
        }
        if (planner_goal && cancel_late) {
          if (!cancel_planner_goal(planner_goal)) complete_after_stop(mission);
          return;
        }
        if (!planner_goal) {
          if (is_canceling(mission)) {
            complete_after_stop(mission);
          } else {
            abort(mission, "planner rejected the mission goal");
          }
        }
      };
    options.result_callback =
      [this, mission](const ComputeGoalHandle::WrappedResult & result) {
        nav_msgs::msg::Path path;
        bool process = false;
        bool cancel = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) return;
          // The planner handle is cleared before inspecting its terminal
          // result or starting the controller.
          planner_goal_.reset();
          process = true;
          cancel = cancel_requested_;
          if (cancel) {
            state_ = MissionState::CANCELING;
          } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && !result.result->path.poses.empty())
          {
            state_ = MissionState::SMOOTHER_PENDING;
            path = result.result->path;
          }
        }
        if (!process) return;
        if (cancel) {
          complete_after_stop(mission);
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !result.result || result.result->path.poses.empty()) {
          abort(mission, "global planning failed");
          return;
        }
        start_smoothing(mission, path);
      };
    planner_client_->async_send_goal(goal, options);
  }

  void start_smoothing(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const nav_msgs::msg::Path & path)
  {
    if (!is_current_smoother_pending(mission)) {
      if (is_canceling(mission)) complete_after_stop(mission);
      return;
    }

    nav2_msgs::action::SmoothPath::Goal goal;
    goal.path = path;
    goal.smoother_id = "simple_smoother";
    goal.max_smoothing_duration.sec = 1;
    goal.max_smoothing_duration.nanosec = 0;
    goal.check_for_collisions = true;
    rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, mission](SmootherGoalHandle::SharedPtr smoother_goal) {
        bool cancel_late = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!smoother_goal) {
            if (mission != mission_goal_ || terminal_reported_) return;
            if (cancel_requested_) {
              state_ = MissionState::CANCELING;
            } else {
              state_ = MissionState::IDLE;
            }
          } else if (mission == mission_goal_ && !terminal_reported_ &&
            state_ == MissionState::SMOOTHER_PENDING && !cancel_requested_)
          {
            smoother_goal_ = smoother_goal;
            state_ = MissionState::SMOOTHER_ACTIVE;
          } else {
            cancel_late = true;
          }
        }
        if (smoother_goal && cancel_late) {
          if (!cancel_smoother_goal(smoother_goal)) complete_after_stop(mission);
          return;
        }
        if (!smoother_goal) {
          if (is_canceling(mission)) {
            complete_after_stop(mission);
          } else {
            abort(mission, "smoother rejected the planned path");
          }
        }
      };
    options.result_callback =
      [this, mission](const SmootherGoalHandle::WrappedResult & result) {
        nav_msgs::msg::Path smoothed_path;
        bool process = false;
        bool cancel = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) return;
          smoother_goal_.reset();
          process = true;
          cancel = cancel_requested_;
          if (cancel) {
            state_ = MissionState::CANCELING;
          } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->was_completed &&
            !result.result->path.poses.empty())
          {
            state_ = MissionState::CONTROLLER_PENDING;
            smoothed_path = result.result->path;
          }
        }
        if (!process) return;
        if (cancel) {
          complete_after_stop(mission);
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !result.result || !result.result->was_completed ||
            result.result->path.poses.empty()) {
          abort(mission, "path smoothing failed or was incomplete");
          return;
        }
        start_following(mission, smoothed_path);
      };
    smoother_client_->async_send_goal(goal, options);
  }

  void start_following(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const nav_msgs::msg::Path & path)
  {
    std::string goal_checker_id;
    std::string controller_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission != mission_goal_ || terminal_reported_ || cancel_requested_ ||
        state_ != MissionState::CONTROLLER_PENDING)
      {
        if (mission == mission_goal_ && cancel_requested_) {
          state_ = MissionState::CANCELING;
        }
        // No downstream controller request has been sent yet in this branch.
        // Completion is safe after releasing the mutex.
      } else {
        // The actual send occurs below, outside the mutex.
        goal_checker_id = mission_goal_checker_id_;
        controller_id = mission_controller_id_;
      }
    }
    if (!is_current_controller_pending(mission)) {
      if (is_canceling(mission)) complete_after_stop(mission);
      return;
    }

    FollowPath::Goal goal;
    goal.path = path;
    goal.controller_id = controller_id;
    goal.goal_checker_id = goal_checker_id;
    rclcpp_action::Client<FollowPath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, mission](FollowGoalHandle::SharedPtr controller_goal) {
        bool cancel_late = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!controller_goal) {
            if (mission != mission_goal_ || terminal_reported_) return;
            if (cancel_requested_) {
              state_ = MissionState::CANCELING;
            } else {
              state_ = MissionState::IDLE;
            }
          } else if (mission == mission_goal_ && !terminal_reported_ &&
            state_ == MissionState::CONTROLLER_PENDING && !cancel_requested_)
          {
            controller_goal_ = controller_goal;
            state_ = MissionState::CONTROLLER_ACTIVE;
          } else {
            cancel_late = true;
          }
        }
        if (controller_goal && cancel_late) {
          if (!cancel_controller_goal(controller_goal)) complete_after_stop(mission);
          return;
        }
        if (!controller_goal) {
          if (is_canceling(mission)) {
            complete_after_stop(mission);
          } else {
            abort(mission, "controller rejected the planned path");
          }
        }
      };
    options.feedback_callback =
      [this, mission](
        FollowGoalHandle::SharedPtr,
        const std::shared_ptr<const FollowPath::Feedback> feedback)
      {
        process_feedback(mission, feedback);
      };
    options.result_callback =
      [this, mission](const FollowGoalHandle::WrappedResult & result) {
        bool current = false;
        bool canceled = false;
        bool abort_requested = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) return;
          // The controller handle is cleared before processing its terminal
          // result. A cancellation/result race is therefore terminally safe.
          controller_goal_.reset();
          current = true;
          canceled = cancel_requested_ ||
            result.code == rclcpp_action::ResultCode::CANCELED;
          abort_requested = abort_on_stop_;
          state_ = canceled ? MissionState::CANCELING : MissionState::IDLE;
        }
        if (!current) return;
        if (canceled) {
          complete_public(mission, abort_requested ? Completion::ABORTED :
            Completion::CANCELED);
        } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          complete_public(mission, Completion::SUCCEEDED);
        } else {
          abort(mission, "path following failed");
        }
      };
    controller_client_->async_send_goal(goal, options);
  }

  void process_feedback(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const std::shared_ptr<const FollowPath::Feedback> & feedback)
  {
    if (!feedback) return;
    rclcpp::Time ros_now = get_clock()->now();
    geometry_msgs::msg::PoseStamped current_pose;
    if (!latest_base_pose(current_pose)) {
      request_cancel(mission, "map to base_footprint TF is unavailable");
      return;
    }

    std::shared_ptr<NavigateToPose::Feedback> mission_feedback;
    bool backward_time = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission != mission_goal_ || terminal_reported_ ||
        state_ != MissionState::CONTROLLER_ACTIVE || cancel_requested_)
      {
        return;
      }
      if (ros_now.nanoseconds() < last_ros_time_.nanoseconds() ||
        ros_now.nanoseconds() < mission_start_.nanoseconds())
      {
        backward_time = true;
      } else {
        last_ros_time_ = ros_now;
        mission_feedback = std::make_shared<NavigateToPose::Feedback>();
        mission_feedback->current_pose = current_pose;
        mission_feedback->distance_remaining = feedback->distance_to_goal;
        mission_feedback->navigation_time = ros_now - mission_start_;
      }
    }
    if (backward_time) {
      request_cancel(mission, "ROS navigation time moved backward");
      return;
    }
    if (mission_feedback) {
      mission->publish_feedback(mission_feedback);
    }
  }

  bool latest_base_pose(geometry_msgs::msg::PoseStamped & pose) {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        "map", "base_footprint", tf2::TimePointZero);
      pose.header = transform.header;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Unable to populate mission feedback pose: %s", exception.what());
      return false;
    }
  }

  void request_cancel(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const char * reason)
  {
    RCLCPP_WARN(get_logger(), "Mission cancellation requested: %s", reason);
    const auto targets = mark_cancel(mission, false);
    if (!targets.mission) return;
    cancel_downstream(
      targets.mission, targets.planner, targets.smoother, targets.controller);
    if (!targets.pending_acceptance && !targets.planner && !targets.smoother &&
      !targets.controller) {
      complete_after_stop(targets.mission);
    }
  }

  void abort(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const char * reason)
  {
    RCLCPP_WARN(get_logger(), "Mission aborted: %s", reason);
    bool can_complete = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission == mission_goal_ && !terminal_reported_ &&
        !cancel_requested_ && !planner_goal_ && !smoother_goal_ &&
        !controller_goal_ &&
        state_ != MissionState::PLANNER_PENDING &&
        state_ != MissionState::SMOOTHER_PENDING &&
        state_ != MissionState::CONTROLLER_PENDING)
      {
        can_complete = true;
      }
    }
    if (can_complete) complete_public(mission, Completion::ABORTED);
  }

  void complete_after_stop(const std::shared_ptr<MissionGoalHandle> & mission) {
    Completion completion = Completion::CANCELED;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission != mission_goal_ || terminal_reported_) return;
      if (planner_goal_ || controller_goal_ ||
        state_ == MissionState::PLANNER_PENDING ||
        smoother_goal_ ||
        state_ == MissionState::SMOOTHER_PENDING ||
        state_ == MissionState::CONTROLLER_PENDING)
      {
        return;
      }
      completion = abort_on_stop_ ? Completion::ABORTED : Completion::CANCELED;
    }
    complete_public(mission, completion);
  }

  void complete_public(
    const std::shared_ptr<MissionGoalHandle> & mission,
    Completion completion)
  {
    bool complete = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission != mission_goal_ || terminal_reported_) return;
      terminal_reported_ = true;
      state_ = MissionState::IDLE;
      goal_reserved_ = false;
      cancel_requested_ = false;
      mission_goal_.reset();
      mission_goal_checker_id_.clear();
      reserved_goal_checker_id_.clear();
      mission_controller_id_.clear();
      reserved_controller_id_.clear();
      planner_goal_.reset();
      smoother_goal_.reset();
      controller_goal_.reset();
      complete = true;
    }
    if (!complete || !mission->is_active()) return;
    auto result = std::make_shared<NavigateToPose::Result>();
    switch (completion) {
      case Completion::SUCCEEDED:
        mission->succeed(result);
        break;
      case Completion::CANCELED:
        mission->canceled(result);
        break;
      case Completion::ABORTED:
        mission->abort(result);
        break;
    }
  }

  bool is_canceling(const std::shared_ptr<MissionGoalHandle> & mission) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mission == mission_goal_ && !terminal_reported_ && cancel_requested_;
  }

  bool is_current_controller_pending(
    const std::shared_ptr<MissionGoalHandle> & mission) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return mission == mission_goal_ && !terminal_reported_ &&
      !cancel_requested_ && state_ == MissionState::CONTROLLER_PENDING;
  }

  bool is_current_smoother_pending(
    const std::shared_ptr<MissionGoalHandle> & mission) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return mission == mission_goal_ && !terminal_reported_ &&
      !cancel_requested_ && state_ == MissionState::SMOOTHER_PENDING;
  }

  void cancel_downstream(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const std::shared_ptr<ComputeGoalHandle> & planner_goal,
    const std::shared_ptr<SmootherGoalHandle> & smoother_goal,
    const std::shared_ptr<FollowGoalHandle> & controller_goal)
  {
    (void)mission;
    bool planner_unknown = planner_goal && !cancel_planner_goal(planner_goal);
    bool smoother_unknown = smoother_goal && !cancel_smoother_goal(smoother_goal);
    bool controller_unknown = controller_goal && !cancel_controller_goal(controller_goal);
    const bool cancellation_still_pending =
      (planner_goal && !planner_unknown) ||
      (smoother_goal && !smoother_unknown) ||
      (controller_goal && !controller_unknown);
    if (mission && (planner_unknown || smoother_unknown || controller_unknown) &&
      !cancellation_still_pending) {
      // The action client has already reported this copied handle as terminal;
      // the result callback may race, but no active downstream goal remains.
      complete_after_stop(mission);
    }
  }

  bool cancel_planner_goal(const ComputeGoalHandle::SharedPtr & planner_goal) {
    try {
      planner_client_->async_cancel_goal(planner_goal);
      return true;
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_WARN(get_logger(), "Planner goal was already terminal during cancellation");
      return false;
    }
  }

  bool cancel_smoother_goal(const SmootherGoalHandle::SharedPtr & smoother_goal) {
    try {
      smoother_client_->async_cancel_goal(smoother_goal);
      return true;
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_WARN(get_logger(), "Smoother goal was already terminal during cancellation");
      return false;
    }
  }

  bool cancel_controller_goal(const FollowGoalHandle::SharedPtr & controller_goal) {
    try {
      controller_client_->async_cancel_goal(controller_goal);
      return true;
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_WARN(get_logger(), "Controller goal was already terminal during cancellation");
      return false;
    }
  }

  rclcpp_action::Server<NavigateToPose>::SharedPtr server_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr precise_server_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr retreat_server_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr controller_client_;
  mutable std::mutex mutex_;
  MissionState state_{MissionState::IDLE};
  bool goal_reserved_{false};
  bool cancel_requested_{false};
  bool abort_on_stop_{false};
  bool terminal_reported_{false};
  std::string reserved_goal_checker_id_;
  std::string reserved_controller_id_;
  std::string mission_goal_checker_id_;
  std::string mission_controller_id_;
  std::shared_ptr<MissionGoalHandle> mission_goal_;
  ComputeGoalHandle::SharedPtr planner_goal_;
  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smoother_client_;
  SmootherGoalHandle::SharedPtr smoother_goal_;
  FollowGoalHandle::SharedPtr controller_goal_;
  rclcpp::Time mission_start_;
  rclcpp::Time last_ros_time_;
  tf2_ros::Buffer tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace amr_mission

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_mission::MissionSupervisorNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

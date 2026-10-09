#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <string>

#include "amr_mission/goal_validation.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/smooth_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/create_publisher.hpp"
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

  enum class StatusStage {
    PLANNING,
    SMOOTHING,
    FOLLOWING,
    CANCELING,
    TERMINAL,
  };

  enum class StatusOutcome {
    PENDING,
    SUCCEEDED,
    CANCELED,
    ABORTED,
    FAULT,
  };

  enum class FaultClass {
    NONE,
    OBSTACLE_BLOCKAGE,
    PLANNER_ABORT,
    SMOOTHER_ABORT,
    CONTROLLER_ABORT,
    LOCALIZATION_UNAVAILABLE,
    CANCELLATION,
    NAVIGATION_FAULT,
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
    if (!status_publisher_) {
      try {
        status_publisher_ = rclcpp::create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          *this, "/amr/mission/status", rclcpp::QoS(10));
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "Mission status publisher unavailable: %s", exception.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Mission status publisher unavailable");
      }
    }
    controller_log_subscription_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100),
      [this](const rcl_interfaces::msg::Log::SharedPtr message) {
        controller_log_callback(message);
      });
    server_ = create_mission_server(
      "/amr/mission/navigate_to_pose", "GridBased", "goal_checker", "FollowPath");
    precise_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_precise", "PrecisionGridBased", "placement_goal_checker",
      "PlacementFollowPath");
    retreat_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_retreat", "PrecisionGridBased", "retreat_goal_checker",
      "PlacementFollowPath");
    dispatch_a_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_dispatch_a", "ExactGoalLattice", "goal_checker",
      "FinalPositionFollowPathA");
    dispatch_b_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_dispatch_b", "ExactGoalLattice", "goal_checker",
      "FinalPositionFollowPathB");
    dispatch_a_precise_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_dispatch_a_precise", "PrecisionGridBased",
      "placement_goal_checker", "FinalPositionPlacementFollowPathA");
    dispatch_b_precise_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_dispatch_b_precise", "PrecisionGridBased",
      "placement_goal_checker", "FinalPositionPlacementFollowPathB");
    dispatch_b_clear_approach_server_ = create_mission_server(
      "/amr/mission/navigate_to_pose_dispatch_b_clear_approach", "PrecisionGridBased",
      "retreat_goal_checker", "FinalPositionPlacementFollowPathB");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    // Deactivation is a fail-closed abort, but downstream cancellation still
    // follows the same identity and terminal-result rules as a public cancel.
    std::shared_ptr<MissionGoalHandle> mission;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      mission = mission_goal_;
    }
    const auto targets = mark_cancel(mission, true, "lifecycle deactivation requested");
    if (!targets.mission) {
      return CallbackReturn::SUCCESS;
    }
    cancel_downstream(
      targets.mission, targets.planner, targets.smoother, targets.controller);
    if (!targets.pending_acceptance && !targets.planner && !targets.smoother &&
      !targets.controller) {
      complete_after_stop(targets.mission);
    }
    return CallbackReturn::SUCCESS;
  }

 private:
  static bool is_dispatch_controller(const std::string & controller) {
    return controller == "FinalPositionFollowPathA" || controller == "FinalPositionFollowPathB" ||
      controller == "FinalPositionPlacementFollowPathA" ||
      controller == "FinalPositionPlacementFollowPathB";
  }

  static bool requested_goal_parity(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & requested)
  {
    if (path.poses.empty() || path.header.frame_id != "map") return false;
    for (const auto & point : path.poses) {
      if (point.header.frame_id != "map" || !valid_planar_pose(point.pose)) return false;
    }
    const auto & final = path.poses.back().pose;
    const auto & goal = requested.pose;
    const auto yaw = [](const geometry_msgs::msg::Quaternion & q) {
      return std::atan2(2.0*q.w*q.z, 1.0-2.0*q.z*q.z);
    };
    const double difference = yaw(final.orientation)-yaw(goal.orientation);
    return std::hypot(final.position.x-goal.position.x, final.position.y-goal.position.y) <= 1e-6 &&
      std::abs(std::atan2(std::sin(difference),std::cos(difference))) <= 1e-6;
  }

  struct CancelTargets {
    std::shared_ptr<MissionGoalHandle> mission;
    std::shared_ptr<ComputeGoalHandle> planner;
    std::shared_ptr<SmootherGoalHandle> smoother;
    std::shared_ptr<FollowGoalHandle> controller;
    bool pending_acceptance{false};
  };

  static const char * status_stage_name(StatusStage stage) {
    switch (stage) {
      case StatusStage::PLANNING: return "PLANNING";
      case StatusStage::SMOOTHING: return "SMOOTHING";
      case StatusStage::FOLLOWING: return "FOLLOWING";
      case StatusStage::CANCELING: return "CANCELING";
      case StatusStage::TERMINAL: return "TERMINAL";
    }
    return "TERMINAL";
  }

  static const char * status_outcome_name(StatusOutcome outcome) {
    switch (outcome) {
      case StatusOutcome::PENDING: return "PENDING";
      case StatusOutcome::SUCCEEDED: return "SUCCEEDED";
      case StatusOutcome::CANCELED: return "CANCELED";
      case StatusOutcome::ABORTED: return "ABORTED";
      case StatusOutcome::FAULT: return "FAULT";
    }
    return "FAULT";
  }

  static const char * fault_class_name(FaultClass fault_class) {
    switch (fault_class) {
      case FaultClass::NONE: return "NONE";
      case FaultClass::OBSTACLE_BLOCKAGE: return "OBSTACLE_BLOCKAGE";
      case FaultClass::PLANNER_ABORT: return "PLANNER_ABORT";
      case FaultClass::SMOOTHER_ABORT: return "SMOOTHER_ABORT";
      case FaultClass::CONTROLLER_ABORT: return "CONTROLLER_ABORT";
      case FaultClass::LOCALIZATION_UNAVAILABLE: return "LOCALIZATION_UNAVAILABLE";
      case FaultClass::CANCELLATION: return "CANCELLATION";
      case FaultClass::NAVIGATION_FAULT: return "NAVIGATION_FAULT";
    }
    return "NAVIGATION_FAULT";
  }

  static std::string canonical_goal_uuid(const rclcpp_action::GoalUUID & goal_id) {
    static constexpr char hexadecimal[] = "0123456789abcdef";
    std::string result;
    result.reserve(32);
    for (const auto byte : goal_id) {
      const auto value = static_cast<uint8_t>(byte);
      result.push_back(hexadecimal[(value >> 4U) & 0x0FU]);
      result.push_back(hexadecimal[value & 0x0FU]);
    }
    return result;
  }

  static std::string mission_uuid(const std::shared_ptr<MissionGoalHandle> & mission) {
    return mission ? canonical_goal_uuid(mission->get_goal_id()) : std::string();
  }

  static bool is_controller_collision_log(const rcl_interfaces::msg::Log & message) {
    return message.name == "amr.controller_server" &&
      message.msg == "RegulatedPurePursuitController detected collision ahead!";
  }

  void controller_log_callback(const rcl_interfaces::msg::Log::SharedPtr & message) {
    if (!message || !is_controller_collision_log(*message)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    // The log is evidence for this mission only while its accepted controller
    // goal is active.  A stale or unrelated rosout message must never
    // reclassify a later controller result as an obstacle blockage.
    if (mission_goal_ && !terminal_reported_ && state_ == MissionState::CONTROLLER_ACTIVE) {
      controller_collision_observed_ = true;
    }
  }

  void publish_status(
    const std::shared_ptr<MissionGoalHandle> & mission,
    StatusStage stage,
    StatusOutcome outcome,
    const std::string & reason,
    FaultClass fault_class,
    bool blockage_confirmed)
  {
    publish_status(
      mission_uuid(mission), stage, outcome, reason, fault_class, blockage_confirmed);
  }

  void publish_status(
    const std::string & goal_uuid,
    StatusStage stage,
    StatusOutcome outcome,
    const std::string & reason,
    FaultClass fault_class,
    bool blockage_confirmed)
  {
    auto publisher = status_publisher_;
    if (!publisher || goal_uuid.empty()) {
      if (goal_uuid.empty()) {
        RCLCPP_ERROR(get_logger(), "Unable to publish mission status without a goal UUID");
      }
      return;
    }

    diagnostic_msgs::msg::DiagnosticArray message;
    message.header.stamp = get_clock()->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "amr_mission/mission_supervisor";
    status.message = reason;
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (outcome == StatusOutcome::CANCELED) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else if (outcome == StatusOutcome::FAULT ||
      outcome == StatusOutcome::ABORTED || fault_class != FaultClass::NONE)
    {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    }

    auto add_value = [&status](const char * key, const std::string & value) {
        diagnostic_msgs::msg::KeyValue field;
        field.key = key;
        field.value = value;
        status.values.push_back(field);
      };
    add_value("goal_uuid", goal_uuid);
    add_value("stage", status_stage_name(stage));
    add_value("outcome", status_outcome_name(outcome));
    add_value("reason", reason);
    add_value("blockage_confirmed", blockage_confirmed ? "true" : "false");
    add_value("fault_class", fault_class_name(fault_class));
    message.status.push_back(status);

    // Status is observation-only. A failed publication must never alter the
    // action result, cancellation proof, or any safety decision.
    try {
      publisher->publish(message);
    } catch (const std::exception & exception) {
      RCLCPP_WARN(get_logger(), "Mission status publication failed: %s", exception.what());
    } catch (...) {
      RCLCPP_WARN(get_logger(), "Mission status publication failed with an unknown error");
    }
  }

  rclcpp_action::Server<NavigateToPose>::SharedPtr create_mission_server(
    const std::string & endpoint,
    const std::string & planner_id,
    const std::string & goal_checker_id,
    const std::string & controller_id)
  {
    return rclcpp_action::create_server<NavigateToPose>(
      this, endpoint,
      [this, planner_id, goal_checker_id, controller_id](const rclcpp_action::GoalUUID &,
             std::shared_ptr<const NavigateToPose::Goal> goal) {
        return handle_goal(*goal, planner_id, goal_checker_id, controller_id);
      },
      [this](const std::shared_ptr<MissionGoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this, planner_id, goal_checker_id, controller_id](
        const std::shared_ptr<MissionGoalHandle> goal_handle) {
        start_planning(goal_handle, planner_id, goal_checker_id, controller_id);
      });
  }

  rclcpp_action::GoalResponse handle_goal(
    const NavigateToPose::Goal & goal,
    const std::string & planner_id,
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
    reserved_planner_id_ = planner_id;
    reserved_goal_checker_id_ = goal_checker_id;
    reserved_controller_id_ = controller_id;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<MissionGoalHandle> & goal_handle)
  {
    const auto targets = mark_cancel(goal_handle, false, "mission cancellation requested");
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
    bool abort_on_stop,
    const char * reason)
  {
    CancelTargets targets;
    bool publish_transition = false;
    bool lifecycle_stop = false;
    std::string transition_reason;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!mission || mission != mission_goal_ || terminal_reported_ ||
        !mission->is_active())
      {
        return targets;
      }
      targets.mission = mission;
      targets.pending_acceptance = downstream_acceptance_pending_;
      cancel_requested_ = true;
      abort_on_stop_ = abort_on_stop_ || abort_on_stop;
      lifecycle_stop = abort_on_stop_;
      if (reason && (abort_on_stop || cancel_reason_.empty())) {
        cancel_reason_ = reason;
      }
      if (state_ != MissionState::CANCELING) {
        publish_transition = true;
      }
      transition_reason = cancel_reason_.empty() ?
        (lifecycle_stop ? "lifecycle deactivation requested" :
        "mission cancellation requested") : cancel_reason_;
      state_ = MissionState::CANCELING;
      if (publish_transition) {
        publish_status(
          mission, StatusStage::CANCELING, StatusOutcome::PENDING,
          transition_reason,
          lifecycle_stop ? FaultClass::NAVIGATION_FAULT : FaultClass::CANCELLATION,
          false);
      }
      // Retain every accepted handle until its terminal result callback. The
      // cancel-sent flags make repeated cancellation requests idempotent while
      // preserving the obligation that a result callback must provide proof.
      if (planner_goal_ && !planner_cancel_sent_) {
        targets.planner = planner_goal_;
        planner_cancel_sent_ = true;
      }
      if (smoother_goal_ && !smoother_cancel_sent_) {
        targets.smoother = smoother_goal_;
        smoother_cancel_sent_ = true;
      }
      if (controller_goal_ && !controller_cancel_sent_) {
        targets.controller = controller_goal_;
        controller_cancel_sent_ = true;
      }
    }
    return targets;
  }

  void start_planning(
    const std::shared_ptr<MissionGoalHandle> & mission,
    const std::string & planner_id,
    const std::string & goal_checker_id,
    const std::string & controller_id)
  {
    std::string selected_planner_id;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!mission || !goal_reserved_ || mission_goal_ ||
        reserved_planner_id_ != planner_id ||
        reserved_goal_checker_id_ != goal_checker_id ||
        reserved_controller_id_ != controller_id)
      {
        return;
      }
      mission_goal_ = mission;
      goal_reserved_ = false;
      mission_planner_id_ = planner_id;
      // A goal already inside the normal checker's 0.07 m XY tolerance is
      // a terminal heading adjustment, not another lattice translation.
      // Smac can otherwise create a multi-metre loop for that same-position
      // goal. Keep FollowPath's collision-checked in-place rotation and the
      // existing checker; only choose the exact, swept-footprint planner.
      geometry_msgs::msg::PoseStamped current_pose;
      const auto & target = mission->get_goal()->pose.pose.position;
      if ((planner_id == "GridBased" || planner_id == "ExactGoalLattice") && latest_base_pose(current_pose) &&
        std::isfinite(current_pose.pose.position.x) &&
        std::isfinite(current_pose.pose.position.y))
      {
        const double distance = std::hypot(target.x - current_pose.pose.position.x,
            target.y - current_pose.pose.position.y);
        if (distance <= 0.07) {
          mission_planner_id_ = "PrecisionGridBased";
          RCLCPP_INFO(get_logger(), "Near-position heading goal uses PrecisionGridBased");
        } else if (planner_id == "GridBased" && distance < 1.0) {
          // The 0.5 m-radius lattice cannot make a one-cell lateral correction
          // within a short approach and loops instead (Native46/47 docks).
          mission_planner_id_ = "PrecisionGridBased";
          RCLCPP_INFO(get_logger(), "Short goal (%.2f m) uses PrecisionGridBased", distance);
        }
      }
      selected_planner_id = mission_planner_id_;
      mission_goal_checker_id_ = goal_checker_id;
      mission_controller_id_ = controller_id;
      cancel_requested_ = false;
      abort_on_stop_ = false;
      terminal_reported_ = false;
      state_ = MissionState::PLANNER_PENDING;
      downstream_acceptance_pending_ = true;
      planner_cancel_sent_ = false;
      smoother_cancel_sent_ = false;
      controller_cancel_sent_ = false;
      controller_collision_observed_ = false;
      mission_start_ = get_clock()->now();
      last_ros_time_ = mission_start_;
      publish_status(
        mission, StatusStage::PLANNING, StatusOutcome::PENDING,
        "mission planning started", FaultClass::NONE, false);
    }

    // Fail closed when either required downstream server is unavailable.
    if (!planner_client_->action_server_is_ready() ||
        !smoother_client_->action_server_is_ready() ||
        !controller_client_->action_server_is_ready()) {
      bool canceled = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mission == mission_goal_ && !terminal_reported_) {
          downstream_acceptance_pending_ = false;
          canceled = cancel_requested_;
          if (!canceled) state_ = MissionState::IDLE;
        }
      }
      if (canceled) {
        complete_after_stop(mission);
      } else {
        abort(mission, "planner or controller action is unavailable", FaultClass::PLANNER_ABORT);
      }
      return;
    }

    ComputePath::Goal goal;
    goal.goal = mission->get_goal()->pose;
    goal.planner_id = selected_planner_id;
    goal.use_start = false;
    rclcpp_action::Client<ComputePath>::SendGoalOptions options;
    options.goal_response_callback =
      [this, mission](ComputeGoalHandle::SharedPtr planner_goal) {
        bool cancel_late = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) {
            cancel_late = static_cast<bool>(planner_goal);
          } else {
            downstream_acceptance_pending_ = false;
            if (!planner_goal) {
              planner_cancel_sent_ = false;
              // A rejection is terminal only for the still-current pending
              // request. A cancellation race is completed as canceled.
              if (cancel_requested_) {
                state_ = MissionState::CANCELING;
              } else {
                state_ = MissionState::IDLE;
              }
            } else if (mission == mission_goal_ && !terminal_reported_ &&
              state_ == MissionState::PLANNER_PENDING && !cancel_requested_)
            {
              planner_goal_ = planner_goal;
              planner_cancel_sent_ = false;
              state_ = MissionState::PLANNER_ACTIVE;
            } else {
              planner_goal_ = planner_goal;
              cancel_late = !planner_cancel_sent_;
              planner_cancel_sent_ = true;
            }
          }
        }
        if (planner_goal && cancel_late) {
          cancel_planner_goal(planner_goal);
          return;
        }
        if (!planner_goal) {
          if (is_canceling(mission)) {
            complete_after_stop(mission);
          } else {
            abort(mission, "planner rejected the mission goal", FaultClass::PLANNER_ABORT);
          }
        }
      };
    options.result_callback =
      [this, mission](const ComputeGoalHandle::WrappedResult & result) {
        nav_msgs::msg::Path path;
        bool process = false;
        bool cancel = false;
        bool parity_failed = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) return;
          // A terminal result is the proof that the retained planner
          // obligation is complete.
          planner_goal_.reset();
          planner_cancel_sent_ = false;
          downstream_acceptance_pending_ = false;
          process = true;
          cancel = cancel_requested_;
          if (cancel) {
            state_ = MissionState::CANCELING;
          } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && (is_dispatch_controller(mission_controller_id_) ||
            !result.result->path.poses.empty()))
          {
            parity_failed = is_dispatch_controller(mission_controller_id_) &&
              !requested_goal_parity(result.result->path, mission->get_goal()->pose);
            if (parity_failed) {
              // Terminal proof discharged this request, including pending acceptance.
              // Retain mission ownership until abort completes outside the mutex.
              state_ = MissionState::IDLE;
            } else {
              state_ = MissionState::SMOOTHER_PENDING;
              path = result.result->path;
              publish_status(
                mission, StatusStage::SMOOTHING, StatusOutcome::PENDING,
                "global planning succeeded; path smoothing started", FaultClass::NONE, false);
            }
          }
        }
        if (!process) return;
        if (cancel) {
          complete_after_stop(mission);
          return;
        }
        if (parity_failed) {
          abort(mission, "planned path differs from requested dispatch goal", FaultClass::PLANNER_ABORT);
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !result.result || result.result->path.poses.empty()) {
          abort(mission, "global planning failed", FaultClass::PLANNER_ABORT);
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
    bool send = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission == mission_goal_ && !terminal_reported_ &&
        !cancel_requested_ && state_ == MissionState::SMOOTHER_PENDING)
      {
        downstream_acceptance_pending_ = true;
        smoother_cancel_sent_ = false;
        send = true;
      }
    }
    if (!send) {
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
          if (mission != mission_goal_ || terminal_reported_) {
            cancel_late = static_cast<bool>(smoother_goal);
          } else {
            downstream_acceptance_pending_ = false;
            if (!smoother_goal) {
              smoother_cancel_sent_ = false;
              if (cancel_requested_) {
                state_ = MissionState::CANCELING;
              } else {
                state_ = MissionState::IDLE;
              }
            } else if (mission == mission_goal_ && !terminal_reported_ &&
              state_ == MissionState::SMOOTHER_PENDING && !cancel_requested_)
            {
              smoother_goal_ = smoother_goal;
              smoother_cancel_sent_ = false;
              state_ = MissionState::SMOOTHER_ACTIVE;
            } else {
              smoother_goal_ = smoother_goal;
              cancel_late = !smoother_cancel_sent_;
              smoother_cancel_sent_ = true;
            }
          }
        }
        if (smoother_goal && cancel_late) {
          cancel_smoother_goal(smoother_goal);
          return;
        }
        if (!smoother_goal) {
          if (is_canceling(mission)) {
            complete_after_stop(mission);
          } else {
            abort(mission, "smoother rejected the planned path", FaultClass::SMOOTHER_ABORT);
          }
        }
      };
    options.result_callback =
      [this, mission](const SmootherGoalHandle::WrappedResult & result) {
        nav_msgs::msg::Path smoothed_path;
        bool process = false;
        bool cancel = false;
        bool parity_failed = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) return;
          smoother_goal_.reset();
          smoother_cancel_sent_ = false;
          downstream_acceptance_pending_ = false;
          process = true;
          cancel = cancel_requested_;
          if (cancel) {
            state_ = MissionState::CANCELING;
          } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->was_completed &&
            (is_dispatch_controller(mission_controller_id_) || !result.result->path.poses.empty()))
          {
            parity_failed = is_dispatch_controller(mission_controller_id_) &&
              !requested_goal_parity(result.result->path, mission->get_goal()->pose);
            if (parity_failed) {
              state_ = MissionState::IDLE;
            } else {
              state_ = MissionState::CONTROLLER_PENDING;
              smoothed_path = result.result->path;
              publish_status(
                mission, StatusStage::FOLLOWING, StatusOutcome::PENDING,
                "path smoothing succeeded; path following started", FaultClass::NONE, false);
            }
          }
        }
        if (!process) return;
        if (cancel) {
          complete_after_stop(mission);
          return;
        }
        if (parity_failed) {
          abort(mission, "smoothed path differs from requested dispatch goal", FaultClass::SMOOTHER_ABORT);
          return;
        }
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED ||
            !result.result || !result.result->was_completed ||
            result.result->path.poses.empty()) {
          // In the installed Humble smoother action, collision checking runs
          // after a completed smoother returns its non-empty path.  The
          // action is then aborted by the collision exception, so the
          // collision proof is a non-success result with a completed path;
          // a timeout/incomplete smoother has was_completed == false.
          const bool blockage_confirmed =
            result.code == rclcpp_action::ResultCode::ABORTED && result.result &&
            result.result->was_completed && !result.result->path.poses.empty();
          abort(
            mission,
            blockage_confirmed ?
            "path smoothing reached a collision boundary" :
            "path smoothing failed or was incomplete",
            blockage_confirmed ? FaultClass::OBSTACLE_BLOCKAGE : FaultClass::SMOOTHER_ABORT,
            blockage_confirmed);
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
    bool send = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission == mission_goal_ && !terminal_reported_ &&
        !cancel_requested_ && state_ == MissionState::CONTROLLER_PENDING)
      {
        goal_checker_id = mission_goal_checker_id_;
        controller_id = mission_controller_id_;
        downstream_acceptance_pending_ = true;
        controller_cancel_sent_ = false;
        send = true;
      } else if (mission == mission_goal_ && cancel_requested_) {
        state_ = MissionState::CANCELING;
      }
    }
    if (!send) {
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
          if (mission != mission_goal_ || terminal_reported_) {
            cancel_late = static_cast<bool>(controller_goal);
          } else {
            downstream_acceptance_pending_ = false;
            if (!controller_goal) {
              controller_cancel_sent_ = false;
              if (cancel_requested_) {
                state_ = MissionState::CANCELING;
              } else {
                state_ = MissionState::IDLE;
              }
            } else if (mission == mission_goal_ && !terminal_reported_ &&
              state_ == MissionState::CONTROLLER_PENDING && !cancel_requested_)
            {
              controller_goal_ = controller_goal;
              controller_cancel_sent_ = false;
              state_ = MissionState::CONTROLLER_ACTIVE;
            } else {
              controller_goal_ = controller_goal;
              cancel_late = !controller_cancel_sent_;
              controller_cancel_sent_ = true;
            }
          }
        }
        if (controller_goal && cancel_late) {
          cancel_controller_goal(controller_goal);
          return;
        }
        if (!controller_goal) {
          if (is_canceling(mission)) {
            complete_after_stop(mission);
          } else {
            abort(mission, "controller rejected the planned path", FaultClass::CONTROLLER_ABORT);
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
        bool controller_collision = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (mission != mission_goal_ || terminal_reported_) return;
          // The controller handle is cleared before processing its terminal
          // result. A cancellation/result race is therefore terminally safe,
          // and this result is the terminal proof for the retained handle.
          controller_goal_.reset();
          controller_cancel_sent_ = false;
          downstream_acceptance_pending_ = false;
          current = true;
          canceled = cancel_requested_ ||
            result.code == rclcpp_action::ResultCode::CANCELED;
          abort_requested = abort_on_stop_;
          controller_collision = controller_collision_observed_;
          state_ = canceled ? MissionState::CANCELING : MissionState::IDLE;
        }
        if (!current) return;
        if (canceled) {
          complete_public(
            mission, abort_requested ? Completion::ABORTED : Completion::CANCELED,
            abort_requested ? "mission stopped by lifecycle deactivation" :
            "mission canceled",
            abort_requested ? FaultClass::NAVIGATION_FAULT : FaultClass::CANCELLATION,
            false);
        } else if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          complete_public(
            mission, Completion::SUCCEEDED, "mission completed", FaultClass::NONE, false);
        } else {
          const bool localization_unavailable =
            !controller_collision && map_localization_lags_odometry();
          abort(
            mission,
            controller_collision ? "controller collision boundary reached" :
            localization_unavailable ? "path following lost map localization" :
            "path following failed",
            controller_collision ? FaultClass::OBSTACLE_BLOCKAGE :
            localization_unavailable ? FaultClass::LOCALIZATION_UNAVAILABLE :
            FaultClass::CONTROLLER_ABORT,
            controller_collision);
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

  bool map_localization_lags_odometry() {
    try {
      const auto map_odom = tf_buffer_.lookupTransform("map", "odom", tf2::TimePointZero);
      const auto odom_base = tf_buffer_.lookupTransform(
        "odom", "base_footprint", tf2::TimePointZero);
      const auto valid = [](const geometry_msgs::msg::TransformStamped & transform) {
          const auto & stamp = transform.header.stamp;
          const auto & t = transform.transform.translation;
          const auto & q = transform.transform.rotation;
          return stamp.sec >= 0 && stamp.nanosec < 1000000000u &&
                 (stamp.sec > 0 || stamp.nanosec > 0) &&
                 std::isfinite(t.x) && std::isfinite(t.y) && std::isfinite(t.z) &&
                 std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) &&
                 std::isfinite(q.w) && (q.x != 0.0 || q.y != 0.0 || q.z != 0.0 || q.w != 0.0);
        };
      return valid(map_odom) && valid(odom_base) &&
             rclcpp::Time(map_odom.header.stamp) < rclcpp::Time(odom_base.header.stamp);
    } catch (const tf2::TransformException &) {
      return false;
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
    const auto targets = mark_cancel(mission, false, reason);
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
    const char * reason,
    FaultClass fault_class,
    bool blockage_confirmed = false)
  {
    RCLCPP_WARN(get_logger(), "Mission aborted: %s", reason);
    bool can_complete = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission == mission_goal_ && !terminal_reported_ &&
        !cancel_requested_ && !downstream_acceptance_pending_ &&
        !planner_goal_ && !smoother_goal_ &&
        !controller_goal_ &&
        state_ != MissionState::PLANNER_PENDING &&
        state_ != MissionState::SMOOTHER_PENDING &&
        state_ != MissionState::CONTROLLER_PENDING)
      {
        can_complete = true;
      }
    }
    if (can_complete) {
      complete_public(
        mission, Completion::ABORTED, reason, fault_class, blockage_confirmed);
    }
  }

  void complete_after_stop(const std::shared_ptr<MissionGoalHandle> & mission) {
    Completion completion = Completion::CANCELED;
    FaultClass fault_class = FaultClass::CANCELLATION;
    std::string reason = "mission canceled";
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission != mission_goal_ || terminal_reported_) return;
      if (downstream_acceptance_pending_ || planner_goal_ || controller_goal_ ||
        state_ == MissionState::PLANNER_PENDING ||
        smoother_goal_ ||
        state_ == MissionState::SMOOTHER_PENDING ||
        state_ == MissionState::CONTROLLER_PENDING)
      {
        return;
      }
      if (abort_on_stop_) {
        completion = Completion::ABORTED;
        fault_class = FaultClass::NAVIGATION_FAULT;
        reason = cancel_reason_.empty() ?
          "mission stopped by lifecycle deactivation" : cancel_reason_;
      } else {
        reason = cancel_reason_.empty() ? "mission canceled" : cancel_reason_;
      }
    }
    complete_public(mission, completion, reason, fault_class, false);
  }

  void complete_public(
    const std::shared_ptr<MissionGoalHandle> & mission,
    Completion completion,
    const std::string & reason,
    FaultClass fault_class,
    bool blockage_confirmed)
  {
    bool complete = false;
    std::string goal_uuid;
    StatusOutcome status_outcome = StatusOutcome::FAULT;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mission != mission_goal_ || terminal_reported_) return;
      goal_uuid = mission_uuid(mission);
      switch (completion) {
        case Completion::SUCCEEDED:
          status_outcome = StatusOutcome::SUCCEEDED;
          break;
        case Completion::CANCELED:
          status_outcome = StatusOutcome::CANCELED;
          break;
        case Completion::ABORTED:
          status_outcome = fault_class == FaultClass::NAVIGATION_FAULT ?
            StatusOutcome::ABORTED : StatusOutcome::FAULT;
          break;
      }
      terminal_reported_ = true;
      state_ = MissionState::IDLE;
      goal_reserved_ = false;
      cancel_requested_ = false;
      downstream_acceptance_pending_ = false;
      mission_goal_.reset();
      mission_planner_id_.clear();
      reserved_planner_id_.clear();
      mission_goal_checker_id_.clear();
      reserved_goal_checker_id_.clear();
      mission_controller_id_.clear();
      reserved_controller_id_.clear();
      planner_goal_.reset();
      planner_cancel_sent_ = false;
      smoother_goal_.reset();
      smoother_cancel_sent_ = false;
      controller_goal_.reset();
      controller_cancel_sent_ = false;
      controller_collision_observed_ = false;
      abort_on_stop_ = false;
      cancel_reason_.clear();
      complete = true;
      publish_status(
        goal_uuid, StatusStage::TERMINAL, status_outcome, reason, fault_class,
        blockage_confirmed);
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
    if (planner_goal) (void)cancel_planner_goal(planner_goal);
    if (smoother_goal) (void)cancel_smoother_goal(smoother_goal);
    if (controller_goal) (void)cancel_controller_goal(controller_goal);
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
  rclcpp_action::Server<NavigateToPose>::SharedPtr dispatch_a_server_, dispatch_b_server_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr dispatch_a_precise_server_, dispatch_b_precise_server_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr dispatch_b_clear_approach_server_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr controller_log_subscription_;
  rclcpp_action::Client<ComputePath>::SharedPtr planner_client_;
  rclcpp_action::Client<FollowPath>::SharedPtr controller_client_;
  mutable std::mutex mutex_;
  MissionState state_{MissionState::IDLE};
  bool goal_reserved_{false};
  bool cancel_requested_{false};
  bool abort_on_stop_{false};
  bool terminal_reported_{false};
  bool downstream_acceptance_pending_{false};
  bool planner_cancel_sent_{false};
  bool smoother_cancel_sent_{false};
  bool controller_cancel_sent_{false};
  std::string cancel_reason_;
  std::string reserved_planner_id_;
  std::string reserved_goal_checker_id_;
  std::string reserved_controller_id_;
  std::string mission_planner_id_;
  std::string mission_goal_checker_id_;
  std::string mission_controller_id_;
  std::shared_ptr<MissionGoalHandle> mission_goal_;
  ComputeGoalHandle::SharedPtr planner_goal_;
  rclcpp_action::Client<nav2_msgs::action::SmoothPath>::SharedPtr smoother_client_;
  SmootherGoalHandle::SharedPtr smoother_goal_;
  FollowGoalHandle::SharedPtr controller_goal_;
  bool controller_collision_observed_{false};
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

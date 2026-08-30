#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "amr_interfaces/action/manipulate_product.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr_manipulation {

using namespace std::chrono_literals;
using Action = amr_interfaces::action::ManipulateProduct;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using Status = amr_interfaces::msg::ManipulatorStatus;

// This node owns the public manipulation boundary. The existing Gate 6
// executable remains the motion implementation; this boundary deliberately
// exposes an injectable hook instead of duplicating arm/gripper motion.
class ManipulationSupervisorNode final : public rclcpp::Node {
 public:
  using ExecutionHook = std::function<Action::Result(
    const Action::Goal &, const std::atomic_bool &)>;

  explicit ManipulationSupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("manipulation_supervisor_node", options)
  {
    status_pub_ = create_publisher<Status>(
      "/amr/manipulation/status", amr_interfaces::qos::authority());
    action_server_ = rclcpp_action::create_server<Action>(
      this, "/amr/manipulation/manipulate_product",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        return handle_goal(goal);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        handle_accepted(goal_handle);
      });
    status_timer_ = create_wall_timer(50ms, [this]() { publish_status(); });
    publish_status();
  }

  ~ManipulationSupervisorNode() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancel_requested_.store(true);
    }
    if (worker_.joinable()) worker_.join();
  }

  // The integration adapter can install the existing Gate 6 executor without
  // changing this action contract or introducing another motion publisher.
  void set_execution_hook(ExecutionHook hook)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    execution_hook_ = std::move(hook);
  }

 private:
  static bool valid_product(const std::string & product_id)
  {
    return product_id == "101" || product_id == "102" || product_id == "103";
  }

  static bool valid_station(const Action::Goal & goal)
  {
    if (goal.operation == Action::Goal::PICK) {
      return goal.station_id == "pickup_a" || goal.station_id == "pickup_b" ||
        goal.station_id == "pickup_c";
    }
    if (goal.operation == Action::Goal::PLACE) return goal.station_id == "dispatch";
    return false;
  }

  static bool valid_goal(const Action::Goal & goal)
  {
    return (goal.operation == Action::Goal::PICK || goal.operation == Action::Goal::PLACE) &&
      valid_station(goal) && valid_product(goal.product_id);
  }

  rclcpp_action::GoalResponse handle_goal(
    const std::shared_ptr<const Action::Goal> & goal)
  {
    if (!goal || !valid_goal(*goal)) return rclcpp_action::GoalResponse::REJECT;
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ || stopping_) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goal_handle || goal_handle != active_goal_ || !goal_handle->is_active()) {
      return rclcpp_action::CancelResponse::REJECT;
    }
    cancel_requested_.store(true);
    phase_ = "CANCELING";
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    if (worker_.joinable()) worker_.join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_goal_ || stopping_) {
        if (goal_handle->is_active()) {
          auto result = std::make_shared<Action::Result>();
          result->outcome = Action::Result::INTERLOCK_FAILED;
          result->message = "another manipulation goal is active";
          goal_handle->abort(result);
        }
        return;
      }
      active_goal_ = goal_handle;
      cancel_requested_.store(false);
      phase_ = "VALIDATING";
      state_ = Status::MOVING;
      base_motion_allowed_ = false;
      attached_ = goal_handle->get_goal()->operation == Action::Goal::PLACE;
      product_id_ = goal_handle->get_goal()->product_id;
      detail_ = "manipulation goal accepted; base motion blocked";
    }
    worker_ = std::thread([this, goal_handle]() { execute(goal_handle); });
  }

  Action::Result execute_gate6_hook(
    const Action::Goal & goal, const std::atomic_bool & cancel_requested)
  {
    ExecutionHook hook;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hook = execution_hook_;
    }
    if (cancel_requested.load()) {
      Action::Result result;
      result.outcome = Action::Result::INTERLOCK_FAILED;
      result.message = "manipulation canceled before Gate 6 hook dispatch";
      return result;
    }
    if (!hook) {
      Action::Result result;
      result.outcome = Action::Result::INTERLOCK_FAILED;
      result.message = "Gate 6 executor hook is unavailable; no motion dispatched";
      return result;
    }
    return hook(goal, cancel_requested);
  }

  void execute(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    const auto goal = *goal_handle->get_goal();
    publish_feedback(goal_handle, "VALIDATING");
    auto result = execute_gate6_hook(goal, cancel_requested_);
    const bool canceled = cancel_requested_.load();
    if (canceled) {
      result.outcome = Action::Result::INTERLOCK_FAILED;
      result.message = "manipulation canceled; no subsequent motion was started";
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (canceled) {
        state_ = attached_ ? Status::FAULT : Status::STOWED_EMPTY;
        base_motion_allowed_ = false;
        detail_ = attached_ ? "canceled with product retained; base motion blocked" :
          "manipulation canceled before attachment";
      } else if (result.outcome == Action::Result::SUCCESS) {
        state_ = goal.operation == Action::Goal::PICK ? Status::STOWED_LOADED :
          Status::STOWED_EMPTY;
        attached_ = goal.operation == Action::Goal::PICK;
        base_motion_allowed_ = true;
        detail_ = result.message;
      } else {
        state_ = Status::FAULT;
        base_motion_allowed_ = false;
        detail_ = result.message;
      }
      phase_ = canceled ? "CANCELED" : "COMPLETE";
    }
    if (!goal_handle->is_active()) {
      clear_active(goal_handle);
      return;
    }
    auto result_message = std::make_shared<Action::Result>(result);
    if (canceled) {
      goal_handle->canceled(result_message);
    } else if (result.outcome == Action::Result::SUCCESS) {
      goal_handle->succeed(result_message);
    } else {
      goal_handle->abort(result_message);
    }
    clear_active(goal_handle);
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandle> & goal_handle, const std::string & phase)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = phase;
    }
    if (!goal_handle->is_active()) return;
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->phase = goal_handle->get_goal()->operation;
    feedback->phase_name = phase;
    goal_handle->publish_feedback(feedback);
  }

  void clear_active(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ == goal_handle) active_goal_.reset();
  }

  void publish_status()
  {
    Status status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.header.stamp = now();
      status.source_boot_id = boot_id_;
      status.sequence = ++sequence_;
      status.valid = state_ != Status::STARTING && state_ != Status::FAULT;
      status.state = state_;
      status.base_motion_allowed = base_motion_allowed_ && status.valid;
      status.product_attached = attached_;
      status.product_id = attached_ ? product_id_ : "";
      status.detail = detail_;
    }
    status_pub_->publish(status);
  }

  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  mutable std::mutex mutex_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::thread worker_;
  std::atomic_bool cancel_requested_{false};
  bool stopping_{false};
  uint32_t boot_id_{static_cast<uint32_t>(
      std::chrono::steady_clock::now().time_since_epoch().count())};
  uint32_t sequence_{0};
  uint8_t state_{Status::STARTING};
  bool base_motion_allowed_{false};
  bool attached_{false};
  std::string product_id_;
  std::string phase_{"STARTING"};
  std::string detail_{"Gate 6 executor hook is not installed"};
  ExecutionHook execution_hook_;
};

}  // namespace amr_manipulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_manipulation::ManipulationSupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

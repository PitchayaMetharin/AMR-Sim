#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "amr_interfaces/action/manipulate_product.hpp"
#include "amr_interfaces/action/transport_product.hpp"
#include "amr_interfaces/msg/factory_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "amr_interfaces/srv/set_operation_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace amr_factory {

using namespace std::chrono_literals;
using Transport = amr_interfaces::action::TransportProduct;
using Manipulate = amr_interfaces::action::ManipulateProduct;
using TransportGoalHandle = rclcpp_action::ServerGoalHandle<Transport>;
using ManipulateGoalHandle = rclcpp_action::ClientGoalHandle<Manipulate>;
using FactoryStatus = amr_interfaces::msg::FactoryStatus;
using ManipulatorStatus = amr_interfaces::msg::ManipulatorStatus;
using SetOperationMode = amr_interfaces::srv::SetOperationMode;

class FactorySupervisorNode final : public rclcpp::Node {
 public:
  explicit FactorySupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("factory_supervisor_node", options)
  {
    status_pub_ = create_publisher<FactoryStatus>(
      "/amr/factory/status", amr_interfaces::qos::authority());
    manipulator_sub_ = create_subscription<ManipulatorStatus>(
      "/amr/manipulation/status", amr_interfaces::qos::authority(),
      [this](ManipulatorStatus::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_manipulator_status_ &&
          message->source_boot_id == manipulator_status_.source_boot_id &&
          message->sequence <= manipulator_status_.sequence)
        {
          return;
        }
        manipulator_status_ = *message;
        have_manipulator_status_ = true;
        manipulator_received_ = std::chrono::steady_clock::now();
      });
    manipulation_client_ = rclcpp_action::create_client<Manipulate>(
      this, "/amr/manipulation/manipulate_product");
    transport_server_ = rclcpp_action::create_server<Transport>(
      this, "/amr/factory/transport_product",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Transport::Goal> goal) {
        return handle_goal(goal);
      },
      [this](const std::shared_ptr<TransportGoalHandle> goal_handle) {
        return handle_cancel(goal_handle);
      },
      [this](const std::shared_ptr<TransportGoalHandle> goal_handle) {
        handle_accepted(goal_handle);
      });
    mode_service_ = create_service<SetOperationMode>(
      "/amr/factory/set_operation_mode",
      [this](const std::shared_ptr<SetOperationMode::Request> request,
             std::shared_ptr<SetOperationMode::Response> response) {
        handle_mode(request, response);
      });
    status_timer_ = create_wall_timer(200ms, [this]() { publish_status(); });
    worker_ = std::thread([this]() { worker_loop(); });
    publish_status();
  }

  ~FactorySupervisorNode() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      active_cancel_requested_ = true;
    }
    queue_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  struct QueuedJob {
    std::shared_ptr<TransportGoalHandle> goal;
    std::string pickup_station;
    std::string destination_station;
    std::string product_id;
  };

  struct ManipulationCall {
    Manipulate::Result result;
    bool accepted{false};
    bool canceled{false};
  };

  static bool valid_pickup(const std::string & station)
  {
    return station == "pickup_a" || station == "pickup_b" || station == "pickup_c";
  }

  static bool valid_destination(const std::string & station)
  {
    return station == "dispatch";
  }

  static std::string product_for_pickup(const std::string & station)
  {
    if (station == "pickup_a") return "101";
    if (station == "pickup_b") return "102";
    if (station == "pickup_c") return "103";
    return "";
  }

  bool manipulator_ready_locked() const
  {
    if (!have_manipulator_status_ ||
      std::chrono::steady_clock::now() - manipulator_received_ > 200ms)
    {
      return false;
    }
    const auto & status = manipulator_status_;
    return status.valid && status.source_boot_id != 0U && status.sequence != 0U &&
      status.state == ManipulatorStatus::STOWED_EMPTY &&
      !status.product_attached && status.product_id.empty() &&
      status.base_motion_allowed;
  }

  bool duplicate_product_locked(const std::string & product_id) const
  {
    if (active_goal_ && active_product_id_ == product_id) return true;
    for (const auto & job : queue_) {
      if (job.product_id == product_id) return true;
    }
    return false;
  }

  rclcpp_action::GoalResponse handle_goal(
    const std::shared_ptr<const Transport::Goal> & goal)
  {
    if (!goal || !valid_pickup(goal->pickup_station_id) ||
      !valid_destination(goal->destination_station_id))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto product_id = product_for_pickup(goal->pickup_station_id);
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || fault_latched_ || !manipulator_ready_locked() ||
      duplicate_product_locked(product_id))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }
    const std::size_t capacity = mode_ == FactoryStatus::AUTONOMOUS ? 3U : 1U;
    if ((active_goal_ ? 1U : 0U) + queue_.size() >= capacity) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<TransportGoalHandle> & goal_handle)
  {
    std::shared_ptr<TransportGoalHandle> active;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!goal_handle || !goal_handle->is_active()) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      if (active_goal_ == goal_handle) {
        active_cancel_requested_ = true;
        active = goal_handle;
      } else {
        const auto found = std::find_if(queue_.begin(), queue_.end(),
          [&goal_handle](const QueuedJob & job) { return job.goal == goal_handle; });
        if (found == queue_.end()) return rclcpp_action::CancelResponse::REJECT;
        auto result = std::make_shared<Transport::Result>();
        result->delivered = false;
        result->outcome = Transport::Result::CANCELED;
        result->message = "queued transport goal canceled without starting motion";
        found->goal->canceled(result);
        queue_.erase(found);
        return rclcpp_action::CancelResponse::ACCEPT;
      }
    }
    if (active) cancel_current_manipulation();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<TransportGoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const QueuedJob job{goal_handle, goal->pickup_station_id,
      goal->destination_station_id, product_for_pickup(goal->pickup_station_id)};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      queue_.push_back(job);
      phase_ = queue_.size() == 1U && !active_goal_ ? 1U : 0U;
      detail_ = mode_ == FactoryStatus::AUTONOMOUS ? "queued FIFO transport goal" :
        "queued manual transport goal";
    }
    queue_condition_.notify_one();
  }

  void handle_mode(
    const std::shared_ptr<SetOperationMode::Request> & request,
    const std::shared_ptr<SetOperationMode::Response> & response)
  {
    if (!request || (request->mode != SetOperationMode::Request::MANUAL &&
      request->mode != SetOperationMode::Request::AUTONOMOUS))
    {
      response->accepted = false;
      response->message = "unknown operation mode";
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ || !queue_.empty() || fault_latched_ || !manipulator_ready_locked()) {
      response->accepted = false;
      response->message = "mode changes require an empty queue and fresh empty stow";
      return;
    }
    mode_ = request->mode;
    response->accepted = true;
    response->message = mode_ == FactoryStatus::MANUAL ? "manual mode selected" :
      "autonomous mode selected";
  }

  void worker_loop()
  {
    while (rclcpp::ok()) {
      QueuedJob job;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        queue_condition_.wait(lock, [this]() {
          return stopping_ || !queue_.empty();
        });
        if (stopping_) return;
        job = queue_.front();
        queue_.pop_front();
        active_goal_ = job.goal;
        active_product_id_ = job.product_id;
        active_cancel_requested_ = false;
        phase_ = 2U;
        current_station_id_ = job.pickup_station;
        detail_ = "starting dependency-gated pickup";
      }
      execute_job(job);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        active_goal_.reset();
        active_product_id_.clear();
        current_station_id_.clear();
        current_destination_id_.clear();
        current_product_attached_ = held_product_;
        active_cancel_requested_ = false;
        if (fault_latched_) {
          while (!queue_.empty()) {
            auto queued = queue_.front().goal;
            queue_.pop_front();
            if (queued && queued->is_active()) {
              auto result = std::make_shared<Transport::Result>();
              result->delivered = false;
              result->outcome = Transport::Result::INTERLOCK_FAILED;
              result->message = "queue held after a retained-product failure";
              queued->abort(result);
            }
          }
        }
        phase_ = fault_latched_ ? 5U : (queue_.empty() ? 0U : 1U);
      }
    }
  }

  void execute_job(const QueuedJob & job)
  {
    publish_feedback(job.goal, 1U, job.pickup_station, false);
    auto pick = call_manipulation(
      Manipulate::Goal::PICK, job.pickup_station, job.product_id);
    if (pick.accepted && (pick.result.outcome == Manipulate::Result::SUCCESS || pick.canceled)) {
      std::lock_guard<std::mutex> lock(mutex_);
      held_product_ = true;
      current_product_attached_ = true;
    }
    if (pick.canceled || is_cancel_requested()) {
      finish_canceled(job.goal, "transport canceled during pickup; product retained conservatively");
      std::lock_guard<std::mutex> lock(mutex_);
      fault_latched_ = held_product_;
      return;
    }
    if (pick.result.outcome != Manipulate::Result::SUCCESS) {
      finish_failed(job.goal, Transport::Result::PICK_FAILED,
        pick.result.message.empty() ? "pickup action failed" : pick.result.message);
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      phase_ = 3U;
      current_station_id_ = job.destination_station;
      current_destination_id_ = job.destination_station;
      held_product_ = true;
    }
    publish_feedback(job.goal, 2U, job.destination_station, true);
    auto place = call_manipulation(
      Manipulate::Goal::PLACE, job.destination_station, job.product_id);
    if (place.accepted && place.result.outcome == Manipulate::Result::SUCCESS) {
      std::lock_guard<std::mutex> lock(mutex_);
      held_product_ = false;
      current_product_attached_ = false;
    }
    if (place.canceled || is_cancel_requested()) {
      finish_canceled(job.goal, "transport canceled during placement; no new work started");
      std::lock_guard<std::mutex> lock(mutex_);
      fault_latched_ = held_product_;
      return;
    }
    if (place.result.outcome != Manipulate::Result::SUCCESS) {
      finish_failed(job.goal, Transport::Result::PLACE_FAILED,
        place.result.message.empty() ? "placement action failed; product retained" :
        place.result.message);
      std::lock_guard<std::mutex> lock(mutex_);
      fault_latched_ = held_product_;
      return;
    }
    auto result = std::make_shared<Transport::Result>();
    result->delivered = true;
    result->outcome = Transport::Result::SUCCESS;
    result->message = "transport completed through manipulation boundary";
    if (job.goal->is_active()) job.goal->succeed(result);
  }

  ManipulationCall call_manipulation(
    uint8_t operation, const std::string & station, const std::string & product)
  {
    ManipulationCall call;
    if (!manipulation_client_->wait_for_action_server(2s)) {
      call.result.outcome = Manipulate::Result::INTERLOCK_FAILED;
      call.result.message = "manipulation action dependency is unavailable";
      return call;
    }
    Manipulate::Goal goal;
    goal.operation = operation;
    goal.station_id = station;
    goal.product_id = product;
    auto sent = manipulation_client_->async_send_goal(goal);
    if (sent.wait_for(3s) != std::future_status::ready || !sent.get()) {
      call.result.outcome = Manipulate::Result::INTERLOCK_FAILED;
      call.result.message = "manipulation goal acceptance timed out";
      return call;
    }
    auto goal_handle = sent.get();
    call.accepted = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_manipulation_goal_ = goal_handle;
    }
    auto result_future = manipulation_client_->async_get_result(goal_handle);
    while (rclcpp::ok()) {
      if (result_future.wait_for(50ms) == std::future_status::ready) break;
      if (is_cancel_requested()) {
        call.canceled = true;
        cancel_current_manipulation(goal_handle);
        break;
      }
    }
    if (call.canceled && result_future.wait_for(3s) != std::future_status::ready) {
      call.result.outcome = Manipulate::Result::INTERLOCK_FAILED;
      call.result.message = "manipulation cancellation did not reach a terminal result";
    } else if (result_future.wait_for(0s) == std::future_status::ready) {
      const auto wrapped = result_future.get();
      if (wrapped.result) call.result = *wrapped.result;
      if (wrapped.code == rclcpp_action::ResultCode::CANCELED) call.canceled = true;
      if (!wrapped.result) {
        call.result.outcome = Manipulate::Result::EXECUTION_FAILED;
        call.result.message = "manipulation returned no result";
      }
    } else {
      call.result.outcome = Manipulate::Result::EXECUTION_FAILED;
      call.result.message = "manipulation action stopped without a result";
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_manipulation_goal_ == goal_handle) current_manipulation_goal_.reset();
    }
    return call;
  }

  void cancel_current_manipulation()
  {
    std::shared_ptr<ManipulateGoalHandle> goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal = current_manipulation_goal_;
    }
    if (goal) cancel_current_manipulation(goal);
  }

  void cancel_current_manipulation(const std::shared_ptr<ManipulateGoalHandle> & goal)
  {
    try {
      manipulation_client_->async_cancel_goal(goal);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_WARN(get_logger(), "manipulation goal was already terminal during cancellation");
    }
  }

  bool is_cancel_requested()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_cancel_requested_;
  }

  void finish_canceled(
    const std::shared_ptr<TransportGoalHandle> & goal, const std::string & detail)
  {
    auto result = std::make_shared<Transport::Result>();
    result->delivered = false;
    result->outcome = Transport::Result::CANCELED;
    result->message = detail;
    if (goal->is_active()) goal->canceled(result);
    std::lock_guard<std::mutex> lock(mutex_);
    phase_ = held_product_ ? 5U : 0U;
    detail_ = detail;
  }

  void finish_failed(
    const std::shared_ptr<TransportGoalHandle> & goal, uint8_t outcome,
    const std::string & detail)
  {
    auto result = std::make_shared<Transport::Result>();
    result->delivered = false;
    result->outcome = outcome;
    result->message = detail;
    if (goal->is_active()) goal->abort(result);
    std::lock_guard<std::mutex> lock(mutex_);
    detail_ = detail;
    if (held_product_) {
      fault_latched_ = true;
      phase_ = 5U;
    }
  }

  void publish_feedback(
    const std::shared_ptr<TransportGoalHandle> & goal, uint8_t phase,
    const std::string & station, bool attached)
  {
    if (!goal || !goal->is_active()) return;
    auto feedback = std::make_shared<Transport::Feedback>();
    feedback->phase = phase;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      feedback->queue_position = static_cast<uint32_t>(queue_.size() + 1U);
      feedback->current_station_id = station;
      feedback->product_attached = attached;
    }
    goal->publish_feedback(feedback);
  }

  void publish_status()
  {
    FactoryStatus status;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status.header.stamp = now();
      status.sequence = ++sequence_;
      status.mode = mode_;
      status.phase = phase_;
      status.active = static_cast<bool>(active_goal_);
      status.queue_depth = static_cast<uint32_t>(queue_.size());
      status.pickup_station_id = active_goal_ ? current_station_id_ : "";
      status.destination_station_id = active_goal_ ? current_destination_id_ : "";
      status.product_id = active_goal_ ? active_product_id_ : "";
      status.product_attached = current_product_attached_ || held_product_;
      status.last_outcome = last_outcome_;
      status.detail = detail_;
    }
    status_pub_->publish(status);
  }

  rclcpp_action::Server<Transport>::SharedPtr transport_server_;
  rclcpp_action::Client<Manipulate>::SharedPtr manipulation_client_;
  rclcpp::Service<SetOperationMode>::SharedPtr mode_service_;
  rclcpp::Publisher<FactoryStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<ManipulatorStatus>::SharedPtr manipulator_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::mutex mutex_;
  std::condition_variable queue_condition_;
  std::deque<QueuedJob> queue_;
  std::shared_ptr<TransportGoalHandle> active_goal_;
  std::shared_ptr<ManipulateGoalHandle> current_manipulation_goal_;
  std::thread worker_;
  bool stopping_{false};
  bool active_cancel_requested_{false};
  bool fault_latched_{false};
  bool held_product_{false};
  bool current_product_attached_{false};
  bool have_manipulator_status_{false};
  ManipulatorStatus manipulator_status_;
  std::chrono::steady_clock::time_point manipulator_received_{};
  uint8_t mode_{FactoryStatus::MANUAL};
  uint8_t phase_{0};
  uint8_t last_outcome_{Transport::Result::SUCCESS};
  uint32_t sequence_{0};
  std::string active_product_id_;
  std::string current_station_id_;
  std::string current_destination_id_;
  std::string detail_{"manual mode; waiting for fresh empty-stowed manipulation status"};
};

}  // namespace amr_factory

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_factory::FactorySupervisorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

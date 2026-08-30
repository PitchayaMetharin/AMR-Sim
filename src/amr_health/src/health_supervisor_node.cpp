#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>

#include "amr_interfaces/msg/base_status.hpp"
#include "amr_interfaces/msg/health_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_health {

// Observation-only base health reporting. This node has no recovery,
// lifecycle, or motion authority; it reports whether base evidence is usable.
class HealthSupervisorNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  using BaseStatus = amr_interfaces::msg::BaseStatus;
  using HealthStatus = amr_interfaces::msg::HealthStatus;

  explicit HealthSupervisorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("health_supervisor_node", options) {
    // Evidence expiry and publication rate are verified before activation.
    evidence_timeout_ = std::chrono::milliseconds(
      declare_parameter("evidence_timeout_ms", 300));
    output_frequency_ = declare_parameter("output_frequency", 10.0);
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    // Refuse settings that would make freshness or periodic reporting invalid.
    if (evidence_timeout_.count() <= 0 ||
      !std::isfinite(output_frequency_) || output_frequency_ <= 0.0)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Evidence timeout and output frequency must be finite and positive");
      return CallbackReturn::FAILURE;
    }
    // Publish one diagnostic topic from the base adapter's evidence.
    health_pub_ = create_publisher<HealthStatus>(
      "/amr/health/status", amr_interfaces::qos::diagnostic());
    base_sub_ = create_subscription<BaseStatus>(
      "/amr/base/status", amr_interfaces::qos::diagnostic(),
      [this](BaseStatus::SharedPtr message) { receive_base(*message); });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    // A new activation has no accepted evidence until each source reports.
    health_pub_->on_activate();
    boot_id_ = static_cast<uint32_t>(
      std::chrono::steady_clock::now().time_since_epoch().count());
    sequence_ = 0;
    base_ = Evidence{};
    const auto period = std::chrono::duration<double>(1.0 / output_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { tick(); });
    tick();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    timer_.reset();
    health_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

 private:
  enum class InvalidCause { NONE, INVALID, BACKWARD_TIME };

  struct Evidence {
    // Keep both latest received and latest accepted identity/time information.
    // Invalid input is remembered so health can report why it degraded.
    bool have{false};
    bool have_accepted{false};
    bool current_valid{false};
    bool ready{false};
    bool fault{false};
    uint32_t boot_id{0};
    uint32_t sequence{0};
    int64_t stamp_ns{0};
    InvalidCause invalid_cause{InvalidCause::NONE};
    std::chrono::steady_clock::time_point received{};
  };

  template<typename Message>
  static int64_t stamp_ns(const Message & message) {
    return static_cast<int64_t>(message.header.stamp.sec) * 1000000000LL +
      static_cast<int64_t>(message.header.stamp.nanosec);
  }

  void receive(
    Evidence & evidence, uint32_t boot_id, uint32_t sequence,
    int64_t message_stamp_ns, bool message_valid, bool semantic_valid,
    bool ready, bool fault)
  {
    // Receiving a message is not the same as accepting its evidence.
    evidence.have = true;
    evidence.received = std::chrono::steady_clock::now();
    evidence.current_valid = false;
    evidence.ready = false;
    evidence.fault = false;
    evidence.invalid_cause = InvalidCause::INVALID;

    // A new boot starts a new sequence stream; within one boot, sequence and
    // message timestamp must advance and semantic state/reason pairs must fit.
    const bool new_boot =
      !evidence.have_accepted || boot_id != evidence.boot_id;
    const bool identity_valid = boot_id != 0 && sequence != 0;
    const bool sequence_valid = new_boot || sequence > evidence.sequence;
    if (!identity_valid || !sequence_valid || !message_valid ||
      !semantic_valid)
    {
      return;
    }
    if (!new_boot && message_stamp_ns < evidence.stamp_ns) {
      evidence.invalid_cause = InvalidCause::BACKWARD_TIME;
      return;
    }

    evidence.have_accepted = true;
    evidence.current_valid = true;
    evidence.ready = ready;
    evidence.fault = fault;
    evidence.boot_id = boot_id;
    evidence.sequence = sequence;
    evidence.stamp_ns = message_stamp_ns;
    evidence.invalid_cause = InvalidCause::NONE;
  }

  void receive_base(const BaseStatus & status) {
    // Translate the base-specific state/reason contract to generic evidence.
    const bool ready =
      status.state == BaseStatus::READY &&
      status.reason == BaseStatus::REASON_READY;
    const bool unavailable =
      status.state == BaseStatus::UNAVAILABLE &&
      (status.reason == BaseStatus::REASON_UNAVAILABLE ||
      status.reason == BaseStatus::REASON_ODOMETRY_MISSING_OR_STALE ||
      status.reason == BaseStatus::REASON_JOINT_STATES_MISSING_OR_STALE);
    const bool fault =
      status.state == BaseStatus::FAULT &&
      status.reason == BaseStatus::REASON_FAULT;
    receive(
      base_, status.source_boot_id, status.sequence, stamp_ns(status),
      status.valid, ready || unavailable || fault, ready, fault);
  }

  bool fresh(const Evidence & evidence) const {
    // Use receive time so this check still works while simulation clock pauses.
    return evidence.have &&
      std::chrono::steady_clock::now() - evidence.received <=
      evidence_timeout_;
  }

  void tick() {
    // Evaluate the base evidence before selecting the diagnostic result.
    const bool base_fresh = fresh(base_);
    const bool backward_time = base_.invalid_cause == InvalidCause::BACKWARD_TIME;
    const bool invalid = !base_.current_valid;
    const bool source_fault = base_fresh && base_.current_valid && base_.fault;

    HealthStatus status;
    status.header.stamp = now();
    status.sequence = ++sequence_;
    status.valid = true;
    status.source_boot_id = boot_id_;
    status.base_ready = base_fresh && base_.current_valid && base_.ready;
    status.state = HealthStatus::HEALTHY;
    status.reason = HealthStatus::REASON_HEALTHY;

    // Priority matters: fresh declared source faults outrank stale/invalid
    // evidence, then missing/stale, backward-time, invalid, and not-ready.
    if (source_fault) {
      status.state = HealthStatus::FAULT;
      status.reason = HealthStatus::REASON_SOURCE_FAULT;
    } else if (!base_fresh) {
      status.state = HealthStatus::DEGRADED;
      status.reason = HealthStatus::REASON_EVIDENCE_MISSING_OR_STALE;
    } else if (backward_time) {
      status.state = HealthStatus::DEGRADED;
      status.reason = HealthStatus::REASON_BACKWARD_TIME;
    } else if (invalid) {
      status.state = HealthStatus::DEGRADED;
      status.reason = HealthStatus::REASON_INVALID_EVIDENCE;
    } else if (!status.base_ready) {
      status.state = HealthStatus::DEGRADED;
      status.reason = HealthStatus::REASON_SOURCE_NOT_READY;
    }
    health_pub_->publish(status);
  }

  std::chrono::milliseconds evidence_timeout_{300};
  double output_frequency_{10.0};
  uint32_t boot_id_{0};
  uint32_t sequence_{0};
  Evidence base_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<HealthStatus>::SharedPtr health_pub_;
  rclcpp::Subscription<BaseStatus>::SharedPtr base_sub_;
};

}  // namespace amr_health

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<amr_health::HealthSupervisorNode>()->
    get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}

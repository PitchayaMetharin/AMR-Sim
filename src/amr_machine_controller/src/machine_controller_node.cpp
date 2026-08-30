#include <chrono>
#include <cstdint>
#include <memory>

#include "amr_interfaces/msg/machine_actuator_command.hpp"
#include "amr_interfaces/msg/machine_controller_status.hpp"
#include "amr_interfaces/msg/machine_inputs.hpp"
#include "amr_interfaces/msg/machine_state.hpp"
#include "amr_interfaces/srv/request_machine_enable.hpp"
#include "amr_interfaces/srv/request_machine_reset.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace amr_machine_controller {
class MachineControllerNode final : public rclcpp_lifecycle::LifecycleNode {
 public:
  MachineControllerNode() : LifecycleNode("machine_controller_node") {
    input_timeout_ = std::chrono::milliseconds(declare_parameter("input_timeout_ms", 500));
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
    state_pub_ = create_publisher<amr_interfaces::msg::MachineState>("/amr/machine/state", 5);
    status_pub_ = create_publisher<amr_interfaces::msg::MachineControllerStatus>("/amr/machine/status", 5);
    actuator_pub_ = create_publisher<amr_interfaces::msg::MachineActuatorCommand>("/amr/machine/actuator_command", 5);
    inputs_sub_ = create_subscription<amr_interfaces::msg::MachineInputs>("/amr/machine/inputs", 5,
      [this](amr_interfaces::msg::MachineInputs::SharedPtr inputs) { receive_inputs(*inputs); });
    enable_service_ = create_service<amr_interfaces::srv::RequestMachineEnable>("/amr/machine/request_enable",
      [this](const amr_interfaces::srv::RequestMachineEnable::Request::SharedPtr request,
             amr_interfaces::srv::RequestMachineEnable::Response::SharedPtr response) {
        handle_enable(*request, *response);
      });
    reset_service_ = create_service<amr_interfaces::srv::RequestMachineReset>("/amr/machine/request_reset",
      [this](const amr_interfaces::srv::RequestMachineReset::Request::SharedPtr request,
             amr_interfaces::srv::RequestMachineReset::Response::SharedPtr response) {
        handle_reset(*request, *response);
      });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
    state_pub_->on_activate(); status_pub_->on_activate(); actuator_pub_->on_activate();
    boot_id_ = static_cast<uint32_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    sequence_ = 0; have_inputs_ = false; enable_acknowledged_ = false; reset_requested_ = false;
    fault_latched_ = false;
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
    tick();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
    timer_.reset(); state_pub_->on_deactivate(); status_pub_->on_deactivate(); actuator_pub_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

 private:
  static constexpr uint32_t kAllInputsValid = 0x1fff;

  void receive_inputs(const amr_interfaces::msg::MachineInputs & inputs) {
    inputs_ = inputs; have_inputs_ = true; last_input_ = std::chrono::steady_clock::now();
  }

  bool fresh_valid() const {
    return have_inputs_ && inputs_.valid && (inputs_.validity_mask & kAllInputsValid) == kAllInputsValid &&
      std::chrono::steady_clock::now() - last_input_ <= input_timeout_;
  }
  bool channels_consistent_and_safe() const {
    return inputs_.estop_channel_a_ok == inputs_.estop_channel_b_ok && inputs_.estop_channel_a_ok &&
      inputs_.bumper_channel_a_ok == inputs_.bumper_channel_b_ok && inputs_.bumper_channel_a_ok;
  }
  bool reset_eligible() const {
    return fresh_valid() && channels_consistent_and_safe() && inputs_.battery_ok && inputs_.bms_ok &&
      inputs_.control_rail_ok && inputs_.traction_rail_ok && !inputs_.driver_fault && !inputs_.charging_active;
  }
  bool critical_fault() const {
    return fresh_valid() && (!channels_consistent_and_safe() || inputs_.driver_fault);
  }
  void handle_enable(const amr_interfaces::srv::RequestMachineEnable::Request & request,
                     amr_interfaces::srv::RequestMachineEnable::Response & response) {
    response.accepted = request.valid && request.machine_enable_request && request.sequence != 0 &&
      request.source_boot_id != 0;
    response.acknowledged_sequence = response.accepted ? request.sequence : 0;
    response.reason = response.accepted ? 0 : 1;
    if (response.accepted) enable_acknowledged_ = true;
  }
  void handle_reset(const amr_interfaces::srv::RequestMachineReset::Request & request,
                    amr_interfaces::srv::RequestMachineReset::Response & response) {
    response.accepted = request.valid && request.machine_reset_request && request.sequence != 0 &&
      request.source_boot_id != 0 && reset_eligible();
    response.acknowledged_sequence = response.accepted ? request.sequence : 0;
    response.reason = response.accepted ? 0 : 1;
    if (response.accepted) reset_requested_ = true;
  }
  void tick() {
    if (critical_fault()) fault_latched_ = true;
    if (reset_requested_) { fault_latched_ = false; reset_requested_ = false; enable_acknowledged_ = false; }

    uint8_t state = amr_interfaces::msg::MachineState::INHIBITED;
    bool permission = false, precharge = false, k1 = false, k2 = false;
    uint16_t reason = 1;
    if (fault_latched_) { state = amr_interfaces::msg::MachineState::FAULT; reason = 2; }
    else if (!fresh_valid()) { reason = 3; }
    else if (inputs_.charging_active) { state = amr_interfaces::msg::MachineState::CHARGING; reason = 4; }
    else if (!inputs_.battery_ok || !inputs_.bms_ok || !inputs_.control_rail_ok || !inputs_.traction_rail_ok || !channels_consistent_and_safe()) { reason = 5; }
    else if (!enable_acknowledged_) { reason = 6; }
    else if (!inputs_.precharge_complete) { state = amr_interfaces::msg::MachineState::PRECHARGING; precharge = true; reason = 7; }
    else if (!inputs_.k1_feedback_closed || !inputs_.k2_feedback_closed || !inputs_.traction_bus_ok) { state = amr_interfaces::msg::MachineState::PRECHARGING; precharge = true; k1 = true; k2 = true; reason = 8; }
    else { state = amr_interfaces::msg::MachineState::READY; permission = true; precharge = true; k1 = true; k2 = true; reason = 0; }

    const auto stamp = now(); const auto seq = ++sequence_;
    amr_interfaces::msg::MachineState machine_state;
    machine_state.header.stamp = stamp; machine_state.sequence = seq; machine_state.valid = fresh_valid();
    machine_state.source_boot_id = boot_id_; machine_state.state = state; machine_state.reason = reason;
    machine_state.raw_permission = permission; state_pub_->publish(machine_state);
    amr_interfaces::msg::MachineControllerStatus status;
    status.header.stamp = stamp; status.sequence = seq; status.valid = machine_state.valid;
    status.source_boot_id = boot_id_; status.state = fault_latched_ ? status.FAULT : status.READY;
    status.reason = reason; status_pub_->publish(status);
    amr_interfaces::msg::MachineActuatorCommand command;
    command.header.stamp = stamp; command.sequence = seq; command.valid = fresh_valid(); command.source_boot_id = boot_id_;
    command.precharge_command = precharge; command.k1_command = k1; command.k2_command = k2;
    command.drive_enable_request = permission; command.controlled_stop_request = !permission; command.reason = reason;
    actuator_pub_->publish(command);
  }

  std::chrono::milliseconds input_timeout_{500}; std::chrono::steady_clock::time_point last_input_{};
  amr_interfaces::msg::MachineInputs inputs_{}; bool have_inputs_{false}, enable_acknowledged_{false}, reset_requested_{false}, fault_latched_{false};
  uint32_t boot_id_{0}, sequence_{0}; rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<amr_interfaces::msg::MachineState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<amr_interfaces::msg::MachineControllerStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<amr_interfaces::msg::MachineActuatorCommand>::SharedPtr actuator_pub_;
  rclcpp::Subscription<amr_interfaces::msg::MachineInputs>::SharedPtr inputs_sub_;
  rclcpp::Service<amr_interfaces::srv::RequestMachineEnable>::SharedPtr enable_service_;
  rclcpp::Service<amr_interfaces::srv::RequestMachineReset>::SharedPtr reset_service_;
};
}  // namespace amr_machine_controller

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amr_machine_controller::MachineControllerNode>()->get_node_base_interface());
  rclcpp::shutdown();
}

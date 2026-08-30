#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

#include "amr_simulation/command_watchdog.hpp"

namespace amr_simulation {

// Native Gazebo-side watchdog. It can disable the simulated plant even if the
// ROS adapter or ROS/Gazebo bridge stops refreshing command traffic.
class CommandWatchdogSystem final
    : public gz::sim::System,
      public gz::sim::ISystemConfigure,
      public gz::sim::ISystemPreUpdate {
 public:
  void Configure(
      const gz::sim::Entity & entity,
      const std::shared_ptr<const sdf::Element> & sdf,
      gz::sim::EntityComponentManager & ecm,
      gz::sim::EventManager &) override {
    // The plugin must be attached to a valid model, not an arbitrary entity.
    const gz::sim::Model model(entity);
    if (!model.Valid(ecm)) {
      gzerr << "Command watchdog must be attached to a model" << std::endl;
      return;
    }

    // Read SDF configuration once; reject a timeout that cannot enforce expiry.
    const auto timeout_ms = sdf->Get<int>("command_timeout_ms", 200).first;
    if (timeout_ms <= 0) {
      gzerr << "command_timeout_ms must be positive" << std::endl;
      return;
    }
    watchdog_ = std::make_unique<CommandWatchdog>(
        std::chrono::milliseconds(timeout_ms));

    const auto model_name = model.Name(ecm);
    const auto command_topic = sdf->Get<std::string>(
        "command_topic", "/model/" + model_name + "/cmd_vel").first;
    const auto enable_topic = sdf->Get<std::string>(
        "enable_topic", "/model/" + model_name + "/enable").first;
    // Publish the plant-enable decision over Gazebo transport, outside ROS.
    enable_publisher_ = node_.Advertise<gz::msgs::Boolean>(enable_topic);
    if (!enable_publisher_) {
      gzerr << "Failed to advertise command-watchdog enable topic ["
            << enable_topic << "]" << std::endl;
      watchdog_.reset();
      return;
    }
    // Each native Twist is recorded as a new liveness event in OnCommand().
    if (!node_.Subscribe(
        command_topic, &CommandWatchdogSystem::OnCommand, this)) {
      gzerr << "Failed to subscribe command watchdog to ["
            << command_topic << "]" << std::endl;
      watchdog_.reset();
      return;
    }
    gzmsg << "Command watchdog monitoring [" << command_topic
          << "] with " << timeout_ms << " ms timeout" << std::endl;
  }

  void PreUpdate(
      const gz::sim::UpdateInfo & info,
      gz::sim::EntityComponentManager &) override {
    if (!watchdog_) {
      return;
    }
    // Convert asynchronous transport callbacks into one “new this update”
    // boolean by comparing atomic command epochs.
    const auto epoch = command_epoch_.load(std::memory_order_acquire);
    const bool command_received = epoch != observed_epoch_;
    observed_epoch_ = epoch;
    // The reusable watchdog owns time/reversal/timeout state transitions.
    const auto transition = watchdog_->update(
        info.simTime, command_received,
        info.dt < std::chrono::steady_clock::duration::zero());
    if (transition) {
      gzmsg << "Command watchdog enable=" << std::boolalpha << *transition
            << " at " << info.simTime.count() << std::endl;
      gz::msgs::Boolean message;
      message.set_data(*transition);
      enable_publisher_.Publish(message);
    }
  }

 private:
  void OnCommand(const gz::msgs::Twist &) {
    // Value content is irrelevant to liveness; receipt refreshes the watchdog.
    command_epoch_.fetch_add(1, std::memory_order_release);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher enable_publisher_;
  std::unique_ptr<CommandWatchdog> watchdog_;
  std::atomic<uint64_t> command_epoch_{0};
  uint64_t observed_epoch_{0};
};

}  // namespace amr_simulation

GZ_ADD_PLUGIN(
    amr_simulation::CommandWatchdogSystem,
    gz::sim::System,
    amr_simulation::CommandWatchdogSystem::ISystemConfigure,
    amr_simulation::CommandWatchdogSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    amr_simulation::CommandWatchdogSystem,
    "amr_simulation::CommandWatchdogSystem")

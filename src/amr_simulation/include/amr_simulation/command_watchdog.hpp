#pragma once

#include <chrono>
#include <optional>

namespace amr_simulation {

// Small state machine shared by the Gazebo plugin and its unit tests. update()
// returns a value only when the desired plant-enable state changes.
class CommandWatchdog {
 public:
  using Duration = std::chrono::steady_clock::duration;

  // Store the native simulation-time interval allowed between commands.
  explicit CommandWatchdog(Duration timeout) : timeout_(timeout) {}

  std::optional<bool> update(
      Duration sim_time, bool command_received, bool time_went_backward) {
    // A simulation reset cannot preserve command authority from its old epoch.
    if (time_went_backward) {
      have_command_ = false;
      last_command_time_ = Duration::zero();
      return set_enabled(false);
    }
    // A fresh command enables the plant and restarts its deadline.
    if (command_received) {
      have_command_ = true;
      last_command_time_ = sim_time;
      return set_enabled(true);
    }
    // Before any command, or after expiry, request plant disable.
    if (!have_command_ || sim_time - last_command_time_ > timeout_) {
      return set_enabled(false);
    }
    return std::nullopt;
  }

 private:
  std::optional<bool> set_enabled(bool enabled) {
    // Avoid publishing duplicate enable decisions every simulation update.
    if (initialized_ && enabled_ == enabled) {
      return std::nullopt;
    }
    initialized_ = true;
    enabled_ = enabled;
    return enabled;
  }

  Duration timeout_;
  Duration last_command_time_{Duration::zero()};
  bool initialized_{false};
  bool enabled_{false};
  bool have_command_{false};
};

}  // namespace amr_simulation

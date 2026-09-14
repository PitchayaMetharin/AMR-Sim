#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cmath>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "amr_interfaces/action/execute_product_cycle.hpp"
#include "amr_interfaces/action/navigate_station.hpp"
#include "amr_interfaces/action/run_sequence.hpp"
#include "amr_interfaces/action/transport_product.hpp"
#include "amr_interfaces/msg/factory_status.hpp"
#include "amr_interfaces/msg/manipulator_status.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "amr_interfaces/srv/set_operation_mode.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "yaml-cpp/yaml.h"

namespace amr_factory {

using namespace std::chrono_literals;
using Execute = amr_interfaces::action::ExecuteProductCycle;
using Transport = amr_interfaces::action::TransportProduct;
using RunSequence = amr_interfaces::action::RunSequence;
using NavigateStation = amr_interfaces::action::NavigateStation;
using FactoryStatus = amr_interfaces::msg::FactoryStatus;
using ManipulatorStatus = amr_interfaces::msg::ManipulatorStatus;
using SetOperationMode = amr_interfaces::srv::SetOperationMode;
using Trigger = std_srvs::srv::Trigger;
using ExecuteGoalHandle = rclcpp_action::ClientGoalHandle<Execute>;
using TransportGoalHandle = rclcpp_action::ServerGoalHandle<Transport>;
using SequenceGoalHandle = rclcpp_action::ServerGoalHandle<RunSequence>;
using HomeGoalHandle = rclcpp_action::ServerGoalHandle<NavigateStation>;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

struct ProductRecord {
  std::string product_id;
  std::string pickup_station;
  std::string dispatch_slot;
  bool autonomous_enabled{false};
};

struct StationRecord {
  std::string id;
  std::string role;
  std::array<double, 3> approach{0.0, 0.0, 0.0};
};

struct QueuedJob {
  std::shared_ptr<TransportGoalHandle> goal;
  std::string pickup_station;
  std::string destination_station;
  std::string product_id;
};

struct ExecuteCall {
  Execute::Result result;
  bool accepted{false};
  bool canceled{false};
};

class FactorySupervisorNode final : public rclcpp::Node {
 public:
  explicit FactorySupervisorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("factory_supervisor_node", options)
  {
    declare_parameter("products_config", "");
    declare_parameter("stations_config", "");
    load_registry();

    status_pub_ = create_publisher<FactoryStatus>(
      "/amr/factory/status", amr_interfaces::qos::authority());
    manipulator_sub_ = create_subscription<ManipulatorStatus>(
      "/amr/manipulation/status", amr_interfaces::qos::authority(),
      [this](ManipulatorStatus::SharedPtr message) {
        if (!message) return;
        std::lock_guard<std::mutex> lock(mutex_);
        update_manipulator_status_locked(*message);
      });
    execute_client_ = rclcpp_action::create_client<Execute>(
      this, "/amr/manipulation/execute_product_cycle");
    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/amr/mission/navigate_to_pose");

    transport_server_ = rclcpp_action::create_server<Transport>(
      this, "/amr/factory/transport_product",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Transport::Goal> goal) {
        return handle_transport_goal(goal);
      },
      [this](const std::shared_ptr<TransportGoalHandle> goal_handle) {
        return handle_transport_cancel(goal_handle);
      },
      [this](const std::shared_ptr<TransportGoalHandle> goal_handle) {
        handle_transport_accepted(goal_handle);
      });
    sequence_server_ = rclcpp_action::create_server<RunSequence>(
      this, "/amr/factory/run_sequence",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const RunSequence::Goal> goal) {
        return handle_sequence_goal(goal);
      },
      [this](const std::shared_ptr<SequenceGoalHandle> goal_handle) {
        return handle_sequence_cancel(goal_handle);
      },
      [this](const std::shared_ptr<SequenceGoalHandle> goal_handle) {
        handle_sequence_accepted(goal_handle);
      });
    home_server_ = rclcpp_action::create_server<NavigateStation>(
      this, "/amr/factory/navigate_station",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const NavigateStation::Goal> goal) {
        return handle_home_goal(goal);
      },
      [this](const std::shared_ptr<HomeGoalHandle> goal_handle) {
        return handle_home_cancel(goal_handle);
      },
      [this](const std::shared_ptr<HomeGoalHandle> goal_handle) {
        handle_home_accepted(goal_handle);
      });
    mode_service_ = create_service<SetOperationMode>(
      "/amr/factory/set_operation_mode",
      [this](const std::shared_ptr<SetOperationMode::Request> request,
             std::shared_ptr<SetOperationMode::Response> response) {
        handle_mode(request, response);
      });
    stop_service_ = create_service<Trigger>(
      "/amr/factory/stop_sequence",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!sequence_active_) {
          response->success = false;
          response->message = "no autonomous sequence is active";
          return;
        }
        graceful_stop_requested_ = true;
        response->success = true;
        response->message = "sequence will stop after the active delivery";
      });
    cancel_service_ = create_service<Trigger>(
      "/amr/factory/cancel_sequence",
      [this](const std::shared_ptr<Trigger::Request>,
             std::shared_ptr<Trigger::Response> response) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!sequence_active_ || sequence_terminal_decided_) {
            response->success = false;
            response->message = sequence_terminal_decided_ ?
              "autonomous sequence terminal outcome is already decided" :
              "no autonomous sequence is active";
            return;
          }
          sequence_cancel_requested_ = true;
          active_cancel_requested_ = true;
        }
        cancel_current_manipulation();
        response->success = true;
        response->message = "sequence cancellation accepted; no home motion will be added";
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
      sequence_cancel_requested_ = true;
    }
    cancel_current_manipulation();
    queue_condition_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (sequence_worker_.joinable()) sequence_worker_.join();
    if (home_worker_.joinable()) home_worker_.join();
  }

 private:
  static bool finite(double value) { return std::isfinite(value); }

  void update_manipulator_status_locked(const ManipulatorStatus & message)
  {
    if (have_manipulator_status_ &&
      message.source_boot_id == manipulator_status_.source_boot_id &&
      message.sequence <= manipulator_status_.sequence) return;
    manipulator_status_ = message;
    have_manipulator_status_ = true;
    manipulator_received_ = std::chrono::steady_clock::now();
    current_product_attached_ = message.product_attached;
    if (message.product_attached) {
      held_product_ = true;
    } else if (!fault_latched_ && message.valid &&
      message.source_boot_id != 0U && message.sequence != 0U &&
      message.state == ManipulatorStatus::STOWED_EMPTY &&
      message.product_id.empty() && message.base_motion_allowed)
    {
      held_product_ = false;
    }
  }

  void load_registry()
  {
    products_by_station_.clear();
    products_by_id_.clear();
    stations_.clear();
    registry_valid_ = false;
    std::string package_dir;
    try {
      package_dir = ament_index_cpp::get_package_share_directory("amr_factory");
    } catch (const std::exception &) {
      package_dir = ".";
    }
    const auto configured_products = get_parameter("products_config").as_string();
    const auto configured_stations = get_parameter("stations_config").as_string();
    const auto products_path = configured_products.empty() ?
      package_dir + "/config/products.yaml" : configured_products;
    const auto stations_path = configured_stations.empty() ?
      package_dir + "/config/stations.yaml" : configured_stations;
    try {
      const auto products_yaml = YAML::LoadFile(products_path);
      const auto stations_yaml = YAML::LoadFile(stations_path);
      const auto products = products_yaml["products"];
      const auto stations = stations_yaml["stations"];
      const auto slots = products_yaml["dispatch_slots"];
      if (!products || !products.IsMap() || !stations || !stations.IsMap() ||
        !slots || !slots.IsSequence() || slots.size() == 0U)
      {
        throw std::runtime_error("products, stations, and dispatch_slots have invalid YAML types");
      }

      const auto required_string = [](const YAML::Node & node, const std::string & key,
        const std::string & label) {
          const auto value = node[key];
          if (!value || !value.IsScalar())
            throw std::runtime_error(label + " is missing or not a scalar");
          const auto text = value.as<std::string>();
          if (text.empty()) throw std::runtime_error(label + " must not be empty");
          return text;
        };
      const auto strict_bool = [](const YAML::Node & node, const std::string & key,
        const std::string & label) {
          const auto value = node[key];
          if (!value || !value.IsScalar())
            throw std::runtime_error(label + " is missing or not a boolean");
          const auto text = value.Scalar();
          if (text == "true") return true;
          if (text == "false") return false;
          throw std::runtime_error(label + " must be true or false");
        };
      const auto parse_pose = [](const YAML::Node & node, const std::string & key,
        const std::string & label, const bool allow_null) -> std::optional<std::array<double, 3>> {
          const auto value = node[key];
          if ((!value || value.IsNull()) && allow_null) return std::nullopt;
          if (!value || !value.IsMap())
            throw std::runtime_error(label + " is missing or not a pose mapping");
          std::array<double, 3> pose{};
          try {
            pose = {value["x"].as<double>(), value["y"].as<double>(),
              value["yaw"].as<double>()};
          } catch (const YAML::Exception & error) {
            throw std::runtime_error(label + " is invalid: " + std::string(error.what()));
          }
          if (!std::all_of(pose.begin(), pose.end(), finite))
            throw std::runtime_error(label + " contains a non-finite value");
          return pose;
        };

      std::unordered_set<int> station_tag_ids;
      std::unordered_map<std::string, int> station_roles;
      for (const auto & entry : stations) {
        if (!entry.first.IsScalar() || !entry.second.IsMap())
          throw std::runtime_error("station entries must be scalar IDs and mappings");
        const auto id = entry.first.as<std::string>();
        if (id.empty() || stations_.count(id) != 0U)
          throw std::runtime_error("duplicate or empty station ID: " + id);
        const auto role = required_string(entry.second, "role", id + ".role");
        if (role != "home" && role != "pickup" && role != "dispatch")
          throw std::runtime_error("invalid station role for " + id + ": " + role);
        const auto approach = parse_pose(entry.second, "approach", id + ".approach", false);
        const auto dock = parse_pose(entry.second, "dock", id + ".dock", true);
        const auto egress = parse_pose(entry.second, "egress", id + ".egress", true);
        if (role == "home" && (dock || egress))
          throw std::runtime_error("home station must not have dock or egress poses: " + id);
        if (role == "pickup" && (!dock || !egress))
          throw std::runtime_error("pickup station needs dock and egress poses: " + id);
        if (role == "dispatch" && (!dock || egress))
          throw std::runtime_error("dispatch station needs only an approach and dock pose: " + id);
        const auto tag = entry.second["tag_id"];
        if (tag) {
          int tag_id = 0;
          try { tag_id = tag.as<int>(); } catch (const YAML::Exception & error) {
            throw std::runtime_error(id + ".tag_id is invalid: " + std::string(error.what()));
          }
          if (tag_id <= 0 || !station_tag_ids.insert(tag_id).second)
            throw std::runtime_error("duplicate or invalid station tag ID: " + id);
        }
        StationRecord record;
        record.id = id;
        record.role = role;
        record.approach = *approach;
        stations_.emplace(id, record);
        ++station_roles[role];
      }
      if (station_roles["home"] != 1 || station_roles["dispatch"] != 1)
        throw std::runtime_error("registry must contain exactly one home and one dispatch station");

      std::unordered_set<std::string> slot_ids;
      for (const auto & slot : slots) {
        if (!slot.IsMap()) throw std::runtime_error("dispatch slot must be a mapping");
        const auto id = required_string(slot, "id", "dispatch slot ID");
        if (!slot_ids.insert(id).second)
          throw std::runtime_error("duplicate dispatch slot ID: " + id);
        try {
          const std::array<double, 3> position{
            slot["x"].as<double>(), slot["y"].as<double>(), slot["z"].as<double>()};
          if (!std::all_of(position.begin(), position.end(), finite))
            throw std::runtime_error("dispatch slot contains a non-finite pose: " + id);
        } catch (const YAML::Exception & error) {
          throw std::runtime_error("dispatch slot is invalid: " + id + ": " + error.what());
        }
      }

      std::unordered_set<int> product_tag_ids;
      std::unordered_set<std::string> product_pickup_ids;
      for (const auto & entry : products) {
        if (!entry.first.IsScalar() || !entry.second.IsMap())
          throw std::runtime_error("product entries must be scalar IDs and mappings");
        const auto model = entry.first.as<std::string>();
        int tag_id = 0;
        double mass = 0.0;
        try {
          tag_id = entry.second["tag_id"].as<int>();
          mass = entry.second["mass"].as<double>();
        } catch (const YAML::Exception & error) {
          throw std::runtime_error("product " + model + " has invalid tag or mass: " + error.what());
        }
        if (tag_id <= 0 || !product_tag_ids.insert(tag_id).second)
          throw std::runtime_error("duplicate or invalid product tag ID: " + model);
        if (!finite(mass) || mass <= 0.0)
          throw std::runtime_error("product " + model + " has an invalid mass");
        const auto pickup = required_string(entry.second, "pickup_station", model + ".pickup_station");
        const auto slot = required_string(entry.second, "dispatch_slot", model + ".dispatch_slot");
        const bool enabled = strict_bool(entry.second, "autonomous_enabled", model + ".autonomous_enabled");
        const auto station = stations_.find(pickup);
        if (station == stations_.end() || station->second.role != "pickup")
          throw std::runtime_error("product " + model + " references a non-pickup station: " + pickup);
        if (!slot_ids.count(slot))
          throw std::runtime_error("product " + model + " references a missing dispatch slot: " + slot);
        if (!product_pickup_ids.insert(pickup).second)
          throw std::runtime_error("multiple products reference pickup station: " + pickup);
        const auto id = std::to_string(tag_id);
        if (id == "103" && enabled)
          throw std::runtime_error("product 103 must remain disabled for autonomous work");
        ProductRecord record{id, pickup, slot, enabled};
        products_by_station_.emplace(pickup, record);
        if (!products_by_id_.emplace(id, record).second)
          throw std::runtime_error("duplicate product ID: " + id);
      }
      if (products_by_station_.empty() || products_by_station_.size() != products_by_id_.size())
        throw std::runtime_error("product registry mappings are incomplete");
      registry_valid_ = true;
    } catch (const YAML::Exception & error) {
      RCLCPP_ERROR(get_logger(), "factory YAML registry is invalid: %s", error.what());
      products_by_station_.clear();
      products_by_id_.clear();
      stations_.clear();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "factory station/product registry is invalid: %s", error.what());
      products_by_station_.clear();
      products_by_id_.clear();
      stations_.clear();
    }
  }

  bool valid_pickup(const std::string & station) const
  {
    return stations_.count(station) == 1 && stations_.at(station).role == "pickup" &&
      products_by_station_.count(station) == 1;
  }

  bool valid_destination(const std::string & station) const
  {
    return stations_.count(station) == 1 && stations_.at(station).role == "dispatch";
  }

  bool autonomous_pickup(const std::string & station) const
  {
    const auto found = products_by_station_.find(station);
    return valid_pickup(station) && found != products_by_station_.end() &&
      found->second.autonomous_enabled && found->second.product_id != "103";
  }

  bool manipulator_ready_locked() const
  {
    if (!have_manipulator_status_ ||
      std::chrono::steady_clock::now() - manipulator_received_ > 200ms) return false;
    const auto & status = manipulator_status_;
    return status.valid && status.source_boot_id != 0U && status.sequence != 0U &&
      status.state == ManipulatorStatus::STOWED_EMPTY && !status.product_attached &&
      status.product_id.empty() && status.base_motion_allowed;
  }

  bool safe_empty_stow_locked() const
  {
    return !fault_latched_ && !held_product_ && !current_product_attached_ &&
      manipulator_ready_locked();
  }

  bool wait_for_manipulator_ready(std::chrono::seconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (manipulator_ready_locked()) return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  bool duplicate_product_locked(const std::string & product_id) const
  {
    if (active_product_id_ == product_id && (active_goal_ || sequence_active_)) return true;
    return std::any_of(queue_.begin(), queue_.end(), [&product_id](const QueuedJob & job) {
      return job.product_id == product_id;
    });
  }

  rclcpp_action::GoalResponse handle_transport_goal(
    const std::shared_ptr<const Transport::Goal> & goal)
  {
    if (!goal || !registry_valid_ || !valid_pickup(goal->pickup_station_id) ||
      !valid_destination(goal->destination_station_id)) return rclcpp_action::GoalResponse::REJECT;
    const auto record = products_by_station_.at(goal->pickup_station_id);
    std::lock_guard<std::mutex> lock(mutex_);
    if (record.product_id == "103" || stopping_ || fault_latched_ || sequence_active_ ||
      home_active_ || pending_transport_reservation_ || active_goal_ || !queue_.empty() ||
      !manipulator_ready_locked() || duplicate_product_locked(record.product_id))
      return rclcpp_action::GoalResponse::REJECT;
    const std::size_t capacity = mode_ == FactoryStatus::AUTONOMOUS ? 3U : 1U;
    if ((active_goal_ ? 1U : 0U) + queue_.size() >= capacity) return rclcpp_action::GoalResponse::REJECT;
    pending_transport_reservation_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_transport_cancel(
    const std::shared_ptr<TransportGoalHandle> & goal_handle)
  {
    std::shared_ptr<TransportGoalHandle> active;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!goal_handle || !goal_handle->is_active()) return rclcpp_action::CancelResponse::REJECT;
      if (active_goal_ == goal_handle) {
        if (active_transport_terminal_decided_) return rclcpp_action::CancelResponse::REJECT;
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
        if (found->goal->is_canceling()) found->goal->canceled(result);
        else found->goal->abort(result);
        queue_.erase(found);
        last_outcome_ = result->outcome;
        return rclcpp_action::CancelResponse::ACCEPT;
      }
    }
    if (active) cancel_current_manipulation();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_transport_accepted(const std::shared_ptr<TransportGoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto record = products_by_station_.at(goal->pickup_station_id);
    const QueuedJob job{goal_handle, goal->pickup_station_id,
      goal->destination_station_id, record.product_id};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_transport_reservation_ = false;
      queue_.push_back(job);
      phase_ = queue_.size() == 1U && !active_goal_ ? 1U : 0U;
      detail_ = mode_ == FactoryStatus::AUTONOMOUS ? "queued FIFO transport goal" :
        "queued manual transport goal";
    }
    queue_condition_.notify_one();
  }

  rclcpp_action::GoalResponse handle_sequence_goal(
    const std::shared_ptr<const RunSequence::Goal> & goal)
  {
    if (!goal || !registry_valid_ || goal->pickup_station_ids.empty() ||
      goal->pickup_station_ids.size() > 16U ||
      (goal->final_station_id != "" && goal->final_station_id != "home"))
      return rclcpp_action::GoalResponse::REJECT;
    for (const auto & station : goal->pickup_station_ids)
      if (!autonomous_pickup(station)) return rclcpp_action::GoalResponse::REJECT;
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ != FactoryStatus::AUTONOMOUS || stopping_ || fault_latched_ || sequence_active_ ||
      sequence_reserved_ || active_goal_ || !queue_.empty() || home_active_ || home_reserved_ ||
      pending_transport_reservation_ || !manipulator_ready_locked())
      return rclcpp_action::GoalResponse::REJECT;
    sequence_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_sequence_cancel(
    const std::shared_ptr<SequenceGoalHandle> & goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goal_handle || goal_handle != sequence_goal_ || sequence_terminal_decided_ ||
      !goal_handle->is_active())
      return rclcpp_action::CancelResponse::REJECT;
    sequence_cancel_requested_ = true;
    active_cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_sequence_accepted(const std::shared_ptr<SequenceGoalHandle> & goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sequence_reserved_ = false;
      sequence_goal_ = goal_handle;
      sequence_template_ = goal->pickup_station_ids;
      sequence_cycle_limit_ = goal->cycle_count;
      sequence_final_station_ = goal->final_station_id;
      sequence_active_ = true;
      sequence_terminal_decided_ = false;
      sequence_cancel_requested_ = false;
      graceful_stop_requested_ = false;
      completed_jobs_ = 0;
      completed_cycles_ = 0;
      current_cycle_ = 0;
      sequence_index_ = 0;
      detail_ = "autonomous sequence accepted; scheduling one cycle at a time";
    }
    if (sequence_worker_.joinable()) sequence_worker_.join();
    sequence_worker_ = std::thread([this]() { sequence_loop(); });
  }

  rclcpp_action::GoalResponse handle_home_goal(
    const std::shared_ptr<const NavigateStation::Goal> & goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goal || goal->station_id != "home" || !registry_valid_ || stopping_ ||
      fault_latched_ || sequence_active_ || sequence_reserved_ || active_goal_ ||
      !queue_.empty() || home_active_ || home_reserved_ || pending_transport_reservation_ ||
      !manipulator_ready_locked()) return rclcpp_action::GoalResponse::REJECT;
    home_reserved_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_home_cancel(const std::shared_ptr<HomeGoalHandle> & goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!goal_handle || goal_handle != home_goal_ || !goal_handle->is_active())
      return rclcpp_action::CancelResponse::REJECT;
    home_cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_home_accepted(const std::shared_ptr<HomeGoalHandle> & goal_handle)
  {
    if (home_worker_.joinable()) home_worker_.join();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      home_reserved_ = false;
      home_goal_ = goal_handle;
      home_active_ = true;
      home_cancel_requested_ = false;
      detail_ = "navigating to registered home approach";
    }
    home_worker_ = std::thread([this, goal_handle]() { execute_home(goal_handle); });
  }

  void handle_mode(const std::shared_ptr<SetOperationMode::Request> & request,
    const std::shared_ptr<SetOperationMode::Response> & response)
  {
    if (!request || (request->mode != SetOperationMode::Request::MANUAL &&
      request->mode != SetOperationMode::Request::AUTONOMOUS)) {
      response->accepted = false;
      response->message = "unknown operation mode";
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ || !queue_.empty() || pending_transport_reservation_ || sequence_reserved_ ||
      sequence_active_ || home_reserved_ || home_active_ || fault_latched_ ||
      !manipulator_ready_locked()) {
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
        queue_condition_.wait(lock, [this]() { return stopping_ || !queue_.empty(); });
        if (stopping_) return;
        job = queue_.front();
        queue_.pop_front();
        active_goal_ = job.goal;
        active_product_id_ = job.product_id;
        active_cancel_requested_ = false;
        active_transport_terminal_decided_ = false;
        phase_ = 2U;
        current_station_id_ = job.pickup_station;
        current_destination_id_ = job.destination_station;
        detail_ = "starting dependency-gated product cycle";
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
        phase_ = fault_latched_ ? 5U : (queue_.empty() ? 0U : 1U);
        if (fault_latched_) abort_queued_jobs_locked();
      }
    }
  }

  void execute_job(const QueuedJob & job)
  {
    publish_transport_feedback(job.goal, 1U, job.pickup_station, false);
    auto call = call_execute_cycle(job.pickup_station, job.destination_station, job.product_id);
    const bool explicit_interlock = call.result.outcome == Execute::Result::RETAINED_PRODUCT_FAULT ||
      call.result.outcome == Execute::Result::INTERLOCK_FAILED;
    const bool proof_required = !explicit_interlock;
    const bool proof = proof_required ? wait_for_manipulator_ready(2s) : false;
    auto result = std::make_shared<Transport::Result>();
    int terminal_kind = 3;  // abort
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (active_transport_terminal_decided_) return;

      const bool safe_empty_stow = proof && safe_empty_stow_locked();
      const bool cancellation_requested = active_cancel_requested_ || stopping_;
      uint8_t outcome = Transport::Result::PLACE_FAILED;
      std::string message;
      bool committed_success = false;
      bool committed_cancellation = false;

      if (explicit_interlock) {
        outcome = Transport::Result::INTERLOCK_FAILED;
        message = call.result.message.empty() ?
          "product cycle reported an interlock failure" : call.result.message;
        fault_latched_ = true;
        if (call.result.outcome == Execute::Result::RETAINED_PRODUCT_FAULT) {
          held_product_ = true;
          current_product_attached_ = true;
        }
      } else if (call.result.outcome != Execute::Result::SUCCESS &&
        call.result.outcome != Execute::Result::CANCELED)
      {
        if (call.result.outcome == Execute::Result::DEPENDENCY_UNAVAILABLE) {
          outcome = Transport::Result::DEPENDENCY_UNAVAILABLE;
        } else if (call.result.outcome == Execute::Result::PREPARATION_FAILED) {
          outcome = Transport::Result::PICK_FAILED;
        }
        message = call.result.message.empty() ? "product cycle action failed" : call.result.message;
        if (!safe_empty_stow) {
          outcome = Transport::Result::INTERLOCK_FAILED;
          fault_latched_ = true;
          message = "product cycle failure ended without a fresh empty-stow proof";
        }
      } else if ((call.result.outcome == Execute::Result::CANCELED || call.canceled) &&
        (call.result.outcome != Execute::Result::SUCCESS || call.result.delivered)) {
        if (safe_empty_stow) {
          outcome = Transport::Result::CANCELED;
          message = "transport canceled during product cycle; no new work started";
          committed_cancellation = true;
        } else {
          outcome = Transport::Result::INTERLOCK_FAILED;
          fault_latched_ = true;
          message = "transport cancellation ended without a fresh empty-stow proof";
        }
      } else if (!call.result.delivered) {
        outcome = Transport::Result::PLACE_FAILED;
        message = "product cycle returned success without delivered=true";
        if (!safe_empty_stow) {
          outcome = Transport::Result::INTERLOCK_FAILED;
          fault_latched_ = true;
          message = "product cycle returned delivered=false without a fresh empty-stow proof";
        }
      } else if (!safe_empty_stow) {
        outcome = Transport::Result::INTERLOCK_FAILED;
        fault_latched_ = true;
        message = "delivery succeeded without a fresh empty-stow proof";
      } else if (cancellation_requested) {
        outcome = Transport::Result::CANCELED;
        message = "transport canceled during product cycle; no new work started";
        committed_cancellation = true;
      } else {
        outcome = Transport::Result::SUCCESS;
        message = "transport completed through ExecuteProductCycle";
        committed_success = true;
      }

      result->delivered = committed_success;
      result->outcome = outcome;
      result->message = message;
      active_transport_terminal_decided_ = true;
      active_cancel_requested_ = false;
      last_outcome_ = outcome;
      detail_ = message;
      phase_ = fault_latched_ ? 5U : 0U;
      if (committed_success) ++completed_jobs_;
      terminal_kind = committed_cancellation ? 2 : (committed_success ? 1 : 3);
    }

    if (!job.goal || !job.goal->is_active()) return;
    if (terminal_kind == 1) {
      job.goal->succeed(result);
    } else if (terminal_kind == 2 && job.goal->is_canceling()) {
      job.goal->canceled(result);
    } else {
      job.goal->abort(result);
    }
  }

  ExecuteCall call_execute_cycle(const std::string & pickup, const std::string & destination,
    const std::string & product_id)
  {
    ExecuteCall call;
    const auto reject_pre_dispatch_locked = [&](const char * cancellation_message) {
      const bool had_fault = fault_latched_;
      const bool retained_product = held_product_ || current_product_attached_;
      const bool fresh_empty_stow = manipulator_ready_locked();
      call.result.delivered = false;
      call.result.product_id = product_id;
      if (had_fault || retained_product || !fresh_empty_stow) {
        fault_latched_ = true;
        call.result.outcome = Execute::Result::INTERLOCK_FAILED;
        call.result.message = had_fault ?
          "cycle dispatch blocked by a latched factory fault" :
          retained_product ?
          "cycle dispatch blocked by retained or attached product" :
          "cycle dispatch blocked without a fresh empty-stow proof";
        return true;
      }
      if (sequence_cancel_requested_ || active_cancel_requested_ || stopping_) {
        call.canceled = true;
        call.result.outcome = Execute::Result::CANCELED;
        call.result.message = cancellation_message;
        return true;
      }
      return false;
    };
    const auto reject_pre_dispatch = [&](const char * cancellation_message) {
      std::lock_guard<std::mutex> lock(mutex_);
      return reject_pre_dispatch_locked(cancellation_message);
    };
    if (reject_pre_dispatch("cycle cancellation was requested before dispatch")) return call;
    if (!execute_client_->wait_for_action_server(2s)) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reject_pre_dispatch_locked(
            "cycle cancellation was requested while waiting for the action server")) return call;
      }
      call.result.outcome = Execute::Result::DEPENDENCY_UNAVAILABLE;
      call.result.product_id = product_id;
      call.result.message = "manipulation action dependency is unavailable";
      return call;
    }
    if (reject_pre_dispatch(
        "cycle cancellation was requested while waiting for the action server")) return call;
    Execute::Goal goal;
    goal.pickup_station_id = pickup;
    goal.destination_station_id = destination;
    struct PendingGoal {
      std::mutex mutex;
      std::condition_variable condition;
      bool response_ready{false};
      bool abandoned{false};
      std::shared_ptr<ExecuteGoalHandle> handle;
    };
    const auto pending = std::make_shared<PendingGoal>();
    rclcpp_action::Client<Execute>::SendGoalOptions options;
    options.goal_response_callback =
      [this, pending](std::shared_ptr<ExecuteGoalHandle> goal_handle) {
        bool cancel_late = false;
        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          pending->handle = goal_handle;
          pending->response_ready = true;
          cancel_late = pending->abandoned;
        }
        pending->condition.notify_all();
        if (goal_handle && cancel_late) cancel_current_manipulation(goal_handle);
      };
    std::unique_lock<std::mutex> lock(mutex_);
    if (reject_pre_dispatch_locked("cycle cancellation was requested before dispatch")) return call;
    execute_client_->async_send_goal(goal, options);
    lock.unlock();
    bool canceled_before_acceptance = false;
    const auto acceptance_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> pending_lock(pending->mutex);
      if (pending->response_ready) break;
      pending_lock.unlock();
      if (is_cancel_requested()) {
        canceled_before_acceptance = true;
        std::lock_guard<std::mutex> lock(pending->mutex);
        pending->abandoned = true;
      }
      pending_lock.lock();
      if (pending->response_ready) break;
      if (std::chrono::steady_clock::now() >= acceptance_deadline) {
        pending->abandoned = true;
        break;
      }
      pending->condition.wait_for(pending_lock, 50ms);
    }
    std::shared_ptr<ExecuteGoalHandle> goal_handle;
    bool response_ready = false;
    {
      std::lock_guard<std::mutex> lock(pending->mutex);
      response_ready = pending->response_ready;
      if (response_ready) goal_handle = pending->handle;
      else pending->abandoned = true;
    }
    if (!response_ready || !goal_handle) {
      call.result.outcome = canceled_before_acceptance ? Execute::Result::CANCELED :
        Execute::Result::INTERLOCK_FAILED;
      call.result.product_id = product_id;
      call.result.message = canceled_before_acceptance ?
        "cycle cancellation was not resolved before ExecuteProductCycle acceptance" :
        "ExecuteProductCycle goal acceptance timed out without retained cancellation ownership";
      call.canceled = canceled_before_acceptance;
      return call;
    }
    call.accepted = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_execute_goal_ = goal_handle;
    }
    auto result_future = execute_client_->async_get_result(goal_handle);
    if (canceled_before_acceptance || is_cancel_requested()) {
      call.canceled = true;
      cancel_current_manipulation(goal_handle);
    }
    while (rclcpp::ok()) {
      if (result_future.wait_for(50ms) == std::future_status::ready) break;
      if (is_cancel_requested()) {
        call.canceled = true;
        cancel_current_manipulation(goal_handle);
        break;
      }
    }
    if (call.canceled && result_future.wait_for(5s) != std::future_status::ready) {
      call.result.outcome = Execute::Result::INTERLOCK_FAILED;
      call.result.product_id = product_id;
      call.result.message = "manipulation cancellation did not reach a terminal result";
    } else if (result_future.wait_for(0s) == std::future_status::ready) {
      const auto wrapped = result_future.get();
      if (wrapped.result) call.result = *wrapped.result;
      if (wrapped.code == rclcpp_action::ResultCode::CANCELED) call.canceled = true;
      if (!wrapped.result) {
        call.result.outcome = Execute::Result::EXECUTION_FAILED;
        call.result.product_id = product_id;
        call.result.message = "manipulation returned no result";
      }
    } else {
      call.result.outcome = Execute::Result::EXECUTION_FAILED;
      call.result.product_id = product_id;
      call.result.message = "manipulation action stopped without a result";
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (current_execute_goal_ == goal_handle) current_execute_goal_.reset();
    }
    return call;
  }

  void sequence_loop()
  {
    RunSequence::Result result;
    result.outcome = RunSequence::Result::SUCCESS;
    std::string terminal_message = "autonomous sequence completed";
    bool canceled = false;
    bool failed = false;
    bool dependency_failed = false;
    bool interlock_failed = false;
    bool cancellation_proof_established = false;
    while (true) {
      for (std::size_t index = 0; index < sequence_template_.size(); ++index) {
        std::string product_id;
        bool cancel_requested = false;
        bool dispatch_interlock = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          cancel_requested = sequence_cancel_requested_ || active_cancel_requested_ || stopping_;
          dispatch_interlock = fault_latched_ || held_product_ || current_product_attached_ ||
            !manipulator_ready_locked();
        }
        if (cancel_requested || dispatch_interlock) {
          const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
          std::lock_guard<std::mutex> lock(mutex_);
          cancel_requested = sequence_cancel_requested_ || active_cancel_requested_ || stopping_;
          const bool had_fault = fault_latched_;
          const bool retained_product = held_product_ || current_product_attached_;
          const bool safe_empty_stow = fresh_empty_stow && safe_empty_stow_locked();
          if (!safe_empty_stow) {
            fault_latched_ = true;
            failed = true;
            interlock_failed = true;
            terminal_message = had_fault ?
              "sequence cancellation blocked by a latched factory fault" :
              retained_product ?
              "sequence dispatch blocked by retained or attached product" :
              "sequence dispatch blocked without a fresh empty-stow proof";
          } else if (cancel_requested) {
            canceled = true;
            cancellation_proof_established = true;
            terminal_message = "autonomous sequence canceled; no home motion was added";
          }
          if (failed || canceled) break;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          sequence_index_ = static_cast<uint32_t>(index);
          current_cycle_ = completed_cycles_ + 1U;
          current_station_id_ = sequence_template_[index];
          current_destination_id_ = "dispatch";
          product_id = products_by_station_.at(sequence_template_[index]).product_id;
          current_product_id_ = product_id;
          active_product_id_ = product_id;
          phase_ = 2U;
          detail_ = "executing ordered autonomous product cycle";
        }
        publish_sequence_feedback(1U, "PREPARING");
        const auto call = call_execute_cycle(
          sequence_template_[index], "dispatch", product_id);
        if (call.result.outcome != Execute::Result::SUCCESS &&
          call.result.outcome != Execute::Result::CANCELED)
        {
          failed = true;
          dependency_failed = call.result.outcome == Execute::Result::DEPENDENCY_UNAVAILABLE;
          const bool explicit_interlock = call.result.outcome == Execute::Result::RETAINED_PRODUCT_FAULT ||
            call.result.outcome == Execute::Result::INTERLOCK_FAILED;
          interlock_failed = explicit_interlock;
          terminal_message = call.result.message.empty() ?
            "autonomous sequence stopped after product cycle failure" : call.result.message;
          if (explicit_interlock) {
            std::lock_guard<std::mutex> lock(mutex_);
            fault_latched_ = true;
            if (call.result.outcome == Execute::Result::RETAINED_PRODUCT_FAULT) {
              held_product_ = true;
              current_product_attached_ = true;
            }
          } else {
            const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
            std::lock_guard<std::mutex> lock(mutex_);
            const bool safe_empty_stow = fresh_empty_stow && safe_empty_stow_locked();
            const bool had_fault = fault_latched_;
            const bool retained_product = held_product_ || current_product_attached_;
            if (!safe_empty_stow) {
              fault_latched_ = true;
              dependency_failed = false;
              interlock_failed = true;
              terminal_message = had_fault ?
                "autonomous sequence stopped after a latched factory fault" :
                retained_product ?
                "autonomous sequence failed with retained or attached product" :
                "autonomous sequence failed without a fresh empty-stow proof";
            }
          }
          break;
        }
        if (call.result.outcome == Execute::Result::CANCELED ||
          (call.canceled &&
          (call.result.outcome != Execute::Result::SUCCESS || call.result.delivered)) ||
          (is_cancel_requested() && call.result.delivered))
        {
          const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
          std::lock_guard<std::mutex> lock(mutex_);
          if (!fresh_empty_stow || !safe_empty_stow_locked()) {
            failed = true;
            interlock_failed = true;
            terminal_message = "sequence cancellation ended without a fresh empty-stow proof";
            fault_latched_ = true;
          } else {
            canceled = true;
            cancellation_proof_established = true;
            terminal_message = "autonomous sequence canceled; no home motion was added";
          }
          break;
        }
        const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const bool safe_empty_stow = fresh_empty_stow && safe_empty_stow_locked();
          if (call.result.delivered && safe_empty_stow) {
            // The successful-delivery commit below performs the final, locked
            // proof and cancellation check.
          } else {
            failed = true;
            interlock_failed = !safe_empty_stow;
            terminal_message = !call.result.delivered && safe_empty_stow ?
              "cycle returned success without delivered=true" :
              "cycle completed without a fresh empty-stow proof";
            if (!safe_empty_stow) fault_latched_ = true;
            break;
          }
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const bool safe_empty_stow = safe_empty_stow_locked();
          const bool had_fault = fault_latched_;
          if (!safe_empty_stow) {
            failed = true;
            interlock_failed = true;
            terminal_message = had_fault ?
              "cycle completion observed a latched factory fault" :
              "cycle completion observed without a current fresh empty-stow proof";
            fault_latched_ = true;
            break;
          }
          const bool cancel_after_proof = sequence_cancel_requested_ ||
            active_cancel_requested_ || stopping_;
          ++completed_jobs_;
          last_outcome_ = Execute::Result::SUCCESS;
          if ((index + 1U) == sequence_template_.size()) ++completed_cycles_;
          current_product_attached_ = false;
          held_product_ = false;
          active_cancel_requested_ = false;
          if (cancel_after_proof) {
            canceled = true;
            cancellation_proof_established = true;
            terminal_message =
              "autonomous sequence canceled after the active delivery; no home motion was added";
          }
          if (!canceled && graceful_stop_requested_) {
            terminal_message = "graceful stop completed the active delivery";
            break;
          }
        }
        if (canceled || failed) break;
        publish_sequence_feedback(2U, "COMPLETE");
      }
      std::lock_guard<std::mutex> lock(mutex_);
      if (canceled || failed || graceful_stop_requested_ ||
        (sequence_cycle_limit_ != 0U && completed_cycles_ >= sequence_cycle_limit_)) break;
    }
    bool terminal_cancel_requested = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      terminal_cancel_requested = sequence_cancel_requested_ || active_cancel_requested_ || stopping_;
    }
    if (!canceled && !failed && terminal_cancel_requested) {
      const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
      std::lock_guard<std::mutex> lock(mutex_);
      const bool safe_empty_stow = fresh_empty_stow && safe_empty_stow_locked();
      const bool had_fault = fault_latched_;
      if (!safe_empty_stow) {
        fault_latched_ = true;
        failed = true;
        interlock_failed = true;
        terminal_message = had_fault ?
          "sequence cancellation blocked by a latched factory fault" :
          "sequence cancellation ended without a fresh empty-stow proof";
      } else {
        canceled = true;
        cancellation_proof_established = true;
        terminal_message = "autonomous sequence canceled; no home motion was added";
      }
    }
    std::string final_station;
    bool fault_latched = false;
    bool home_failed = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      final_station = sequence_final_station_;
      fault_latched = fault_latched_;
    }
    if (!canceled && !failed && !fault_latched && final_station == "home") {
      const bool home_success = navigate_home();
      bool canceled_during_home = false;
      bool home_cancel_failed = false;
      bool home_interlock_failed = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        canceled_during_home = sequence_cancel_requested_ || active_cancel_requested_ || stopping_;
        home_cancel_failed = home_cancel_confirmation_failed_;
        home_interlock_failed = home_interlock_failed_;
      }
      if (!home_success) {
        if (home_interlock_failed) {
          std::lock_guard<std::mutex> lock(mutex_);
          fault_latched_ = true;
          failed = true;
          interlock_failed = true;
          terminal_message = "home navigation blocked by a fresh empty-stow interlock";
        } else if (canceled_during_home) {
          if (home_cancel_failed) {
            std::lock_guard<std::mutex> lock(mutex_);
            fault_latched_ = true;
            failed = true;
            interlock_failed = true;
            terminal_message = "home cancellation did not reach a confirmed CANCELED result";
          } else {
            const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
            std::lock_guard<std::mutex> lock(mutex_);
            const bool safe_empty_stow = fresh_empty_stow && safe_empty_stow_locked();
            const bool had_fault = fault_latched_;
            if (!safe_empty_stow) {
              fault_latched_ = true;
              failed = true;
              interlock_failed = true;
              terminal_message = had_fault ?
                "sequence cancellation blocked by a latched factory fault" :
                "sequence cancellation ended without a fresh empty-stow proof";
            } else {
              canceled = true;
              cancellation_proof_established = true;
              terminal_message = "autonomous sequence canceled during home; no further motion added";
            }
          }
        } else {
          home_failed = true;
          terminal_message = "deliveries completed but registered home navigation failed";
        }
      } else {
        const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
        std::lock_guard<std::mutex> lock(mutex_);
        if (!fresh_empty_stow || !safe_empty_stow_locked()) {
          fault_latched_ = true;
          failed = true;
          interlock_failed = true;
          terminal_message = "home navigation ended without a fresh empty-stow proof";
        }
      }
    }
    std::shared_ptr<SequenceGoalHandle> goal;
    bool terminal_committed = false;
    while (!terminal_committed) {
      bool cancellation_needs_proof = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool cancel_requested = sequence_cancel_requested_ || active_cancel_requested_ || stopping_;
        if (!failed && (canceled || cancel_requested)) {
          if (!cancellation_proof_established) {
            cancellation_needs_proof = true;
          } else if (!safe_empty_stow_locked()) {
            fault_latched_ = true;
            canceled = false;
            failed = true;
            interlock_failed = true;
            terminal_message = "sequence cancellation ended without a current fresh empty-stow proof";
          }
        }
        if (!cancellation_needs_proof) {
          if (!canceled && !failed && cancel_requested) {
            canceled = true;
            terminal_message = "autonomous sequence canceled; no further motion was added";
          }
          if (!interlock_failed && !fault_latched_ && !safe_empty_stow_locked()) {
            const bool retained_product = held_product_ || current_product_attached_;
            fault_latched_ = true;
            canceled = false;
            failed = true;
            interlock_failed = true;
            dependency_failed = false;
            terminal_message = retained_product ?
              "sequence terminal commit blocked by retained or attached product" :
              "sequence terminal commit blocked without a current fresh empty-stow proof";
          }
          if (fault_latched_) {
            if (canceled) {
              terminal_message = "autonomous sequence cancellation ended with a latched factory fault";
            }
            canceled = false;
            failed = true;
            dependency_failed = false;
            interlock_failed = true;
          }
          if (canceled) result.outcome = RunSequence::Result::CANCELED;
          else if (failed) {
            if (dependency_failed) result.outcome = RunSequence::Result::DEPENDENCY_UNAVAILABLE;
            else if (interlock_failed || fault_latched_)
              result.outcome = RunSequence::Result::INTERLOCK_FAILED;
            else result.outcome = RunSequence::Result::JOB_FAILED;
          }
          else if (home_failed) result.outcome = RunSequence::Result::HOME_FAILED;
          else if (graceful_stop_requested_) result.outcome = RunSequence::Result::STOPPED;
          result.completed_jobs = completed_jobs_;
          result.completed_cycles = completed_cycles_;
          result.message = terminal_message;
          sequence_terminal_decided_ = true;
          goal = sequence_goal_;
          last_outcome_ = result.outcome;
          sequence_active_ = false;
          sequence_goal_.reset();
          active_product_id_.clear();
          current_product_id_.clear();
          current_station_id_.clear();
          current_destination_id_.clear();
          active_cancel_requested_ = false;
          sequence_cancel_requested_ = false;
          phase_ = fault_latched_ ? 5U : 0U;
          result.message = terminal_message;
          terminal_committed = true;
        }
      }
      if (cancellation_needs_proof) {
        const bool fresh_empty_stow = wait_for_manipulator_ready(2s);
        std::lock_guard<std::mutex> lock(mutex_);
        const bool safe_empty_stow = fresh_empty_stow && safe_empty_stow_locked();
        const bool had_fault = fault_latched_;
        if (!safe_empty_stow) {
          fault_latched_ = true;
          canceled = false;
          failed = true;
          interlock_failed = true;
          terminal_message = had_fault ?
            "sequence cancellation blocked by a latched factory fault" :
            "sequence cancellation ended without a fresh empty-stow proof";
        } else {
          canceled = true;
          cancellation_proof_established = true;
          terminal_message = "autonomous sequence canceled; no further motion was added";
        }
      }
    }
    if (goal && goal->is_active()) {
      if (result.outcome == RunSequence::Result::SUCCESS ||
        result.outcome == RunSequence::Result::STOPPED) goal->succeed(
          std::make_shared<RunSequence::Result>(result));
      else if (result.outcome == RunSequence::Result::CANCELED) {
        auto terminal = std::make_shared<RunSequence::Result>(result);
        if (goal->is_canceling()) goal->canceled(terminal);
        else goal->abort(terminal);
      }
      else goal->abort(std::make_shared<RunSequence::Result>(result));
    }
  }

  bool cancel_home_navigation_goal(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> & goal_handle,
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult> & result)
  {
    try {
      auto cancel = navigation_client_->async_cancel_goal(goal_handle);
      if (cancel.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "home navigation cancellation response timed out");
        return false;
      }
      const auto response = cancel.get();
      if (!response) {
        RCLCPP_ERROR(get_logger(), "home navigation cancellation returned no response");
        return false;
      }
      const auto & goal_id = goal_handle->get_goal_id();
      const bool listed = std::any_of(
        response->goals_canceling.begin(), response->goals_canceling.end(),
        [&goal_id](const auto & goal_info) { return goal_info.goal_id.uuid == goal_id; });
      if (!listed) {
        RCLCPP_ERROR(get_logger(), "home navigation cancellation did not acknowledge the goal");
        return false;
      }
      if (result.wait_for(3s) != std::future_status::ready) {
        RCLCPP_ERROR(get_logger(), "home navigation did not reach terminal CANCELED");
        return false;
      }
      const auto terminal = result.get();
      if (terminal.code != rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_ERROR(get_logger(),
          "home navigation terminal result after cancellation was code %d",
          static_cast<int>(terminal.code));
        return false;
      }
      return true;
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_ERROR(get_logger(), "home navigation cancellation found an unknown goal handle");
      return false;
    }
  }

  bool navigate_home()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      home_cancel_confirmation_failed_ = false;
      home_interlock_failed_ = false;
    }
    if (!navigation_client_->wait_for_action_server(2s)) return false;
    const auto pose = stations_.at("home").approach;
    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = pose[0];
    goal.pose.pose.position.y = pose[1];
    goal.pose.pose.orientation.z = std::sin(pose[2] * 0.5);
    goal.pose.pose.orientation.w = std::cos(pose[2] * 0.5);
    std::unique_lock<std::mutex> lock(mutex_);
    if (!safe_empty_stow_locked()) {
      home_interlock_failed_ = true;
      return false;
    }
    if (home_cancel_requested_ || sequence_cancel_requested_ || stopping_) return false;
    struct PendingHomeGoal {
      std::mutex mutex;
      std::condition_variable condition;
      bool response_ready{false};
      bool abandoned{false};
      std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> handle;
    };
    const auto pending = std::make_shared<PendingHomeGoal>();
    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, pending](
      std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> goal_handle) {
        bool cancel_late = false;
        {
          std::lock_guard<std::mutex> pending_lock(pending->mutex);
          pending->handle = goal_handle;
          pending->response_ready = true;
          cancel_late = pending->abandoned;
        }
        pending->condition.notify_all();
        if (goal_handle && cancel_late) {
          try {
            (void)navigation_client_->async_cancel_goal(goal_handle);
          } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
            RCLCPP_WARN(get_logger(), "late home goal was already terminal during cancellation");
          }
        }
      };
    navigation_client_->async_send_goal(goal, options);
    lock.unlock();
    bool canceled_before_acceptance = false;
    const auto acceptance_deadline = std::chrono::steady_clock::now() + 3s;
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> pending_lock(pending->mutex);
      if (pending->response_ready) break;
      pending_lock.unlock();
      {
        std::lock_guard<std::mutex> state_lock(mutex_);
        canceled_before_acceptance = home_cancel_requested_ ||
          sequence_cancel_requested_ || stopping_;
      }
      if (canceled_before_acceptance) {
        std::lock_guard<std::mutex> abandoned_lock(pending->mutex);
        pending->abandoned = true;
      }
      pending_lock.lock();
      if (pending->response_ready) break;
      if (std::chrono::steady_clock::now() >= acceptance_deadline) {
        pending->abandoned = true;
        break;
      }
      pending->condition.wait_for(pending_lock, 50ms);
    }
    std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>> goal_handle;
    bool response_ready = false;
    {
      std::lock_guard<std::mutex> pending_lock(pending->mutex);
      response_ready = pending->response_ready;
      if (response_ready) goal_handle = pending->handle;
      else pending->abandoned = true;
    }
    if (!response_ready || !goal_handle) {
      std::lock_guard<std::mutex> state_lock(mutex_);
      if (canceled_before_acceptance) home_cancel_confirmation_failed_ = true;
      else home_interlock_failed_ = true;
      return false;
    }
    auto result = navigation_client_->async_get_result(goal_handle);
    if (canceled_before_acceptance) {
      if (!cancel_home_navigation_goal(goal_handle, result)) {
        std::lock_guard<std::mutex> state_lock(mutex_);
        home_cancel_confirmation_failed_ = true;
      }
      return false;
    }
    while (rclcpp::ok()) {
      if (result.wait_for(50ms) == std::future_status::ready) {
        const auto terminal = result.get();
        bool cancel_requested = false;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          cancel_requested = home_cancel_requested_ || sequence_cancel_requested_ || stopping_;
        }
        if (terminal.code == rclcpp_action::ResultCode::SUCCEEDED) {
          std::lock_guard<std::mutex> lock(mutex_);
          if (!safe_empty_stow_locked()) {
            home_interlock_failed_ = true;
            return false;
          }
          return true;
        }
        if (cancel_requested) {
          std::lock_guard<std::mutex> lock(mutex_);
          home_cancel_confirmation_failed_ = true;
          return false;
        }
        return false;
      }
      bool cancel_requested = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        cancel_requested = home_cancel_requested_ || sequence_cancel_requested_ || stopping_;
      }
      if (cancel_requested) {
        const bool confirmed = cancel_home_navigation_goal(goal_handle, result);
        if (!confirmed) {
          std::lock_guard<std::mutex> lock(mutex_);
          home_cancel_confirmation_failed_ = true;
        }
        return false;
      }
    }
    return false;
  }

  void execute_home(const std::shared_ptr<HomeGoalHandle> & goal_handle)
  {
    auto result = std::make_shared<NavigateStation::Result>();
    result->outcome = NavigateStation::Result::NAVIGATION_FAILED;
    result->message = "registered home navigation failed";
    auto feedback = std::make_shared<NavigateStation::Feedback>();
    feedback->phase = 1U;
    feedback->current_station_id = "home";
    if (goal_handle->is_active()) goal_handle->publish_feedback(feedback);
    const bool success = navigate_home();
    bool terminal_committed = false;
    while (!terminal_committed) {
      bool proof_required = false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        proof_required = !home_interlock_failed_ && !home_cancel_confirmation_failed_;
      }
      const bool fresh_empty_stow = proof_required ? wait_for_manipulator_ready(2s) : false;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool cancellation_requested = home_cancel_requested_ ||
          sequence_cancel_requested_ || stopping_;
        const bool safe_empty_stow = proof_required && fresh_empty_stow &&
          safe_empty_stow_locked();
        if (home_interlock_failed_) {
          fault_latched_ = true;
          phase_ = 5U;
          result->outcome = NavigateStation::Result::INTERLOCK_FAILED;
          result->message = "home navigation blocked by a fresh empty-stow interlock";
        } else if (home_cancel_confirmation_failed_) {
          fault_latched_ = true;
          phase_ = 5U;
          result->outcome = NavigateStation::Result::INTERLOCK_FAILED;
          result->message = "home cancellation did not reach a confirmed CANCELED result";
        } else if (!safe_empty_stow) {
          const bool retained_product = held_product_ || current_product_attached_;
          const bool had_fault = fault_latched_;
          fault_latched_ = true;
          phase_ = 5U;
          result->outcome = NavigateStation::Result::INTERLOCK_FAILED;
          result->message = had_fault ?
            "home navigation ended with a latched factory fault" :
            retained_product ?
            "home navigation ended with retained or attached product" :
            "home navigation ended without a fresh empty-stow proof";
        } else if (cancellation_requested) {
          if (safe_empty_stow) {
            result->outcome = NavigateStation::Result::CANCELED;
            result->message = "home navigation canceled";
          }
        } else if (success) {
          result->outcome = NavigateStation::Result::SUCCESS;
          result->message = "arrived at registered home approach";
        } else {
          result->outcome = NavigateStation::Result::NAVIGATION_FAILED;
          result->message = "registered home navigation failed";
        }
        last_outcome_ = result->outcome;
        home_active_ = false;
        home_cancel_requested_ = false;
        home_cancel_confirmation_failed_ = false;
        home_interlock_failed_ = false;
        home_goal_.reset();
        terminal_committed = true;
      }
    }
    if (!goal_handle->is_active()) return;
    if (result->outcome == NavigateStation::Result::SUCCESS) goal_handle->succeed(result);
    else if (result->outcome == NavigateStation::Result::CANCELED) {
      if (goal_handle->is_canceling()) goal_handle->canceled(result);
      else goal_handle->abort(result);
    }
    else goal_handle->abort(result);
  }

  void cancel_current_manipulation()
  {
    std::shared_ptr<ExecuteGoalHandle> goal;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal = current_execute_goal_;
    }
    if (goal) cancel_current_manipulation(goal);
  }

  void cancel_current_manipulation(const std::shared_ptr<ExecuteGoalHandle> & goal)
  {
    try {
      execute_client_->async_cancel_goal(goal);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      RCLCPP_WARN(get_logger(), "manipulation goal was already terminal during cancellation");
    }
  }

  bool is_cancel_requested()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_cancel_requested_ || stopping_;
  }

  void finish_transport_canceled(const std::shared_ptr<TransportGoalHandle> & goal,
    const std::string & detail)
  {
    auto result = std::make_shared<Transport::Result>();
    result->delivered = false;
    result->outcome = Transport::Result::CANCELED;
    result->message = detail;
    if (goal && goal->is_active()) {
      if (goal->is_canceling()) goal->canceled(result);
      else goal->abort(result);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    detail_ = detail;
    last_outcome_ = result->outcome;
    phase_ = held_product_ ? 5U : 0U;
  }

  void finish_transport_failed(const std::shared_ptr<TransportGoalHandle> & goal,
    uint8_t outcome, const std::string & detail)
  {
    auto result = std::make_shared<Transport::Result>();
    result->delivered = false;
    result->outcome = outcome;
    result->message = detail;
    if (goal && goal->is_active()) goal->abort(result);
    std::lock_guard<std::mutex> lock(mutex_);
    detail_ = detail;
    last_outcome_ = outcome;
    if (held_product_ || outcome == Transport::Result::INTERLOCK_FAILED) {
      fault_latched_ = true;
      phase_ = 5U;
    }
  }

  void abort_queued_jobs_locked()
  {
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

  void publish_transport_feedback(const std::shared_ptr<TransportGoalHandle> & goal,
    uint8_t phase, const std::string & station, bool attached)
  {
    if (!goal || !goal->is_active()) return;
    auto feedback = std::make_shared<Transport::Feedback>();
    feedback->phase = phase;
    std::lock_guard<std::mutex> lock(mutex_);
    feedback->queue_position = static_cast<uint32_t>(queue_.size() + 1U);
    feedback->current_station_id = station;
    feedback->product_attached = attached;
    goal->publish_feedback(feedback);
  }

  void publish_sequence_feedback(uint8_t phase, const std::string & phase_name)
  {
    std::shared_ptr<SequenceGoalHandle> goal;
    auto feedback = std::make_shared<RunSequence::Feedback>();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      goal = sequence_goal_;
      feedback->current_cycle = current_cycle_;
      feedback->sequence_index = sequence_index_;
      feedback->completed_jobs = completed_jobs_;
      feedback->pickup_station_id = current_station_id_;
      feedback->product_id = current_product_id_;
      feedback->phase = phase;
      feedback->phase_name = phase_name;
      feedback->product_attached = current_product_attached_ || held_product_;
    }
    if (goal && goal->is_active()) goal->publish_feedback(feedback);
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
      status.active = static_cast<bool>(active_goal_) || sequence_active_ || home_active_;
      status.queue_depth = static_cast<uint32_t>(queue_.size());
      status.pickup_station_id = active_goal_ || sequence_active_ ? current_station_id_ : "";
      status.destination_station_id = active_goal_ || sequence_active_ ? current_destination_id_ : "";
      status.product_id = active_goal_ ? active_product_id_ : current_product_id_;
      status.product_attached = current_product_attached_ || held_product_;
      status.last_outcome = last_outcome_;
      status.detail = detail_;
      status.sequence_active = sequence_active_;
      status.current_cycle = current_cycle_;
      status.sequence_index = sequence_index_;
      status.completed_jobs = completed_jobs_;
      status.completed_cycles = completed_cycles_;
      status.final_station_id = sequence_final_station_;
      status.graceful_stop_requested = graceful_stop_requested_;
      status.fault_latched = fault_latched_;
    }
    status_pub_->publish(status);
  }

  rclcpp_action::Server<Transport>::SharedPtr transport_server_;
  rclcpp_action::Server<RunSequence>::SharedPtr sequence_server_;
  rclcpp_action::Server<NavigateStation>::SharedPtr home_server_;
  rclcpp_action::Client<Execute>::SharedPtr execute_client_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp::Service<SetOperationMode>::SharedPtr mode_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<FactoryStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<ManipulatorStatus>::SharedPtr manipulator_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::mutex mutex_;
  std::condition_variable queue_condition_;
  std::deque<QueuedJob> queue_;
  std::thread worker_;
  std::thread sequence_worker_;
  std::thread home_worker_;
  bool stopping_{false};
  bool pending_transport_reservation_{false};
  bool active_transport_terminal_decided_{false};
  bool sequence_reserved_{false};
  bool home_reserved_{false};
  bool active_cancel_requested_{false};
  bool sequence_cancel_requested_{false};
  bool graceful_stop_requested_{false};
  bool fault_latched_{false};
  bool held_product_{false};
  bool current_product_attached_{false};
  bool have_manipulator_status_{false};
  bool registry_valid_{false};
  bool sequence_active_{false};
  bool sequence_terminal_decided_{false};
  bool home_active_{false};
  bool home_cancel_requested_{false};
  bool home_cancel_confirmation_failed_{false};
  bool home_interlock_failed_{false};
  ManipulatorStatus manipulator_status_;
  std::chrono::steady_clock::time_point manipulator_received_{};
  std::unordered_map<std::string, ProductRecord> products_by_station_;
  std::unordered_map<std::string, ProductRecord> products_by_id_;
  std::unordered_map<std::string, StationRecord> stations_;
  std::shared_ptr<TransportGoalHandle> active_goal_;
  std::shared_ptr<ExecuteGoalHandle> current_execute_goal_;
  std::shared_ptr<SequenceGoalHandle> sequence_goal_;
  std::shared_ptr<HomeGoalHandle> home_goal_;
  std::vector<std::string> sequence_template_;
  uint32_t sequence_cycle_limit_{0};
  uint32_t current_cycle_{0};
  uint32_t sequence_index_{0};
  uint32_t completed_jobs_{0};
  uint32_t completed_cycles_{0};
  uint32_t sequence_{0};
  uint8_t mode_{FactoryStatus::MANUAL};
  uint8_t phase_{0};
  uint8_t last_outcome_{Transport::Result::SUCCESS};
  std::string active_product_id_;
  std::string current_product_id_;
  std::string current_station_id_;
  std::string current_destination_id_;
  std::string sequence_final_station_;
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

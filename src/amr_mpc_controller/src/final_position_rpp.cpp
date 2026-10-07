#include "amr_mpc_controller/final_position_rpp.hpp"

#include <algorithm>
#include <cmath>
#include "amr_interfaces/final_placement_stance.hpp"
#include "amr_interfaces/qos_profiles.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/node_utils.hpp"

namespace amr_mpc_controller
{
namespace
{
constexpr double kNominal = 0.50;
constexpr double kPrecisionBound = 0.10;
constexpr int64_t kAgeNs = 200000000;
int64_t steady_ns(std::chrono::steady_clock::time_point time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
const std::array<std::string, 4> kNames{
  "FinalPositionFollowPathA", "FinalPositionFollowPathB",
  "FinalPositionPlacementFollowPathA", "FinalPositionPlacementFollowPathB"};

bool valid_pose(const geometry_msgs::msg::Pose & p)
{
  const auto & q = p.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(norm) &&
         std::abs(norm - 1.0) <= 1e-6;
}

std::array<double, 3> canonical(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & name,
  std::vector<rclcpp::Parameter> & expected)
{
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(std::vector<double>{}));
  const auto p = node->get_parameter(name);
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    throw nav2_core::PlannerException("Final-position canonical geometry must be a double array");
  }
  const auto values = p.as_double_array();
  if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
      [](double v) {return std::isfinite(v);}))
  {
    throw nav2_core::PlannerException("Final-position canonical geometry missing or invalid");
  }
  expected.push_back(p);
  return {values[0], values[1], values[2]};
}
}  // namespace

template<class Base>
void ProfiledRPP<Base>::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (configured_) {throw nav2_core::PlannerException("Final-position controller already configured");}
  const auto node = parent.lock();
  if (!node) {throw nav2_core::PlannerException("Final-position parent expired");}
  const auto found = std::find(kNames.begin(), kNames.end(), name);
  if (found == kNames.end()) {throw nav2_core::PlannerException("Unknown final-position controller ID");}
  const auto index = static_cast<size_t>(found - kNames.begin());
  const bool precise = index >= 2;
  if (precise != std::is_same<Base, UpstreamRPP>::value) {
    throw nav2_core::PlannerException("Final-position controller mode/ID mismatch");
  }
  expected_.clear();
  const auto dock = canonical(node, "final_position_profiles.dispatch_dock", expected_);
  const auto a = canonical(node, "final_position_profiles.slot_a", expected_);
  const auto b = canonical(node, "final_position_profiles.slot_b", expected_);
  std::array<std::array<double, 3>, 2> references;
  try {
    references = {amr_interfaces::placement::final_placement_stance(101, dock, a).physical,
      amr_interfaces::placement::final_placement_stance(102, dock, b).physical};
  } catch (const std::exception & error) {
    throw nav2_core::PlannerException(error.what());
  }
  for (size_t i = 0; i < kNames.size(); ++i) {
    const auto & prefix = kNames[i];
    const bool placement = i >= 2;
    expected_.emplace_back(prefix + ".plugin", placement ?
      "amr_mpc_controller::FinalPositionPlacementRPP" : "amr_mpc_controller::FinalPositionRPP");
    expected_.emplace_back(prefix + ".desired_linear_vel", kNominal);
    expected_.emplace_back(prefix + ".min_approach_linear_velocity", placement ? 0.02 : 0.05);
    expected_.emplace_back(prefix + ".approach_velocity_scaling_dist", 0.60);
    expected_.emplace_back(prefix + ".allow_reversing", placement);
    expected_.emplace_back(prefix + ".use_rotate_to_heading", !placement);
    expected_.emplace_back(prefix + ".product_id", static_cast<int64_t>(101 + i%2));
    const auto & r = references[i%2];
    expected_.emplace_back(prefix + ".final_position_reference", std::vector<double>(r.begin(), r.end()));
  }
  // Predeclare ALL private contracts before the first guard. Later Base configure
  // reuses these declarations; nominal updates can therefore always be rejected.
  for (const auto & p : expected_) {
    nav2_util::declare_parameter_if_not_declared(node, p.get_name(), p.get_parameter_value());
    const auto actual = node->get_parameter(p.get_name());
    if (actual.get_type() != p.get_type() || actual.get_parameter_value() != p.get_parameter_value()) {
      throw nav2_core::PlannerException("Conflicting final-position contract: " + p.get_name());
    }
  }
  Base::configure(parent, name, std::move(tf), std::move(costmap_ros));
  reference_ = references[index%2];
  product_ = 101 + index%2;
  floor_ = precise ? 0.02 : 0.05;
  external_cap_ = kNominal;
  external_valid_ = true;
  generation_ = 0;
  last_band_ = -1;
  last_log_ = {};
  uint64_t token;
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    reset_sample();
    sampling_ = true;
    pending_allowed_ = false;
    token = ++sampling_token_;
  }
  subscription_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/amr/simulation/ground_truth/pose", amr_interfaces::qos::sensor(),
    [this, token](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) {
      receive(*message, token);
    });
  configured_ = true;
  active_ = false;
  install_guard();
}

template<class Base>
void ProfiledRPP<Base>::install_guard()
{
  guard_.reset();
  guard_ = this->node_.lock()->add_on_set_parameters_callback(
    [expected = expected_](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        for (const auto & e : expected) {
          if (p.get_name() != e.get_name()) {continue;}
          const bool nominal = e.get_name().find(".desired_linear_vel") != std::string::npos;
          if (nominal || p.get_type() != e.get_type() ||
            p.get_parameter_value() != e.get_parameter_value())
          {
            result.successful = false;
            result.reason = "Immutable final-position contract: " + e.get_name();
            return result;
          }
        }
      }
      return result;
    });
}

template<class Base>
void ProfiledRPP<Base>::reset_sample()
{
  eligible_ = false;
  high_water_ = 0;
  last_clock_ = -1;
  sample_ = geometry_msgs::msg::PoseStamped{};
  receipt_ = {};
  pending_state_ = PendingState::None;
  pending_ = geometry_msgs::msg::PoseStamped{};
  pending_receipt_ = pending_episode_start_ = {};
}

template<class Base>
bool ProfiledRPP<Base>::observe_clock(int64_t now)
{
  const bool rollback = last_clock_ >= 0 && now < last_clock_;
  if (rollback) {
    eligible_ = false; high_water_ = 0;
    pending_state_ = PendingState::Failed;
  }
  last_clock_ = now;
  return rollback;
}

template<class Base>
void ProfiledRPP<Base>::receive(const geometry_msgs::msg::PoseStamped & pose, uint64_t token)
{
  const auto callback_receipt = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lock(sample_mutex_);
  if (token != sampling_token_) {return;}  // Obsolete subscription cannot mutate a new lifecycle.
  last_receive_ = {};
  auto & observation = last_receive_;
  observation.sequence = ++receive_sequence_;
  observation.sec = pose.header.stamp.sec;
  observation.nsec = pose.header.stamp.nanosec;
  observation.acquired = static_cast<int64_t>(observation.sec)*1000000000 + observation.nsec;
  observation.previous_clock = last_clock_;
  observation.high_water_before = high_water_;
  observation.eligible_before = eligible_;
  observation.sampling = sampling_;
  const auto record = [&](ReceiveDecision decision) {
      observation.decision = decision;
      observation.callbacks = callbacks_;
      observation.high_water_after = high_water_;
      observation.eligible_after = eligible_;
      observation.receipt_ns = steady_ns(receipt_);
      observation.steady_ns = steady_ns(std::chrono::steady_clock::now());
    };
  if (!sampling_) {record(ReceiveDecision::NotSampling); return;}
  ++callbacks_;
  const int64_t now = this->clock_->now().nanoseconds();
  observation.now = now;
  observation.rollback = observe_clock(now);
  const auto & stamp = pose.header.stamp;
  const int64_t acquired = static_cast<int64_t>(stamp.sec)*1000000000 + stamp.nanosec;
  observation.bad_frame = pose.header.frame_id != "factory_world";
  observation.bad_pose = !valid_pose(pose.pose);
  observation.bad_sec = stamp.sec < 0;
  observation.bad_nsec = stamp.nanosec >= 1000000000;
  observation.nonpositive = acquired <= 0;
  observation.future = acquired > now;
  observation.stale = static_cast<long double>(now) - acquired > kAgeNs;
  if (observation.bad_frame || observation.bad_pose || observation.bad_sec ||
    observation.bad_nsec || observation.nonpositive || observation.stale || now <= 0)
  {
    eligible_ = false;
    pending_state_ = PendingState::Failed;
    record(ReceiveDecision::Invalid);
    return;
  }
  if (acquired > now) {
    eligible_ = false;
    if (!pending_allowed_ || pending_state_ == PendingState::Failed || observation.rollback ||
      (pending_state_ == PendingState::Waiting &&
      callback_receipt-pending_episode_start_ > std::chrono::nanoseconds(kAgeNs)))
    {
      pending_state_ = PendingState::Failed;
      record(ReceiveDecision::Invalid);
      return;
    }
    if (pending_state_ == PendingState::Waiting) {
      const int64_t previous = static_cast<int64_t>(pending_.header.stamp.sec)*1000000000 +
        pending_.header.stamp.nanosec;
      if (acquired == previous) {record(ReceiveDecision::Duplicate); return;}
      if (acquired < previous) {
        pending_state_ = PendingState::Failed;
        record(ReceiveDecision::Reordered);
        return;
      }
    } else {
      pending_episode_start_ = callback_receipt;
    }
    pending_ = pose;
    pending_receipt_ = callback_receipt;
    pending_state_ = PendingState::Waiting;
    record(ReceiveDecision::Pending);
    return;
  }
  if (acquired == high_water_) {record(ReceiveDecision::Duplicate); return;}  // No receipt-time refresh.
  if (acquired < high_water_) {
    eligible_ = false; pending_state_ = PendingState::Failed;
    record(ReceiveDecision::Reordered); return;
  }
  sample_ = pose;
  high_water_ = acquired;
  receipt_ = callback_receipt;
  eligible_ = true;
  pending_state_ = PendingState::None;
  record(ReceiveDecision::Accepted);
}

template<class Base>
void ProfiledRPP<Base>::activate()
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (!configured_) {throw nav2_core::PlannerException("Final-position controller unconfigured");}
  Base::activate();
  install_guard();  // Newest guard precedes all of this Base's callbacks.
  active_ = true;
  std::lock_guard<std::mutex> sample_lock(sample_mutex_);
  pending_allowed_ = true;
}

template<class Base>
void ProfiledRPP<Base>::deactivate()
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (configured_ && active_) {Base::deactivate();}
  active_ = false;
  std::lock_guard<std::mutex> sample_lock(sample_mutex_);
  pending_allowed_ = false;
  if (pending_state_ == PendingState::Waiting) {pending_state_ = PendingState::Failed;}
}

template<class Base>
void ProfiledRPP<Base>::cleanup()
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    sampling_ = false;
    pending_allowed_ = false;
    ++sampling_token_;
    reset_sample();
  }
  subscription_.reset();
  guard_.reset();
  if (configured_) {Base::cleanup();}
  configured_ = active_ = false;
  external_valid_ = true;
  external_cap_ = kNominal;
}

template<class Base>
void ProfiledRPP<Base>::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (!configured_) {throw nav2_core::PlannerException("Final-position controller unconfigured");}
  Base::setPlan(path);
  ++generation_;
}

template<class Base>
void ProfiledRPP<Base>::setSpeedLimit(const double & limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  external_valid_ = std::isfinite(limit) && limit >= 0.0;
  if (!external_valid_) {return;}
  external_cap_ = limit == nav2_costmap_2d::NO_SPEED_LIMIT ? kNominal :
    (percentage ? kNominal*std::min(limit, 100.0)/100.0 : std::min(limit, kNominal));
}

template<class Base>
geometry_msgs::msg::TwistStamped ProfiledRPP<Base>::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * checker)
{
  std::lock_guard<std::mutex> lock(profile_mutex_);
  if (!configured_) {throw nav2_core::PlannerException("Final-position controller unconfigured");}
  geometry_msgs::msg::PoseStamped physical;
  int64_t now;
  double receipt_age;
  {
    std::lock_guard<std::mutex> sample_lock(sample_mutex_);
    now = this->clock_->now().nanoseconds();
    const int64_t previous_clock = last_clock_;
    const int64_t previous_high_water = high_water_;
    const bool rollback = observe_clock(now);
    const auto steady_now = std::chrono::steady_clock::now();
    if (pending_state_ == PendingState::Waiting) {
      const auto & stamp = pending_.header.stamp;
      const int64_t pending_acquired = static_cast<int64_t>(stamp.sec)*1000000000 + stamp.nanosec;
      const bool valid_pending = active_ && sampling_ && !rollback && now > 0 &&
        pending_.header.frame_id == "factory_world" && valid_pose(pending_.pose) &&
        stamp.sec >= 0 && stamp.nanosec < 1000000000 && pending_acquired > high_water_ &&
        steady_now-pending_receipt_ <= std::chrono::nanoseconds(kAgeNs) &&
        steady_now-pending_episode_start_ <= std::chrono::nanoseconds(kAgeNs);
      if (!valid_pending || (pending_acquired <= now && now-pending_acquired > kAgeNs)) {
        eligible_ = false;
        pending_state_ = PendingState::Failed;
      } else if (pending_acquired > now) {
        if (!external_valid_) {throw nav2_core::PlannerException("Invalid external speed limit");}
        geometry_msgs::msg::TwistStamped zero;
        zero.header.stamp = rclcpp::Time(now, this->clock_->get_clock_type());
        zero.header.frame_id = this->costmap_ros_->getBaseFrameID();
        return zero;  // No inherited motion/collision computation or physical-stop proof.
      } else {
        sample_ = pending_;
        high_water_ = pending_acquired;
        receipt_ = pending_receipt_;  // Original callback entry, never admission time.
        eligible_ = true;
        pending_state_ = PendingState::None;
      }
    }
    const int64_t acquired = static_cast<int64_t>(sample_.header.stamp.sec)*1000000000 +
      sample_.header.stamp.nanosec;
    receipt_age = std::chrono::duration<double>(steady_now-receipt_).count();
    if (!active_ || rollback || !eligible_ || acquired > now || now - acquired > kAgeNs ||
      receipt_age > 0.200)
    {
      last_failure_ = {};
      auto & failure = last_failure_;
      failure.sequence = ++failure_sequence_;
      failure.callbacks = callbacks_;
      failure.generation = generation_;
      failure.now = now;
      failure.previous_clock = previous_clock;
      failure.high_water_before = previous_high_water;
      failure.high_water = high_water_;
      failure.acquired = acquired;
      failure.sec = sample_.header.stamp.sec;
      failure.nsec = sample_.header.stamp.nanosec;
      failure.steady_ns = steady_ns(steady_now);
      failure.receipt_ns = steady_ns(receipt_);
      failure.receipt_age = receipt_age;
      const auto & p = sample_.pose;
      failure.cached_pose = {p.position.x, p.position.y, p.position.z,
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
      failure.configured = configured_;
      failure.active = active_;
      failure.sampling = sampling_;
      failure.eligible = eligible_;
      failure.inactive = !active_;
      failure.rollback = rollback;
      failure.ineligible = !eligible_;
      failure.future = acquired > now;
      failure.acquisition_stale = static_cast<long double>(now) - acquired > kAgeNs;
      failure.receipt_stale = receipt_age > 0.200;
      failure.receive = last_receive_;
      const auto & r = failure.receive;
      // One record per existing throw, never receive/hot-path streaming. Logging
      // occurs after the decision under the same locks and adds failure-path latency.
      RCLCPP_ERROR(this->logger_,
        "FINAL_POSITION_OBSERVATION_FAILURE controller=%.64s product=%d failure_seq=%llu "
        "generation=%llu callbacks=%llu configured=%d active=%d sampling=%d eligible=%d "
        "fail_inactive=%d fail_rollback=%d fail_ineligible=%d fail_future=%d "
        "fail_acquisition_stale=%d fail_receipt_stale=%d command_ros_ns=%lld previous_clock_ns=%lld "
        "highwater_before_ns=%lld highwater_ns=%lld acq_sec=%d acq_nsec=%u acquired_ns=%lld acquisition_age_ns=%.0Lf "
        "steady_ns=%lld receipt_ns=%lld receipt_age_s=%.17g "
        "cached_x=%.17g cached_y=%.17g cached_z=%.17g cached_qx=%.17g cached_qy=%.17g "
        "cached_qz=%.17g cached_qw=%.17g receive_seq=%llu receive_decision=%u "
        "receive_callbacks=%llu receive_sec=%d receive_nsec=%u receive_acquired_ns=%lld "
        "receive_ros_ns=%lld receive_previous_clock_ns=%lld receive_highwater_before_ns=%lld "
        "receive_highwater_after_ns=%lld receive_steady_ns=%lld receive_receipt_ns=%lld "
        "receive_sampling=%d receive_rollback=%d receive_eligible_before=%d receive_eligible_after=%d "
        "receive_bad_frame=%d receive_bad_pose=%d receive_bad_sec=%d receive_bad_nsec=%d "
        "receive_nonpositive=%d receive_future=%d receive_stale=%d",
        this->plugin_name_.c_str(), product_, static_cast<unsigned long long>(failure.sequence),
        static_cast<unsigned long long>(failure.generation), static_cast<unsigned long long>(failure.callbacks),
        failure.configured, failure.active, failure.sampling, failure.eligible,
        failure.inactive, failure.rollback, failure.ineligible, failure.future,
        failure.acquisition_stale, failure.receipt_stale, static_cast<long long>(now),
        static_cast<long long>(previous_clock), static_cast<long long>(previous_high_water),
        static_cast<long long>(high_water_),
        failure.sec, failure.nsec, static_cast<long long>(acquired), static_cast<long double>(now)-acquired,
        static_cast<long long>(failure.steady_ns), static_cast<long long>(failure.receipt_ns), receipt_age,
        p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
        static_cast<unsigned long long>(r.sequence), static_cast<unsigned int>(r.decision),
        static_cast<unsigned long long>(r.callbacks), r.sec, r.nsec, static_cast<long long>(r.acquired),
        static_cast<long long>(r.now), static_cast<long long>(r.previous_clock),
        static_cast<long long>(r.high_water_before), static_cast<long long>(r.high_water_after),
        static_cast<long long>(r.steady_ns), static_cast<long long>(r.receipt_ns),
        r.sampling, r.rollback, r.eligible_before, r.eligible_after, r.bad_frame, r.bad_pose,
        r.bad_sec, r.bad_nsec, r.nonpositive, r.future, r.stale);
      throw nav2_core::PlannerException("Final-position physical observation missing, stale or inactive");
    }
    physical = sample_;
  }
  if (!external_valid_) {throw nav2_core::PlannerException("Invalid external speed limit");}
  const double d = std::hypot(physical.pose.position.x-reference_[0],
    physical.pose.position.y-reference_[1]);
  if (!std::isfinite(d)) {throw nav2_core::PlannerException("Invalid final-position distance");}
  const double profile = std::max(floor_, std::min(kNominal, kNominal*d));
  const double mode_bound = std::is_same<Base, UpstreamRPP>::value ? kPrecisionBound : kNominal;
  const double cap = std::min({profile, kNominal, external_cap_, mode_bound});
  {
    std::lock_guard<std::mutex> inherited_lock(this->mutex_);
    UpstreamRPP::setSpeedLimit(cap, false);
  }  // Base compute must acquire its own nonrecursive controller->costmap locks.
  const auto command = Base::computeVelocityCommands(pose, speed, checker);
  const int band = d <= 0.30 ? 0 : (d <= 1.0 ? 1 : 2);
  const auto wall = std::chrono::steady_clock::now();
  if (band != last_band_ || wall-last_log_ >= std::chrono::seconds(1)) {
    RCLCPP_INFO(this->logger_,
      "FINAL_POSITION_PROFILE controller=%s product=%d acq_sec=%d acq_nsec=%u "
      "command_ros_ns=%lld generation=%llu receipt_age_s=%.17g physical_x=%.17g physical_y=%.17g "
      "reference_x=%.17g reference_y=%.17g distance=%.17g profile_cap=%.17g "
      "external_cap=%.17g combined_cap=%.17g linear=%.17g angular=%.17g",
      this->plugin_name_.c_str(), product_, physical.header.stamp.sec, physical.header.stamp.nanosec,
      static_cast<long long>(now), static_cast<unsigned long long>(generation_),
      receipt_age, physical.pose.position.x, physical.pose.position.y, reference_[0], reference_[1],
      d, profile, external_cap_, cap, command.twist.linear.x, command.twist.angular.z);
    last_band_ = band;
    last_log_ = wall;
  }
  return command;
}

template class ProfiledRPP<HeadingLatchedRPP>;
template class ProfiledRPP<UpstreamRPP>;
}  // namespace amr_mpc_controller

PLUGINLIB_EXPORT_CLASS(amr_mpc_controller::FinalPositionRPP, nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(amr_mpc_controller::FinalPositionPlacementRPP, nav2_core::Controller)

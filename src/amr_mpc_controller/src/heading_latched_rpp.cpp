#include "amr_mpc_controller/heading_latched_rpp.hpp"

#include <cmath>
#include <utility>
#include "nav2_core/exceptions.hpp"
#include "tf2/utils.h"

namespace amr_mpc_controller
{
namespace
{
using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;
constexpr double kStoppedLinear = 0.01;  // m/s measured
constexpr double kStoppedAngular = 0.03;  // rad/s measured

// transformGlobalPlan prunes its input. Inspection must not advance a loop's
// bounded nearest-point search; restore before releasing either inherited lock.
class PlanRestore
{
public:
  explicit PlanRestore(nav_msgs::msg::Path & plan) : plan_(plan), saved_(plan) {}
  ~PlanRestore() {plan_ = std::move(saved_);}
private:
  nav_msgs::msg::Path & plan_;
  nav_msgs::msg::Path saved_;
};

bool finitePose(const geometry_msgs::msg::Pose & p)
{
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
         std::isfinite(p.position.z) && std::isfinite(p.orientation.x) &&
         std::isfinite(p.orientation.y) && std::isfinite(p.orientation.z) &&
         std::isfinite(p.orientation.w);
}
bool finiteSpeed(const geometry_msgs::msg::Twist & v)
{
  return std::isfinite(v.linear.x) && std::isfinite(v.linear.y) &&
         std::isfinite(v.linear.z) && std::isfinite(v.angular.x) &&
         std::isfinite(v.angular.y) && std::isfinite(v.angular.z);
}
}  // namespace

void HeadingLatchedRPP::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  Base::configure(parent, std::move(name), std::move(tf), std::move(costmap_ros));
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::NORMAL;
  auto node = node_.lock();
  // Check requested parameters too: upstream silently disables reversing when
  // both are requested. This plugin has only the forward/rotation contract.
  if (!use_rotate_to_heading_ || allow_reversing_ ||
    node->get_parameter(plugin_name_ + ".allow_reversing").as_bool())
  {
    throw nav2_core::PlannerException("HeadingLatchedRPP requires rotation and forward-only plans");
  }
}

void HeadingLatchedRPP::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Base::setPlan(path);
  phase_ = Phase::NORMAL;
}

void HeadingLatchedRPP::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::NORMAL;
  contract_callback_.reset();
  Base::cleanup();
}

void HeadingLatchedRPP::activate()
{
  Base::activate();
  // Humble rclcpp invokes callbacks in reverse registration order. Reject an
  // unsupported batch before upstream's callback can mutate controller fields.
  contract_callback_ = node_.lock()->add_on_set_parameters_callback(
    [prefix = plugin_name_](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        if ((parameter.get_name() == prefix + ".allow_reversing" && parameter.as_bool()) ||
          (parameter.get_name() == prefix + ".use_rotate_to_heading" && !parameter.as_bool()))
        {
          result.successful = false;
          result.reason = "HeadingLatchedRPP requires rotation and forward-only plans";
          break;
        }
      }
      return result;
    });
}

void HeadingLatchedRPP::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  phase_ = Phase::NORMAL;
  contract_callback_.reset();
  Base::deactivate();
}

geometry_msgs::msg::TwistStamped HeadingLatchedRPP::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & speed, nav2_core::GoalChecker * goal_checker)
{
  std::unique_lock<std::mutex> controller_lock(mutex_);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> costmap_lock(*costmap_->getMutex());
  if (!use_rotate_to_heading_ || allow_reversing_) {
    throw nav2_core::PlannerException("HeadingLatchedRPP requires rotation and forward-only plans");
  }
  if (!finitePose(pose.pose) || !finiteSpeed(speed) || !goal_checker) {
    throw nav2_core::PlannerException("HeadingLatchedRPP received invalid pose, speed or goal checker");
  }
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (!goal_checker->getTolerances(tolerance, velocity_tolerance)) {
    throw nav2_core::PlannerException("HeadingLatchedRPP requires goal tolerances");
  }
  const double yaw_tolerance = std::abs(tf2::getYaw(tolerance.orientation));
  if (!std::isfinite(tolerance.position.x) || tolerance.position.x <= 0.0 ||
    !std::isfinite(yaw_tolerance) || yaw_tolerance <= 0.0)
  {
    throw nav2_core::PlannerException("HeadingLatchedRPP received invalid goal tolerances");
  }
  goal_dist_tol_ = tolerance.position.x;
  double bearing, carrot_distance;
  {
    PlanRestore restore(global_plan_);
    // Upstream ignores a per-point transform failure. Check the same transform
    // chain and timestamp before accepting any transformed carrot.
    geometry_msgs::msg::PoseStamped base_pose;
    if (!transformPose(costmap_ros_->getBaseFrameID(), pose, base_pose)) {
      throw nav2_core::PlannerException("HeadingLatchedRPP unable to transform into robot frame");
    }
    const auto transformed = transformGlobalPlan(pose);
    for (const auto & point : transformed.poses) {
      if (!finitePose(point.pose) || point.header.frame_id != costmap_ros_->getBaseFrameID()) {
        throw nav2_core::PlannerException("HeadingLatchedRPP received invalid transformed plan");
      }
    }
    const auto carrot = getLookAheadPoint(getLookAheadDistance(speed), transformed);
    carrot_distance = std::hypot(carrot.pose.position.x, carrot.pose.position.y);
    bearing = std::atan2(carrot.pose.position.y, carrot.pose.position.x);
    if (!std::isfinite(carrot_distance) || !std::isfinite(bearing)) {
      throw nav2_core::PlannerException("HeadingLatchedRPP received invalid path bearing");
    }
  }
  const auto delegate = [&]() {
      costmap_lock.unlock();
      controller_lock.unlock();
      return Base::computeVelocityCommands(pose, speed, goal_checker);
    };
  // Terminal heading belongs to upstream RPP and the active goal checker.
  if (carrot_distance <= goal_dist_tol_) {
    phase_ = Phase::NORMAL;
    return delegate();
  }
  geometry_msgs::msg::TwistStamped command;
  command.header = pose.header;
  const bool stopped_linear = std::abs(speed.linear.x) <= kStoppedLinear;
  if (phase_ == Phase::NORMAL) {
    if (std::abs(bearing) <= rotate_to_heading_min_angle_) {return delegate();}
    phase_ = Phase::STOPPING;
    // Always send a complete stop on entry, even if the latest speed is zero.
  } else if (phase_ == Phase::STOPPING) {
    if (stopped_linear) {phase_ = Phase::ROTATING;}
  }
  if (phase_ == Phase::ROTATING) {
    if (std::abs(bearing) <= yaw_tolerance) {
      phase_ = Phase::SETTLING;
    } else {
      rotateToHeading(command.twist.linear.x, command.twist.angular.z, bearing, speed);
    }
  }
  if (phase_ == Phase::SETTLING) {
    if (std::abs(bearing) > yaw_tolerance) {
      phase_ = Phase::ROTATING;
      rotateToHeading(command.twist.linear.x, command.twist.angular.z, bearing, speed);
    } else if (stopped_linear && std::abs(speed.angular.z) <= kStoppedAngular) {
      phase_ = Phase::NORMAL;
      return delegate();
    }
  }
  if (use_collision_detection_ && isCollisionImminent(
      pose, command.twist.linear.x, command.twist.angular.z, carrot_distance))
  {
    throw nav2_core::PlannerException("RegulatedPurePursuitController detected collision ahead!");
  }
  return command;
}
}  // namespace amr_mpc_controller

PLUGINLIB_EXPORT_CLASS(amr_mpc_controller::HeadingLatchedRPP, nav2_core::Controller)

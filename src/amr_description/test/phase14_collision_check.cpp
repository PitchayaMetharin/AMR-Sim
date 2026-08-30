#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/collision_detection_fcl/collision_env_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

namespace
{
constexpr double kHalfFootprintX = 0.60;
constexpr double kHalfFootprintY = 0.40;
constexpr double kTolerance = 1e-6;

bool belongsToCompositeAttachment(const std::string& link_name)
{
  return link_name.rfind("arm_", 0) == 0 || link_name.rfind("gripper_", 0) == 0 ||
         link_name == "product_camera_link" || link_name == "stowed_product_link";
}

void includePoint(const Eigen::Vector3d& point, Eigen::Vector2d& lower, Eigen::Vector2d& upper)
{
  lower.x() = std::min(lower.x(), point.x());
  lower.y() = std::min(lower.y(), point.y());
  upper.x() = std::max(upper.x(), point.x());
  upper.y() = std::max(upper.y(), point.y());
}

void includeCenteredExtent(const Eigen::Isometry3d& transform, const Eigen::Vector3d& half_extent,
                           Eigen::Vector2d& lower, Eigen::Vector2d& upper)
{
  const Eigen::Matrix3d absolute_rotation = transform.linear().cwiseAbs();
  const Eigen::Vector3d world_extent = absolute_rotation * half_extent;
  includePoint(transform.translation() - world_extent, lower, upper);
  includePoint(transform.translation() + world_extent, lower, upper);
}

void includeShape(const shapes::Shape& shape, const Eigen::Isometry3d& transform,
                  Eigen::Vector2d& lower, Eigen::Vector2d& upper)
{
  switch (shape.type)
  {
    case shapes::BOX:
    {
      const auto& box = static_cast<const shapes::Box&>(shape);
      includeCenteredExtent(transform, Eigen::Vector3d(box.size[0], box.size[1], box.size[2]) * 0.5,
                            lower, upper);
      return;
    }
    case shapes::SPHERE:
    {
      const auto& sphere = static_cast<const shapes::Sphere&>(shape);
      includeCenteredExtent(transform, Eigen::Vector3d::Constant(sphere.radius), lower, upper);
      return;
    }
    case shapes::CYLINDER:
    {
      const auto& cylinder = static_cast<const shapes::Cylinder&>(shape);
      const Eigen::Matrix3d& rotation = transform.linear();
      Eigen::Vector2d extent;
      for (int row = 0; row < 2; ++row)
      {
        extent[row] = cylinder.radius * std::hypot(rotation(row, 0), rotation(row, 1)) +
                      0.5 * cylinder.length * std::abs(rotation(row, 2));
      }
      includePoint(Eigen::Vector3d(transform.translation().x() - extent.x(),
                                   transform.translation().y() - extent.y(), 0.0), lower, upper);
      includePoint(Eigen::Vector3d(transform.translation().x() + extent.x(),
                                   transform.translation().y() + extent.y(), 0.0), lower, upper);
      return;
    }
    case shapes::MESH:
    {
      const auto& mesh = static_cast<const shapes::Mesh&>(shape);
      for (unsigned int index = 0; index < mesh.vertex_count; ++index)
      {
        includePoint(transform * Eigen::Vector3d(mesh.vertices[3 * index], mesh.vertices[3 * index + 1],
                                                  mesh.vertices[3 * index + 2]),
                     lower, upper);
      }
      return;
    }
    default:
      throw std::runtime_error("unsupported collision shape in footprint check");
  }
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage: phase14_collision_check MODEL.urdf MODEL.srdf\n";
    return 2;
  }

  const urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDFFile(argv[1]);
  if (!urdf_model)
  {
    std::cerr << "failed to parse URDF: " << argv[1] << '\n';
    return 2;
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  if (!srdf_model->initFile(*urdf_model, argv[2]))
  {
    std::cerr << "failed to parse SRDF: " << argv[2] << '\n';
    return 2;
  }

  auto robot_model = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  const std::array<double, 6> stow{ 0.0, -1.5708, 1.5708, 0.0, 0.0, 0.0 };
  for (std::size_t index = 0; index < stow.size(); ++index)
    state.setVariablePosition("arm_joint_" + std::to_string(index + 1), stow[index]);
  state.setVariablePosition("gripper_finger_joint", 0.02);
  state.update();

  collision_detection::CollisionEnvFCL collision_environment(robot_model);
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 1000;
  request.max_contacts_per_pair = 10;
  collision_detection::CollisionResult raw_result;
  collision_environment.checkSelfCollision(request, raw_result, state);
  double mount_penetration = 0.0;
  for (const auto& [pair, contacts] : raw_result.contacts)
  {
    const bool mount_pair = (pair.first == "arm_base_link" && pair.second == "base_link") ||
                            (pair.first == "base_link" && pair.second == "arm_base_link");
    if (mount_pair)
      for (const auto& contact : contacts)
        mount_penetration = std::max(mount_penetration, contact.depth);
  }
  std::cout << "arm mount maximum contact depth: " << mount_penetration << " m\n";

  collision_detection::AllowedCollisionMatrix allowed_collisions(*srdf_model);
  collision_detection::AllowedCollision::Type mount_collision_type;
  const bool mount_collision_allowed =
      allowed_collisions.getEntry("base_link", "arm_base_link", mount_collision_type) &&
      mount_collision_type == collision_detection::AllowedCollision::ALWAYS;
  if (!mount_collision_allowed)
    std::cerr << "SRDF does not allow the base_link <-> arm_base_link collision pair\n";

  if (robot_model->hasLinkModel("stowed_product_link"))
  {
    allowed_collisions.setEntry("gripper_left_finger_link", "stowed_product_link", true);
    allowed_collisions.setEntry("gripper_right_finger_link", "stowed_product_link", true);
  }
  collision_detection::CollisionResult result;
  collision_environment.checkSelfCollision(request, result, state, allowed_collisions);
  if (result.collision)
  {
    std::cerr << "self-collision pairs:\n";
    for (const auto& [pair, contacts] : result.contacts)
      std::cerr << "  " << pair.first << " <-> " << pair.second << " (" << contacts.size() << ")\n";
  }

  Eigen::Vector2d lower = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d upper = Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());
  for (const moveit::core::LinkModel* link : robot_model->getLinkModelsWithCollisionGeometry())
  {
    if (!belongsToCompositeAttachment(link->getName()))
      continue;
    Eigen::Vector2d link_lower = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector2d link_upper = Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());
    const auto& shapes = link->getShapes();
    for (std::size_t index = 0; index < shapes.size(); ++index)
    {
      includeShape(*shapes[index], state.getCollisionBodyTransform(link, index), link_lower, link_upper);
      includeShape(*shapes[index], state.getCollisionBodyTransform(link, index), lower, upper);
    }
    std::cout << "  " << link->getName() << ": [" << link_lower.x() << ", " << link_upper.x()
              << "] x [" << link_lower.y() << ", " << link_upper.y() << "] m\n";
  }
  std::cout << "attachment XY bounds: [" << lower.x() << ", " << upper.x() << "] x [" << lower.y()
            << ", " << upper.y() << "] m\n";
  const bool inside_footprint = lower.x() >= -kHalfFootprintX - kTolerance &&
                                upper.x() <= kHalfFootprintX + kTolerance &&
                                lower.y() >= -kHalfFootprintY - kTolerance &&
                                upper.y() <= kHalfFootprintY + kTolerance;
  if (!inside_footprint)
    std::cerr << "attachment exceeds the 1.20 x 0.80 m chassis footprint\n";

  return result.collision || !inside_footprint || !mount_collision_allowed ? 1 : 0;
}

// Captured-scene clearance replay of the installed PrecisionNavfnPlanner plugin
// and the production-budget (1 s) SimpleSmoother.
//
//   production_precision_replay --scene SCENE.json --output REPORT.json
//   production_precision_replay --self-test
//   production_precision_replay --self-test-footprint   (footprint groups only)
//   production_precision_replay --self-test-smoother-completion   (completion groups only)
//   production_precision_replay --self-test-planner-map   (planner-map groups only)
//
// The scene is native45_clearance_scene_v1. Every admitted case/stage is planned
// through the pluginlib-loaded plugin on the phase-selected captured global grid,
// smoothed for one second, and the raw AND smoothed paths (plus explicit start and
// terminal heading sweeps) are swept with the complete padded polygon (boundary and
// interior cells) against all four captured grids. The first failure stops the replay.
//
// This is geometric clearance evidence only. Settling, fresh bias, controller
// tracking, action execution and terminal-state guarantees stay unproved.

#define main replay_backend_main
#include "planner_replay_backend.cpp"
#undef main

#include <algorithm>
#include <array>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <utility>

#include "amr_interfaces/final_placement_stance.hpp"
#include "amr_navigation/precision_navfn_planner.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace native45
{

// Production mission budget: mission_supervisor_node.cpp goal.max_smoothing_duration.
constexpr int kSmootherBudgetSeconds = 1;
constexpr double kFinalHeadingMarginRad = 0.03;
constexpr double kStanceAgreementTolerance = 1.0e-9;
constexpr double kPaddingTolerance = 1.0e-9;
// Plugin/scene consistency only: Costmap2DROS parses the footprint string as float, so its
// padded polygon differs from the decimal scene polygon by float rounding. This allowance
// admits that difference per X/Y coordinate; it never changes the nominal scene shape, the
// stance gate, the padding, or the collision geometry (see conservative_envelope).
constexpr double kPluginSceneConsistencyAllowance = 0.02;
// Double-subtraction noise only (0.63 - 0.61 > 0.02 in binary); a delta of exactly 0.02 m
// passes, 0.02 m + 1e-9 m does not.
constexpr double kAllowanceBoundaryNoise = 1.0e-12;
constexpr double kPlannerMapGeometryTolerance = 1.0e-9;
constexpr unsigned char kInscribedCost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
constexpr unsigned char kLethalCost = nav2_costmap_2d::LETHAL_OBSTACLE;
constexpr unsigned char kUnknownCost = nav2_costmap_2d::NO_INFORMATION;
constexpr std::size_t kExpectedCaptures = 4;
constexpr std::size_t kExpectedNominal = 4;
constexpr std::size_t kExpectedBias = 16;
constexpr std::size_t kExpectedRepair = 80;
constexpr std::size_t kMaxSamplesPerMotion = 1000000;
constexpr std::size_t kMaxReportedPathPoses = 400;
constexpr std::size_t kMaxReportBytes = 8U * 1024U * 1024U;
constexpr std::uintmax_t kMaxSceneBytes = 64U * 1024U * 1024U;
constexpr const char * kPlannerName = "PrecisionGridBased";
constexpr const char * kPluginClass = "amr_navigation/PrecisionNavfnPlanner";
constexpr const char * kSceneSchema = "native45_clearance_scene_v1";
constexpr const char * kReportSchema = "native45_clearance_report_v1";

// Indices into Scene::grids.
constexpr std::size_t kClearGlobal = 0;
constexpr std::size_t kClearLocal = 1;
constexpr std::size_t kDockGlobal = 2;
constexpr std::size_t kDockLocal = 3;

using Point2 = std::pair<double, double>;
using Polygon = std::vector<Point2>;

struct SceneError : public std::runtime_error
{
  using std::runtime_error::runtime_error;
};

void expect(const bool condition, const std::string & reason)
{
  if (!condition) {
    throw SceneError(reason);
  }
}

double wrap_rad(const double angle)
{
  return std::remainder(angle, 2.0 * M_PI);
}

bool finite_pose(const PoseData & pose)
{
  return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
}

json pose_json(const PoseData & pose)
{
  return json{{"x", pose.x}, {"y", pose.y}, {"yaw", pose.yaw}};
}

// ---------------------------------------------------------------------------
// JSON accessors that name the offending key.
// ---------------------------------------------------------------------------

const json & jmember(const json & parent, const std::string & key, const std::string & where)
{
  expect(parent.is_object(), where + ": not a JSON object");
  const auto found = parent.find(key);
  expect(found != parent.end(), where + ": missing key '" + key + "'");
  return *found;
}

double jnum(const json & parent, const std::string & key, const std::string & where)
{
  const json & value = jmember(parent, key, where);
  expect(value.is_number() && std::isfinite(value.get<double>()),
    where + "." + key + ": not a finite number");
  return value.get<double>();
}

long long jint(const json & parent, const std::string & key, const std::string & where)
{
  const json & value = jmember(parent, key, where);
  expect(value.is_number_integer(), where + "." + key + ": not an integer");
  return value.get<long long>();
}

std::string jstr(const json & parent, const std::string & key, const std::string & where)
{
  const json & value = jmember(parent, key, where);
  expect(value.is_string() && !value.get<std::string>().empty(),
    where + "." + key + ": not a non-empty string");
  return value.get<std::string>();
}

bool jbool(const json & parent, const std::string & key, const std::string & where)
{
  const json & value = jmember(parent, key, where);
  expect(value.is_boolean(), where + "." + key + ": not a boolean");
  return value.get<bool>();
}

std::vector<double> jvector(
  const json & parent, const std::string & key, const std::string & where, const std::size_t count)
{
  const json & value = jmember(parent, key, where);
  expect(value.is_array() && value.size() == count,
    where + "." + key + ": expected an array of " + std::to_string(count) + " numbers");
  std::vector<double> out;
  for (const auto & item : value) {
    expect(item.is_number() && std::isfinite(item.get<double>()),
      where + "." + key + ": element is not a finite number");
    out.push_back(item.get<double>());
  }
  return out;
}

Polygon jpolygon(const json & parent, const std::string & key, const std::string & where)
{
  const json & value = jmember(parent, key, where);
  expect(value.is_array() && value.size() >= 3 && value.size() <= 64,
    where + "." + key + ": not a polygon of 3..64 vertices");
  Polygon out;
  for (const auto & vertex : value) {
    expect(vertex.is_array() && vertex.size() == 2 && vertex[0].is_number() &&
      vertex[1].is_number() && std::isfinite(vertex[0].get<double>()) &&
      std::isfinite(vertex[1].get<double>()),
      where + "." + key + ": vertex is not a finite [x, y]");
    out.emplace_back(vertex[0].get<double>(), vertex[1].get<double>());
  }
  return out;
}

PoseData jpose(const json & value, const std::string & where)
{
  expect(value.is_object(), where + ": pose is not an object");
  return PoseData{jnum(value, "x", where), jnum(value, "y", where), jnum(value, "yaw", where)};
}

// ---------------------------------------------------------------------------
// Captured grids and the strict (complete polygon) collision checker.
// ---------------------------------------------------------------------------

struct GridView
{
  std::string label;  // clear.global, clear.local, pre_dock.global, pre_dock.local
  std::string id;
  std::string frame_id;
  long long stamp_ns = 0;
  unsigned int width = 0;
  unsigned int height = 0;
  double resolution = 0.0;
  double origin_x = 0.0;
  double origin_y = 0.0;
  std::vector<unsigned char> costs;
  std::string data_sha256;
  bool has_transform = false;  // local grids carry map<-odom
  double map_to_odom_x = 0.0;
  double map_to_odom_y = 0.0;
  double map_to_odom_yaw = 0.0;
};

// Map-frame pose -> the grid's own frame. map_to_odom is map<-odom, so it is inverted.
PoseData to_grid_frame(const GridView & grid, const PoseData & map_pose)
{
  if (!grid.has_transform) {
    return map_pose;
  }
  const double c = std::cos(grid.map_to_odom_yaw);
  const double s = std::sin(grid.map_to_odom_yaw);
  const double dx = map_pose.x - grid.map_to_odom_x;
  const double dy = map_pose.y - grid.map_to_odom_y;
  return PoseData{c * dx + s * dy, -s * dx + c * dy, wrap_rad(map_pose.yaw - grid.map_to_odom_yaw)};
}

double circumradius(const Polygon & polygon)
{
  double radius = 0.0;
  for (const auto & vertex : polygon) {
    radius = std::max(radius, std::hypot(vertex.first, vertex.second));
  }
  return radius;
}

struct StrictFailure
{
  std::string reason;  // empty_path, nonfinite_pose, outside_map, center_cost_inscribed_or_worse,
                       // lethal_cell, unknown_cell, bad_polygon, motion_too_long
  std::string role;    // input, center, footprint_vertex, footprint_cell
  std::string grid;
  std::string path_label;
  long long pose_index = -1;
  long long sample_index = -1;
  PoseData map_pose{0.0, 0.0, 0.0};
  PoseData grid_pose{0.0, 0.0, 0.0};
  long long cell_x = -1;
  long long cell_y = -1;
  int cost = -1;
  double vertex_x = std::numeric_limits<double>::quiet_NaN();
  double vertex_y = std::numeric_limits<double>::quiet_NaN();
};

struct StrictStats
{
  std::size_t samples = 0;
  std::size_t cells = 0;
};

struct StrictVerdict
{
  bool ok = true;
  StrictFailure failure;
  StrictStats stats;
};

std::vector<Point2> clip_y(const std::vector<Point2> & input, const double bound, const bool keep_above)
{
  std::vector<Point2> output;
  const auto inside = [&](const Point2 & point) {
      return keep_above ? point.second >= bound : point.second <= bound;
    };
  for (std::size_t index = 0; index < input.size(); ++index) {
    const Point2 & a = input[index];
    const Point2 & b = input[(index + 1) % input.size()];
    const bool a_in = inside(a);
    const bool b_in = inside(b);
    if (a_in) {
      output.push_back(a);
    }
    if (a_in != b_in) {
      const double t = (bound - a.second) / (b.second - a.second);
      output.emplace_back(a.first + t * (b.first - a.first), bound);
    }
  }
  return output;
}

bool fail_sample(
  StrictVerdict & verdict, const GridView & grid, const std::string & path_label,
  const long long pose_index, const long long sample_index, const PoseData & map_pose,
  const PoseData & grid_pose, const std::string & reason, const std::string & role,
  const long long cell_x, const long long cell_y, const int cost)
{
  verdict.ok = false;
  StrictFailure & failure = verdict.failure;
  failure.reason = reason;
  failure.role = role;
  failure.grid = grid.label;
  failure.path_label = path_label;
  failure.pose_index = pose_index;
  failure.sample_index = sample_index;
  failure.map_pose = map_pose;
  failure.grid_pose = grid_pose;
  failure.cell_x = cell_x;
  failure.cell_y = cell_y;
  failure.cost = cost;
  return false;
}

// One pose. The center cell uses the production rule (cost >= 253 rejects); every
// other cell the full polygon touches (boundary and interior) rejects only lethal
// (254) or unknown (255). Polygon vertices outside the grid are a failure, never
// clipped. Cell coverage = cells crossed by the boundary plus cells whose interior
// the polygon covers, found by clipping the polygon to each cell row.
bool check_sample(
  const GridView & grid, const Polygon & polygon, const PoseData & map_pose,
  const std::string & path_label, const long long pose_index, const long long sample_index,
  StrictVerdict & verdict)
{
  ++verdict.stats.samples;
  if (!finite_pose(map_pose)) {
    return fail_sample(verdict, grid, path_label, pose_index, sample_index, map_pose, map_pose,
        "nonfinite_pose", "input", -1, -1, -1);
  }
  const PoseData pose = to_grid_frame(grid, map_pose);
  const double width = static_cast<double>(grid.width);
  const double height = static_cast<double>(grid.height);
  const double center_u = (pose.x - grid.origin_x) / grid.resolution;
  const double center_v = (pose.y - grid.origin_y) / grid.resolution;
  if (!(center_u >= 0.0 && center_v >= 0.0 && center_u < width && center_v < height)) {
    return fail_sample(verdict, grid, path_label, pose_index, sample_index, map_pose, pose,
        "outside_map", "center", -1, -1, -1);
  }
  const auto center_x = static_cast<long long>(std::floor(center_u));
  const auto center_y = static_cast<long long>(std::floor(center_v));
  const int center_cost = grid.costs[static_cast<std::size_t>(center_x) +
      static_cast<std::size_t>(center_y) * grid.width];
  ++verdict.stats.cells;
  if (center_cost >= kInscribedCost) {
    return fail_sample(verdict, grid, path_label, pose_index, sample_index, map_pose, pose,
        "center_cost_inscribed_or_worse", "center", center_x, center_y, center_cost);
  }

  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  std::vector<Point2> cell_space;  // polygon in grid-cell units (cell i spans [i, i + 1])
  cell_space.reserve(polygon.size());
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (const auto & vertex : polygon) {
    const double vx = pose.x + cos_yaw * vertex.first - sin_yaw * vertex.second;
    const double vy = pose.y + sin_yaw * vertex.first + cos_yaw * vertex.second;
    const double u = (vx - grid.origin_x) / grid.resolution;
    const double v = (vy - grid.origin_y) / grid.resolution;
    if (!(u >= 0.0 && v >= 0.0 && u < width && v < height)) {
      fail_sample(verdict, grid, path_label, pose_index, sample_index, map_pose, pose,
        "outside_map", "footprint_vertex", -1, -1, -1);
      verdict.failure.vertex_x = vx;
      verdict.failure.vertex_y = vy;
      return false;
    }
    cell_space.emplace_back(u, v);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  const auto row_first = static_cast<long long>(std::floor(min_v));
  const auto row_last = std::min(static_cast<long long>(std::floor(max_v)),
      static_cast<long long>(grid.height) - 1);
  for (long long row = row_first; row <= row_last; ++row) {
    const auto above = clip_y(cell_space, static_cast<double>(row), true);
    if (above.empty()) {
      continue;
    }
    const auto strip = clip_y(above, static_cast<double>(row + 1), false);
    if (strip.empty()) {
      continue;
    }
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    for (const auto & point : strip) {
      min_u = std::min(min_u, point.first);
      max_u = std::max(max_u, point.first);
    }
    const auto column_first = std::max(0LL, static_cast<long long>(std::floor(min_u)));
    const auto column_last = std::min(static_cast<long long>(std::floor(max_u)),
        static_cast<long long>(grid.width) - 1);
    for (long long column = column_first; column <= column_last; ++column) {
      const int cost = grid.costs[static_cast<std::size_t>(column) +
          static_cast<std::size_t>(row) * grid.width];
      ++verdict.stats.cells;
      if (cost >= kLethalCost) {
        return fail_sample(verdict, grid, path_label, pose_index, sample_index, map_pose, pose,
            cost == kUnknownCost ? "unknown_cell" : "lethal_cell", "footprint_cell", column, row,
            cost);
      }
    }
  }
  return true;
}

// Interpolates from `from` to `to` with spacing <= 0.5 * resolution of translation plus
// corner rotation travel (circumradius * |dyaw|), checking every sample after `from`.
bool check_motion(
  const GridView & grid, const Polygon & polygon, const double radius, const PoseData & from,
  const PoseData & to, const std::string & path_label, const long long pose_index,
  StrictVerdict & verdict)
{
  if (!finite_pose(from) || !finite_pose(to)) {
    return fail_sample(verdict, grid, path_label, pose_index, 0, to, to, "nonfinite_pose",
        "input", -1, -1, -1);
  }
  const double dyaw = wrap_rad(to.yaw - from.yaw);
  const double travel = std::hypot(to.x - from.x, to.y - from.y) + radius * std::abs(dyaw);
  const double spacing = kFootprintSampleSpacingFraction * grid.resolution;
  const double wanted = std::max(1.0, std::ceil(travel / spacing));
  if (!std::isfinite(wanted) || wanted > static_cast<double>(kMaxSamplesPerMotion)) {
    return fail_sample(verdict, grid, path_label, pose_index, 0, to, to, "motion_too_long",
        "input", -1, -1, -1);
  }
  const auto steps = static_cast<std::size_t>(wanted);
  for (std::size_t step = 1; step <= steps; ++step) {
    const double fraction = static_cast<double>(step) / static_cast<double>(steps);
    const PoseData sample{
      from.x + fraction * (to.x - from.x), from.y + fraction * (to.y - from.y),
      wrap_rad(from.yaw + fraction * dyaw)};
    if (!check_sample(grid, polygon, sample, path_label, pose_index,
      static_cast<long long>(step), verdict))
    {
      return false;
    }
  }
  return true;
}

StrictVerdict input_failure(
  const GridView & grid, const std::string & path_label, const std::string & reason)
{
  StrictVerdict verdict;
  verdict.ok = false;
  verdict.failure.reason = reason;
  verdict.failure.role = "input";
  verdict.failure.grid = grid.label;
  verdict.failure.path_label = path_label;
  return verdict;
}

StrictVerdict strict_check_path(
  const GridView & grid, const Polygon & polygon, const std::vector<PoseData> & path,
  const std::string & path_label)
{
  const double radius = circumradius(polygon);
  if (polygon.size() < 3 || !(radius > 0.0) || !std::isfinite(radius)) {
    return input_failure(grid, path_label, "bad_polygon");
  }
  if (path.empty()) {
    return input_failure(grid, path_label, "empty_path");
  }
  StrictVerdict verdict;
  if (!check_sample(grid, polygon, path.front(), path_label, 0, 0, verdict)) {
    return verdict;
  }
  for (std::size_t index = 1; index < path.size(); ++index) {
    if (!check_motion(grid, polygon, radius, path[index - 1], path[index], path_label,
      static_cast<long long>(index), verdict))
    {
      return verdict;
    }
  }
  return verdict;
}

// Pure rotation about (x, y). Mandatory for start and terminal headings even when the
// plugin path emits no intermediate heading motion.
StrictVerdict strict_check_heading_sweep(
  const GridView & grid, const Polygon & polygon, const double x, const double y,
  const double yaw_from, const double yaw_to, const std::string & path_label)
{
  const double radius = circumradius(polygon);
  if (polygon.size() < 3 || !(radius > 0.0) || !std::isfinite(radius)) {
    return input_failure(grid, path_label, "bad_polygon");
  }
  StrictVerdict verdict;
  const PoseData from{x, y, yaw_from};
  const PoseData to{x, y, yaw_to};
  if (!finite_pose(from) || !finite_pose(to)) {
    return input_failure(grid, path_label, "nonfinite_pose");
  }
  if (!check_sample(grid, polygon, from, path_label, 0, 0, verdict)) {
    return verdict;
  }
  check_motion(grid, polygon, radius, from, to, path_label, 1, verdict);
  return verdict;
}

// ---------------------------------------------------------------------------
// Plugin/scene footprint consistency and the conservative strict polygon.
// ---------------------------------------------------------------------------

struct FootprintComparison
{
  Polygon actual;        // Costmap2DROS padded footprint (the plugin's real input)
  Polygon signed_delta;  // actual - nominal, per vertex
  double max_abs_delta = 0.0;
};

std::string metres(const double value)
{
  std::ostringstream text;
  text << std::setprecision(17) << value << " m";
  return text.str();
}

// Compares the actual padded polygon to the nominal scene polygon vertex by vertex. Every
// X/Y coordinate must be finite and within `allowance` (inclusive) of its nominal value.
// The actual polygon must keep the nominal vertex count and order and be an axis-aligned
// rectangle. Throws SceneError naming phase, vertex, coordinate, actual, expected, signed
// delta and allowance.
FootprintComparison compare_plugin_footprint(
  const std::string & phase, const Polygon & actual, const Polygon & nominal, const double allowance)
{
  const std::string head =
    "plugin padded footprint disagrees with the scene padded footprint: phase " + phase;
  expect(actual.size() == nominal.size() && actual.size() == 4,
    head + ": vertex count " + std::to_string(actual.size()) + " differs from the scene's " +
    std::to_string(nominal.size()) + " (four-corner rectangle required)");
  for (std::size_t index = 0; index < actual.size(); ++index) {
    expect(std::isfinite(actual[index].first) && std::isfinite(actual[index].second),
      head + ": vertex " + std::to_string(index) + " is not finite");
  }
  expect(std::abs(actual[0].first - actual[1].first) <= kPaddingTolerance &&
    std::abs(actual[2].first - actual[3].first) <= kPaddingTolerance &&
    std::abs(actual[0].second - actual[3].second) <= kPaddingTolerance &&
    std::abs(actual[1].second - actual[2].second) <= kPaddingTolerance &&
    actual[0].first > actual[2].first && actual[0].second > actual[1].second,
    head + ": polygon is not the axis-aligned (+x,+y),(+x,-y),(-x,-y),(-x,+y) rectangle");
  FootprintComparison result;
  result.actual = actual;
  for (std::size_t index = 0; index < actual.size(); ++index) {
    const double delta[2] = {
      actual[index].first - nominal[index].first, actual[index].second - nominal[index].second};
    const double got[2] = {actual[index].first, actual[index].second};
    const double want[2] = {nominal[index].first, nominal[index].second};
    for (int axis = 0; axis < 2; ++axis) {
      expect(std::abs(delta[axis]) <= allowance + kAllowanceBoundaryNoise,
        head + " vertex " + std::to_string(index) + " coordinate " + (axis == 0 ? "x" : "y") +
        ": actual " + metres(got[axis]) + ", expected " + metres(want[axis]) + ", signed delta " +
        metres(delta[axis]) + " exceeds the +/-" + metres(allowance) + " allowance");
      result.max_abs_delta = std::max(result.max_abs_delta, std::abs(delta[axis]));
    }
    result.signed_delta.emplace_back(delta[0], delta[1]);
  }
  return result;
}

bool polygon_encloses(const Polygon & outer, const Polygon & inner)
{
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = max_x;
  for (const auto & vertex : outer) {
    min_x = std::min(min_x, vertex.first);
    max_x = std::max(max_x, vertex.first);
    min_y = std::min(min_y, vertex.second);
    max_y = std::max(max_y, vertex.second);
  }
  for (const auto & vertex : inner) {
    if (!(vertex.first >= min_x && vertex.first <= max_x && vertex.second >= min_y &&
      vertex.second <= max_y))
    {
      return false;
    }
  }
  return true;
}

// Smallest axis-aligned rectangle enclosing every vertex of every input polygon, in the
// production vertex order (+maxX,+maxY),(+maxX,minY),(minX,minY),(minX,+maxY). Bounds are
// never shrunk and the consistency allowance is not subtracted: this is the only polygon
// the independent strict collision checks use.
Polygon conservative_envelope(const std::vector<Polygon> & polygons)
{
  expect(!polygons.empty(), "conservative footprint envelope has no input polygon");
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = max_x;
  for (const auto & polygon : polygons) {
    for (const auto & vertex : polygon) {
      expect(std::isfinite(vertex.first) && std::isfinite(vertex.second),
        "conservative footprint envelope input vertex is not finite");
      min_x = std::min(min_x, vertex.first);
      max_x = std::max(max_x, vertex.first);
      min_y = std::min(min_y, vertex.second);
      max_y = std::max(max_y, vertex.second);
    }
  }
  expect(std::isfinite(min_x + max_x + min_y + max_y) && max_x > min_x && max_y > min_y,
    "conservative footprint envelope is degenerate");
  const Polygon envelope{{max_x, max_y}, {max_x, min_y}, {min_x, min_y}, {min_x, max_y}};
  for (const auto & polygon : polygons) {
    expect(polygon_encloses(envelope, polygon),
      "conservative footprint envelope does not enclose an input polygon");
  }
  return envelope;
}

// ---------------------------------------------------------------------------
// Scene model and exact schema / roster validation.
// ---------------------------------------------------------------------------

struct StageData
{
  std::string name;
  std::string phase;
  std::string provenance;
  PoseData start{0.0, 0.0, 0.0};
  PoseData goal{0.0, 0.0, 0.0};
};

struct CaseData
{
  std::string id;
  std::string kind;
  std::string provenance;
  std::string admission_reason;
  bool synthetic = false;
  bool admitted = false;
  PoseData bias{0.0, 0.0, 0.0};
  std::vector<StageData> stages;
};

struct Scene
{
  Polygon padded_polygon;
  std::string footprint_string;
  double footprint_padding = 0.0;
  double costmap_resolution = 0.0;
  double planner_tolerance = 0.0;
  bool planner_allow_unknown = false;
  bool planner_use_astar = false;
  std::array<GridView, 4> grids;
  std::array<double, 3> physical_stance{0.0, 0.0, 0.0};
  bool center_slot = false;
  std::vector<CaseData> cases;
  std::size_t excluded_repairs = 0;
  json runtime_properties;
  json limitations;
  json identity;
  json extraction;
  json source_constants;
  json config_summary;
  json registry;
};

bool matches_current_padded_shape(const Polygon & polygon)
{
  static const Polygon expected{{0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}, {-0.61, 0.41}};
  if (polygon.size() != expected.size()) {
    return false;
  }
  for (std::size_t index = 0; index < expected.size(); ++index) {
    if (std::abs(polygon[index].first - expected[index].first) > kPaddingTolerance ||
      std::abs(polygon[index].second - expected[index].second) > kPaddingTolerance)
    {
      return false;
    }
  }
  return true;
}

GridView parse_grid(
  const json & value, const std::string & label, const std::string & frame, const bool local,
  const double config_resolution)
{
  const std::string where = "scenes." + label;
  expect(value.is_object(), where + ": not a JSON object");
  GridView grid;
  grid.label = label;
  grid.id = jstr(value, "id", where);
  grid.frame_id = jstr(value, "frame_id", where);
  expect(grid.frame_id == frame,
    where + ": wrong frame_id '" + grid.frame_id + "' (expected '" + frame + "')");
  const long long width = jint(value, "width", where);
  const long long height = jint(value, "height", where);
  expect(width > 0 && height > 0 && width <= 20000 && height <= 20000,
    where + ": malformed grid size");
  grid.width = static_cast<unsigned int>(width);
  grid.height = static_cast<unsigned int>(height);
  grid.resolution = jnum(value, "resolution", where);
  expect(grid.resolution > 0.0 && std::abs(grid.resolution - config_resolution) <= 1.0e-6,
    where + ": resolution disagrees with config.global_costmap_resolution");
  grid.origin_x = jnum(value, "origin_x", where);
  grid.origin_y = jnum(value, "origin_y", where);
  grid.stamp_ns = jint(value, "stamp_ns", where);
  expect(grid.stamp_ns > 0, where + ": invalid stamp_ns");
  const json & data = jmember(value, "data", where);
  expect(data.is_array() && data.size() == static_cast<std::size_t>(width * height),
    where + ": raw cost length does not match geometry");
  grid.costs.reserve(data.size());
  for (const auto & item : data) {
    expect(item.is_number_integer(), where + ": raw cost is not an integer");
    const long long cost = item.get<long long>();
    expect(cost >= 0 && cost <= 255, where + ": raw cost outside 0..255");
    grid.costs.push_back(static_cast<unsigned char>(cost));
  }
  grid.data_sha256 = jstr(value, "data_sha256", where);
  expect(grid.data_sha256.size() == 64, where + ": data_sha256 is not 64 hex characters");
  const json & transform = jmember(value, "map_to_odom", where);
  if (local) {
    expect(transform.is_object(), where + ": local grid needs a map_to_odom transform");
    grid.has_transform = true;
    grid.map_to_odom_x = jnum(transform, "x", where + ".map_to_odom");
    grid.map_to_odom_y = jnum(transform, "y", where + ".map_to_odom");
    grid.map_to_odom_yaw = jnum(transform, "yaw", where + ".map_to_odom");
    expect(jint(transform, "stamp_ns", where + ".map_to_odom") == grid.stamp_ns,
      where + ": map_to_odom stamp is not the grid acquisition stamp");
  } else {
    expect(transform.is_null(), where + ": global grid must not carry map_to_odom");
  }
  return grid;
}

std::vector<std::string> stage_names(const bool repair, const bool with_clear_motion)
{
  const std::string suffix = repair ? "_repair" : "";
  std::vector<std::string> names;
  if (with_clear_motion) {
    names.push_back("tangent_heading" + suffix);
    names.push_back("clear_translation" + suffix);
  }
  names.push_back("arrival_heading" + suffix);
  names.push_back("dock");
  names.push_back("final_heading_margin");
  return names;
}

bool allowed_repair_exclusion(const std::string & reason)
{
  static const std::set<std::string> exact{
    "registered_approach_distance_exceeded", "repair_start_within_clear_tolerance",
    "repair_did_not_converge", "clear_translation_bound_exceeded"};
  return exact.count(reason) > 0 ||
         reason.rfind("clear_reference_displacement_exceeded:", 0) == 0;
}

bool near_pose(const PoseData & a, const PoseData & b, const double tolerance)
{
  return std::hypot(a.x - b.x, a.y - b.y) <= tolerance &&
         std::abs(wrap_rad(a.yaw - b.yaw)) <= tolerance;
}

std::vector<StageData> parse_stages(
  const json & case_json, const CaseData & parsed, const Scene & scene, const std::string & where)
{
  const json & stages_json = jmember(case_json, "stages", where);
  expect(stages_json.is_array(), where + ".stages: not an array");
  const bool repair = parsed.kind == "synthetic_repair";
  const bool full = stages_json.size() == stage_names(repair, true).size();
  const auto names = stage_names(repair, full);
  expect(stages_json.size() == names.size(), where + ": stage names do not form a valid sequence");
  std::vector<StageData> stages;
  for (std::size_t index = 0; index < names.size(); ++index) {
    const std::string at = where + ".stages[" + std::to_string(index) + "]";
    const json & item = stages_json[index];
    StageData stage;
    stage.name = jstr(item, "name", at);
    expect(stage.name == names[index], where + ": stage names do not match the expected sequence");
    stage.phase = jstr(item, "phase", at);
    const bool dock_phase = stage.name == "dock" || stage.name == "final_heading_margin";
    expect(stage.phase == (dock_phase ? "dock" : "clear"), at + ": wrong phase '" + stage.phase + "'");
    stage.provenance = jstr(item, "provenance", at);
    stage.start = jpose(jmember(item, "start", at), at + ".start");
    stage.goal = jpose(jmember(item, "goal", at), at + ".goal");
    const bool heading_only = stage.name != "dock" && stage.name != "clear_translation" &&
      stage.name != "clear_translation_repair";
    if (heading_only) {
      expect(std::hypot(stage.goal.x - stage.start.x, stage.goal.y - stage.start.y) <= 1.0e-9,
        at + ": heading stage must not translate");
    }
    stages.push_back(stage);
  }
  // Source-backed placement geometry relative to the case bias.
  const auto & stance = scene.physical_stance;
  const PoseData dock_goal{
    stance[0] - parsed.bias.x, stance[1] - parsed.bias.y, wrap_rad(stance[2] - parsed.bias.yaw)};
  const StageData & dock = stages[stages.size() - 2];
  const StageData & final_stage = stages.back();
  expect(near_pose(dock.goal, dock_goal, 1.0e-9),
    where + ": dock goal is not the bias-corrected stance");
  expect(std::hypot(final_stage.goal.x - dock_goal.x, final_stage.goal.y - dock_goal.y) <= 1.0e-9 &&
    std::abs(wrap_rad(final_stage.goal.yaw -
    (stance[2] - parsed.bias.yaw - kFinalHeadingMarginRad))) <= 1.0e-9,
    where + ": final heading goal is not stance_yaw - bias_yaw - kFinalHeadingGoalMargin");
  for (std::size_t index = 1; index < stages.size(); ++index) {
    expect(near_pose(stages[index - 1].goal, stages[index].start, 1.0e-9),
      where + ": stage continuity broken between '" + stages[index - 1].name + "' and '" +
      stages[index].name + "'");
  }
  return stages;
}

Scene validate_scene_inner(const json & doc)
{
  expect(doc.is_object(), "scene: not a JSON object");
  expect(jstr(doc, "schema", "scene") == kSceneSchema, "scene: unsupported schema");
  Scene scene;

  const json & config = jmember(doc, "config", "scene");
  scene.costmap_resolution = jnum(config, "global_costmap_resolution", "config");
  expect(scene.costmap_resolution > 0.0, "config: global_costmap_resolution must be positive");
  scene.footprint_string = jstr(config, "footprint_string", "config");
  scene.footprint_padding = jnum(config, "footprint_padding", "config");
  expect(std::abs(scene.footprint_padding - 0.01) <= 1.0e-12,
    "config: footprint_padding is not the current 0.01 m");
  const Polygon footprint = jpolygon(config, "footprint", "config");
  scene.padded_polygon = jpolygon(config, "padded_footprint", "config");
  expect(footprint.size() == scene.padded_polygon.size(), "config: padded footprint vertex count differs");
  for (std::size_t index = 0; index < footprint.size(); ++index) {
    const double pad_x = footprint[index].first >= 0.0 ? scene.footprint_padding : -scene.footprint_padding;
    const double pad_y = footprint[index].second >= 0.0 ? scene.footprint_padding : -scene.footprint_padding;
    expect(std::abs(scene.padded_polygon[index].first - (footprint[index].first + pad_x)) <= kPaddingTolerance &&
      std::abs(scene.padded_polygon[index].second - (footprint[index].second + pad_y)) <= kPaddingTolerance,
      "config: padded footprint is not the footprint plus footprint_padding");
  }
  expect(matches_current_padded_shape(scene.padded_polygon),
    "config: padded footprint is not the current +/-0.61 x +/-0.41 m rectangle");

  const json & precision = jmember(config, "precision_grid_based", "config");
  expect(jstr(precision, "plugin", "config.precision_grid_based") == kPluginClass,
    "config.precision_grid_based: unexpected plugin");
  scene.planner_tolerance = jnum(precision, "tolerance", "config.precision_grid_based");
  expect(scene.planner_tolerance > 0.0, "config.precision_grid_based: tolerance must be positive");
  scene.planner_allow_unknown = jbool(precision, "allow_unknown", "config.precision_grid_based");
  expect(!scene.planner_allow_unknown, "config.precision_grid_based: allow_unknown must be false");
  scene.planner_use_astar = jbool(precision, "use_astar", "config.precision_grid_based");

  // The backend smoother helper declares these exact values; refuse any other config.
  const json & smoother = jmember(config, "simple_smoother", "config");
  expect(jnum(smoother, "tolerance", "config.simple_smoother") == 1.0e-10 &&
    jint(smoother, "max_its", "config.simple_smoother") == 1000 &&
    jnum(smoother, "w_data", "config.simple_smoother") == 0.2 &&
    jnum(smoother, "w_smooth", "config.simple_smoother") == 0.0 &&
    jbool(smoother, "do_refinement", "config.simple_smoother"),
    "config.simple_smoother: parameters differ from the unchanged SimpleSmoother configuration");
  expect(jnum(config, "smoother_budget_s", "config") == static_cast<double>(kSmootherBudgetSeconds),
    "config: smoother_budget_s is not the production 1 s budget");
  scene.config_summary = json{
    {"padded_footprint", jmember(config, "padded_footprint", "config")},
    {"footprint_padding", scene.footprint_padding},
    {"footprint_string", scene.footprint_string},
    {"simple_smoother", smoother},
    {"smoother_budget_s", kSmootherBudgetSeconds},
    {"smoother_budget_source", config.value("smoother_budget_source", std::string())},
    {"precision_grid_based", precision}};

  const json & registry = jmember(doc, "registry", "scene");
  scene.registry = registry;
  const long long product_id = jint(registry, "product_id", "registry");
  const auto dock = jvector(registry, "dispatch_dock", "registry", 3);
  const auto slot = jvector(registry, "selected_slot", "registry", 3);
  const auto stance = jvector(registry, "physical_stance", "registry", 3);
  jvector(registry, "dispatch_approach", "registry", 3);
  jvector(registry, "clear_point", "registry", 2);
  try {
    const auto computed = amr_interfaces::placement::final_placement_stance(
      static_cast<int>(product_id), {dock[0], dock[1], dock[2]}, {slot[0], slot[1], slot[2]});
    expect(std::abs(computed.physical[0] - stance[0]) <= kStanceAgreementTolerance &&
      std::abs(computed.physical[1] - stance[1]) <= kStanceAgreementTolerance &&
      std::abs(wrap_rad(computed.physical[2] - stance[2])) <= kStanceAgreementTolerance,
      "registry: physical_stance disagrees with final_placement_stance()");
    scene.center_slot = computed.product102_center_slot;
  } catch (const SceneError &) {
    throw;
  } catch (const std::exception & error) {
    throw SceneError(std::string("registry: final_placement_stance() rejected inputs: ") + error.what());
  }
  scene.physical_stance = {stance[0], stance[1], stance[2]};

  const json & constants = jmember(doc, "source_constants", "scene");
  scene.source_constants = constants;
  expect(std::abs(jnum(constants, "kFinalHeadingGoalMargin", "source_constants") -
    kFinalHeadingMarginRad) <= 1.0e-12, "source_constants: kFinalHeadingGoalMargin is not 0.03 rad");
  expect(std::abs(jnum(constants, "kDesiredProduct102SlotBaseX", "source_constants") -
    amr_interfaces::placement::kDesiredProduct102SlotBaseX) <= 1.0e-12 &&
    std::abs(jnum(constants, "kDesiredProduct102SlotBaseY", "source_constants") -
    amr_interfaces::placement::kDesiredProduct102SlotBaseY) <= 1.0e-12,
    "source_constants: product 102 slot base disagrees with the shared helper");

  const json & runtime = jmember(doc, "runtime_properties", "scene");
  expect(runtime.is_object() && runtime.size() >= 6, "scene: runtime_properties incomplete");
  for (const auto & key : {"action_execution", "controller_tracking", "fresh_post_heading_bias",
      "observer_settling", "stationary_start", "terminal_state_guarantees"})
  {
    expect(jstr(runtime, key, "runtime_properties") == "not_exercised",
      std::string("runtime_properties: ") + key + " must stay not_exercised");
  }
  for (const auto & item : runtime.items()) {
    expect(item.value().is_string() && item.value().get<std::string>() == "not_exercised",
      "runtime_properties: " + item.key() + " must stay not_exercised");
  }
  scene.runtime_properties = runtime;
  scene.limitations = jmember(doc, "limitations", "scene");
  expect(scene.limitations.is_array(), "scene: limitations is not an array");
  scene.identity = doc.value("identity", json::object());
  scene.extraction = doc.value("extraction", json::object());

  const json & phases = jmember(doc, "planning_grid_by_phase", "scene");
  expect(jstr(phases, "clear", "planning_grid_by_phase") == "clear.global" &&
    jstr(phases, "dock", "planning_grid_by_phase") == "pre_dock.global",
    "planning_grid_by_phase: planning_grid mapping is not clear->clear.global, dock->pre_dock.global");

  const json & scenes = jmember(doc, "scenes", "scene");
  std::vector<std::string> clear_ids;
  std::vector<std::string> dock_ids;
  const std::array<std::pair<const char *, std::size_t>, 2> scene_names{
    {{"clear", kClearGlobal}, {"pre_dock", kDockGlobal}}};
  for (const auto & entry : scene_names) {
    const std::string name = entry.first;
    const json & item = jmember(scenes, name, "scenes");
    const json & grids = jmember(item, "grids", "scenes." + name);
    scene.grids[entry.second] = parse_grid(
      jmember(grids, "global", "scenes." + name + ".grids"), name + ".global", "map", false,
      scene.costmap_resolution);
    scene.grids[entry.second + 1] = parse_grid(
      jmember(grids, "local", "scenes." + name + ".grids"), name + ".local", "odom", true,
      scene.costmap_resolution);
    const json & captures = jmember(item, "captures", "scenes." + name);
    expect(captures.is_array() && captures.size() == kExpectedCaptures,
      "scenes." + name + ": expected exactly 4 captures");
    for (const auto & capture : captures) {
      (name == "clear" ? clear_ids : dock_ids).push_back(
        jstr(capture, "id", "scenes." + name + ".captures"));
    }
  }

  // Exact roster: every clear capture x (own bias + each pre-dock bias) x
  // (nominal/bias case + four +/-X/Y synthetic repair misses).
  const std::vector<std::string> variants{"", "|repair_miss=+x", "|repair_miss=-x",
    "|repair_miss=+y", "|repair_miss=-y"};
  std::map<std::string, std::string> expected_kind;
  for (const auto & clear_id : clear_ids) {
    std::vector<std::pair<std::string, std::string>> labels{{"own", "nominal"}};
    for (const auto & dock_id : dock_ids) {
      labels.emplace_back(dock_id, "bias_sensitivity");
    }
    for (const auto & label : labels) {
      for (const auto & variant : variants) {
        expected_kind[clear_id + "|bias=" + label.first + variant] =
          variant.empty() ? label.second : "synthetic_repair";
      }
    }
  }
  const json & cases = jmember(doc, "cases", "scene");
  expect(cases.is_array() && cases.size() == expected_kind.size() &&
    expected_kind.size() == kExpectedNominal + kExpectedBias + kExpectedRepair,
    "scene: case roster is not the expected 4 nominal + 16 bias_sensitivity + 80 synthetic_repair");
  std::set<std::string> seen;
  std::map<std::string, std::size_t> kind_counts;
  for (std::size_t index = 0; index < cases.size(); ++index) {
    const std::string where = "cases[" + std::to_string(index) + "]";
    const json & item = cases[index];
    CaseData parsed;
    parsed.id = jstr(item, "id", where);
    expect(seen.insert(parsed.id).second, where + ": duplicate case id " + parsed.id);
    const auto expected = expected_kind.find(parsed.id);
    expect(expected != expected_kind.end(), where + ": case id is not in the expected roster: " + parsed.id);
    parsed.kind = jstr(item, "kind", where);
    expect(parsed.kind == expected->second,
      where + ": roster kind '" + parsed.kind + "' is not the expected '" + expected->second + "'");
    parsed.synthetic = jbool(item, "synthetic", where);
    expect(parsed.synthetic == (parsed.kind == "synthetic_repair"), where + ": synthetic flag disagrees with kind");
    parsed.provenance = jstr(item, "provenance", where);
    parsed.admitted = jbool(item, "admitted", where);
    parsed.admission_reason = jstr(item, "admission_reason", where);
    parsed.bias = jpose(jmember(item, "bias", where), where + ".bias");
    ++kind_counts[parsed.kind];
    if (!parsed.admitted) {
      expect(parsed.kind == "synthetic_repair",
        where + ": mandatory " + parsed.kind + " case was excluded (" + parsed.admission_reason + ")");
      expect(allowed_repair_exclusion(parsed.admission_reason),
        where + ": repair exclusion lacks an exact admission reason: " + parsed.admission_reason);
      const json & stages = jmember(item, "stages", where);
      expect(stages.is_array() && stages.empty(), where + ": excluded case must carry no stages");
      ++scene.excluded_repairs;
    } else {
      expect(parsed.admission_reason == "admitted", where + ": admitted case has a different admission reason");
      parsed.stages = parse_stages(item, parsed, scene, where);
    }
    scene.cases.push_back(std::move(parsed));
  }
  expect(kind_counts["nominal"] == kExpectedNominal && kind_counts["bias_sensitivity"] == kExpectedBias &&
    kind_counts["synthetic_repair"] == kExpectedRepair, "scene: case kind roster is not 4/16/80");
  return scene;
}

Scene validate_scene(const json & doc)
{
  try {
    return validate_scene_inner(doc);
  } catch (const json::exception & error) {
    throw SceneError(std::string("scene: malformed JSON structure: ") + error.what());
  }
}

json load_json_file(const std::string & path)
{
  std::error_code code;
  const auto size = std::filesystem::file_size(path, code);
  expect(!code, "scene file cannot be read: " + path);
  expect(size <= kMaxSceneBytes, "scene file exceeds the 64 MB input bound");
  std::ifstream input(path);
  expect(static_cast<bool>(input), "scene file cannot be opened: " + path);
  try {
    return json::parse(input);
  } catch (const json::exception & error) {
    throw SceneError(std::string("scene file is not valid JSON: ") + error.what());
  }
}

// ---------------------------------------------------------------------------
// Production plugin instance (pluginlib + Costmap2DROS fixture).
// ---------------------------------------------------------------------------

struct PlanningFixture
{
  std::string label;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> host;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
  // The loader must outlive the plugin instance created from it.
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> loader;
  std::shared_ptr<nav2_core::GlobalPlanner> plugin;
  ReplayData data{};
  nav2_costmap_2d::Footprint padded_footprint;
  FootprintComparison footprint;  // verified plugin vs scene padded polygon
  std::string library_path;
  bool costmap_configured = false;
  bool plugin_active = false;

  PlanningFixture() = default;
  PlanningFixture(const PlanningFixture &) = delete;
  PlanningFixture & operator=(const PlanningFixture &) = delete;
  ~PlanningFixture() {shutdown();}

  void shutdown() noexcept
  {
    try {
      if (plugin && plugin_active) {
        plugin->deactivate();
        plugin->cleanup();
      }
      plugin_active = false;
      plugin.reset();
      if (costmap_ros && costmap_configured) {
        costmap_ros->on_cleanup(rclcpp_lifecycle::State());
      }
      costmap_configured = false;
    } catch (...) {
    }
  }
};

Polygon to_polygon(const nav2_costmap_2d::Footprint & footprint)
{
  Polygon polygon;
  for (const auto & point : footprint) {
    polygon.emplace_back(point.x, point.y);
  }
  return polygon;
}

json polygon_json(const Polygon & polygon)
{
  json out = json::array();
  for (const auto & vertex : polygon) {
    out.push_back(json::array({vertex.first, vertex.second}));
  }
  return out;
}

json phase_footprint_json(const FootprintComparison & comparison)
{
  return json{
    {"actual_padded_polygon", polygon_json(comparison.actual)},
    {"signed_vertex_deltas_m", polygon_json(comparison.signed_delta)},
    {"max_abs_coordinate_delta_m", comparison.max_abs_delta}};
}

// Additive report block. The allowance is a consistency allowance only: the plugin/scene
// footprints are NOT claimed equal, and the strict polygon grants no collision slack.
json footprint_evidence_json(
  const Polygon & nominal, const FootprintComparison & clear, const FootprintComparison & dock,
  const Polygon & strict)
{
  return json{
    {"nominal_padded_polygon", polygon_json(nominal)},
    {"plugin_scene_consistency_allowance_m", kPluginSceneConsistencyAllowance},
    {"allowance_boundary_noise_m", kAllowanceBoundaryNoise},
    {"phases", json{{"clear", phase_footprint_json(clear)}, {"dock", phase_footprint_json(dock)}}},
    {"conservative_strict_polygon", polygon_json(strict)},
    {"strict_polygon_rule",
      "axis-aligned bounding rectangle of the nominal, clear and dock actual padded polygons; "
      "the consistency allowance is not subtracted"}};
}

std::unique_ptr<PlanningFixture> make_fixture(
  const Scene & scene, const std::size_t grid_index, const std::string & tag)
{
  const GridView & grid = scene.grids[grid_index];
  auto fixture = std::make_unique<PlanningFixture>();
  fixture->label = grid.label;
  const double width_m = static_cast<double>(grid.width) * scene.costmap_resolution;
  const double height_m = static_cast<double>(grid.height) * scene.costmap_resolution;
  const long long width_whole = std::llround(width_m);
  const long long height_whole = std::llround(height_m);
  expect(std::abs(width_m - static_cast<double>(width_whole)) <= 1.0e-3 &&
    std::abs(height_m - static_cast<double>(height_whole)) <= 1.0e-3,
    grid.label + ": grid is not a whole number of metres, which the Costmap2DROS fixture requires");

  fixture->host = std::make_shared<rclcpp_lifecycle::LifecycleNode>("native45_" + tag + "_planner");
  fixture->tf = std::make_shared<tf2_ros::Buffer>(fixture->host->get_clock());
  fixture->costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("native45_" + tag + "_costmap");
  const auto results = fixture->costmap_ros->set_parameters({
      rclcpp::Parameter("global_frame", "map"),
      rclcpp::Parameter("robot_base_frame", "base_footprint"),
      rclcpp::Parameter("plugins", std::vector<std::string>{}),
      rclcpp::Parameter("width", static_cast<int>(width_whole)),
      rclcpp::Parameter("height", static_cast<int>(height_whole)),
      rclcpp::Parameter("origin_x", grid.origin_x),
      rclcpp::Parameter("origin_y", grid.origin_y),
      rclcpp::Parameter("resolution", scene.costmap_resolution),
      rclcpp::Parameter("footprint", scene.footprint_string),
      rclcpp::Parameter("footprint_padding", scene.footprint_padding)});
  for (const auto & result : results) {
    expect(result.successful, "Costmap2DROS rejected a fixture parameter: " + result.reason);
  }
  expect(fixture->costmap_ros->on_configure(rclcpp_lifecycle::State()) ==
    nav2_util::CallbackReturn::SUCCESS, "Costmap2DROS fixture configuration failed");
  fixture->costmap_configured = true;

  nav2_costmap_2d::Costmap2D * map = fixture->costmap_ros->getCostmap();
  expect(map->getSizeInCellsX() == grid.width && map->getSizeInCellsY() == grid.height &&
    std::abs(map->getOriginX() - grid.origin_x) <= 1.0e-9 &&
    std::abs(map->getOriginY() - grid.origin_y) <= 1.0e-9,
    grid.label + ": Costmap2DROS fixture geometry disagrees with the captured grid");
  for (unsigned int y = 0; y < grid.height; ++y) {
    for (unsigned int x = 0; x < grid.width; ++x) {
      map->setCost(x, y, grid.costs[x + static_cast<std::size_t>(y) * grid.width]);
    }
  }

  // The plugin plans with the Costmap2DROS padded footprint (float-parsed, then padded). It
  // must agree with the scene's nominal polygon within the consistency allowance; the
  // plugin keeps the actual polygon unchanged.
  fixture->padded_footprint = fixture->costmap_ros->getRobotFootprint();
  fixture->footprint = compare_plugin_footprint(
    tag, to_polygon(fixture->padded_footprint), scene.padded_polygon,
    kPluginSceneConsistencyAllowance);

  const std::string name = kPlannerName;
  fixture->host->declare_parameter(name + ".tolerance", scene.planner_tolerance);
  fixture->host->declare_parameter(name + ".use_astar", scene.planner_use_astar);
  fixture->host->declare_parameter(name + ".allow_unknown", scene.planner_allow_unknown);
  fixture->loader = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
    "nav2_core", "nav2_core::GlobalPlanner");
  fixture->plugin = fixture->loader->createSharedInstance(kPluginClass);
  fixture->library_path = fixture->loader->getClassLibraryPath(kPluginClass);
  fixture->plugin->configure(fixture->host, name, fixture->tf, fixture->costmap_ros);
  fixture->plugin->activate();
  fixture->plugin_active = true;

  ReplayData & data = fixture->data;
  data.width = grid.width;
  data.height = grid.height;
  data.resolution = grid.resolution;
  data.origin_x = grid.origin_x;
  data.origin_y = grid.origin_y;
  data.frame_id = "map";
  data.costs = grid.costs;
  data.footprint = fixture->padded_footprint;
  data.start = PoseData{0.0, 0.0, 0.0};
  data.goal = PoseData{0.0, 0.0, 0.0};
  data.recorded_plan = json{{"frame_id", "map"}, {"poses", json::array()}};
  return fixture;
}

geometry_msgs::msg::PoseStamped stamped_pose(const PoseData & pose)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = "map";
  result.pose.position.x = pose.x;
  result.pose.position.y = pose.y;
  result.pose.orientation.z = std::sin(pose.yaw * 0.5);
  result.pose.orientation.w = std::cos(pose.yaw * 0.5);
  return result;
}

// Rejects empty/nonfinite/malformed paths, wrong frames and non-unit or non-planar quaternions.
std::vector<PoseData> poses_from_path(
  const nav_msgs::msg::Path & path, const std::string & frame, const std::string & what)
{
  expect(!path.poses.empty(), what + ": empty path");
  expect(path.header.frame_id == frame,
    what + ": path frame '" + path.header.frame_id + "' is not '" + frame + "'");
  std::vector<PoseData> out;
  out.reserve(path.poses.size());
  for (std::size_t index = 0; index < path.poses.size(); ++index) {
    const auto & stamped = path.poses[index];
    const std::string at = what + ": pose " + std::to_string(index);
    expect(stamped.header.frame_id == frame, at + " has frame '" + stamped.header.frame_id + "'");
    const auto & p = stamped.pose.position;
    const auto & q = stamped.pose.orientation;
    expect(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z), at + " position is not finite");
    expect(std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w),
      at + " orientation is not finite");
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    expect(std::abs(norm - 1.0) <= 1.0e-6, at + " quaternion is not normalized");
    expect(std::abs(q.x) <= 1.0e-6 && std::abs(q.y) <= 1.0e-6, at + " orientation is not planar");
    out.push_back(PoseData{p.x, p.y, std::atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)});
  }
  return out;
}

struct Sweep
{
  double x;
  double y;
  double from;
  double to;
  const char * label;
};

bool same_xy(const PoseData & pose, const double x, const double y)
{
  return std::hypot(pose.x - x, pose.y - y) <= 1.0e-9;
}

// Start sweep: stage start heading -> the heading the path departs with. Terminal sweep:
// the heading the path arrives with -> stage goal heading. A path without translation is
// swept for its whole rotation at both ends.
std::vector<Sweep> heading_sweeps(const StageData & stage, const std::vector<PoseData> & path)
{
  double depart = path.back().yaw;
  for (const auto & pose : path) {
    if (!same_xy(pose, stage.start.x, stage.start.y)) {
      depart = pose.yaw;
      break;
    }
  }
  double arrive = path.front().yaw;
  for (auto iterator = path.rbegin(); iterator != path.rend(); ++iterator) {
    if (!same_xy(*iterator, stage.goal.x, stage.goal.y)) {
      arrive = iterator->yaw;
      break;
    }
  }
  return {
    Sweep{stage.start.x, stage.start.y, stage.start.yaw, depart, "start_heading_sweep"},
    Sweep{stage.goal.x, stage.goal.y, arrive, stage.goal.yaw, "terminal_heading_sweep"}};
}

// ---------------------------------------------------------------------------
// Replay driver.
// ---------------------------------------------------------------------------

struct Totals
{
  std::size_t cases = 0;
  std::size_t stages = 0;
  std::size_t strict_samples = 0;
  std::size_t strict_cells = 0;
  std::size_t smoother_incomplete = 0;
  std::map<std::string, std::size_t> by_kind;
  std::map<std::string, std::size_t> by_stage;
  std::map<std::string, std::size_t> by_routing;
};

json failure_base(
  const std::string & category, const std::string & reason, const CaseData & c,
  const std::size_t case_index, const StageData & s, const std::size_t stage_index)
{
  json failure = json::object();
  failure["category"] = category;
  failure["reason"] = reason;
  failure["case_id"] = c.id;
  failure["case_kind"] = c.kind;
  failure["case_index"] = case_index;
  failure["case_provenance"] = c.provenance;
  failure["stage_name"] = s.name;
  failure["stage_index"] = stage_index;
  failure["stage_phase"] = s.phase;
  failure["stage_provenance"] = s.provenance;
  failure["stage_start"] = pose_json(s.start);
  failure["stage_goal"] = pose_json(s.goal);
  return failure;
}

json failure_from_verdict(
  const StrictFailure & f, const CaseData & c, const std::size_t case_index, const StageData & s,
  const std::size_t stage_index, const std::vector<PoseData> & path)
{
  json failure = failure_base("clearance", f.reason, c, case_index, s, stage_index);
  failure["role"] = f.role;
  failure["path"] = f.path_label;
  failure["grid"] = f.grid;
  failure["pose_index"] = f.pose_index;
  failure["sample_index"] = f.sample_index;
  failure["pose_map"] = pose_json(f.map_pose);
  failure["pose_grid"] = pose_json(f.grid_pose);
  failure["cell"] = json{{"x", f.cell_x}, {"y", f.cell_y}};
  failure["cost"] = f.cost;
  failure["vertex_grid_m"] = json{{"x", f.vertex_x}, {"y", f.vertex_y}};
  json poses = json::array();
  for (std::size_t index = 0; index < path.size() && index < kMaxReportedPathPoses; ++index) {
    poses.push_back(pose_json(path[index]));
  }
  failure["path_poses_reported"] = poses.size();
  failure["path_pose_count"] = path.size();
  failure["path_poses"] = poses;
  return failure;
}

struct ReplayContext
{
  const Scene & scene;
  std::array<std::unique_ptr<PlanningFixture>, 2> fixtures;  // [0] clear, [1] dock
  std::shared_ptr<nav2_util::LifecycleNode> smoother_node;
  Totals totals;
  // Conservative body-frame polygon for every strict check; set after BOTH fixtures are
  // configured and verified, before any stage is processed.
  Polygon strict_polygon;
};

bool sweep_path_against_grids(
  ReplayContext & ctx, const std::vector<PoseData> & path, const std::string & label,
  const CaseData & c, const std::size_t case_index, const StageData & s,
  const std::size_t stage_index, json & failure)
{
  const auto sweeps = heading_sweeps(s, path);
  for (const auto & grid : ctx.scene.grids) {
    auto absorb = [&](const StrictVerdict & verdict, const std::vector<PoseData> & shown) {
        ctx.totals.strict_samples += verdict.stats.samples;
        ctx.totals.strict_cells += verdict.stats.cells;
        if (!verdict.ok) {
          failure = failure_from_verdict(verdict.failure, c, case_index, s, stage_index, shown);
        }
        return verdict.ok;
      };
    if (!absorb(strict_check_path(grid, ctx.strict_polygon, path, label), path)) {
      return false;
    }
    for (const auto & sweep : sweeps) {
      if (!absorb(strict_check_heading_sweep(grid, ctx.strict_polygon, sweep.x, sweep.y,
        sweep.from, sweep.to, label + ":" + sweep.label), path))
      {
        return false;
      }
    }
  }
  return true;
}

// Records what the smoother result actually said, so a rejection is inspectable.
void add_smoother_proof(json & failure, const json & smoother)
{
  const bool is_object = smoother.is_object();
  const bool present = is_object && smoother.contains("smoother_completed");
  failure["smoother_result_is_object"] = is_object;
  failure["smoother_completed_present"] = present;
  failure["smoother_completed_observed"] = present ? smoother.at("smoother_completed") : json(nullptr);
  failure["smoother_budget_observed_s"] =
    is_object && smoother.contains("smoother_budget_s") ? smoother.at("smoother_budget_s") :
    json(nullptr);
  failure["smoother_budget_expected_s"] = kSmootherBudgetSeconds;
}

// Post-smoother consumer used by run_stage and by the focused self-test. The replay calls the
// synchronous SimpleSmoother API, which exposes no action result code; the explicit
// smoother_completed boolean is the completion proof (mission result callback requires
// was_completed). Completion is admitted before any strict sweep or successful record; the
// helper's composite "success" key is deliberately not consulted.
bool consume_smoothed_stage(
  ReplayContext & ctx, const CaseData & c, const std::size_t case_index, const StageData & s,
  const std::size_t stage_index, const std::string & routing, const std::string & planning_grid,
  const json & smoother, const std::vector<PoseData> & raw, const std::vector<PoseData> & smoothed,
  json & record, json & failure)
{
  const auto reject = [&](const std::string & reason) {
      failure = failure_base("smoother", reason, c, case_index, s, stage_index);
      add_smoother_proof(failure, smoother);
      return false;
    };
  if (!smoother.is_object()) {
    return reject("smoother_result_missing_or_not_object");
  }
  if (!smoother.contains("smoother_completed")) {
    return reject("smoother_completion_missing");
  }
  if (!smoother.at("smoother_completed").is_boolean()) {
    return reject("smoother_completion_not_boolean");
  }
  const bool completed = smoother.at("smoother_completed").get<bool>();
  if (!completed) {
    ++ctx.totals.smoother_incomplete;
    return reject("smoother_incomplete");
  }
  const json budget = smoother.contains("smoother_budget_s") ? smoother.at("smoother_budget_s") :
    json(nullptr);
  if (!budget.is_number() || budget.get<double>() != static_cast<double>(kSmootherBudgetSeconds)) {
    return reject("smoother_budget_is_not_1_s");
  }
  if (smoothed.empty()) {
    return reject("empty_smoothed_path");
  }
  if (raw.empty()) {
    return reject("empty_raw_path");
  }

  if (!sweep_path_against_grids(ctx, raw, "raw_plugin_path", c, case_index, s, stage_index, failure) ||
    !sweep_path_against_grids(ctx, smoothed, "smoothed_path", c, case_index, s, stage_index, failure))
  {
    failure["routing"] = routing;
    return false;
  }

  ++ctx.totals.by_routing[routing];
  record = json{
    {"name", s.name}, {"phase", s.phase}, {"planning_grid", planning_grid}, {"routing", routing},
    {"raw_pose_count", raw.size()}, {"smoothed_pose_count", smoothed.size()},
    {"smoother_completed", completed},
    {"raw_start_error_m", std::hypot(raw.front().x - s.start.x, raw.front().y - s.start.y)},
    {"raw_goal_error_m", std::hypot(raw.back().x - s.goal.x, raw.back().y - s.goal.y)}};
  return true;
}

struct PlannedStage
{
  nav_msgs::msg::Path path;
  std::string routing;
};

PlannedStage plan_fixture_stage(
  PlanningFixture & fixture, const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const ReplayData & captured = fixture.data;
  expect(fixture.costmap_ros != nullptr, "planning fixture has no Costmap2DROS");
  nav2_costmap_2d::Costmap2D * live_map = fixture.costmap_ros->getCostmap();
  expect(live_map != nullptr, "planning fixture has no live costmap");
  const std::size_t captured_cell_count =
    static_cast<std::size_t>(captured.width) * static_cast<std::size_t>(captured.height);
  expect(captured.width > 0 && captured.height > 0 && captured.costs.size() == captured_cell_count,
    "captured planning map has a mismatched cost byte count");
  expect(live_map->getSizeInCellsX() == captured.width &&
    live_map->getSizeInCellsY() == captured.height &&
    std::abs(live_map->getResolution() - captured.resolution) <= kPlannerMapGeometryTolerance &&
    std::abs(live_map->getOriginX() - captured.origin_x) <= kPlannerMapGeometryTolerance &&
    std::abs(live_map->getOriginY() - captured.origin_y) <= kPlannerMapGeometryTolerance,
    "live planning map geometry disagrees with captured map");
  expect(fixture.plugin != nullptr, "planning fixture has no configured plugin");
  auto * live_costs = live_map->getCharMap();
  auto * map_mutex = live_map->getMutex();
  expect(live_costs != nullptr && map_mutex != nullptr,
    "live planning map storage or mutex is unavailable");

  std::string routing;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map_mutex);
    std::copy(captured.costs.begin(), captured.costs.end(), live_costs);
    const auto segment = amr_navigation::make_precision_segment(
      start, goal, *live_map, fixture.padded_footprint);
    routing = segment.poses.empty() ? "navfn_fallback" : "direct_precision";
  }

  nav_msgs::msg::Path path = fixture.plugin->createPlan(start, goal);
  return PlannedStage{std::move(path), std::move(routing)};
}

bool run_stage(
  ReplayContext & ctx, const CaseData & c, const std::size_t case_index, const StageData & s,
  const std::size_t stage_index, json & record, json & failure)
{
  PlanningFixture & fixture = *ctx.fixtures[s.phase == "dock" ? 1 : 0];
  const auto start = stamped_pose(s.start);
  const auto goal = stamped_pose(s.goal);

  PlannedStage planned;
  try {
    planned = plan_fixture_stage(fixture, start, goal);
  } catch (const std::exception & error) {
    failure = failure_base("plan", std::string("plugin_exception: ") + error.what(), c,
        case_index, s, stage_index);
    return false;
  }
  std::vector<PoseData> raw;
  try {
    raw = poses_from_path(planned.path, "map", "plugin path");
  } catch (const SceneError & error) {
    failure = failure_base("plan", error.what(), c, case_index, s, stage_index);
    return false;
  }

  ReplayData data = fixture.data;
  data.start = s.start;
  data.goal = s.goal;
  RunOptions options;
  options.allow_unknown = false;
  std::vector<PoseData> smoothed;
  json smoother;
  try {
    smoother = runConfiguredSmoother(
      data, options, ctx.smoother_node, raw, "plugin_raw_path", kSmootherBudgetSeconds, &smoothed);
  } catch (const std::exception & error) {
    failure = failure_base("smoother", std::string("smoother_exception: ") + error.what(), c,
        case_index, s, stage_index);
    return false;
  }
  return consume_smoothed_stage(
    ctx, c, case_index, s, stage_index, planned.routing, fixture.label, smoother, raw, smoothed,
    record, failure);
}

json counts_json(const std::map<std::string, std::size_t> & counts)
{
  json out = json::object();
  for (const auto & entry : counts) {
    out[entry.first] = entry.second;
  }
  return out;
}

json report_skeleton(const std::string & scene_path)
{
  json report = json::object();
  report["schema"] = kReportSchema;
  report["scene_path"] = scene_path;
  report["units"] = json{
    {"position", "m"}, {"angle", "rad"}, {"cost", "unitless 0..255 occupancy cost"},
    {"cell", "grid cell index (x = column, y = row) in the failing grid's own frame"},
    {"smoother_budget", "s"}, {"stamp", "ns"}};
  report["claim_scope"] =
    "geometric full-polygon clearance of the plugin raw path and the 1 s smoothed path against "
    "four captured grids; not a runtime acceptance";
  return report;
}

void add_common_limitations(json & report, const Scene * scene)
{
  json limitations = json::array();
  if (scene != nullptr) {
    for (const auto & item : scene->limitations) {
      limitations.push_back(item);
    }
  }
  for (const auto * text : {
      "rotation between consecutive poses is interpolated along the shortest arc; the controller's actual turn direction is not proved",
      "the smoother budget is wall-clock; completion (smoother_completed == true) is mandatory and a stage fails closed when it is false, missing or malformed, so completion can vary with host load",
      "the plugin and smoother use the phase-selected captured global grid; the other grids are swept geometrically only",
      "settling, fresh post-heading bias, controller tracking, action execution and terminal-state guarantees are not exercised"})
  {
    limitations.push_back(text);
  }
  report["limitations"] = limitations;
  report["runtime_properties"] = scene != nullptr ? scene->runtime_properties : json::object();
}

json replay_scene(
  const Scene & scene, const std::shared_ptr<nav2_util::LifecycleNode> & smoother_node,
  const std::string & scene_path)
{
  json report = report_skeleton(scene_path);
  ReplayContext ctx{scene, {}, smoother_node, {}, {}};
  json first_failure;  // null until a mandatory failure
  json cases_summary = json::array();
  json plugin_info = json::object();
  try {
    ctx.fixtures[0] = make_fixture(scene, kClearGlobal, "clear");
    ctx.fixtures[1] = make_fixture(scene, kDockGlobal, "dock");
    ctx.strict_polygon = conservative_envelope(
      {scene.padded_polygon, ctx.fixtures[0]->footprint.actual, ctx.fixtures[1]->footprint.actual});
    plugin_info["footprint"] = footprint_evidence_json(
      scene.padded_polygon, ctx.fixtures[0]->footprint, ctx.fixtures[1]->footprint,
      ctx.strict_polygon);
    plugin_info["class"] = kPluginClass;
    plugin_info["planner_name"] = kPlannerName;
    plugin_info["library_path"] = ctx.fixtures[0]->library_path;
    plugin_info["tolerance_m"] = scene.planner_tolerance;
    plugin_info["use_astar"] = scene.planner_use_astar;
    plugin_info["allow_unknown"] = scene.planner_allow_unknown;
    plugin_info["costmap_resolution_m"] = scene.costmap_resolution;
    // Exact equality is not claimed: only consistency within the allowance was verified.
    plugin_info["padded_footprint_matches_scene"] =
      std::max(ctx.fixtures[0]->footprint.max_abs_delta, ctx.fixtures[1]->footprint.max_abs_delta) <=
      kPaddingTolerance;
    plugin_info["padded_footprint_consistent_within_allowance"] = true;
  } catch (const std::exception & error) {
    first_failure = json{{"category", "fixture_setup"}, {"reason", error.what()}};
  }

  for (std::size_t case_index = 0; case_index < scene.cases.size() && first_failure.is_null();
    ++case_index)
  {
    const CaseData & c = scene.cases[case_index];
    if (!c.admitted) {
      continue;
    }
    json summary = json{{"id", c.id}, {"kind", c.kind}, {"provenance", c.provenance},
      {"stages", json::array()}};
    for (std::size_t stage_index = 0; stage_index < c.stages.size(); ++stage_index) {
      json record;
      json failure;
      if (!run_stage(ctx, c, case_index, c.stages[stage_index], stage_index, record, failure)) {
        first_failure = failure;
        break;
      }
      summary["stages"].push_back(record);
      ++ctx.totals.stages;
      ++ctx.totals.by_stage[c.stages[stage_index].name];
    }
    if (first_failure.is_null()) {
      ++ctx.totals.cases;
      ++ctx.totals.by_kind[c.kind];
      cases_summary.push_back(summary);
    }
  }

  const bool roster_complete = ctx.totals.by_kind["nominal"] == kExpectedNominal &&
    ctx.totals.by_kind["bias_sensitivity"] == kExpectedBias &&
    ctx.totals.cases + scene.excluded_repairs == scene.cases.size();
  if (first_failure.is_null() && !roster_complete) {
    first_failure = json{{"category", "roster"},
      {"reason", "not every mandatory nominal/bias case and admitted repair case was exercised"}};
  }

  report["status"] = first_failure.is_null() ? "PASS" : "FAIL";
  report["plugin"] = plugin_info;
  report["config"] = scene.config_summary;
  report["registry"] = scene.registry;
  report["source_constants"] = scene.source_constants;
  report["scene_identity"] = scene.identity;
  report["scene_extraction"] = scene.extraction;
  json grids = json::array();
  for (const auto & grid : scene.grids) {
    grids.push_back(json{{"label", grid.label}, {"id", grid.id}, {"frame_id", grid.frame_id},
        {"stamp_ns", grid.stamp_ns}, {"width", grid.width}, {"height", grid.height},
        {"resolution_m", grid.resolution}, {"declared_data_sha256", grid.data_sha256},
        {"transform_map_to_odom", grid.has_transform ?
          json{{"x", grid.map_to_odom_x}, {"y", grid.map_to_odom_y}, {"yaw", grid.map_to_odom_yaw}} :
          json(nullptr)}});
  }
  report["grids"] = grids;
  report["stance_center_slot"] = scene.center_slot;
  report["exercised"] = json{
    {"cases_total_in_scene", scene.cases.size()},
    {"cases_exercised", ctx.totals.cases},
    {"cases_by_kind", counts_json(ctx.totals.by_kind)},
    {"repair_cases_excluded_with_reason", scene.excluded_repairs},
    {"stages_exercised", ctx.totals.stages},
    {"stages_by_name", counts_json(ctx.totals.by_stage)},
    {"routing", counts_json(ctx.totals.by_routing)},
    {"strict_pose_samples", ctx.totals.strict_samples},
    {"strict_cells_examined", ctx.totals.strict_cells},
    {"smoother_budget_s", kSmootherBudgetSeconds},
    {"smoother_incomplete_stages", ctx.totals.smoother_incomplete},
    {"grids_per_path", 4},
    {"paths_per_stage", json::array({"raw_plugin_path", "smoothed_path"})},
    {"heading_sweeps_per_path", json::array({"start_heading_sweep", "terminal_heading_sweep"})}};
  report["first_failure"] = first_failure;
  add_common_limitations(report, &scene);
  report["cases"] = cases_summary;
  return report;
}

json failure_report(
  const std::string & scene_path, const std::string & category, const std::string & reason)
{
  json report = report_skeleton(scene_path);
  report["status"] = "FAIL";
  report["first_failure"] = json{{"category", category}, {"reason", reason}};
  report["exercised"] = json{{"cases_exercised", 0}, {"stages_exercised", 0}};
  add_common_limitations(report, nullptr);
  return report;
}

void write_report(const std::string & path, json report)
{
  std::string text = report.dump(2);
  if (text.size() > kMaxReportBytes && report.contains("cases")) {
    report["cases"] = "dropped: per-case detail exceeded the report size bound";
    text = report.dump(2);
  }
  if (text.size() > kMaxReportBytes) {
    throw std::runtime_error("report exceeds the 8 MB bound even without case detail");
  }
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write report: " + path);
  }
  output << text << "\n";
  if (!output) {
    throw std::runtime_error("failed while writing report: " + path);
  }
}

// ---------------------------------------------------------------------------
// Self-test: a deterministic fixture scene plus negative/positive contract cases.
// ---------------------------------------------------------------------------

constexpr double kFixtureResolution = 0.05000000074505806;
constexpr double kFixtureBiasX = 0.0018073787966876864;
constexpr double kFixtureBiasY = 0.030670954302518174;
constexpr double kFixtureBiasYaw = -0.0070103592421109084;

void ensure(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error("self-test assertion failed: " + message);
  }
}

json fixture_pose(const double x, const double y, const double yaw)
{
  return json{{"x", x}, {"y", y}, {"yaw", wrap_rad(yaw)}};
}

json fixture_stage(
  const std::string & name, const std::string & phase, const json & start, const json & goal)
{
  return json{{"name", name}, {"phase", phase}, {"start", start}, {"goal", goal},
    {"provenance", "self-test fixture stage"}};
}

json fixture_grid(
  const std::string & id, const std::string & frame, const unsigned int width,
  const unsigned int height, const double origin_x, const double origin_y,
  const long long stamp_ns, const json & map_to_odom)
{
  json grid = json::object();
  grid["id"] = id;
  grid["frame_id"] = frame;
  grid["width"] = width;
  grid["height"] = height;
  grid["resolution"] = kFixtureResolution;
  grid["origin_x"] = origin_x;
  grid["origin_y"] = origin_y;
  grid["stamp_ns"] = stamp_ns;
  grid["data"] = std::vector<int>(static_cast<std::size_t>(width) * height, 0);
  grid["data_sha256"] = std::string(64, '0');
  grid["map_to_odom"] = map_to_odom;
  return grid;
}

json fixture_transform(const double x, const double y, const double yaw, const long long stamp_ns)
{
  json transform = json::object();
  transform["x"] = x;
  transform["y"] = y;
  transform["yaw"] = yaw;
  transform["stamp_ns"] = stamp_ns;
  return transform;
}

// Valid minimal scene: all-free grids, the real stamps/transforms, and the full 4/16/80
// roster with 3-stage (arrival heading, dock, final heading) sequences.
json make_fixture_scene()
{
  json doc = json::object();
  doc["schema"] = kSceneSchema;
  json config = json::object();
  config["footprint"] = json::parse("[[0.6,0.4],[0.6,-0.4],[-0.6,-0.4],[-0.6,0.4]]");
  config["footprint_padding"] = 0.01;
  config["footprint_string"] = "[[0.6, 0.4], [0.6, -0.4], [-0.6, -0.4], [-0.6, 0.4]]";
  config["global_costmap_resolution"] = 0.05;
  config["padded_footprint"] = json::parse("[[0.61,0.41],[0.61,-0.41],[-0.61,-0.41],[-0.61,0.41]]");
  json precision = json::object();
  precision["allow_unknown"] = false;
  precision["plugin"] = kPluginClass;
  precision["tolerance"] = 0.01;
  precision["use_astar"] = true;
  config["precision_grid_based"] = precision;
  json smoother = json::object();
  smoother["do_refinement"] = true;
  smoother["max_its"] = 1000;
  smoother["tolerance"] = 1.0e-10;
  smoother["w_data"] = 0.2;
  smoother["w_smooth"] = 0.0;
  config["simple_smoother"] = smoother;
  config["smoother_budget_s"] = 1;
  config["smoother_budget_source"] = "self-test fixture";
  doc["config"] = config;

  const std::array<double, 3> dock{-3.4, 0.0, M_PI};
  const std::array<double, 3> slot{-4.1, 0.0, 0.075};
  const auto stance = amr_interfaces::placement::final_placement_stance(102, dock, slot);
  json registry = json::object();
  registry["product_id"] = 102;
  registry["slot_id"] = "dispatch_2";
  registry["dispatch_dock"] = json::array({dock[0], dock[1], dock[2]});
  registry["selected_slot"] = json::array({slot[0], slot[1], slot[2]});
  registry["dispatch_approach"] = json::array({-2.5, 0.0, M_PI});
  registry["physical_stance"] = json::array({stance.physical[0], stance.physical[1], stance.physical[2]});
  registry["clear_point"] = json::array({-2.5, 0.1});
  doc["registry"] = registry;

  json phases = json::object();
  phases["clear"] = "clear.global";
  phases["dock"] = "pre_dock.global";
  doc["planning_grid_by_phase"] = phases;

  json runtime = json::object();
  for (const auto * key : {"action_execution", "controller_tracking", "fresh_post_heading_bias",
      "observer_settling", "stationary_start", "terminal_state_guarantees"})
  {
    runtime[key] = "not_exercised";
  }
  doc["runtime_properties"] = runtime;
  doc["limitations"] = json::array({"self-test fixture; historical equivalence is not claimed"});
  doc["extraction"] = json::object();
  doc["identity"] = json::object();
  json constants = json::object();
  constants["kFinalHeadingGoalMargin"] = kFinalHeadingMarginRad;
  constants["kDesiredProduct102SlotBaseX"] = amr_interfaces::placement::kDesiredProduct102SlotBaseX;
  constants["kDesiredProduct102SlotBaseY"] = amr_interfaces::placement::kDesiredProduct102SlotBaseY;
  doc["source_constants"] = constants;

  const std::vector<std::string> geometries{"global_costmap", "local_costmap", "global_footprint",
    "local_footprint"};
  json scenes = json::object();
  std::vector<std::string> clear_ids;
  std::vector<std::string> dock_ids;
  for (const std::string name : {"clear", "pre_dock"}) {
    json item = json::object();
    json captures = json::array();
    for (const auto & geometry : geometries) {
      json capture = json::object();
      capture["id"] = name + ":" + geometry;
      captures.push_back(capture);
      (name == "clear" ? clear_ids : dock_ids).push_back(name + ":" + geometry);
    }
    item["captures"] = captures;
    json grids = json::object();
    const bool clear = name == "clear";
    grids["global"] = fixture_grid(name + "_global", "map", 240, 200, -6.0, -5.0,
        clear ? 401766626490LL : 410216625645LL, nullptr);
    grids["local"] = fixture_grid(name + "_local", "odom", 100, 100, clear ? -0.3 : -0.4,
        clear ? -1.0 : -0.95, clear ? 401616626505LL : 409979959002LL,
        clear ? fixture_transform(-4.533362598803633, -1.5455333196271157, 0.007010589299834211,
        401616626505LL) :
        fixture_transform(-4.532143197282093, -1.5441274812079069, 0.006296029611193008,
        409979959002LL));
    item["grids"] = grids;
    scenes[name] = item;
  }
  doc["scenes"] = scenes;

  json cases = json::array();
  for (const auto & clear_id : clear_ids) {
    std::vector<std::string> labels{"own"};
    labels.insert(labels.end(), dock_ids.begin(), dock_ids.end());
    for (const auto & label : labels) {
      for (const std::string variant : {"", "|repair_miss=+x", "|repair_miss=-x",
          "|repair_miss=+y", "|repair_miss=-y"})
      {
        const bool repair = !variant.empty();
        const std::string suffix = repair ? "_repair" : "";
        json item = json::object();
        item["id"] = clear_id + "|bias=" + label + variant;
        item["kind"] = repair ? "synthetic_repair" : (label == "own" ? "nominal" : "bias_sensitivity");
        item["synthetic"] = repair;
        item["admitted"] = true;
        item["admission_reason"] = "admitted";
        item["provenance"] = "self-test fixture case";
        item["bias"] = fixture_pose(kFixtureBiasX, kFixtureBiasY, kFixtureBiasYaw);
        const double stance_yaw = stance.physical[2];
        const json arrival_start = fixture_pose(-2.5 - kFixtureBiasX, 0.1 - kFixtureBiasY,
            stance_yaw - kFixtureBiasYaw - 0.4);
        const json arrival_goal = fixture_pose(-2.5 - kFixtureBiasX, 0.1 - kFixtureBiasY,
            stance_yaw - kFixtureBiasYaw);
        const json dock_goal = fixture_pose(stance.physical[0] - kFixtureBiasX,
            stance.physical[1] - kFixtureBiasY, stance_yaw - kFixtureBiasYaw);
        const json final_goal = fixture_pose(stance.physical[0] - kFixtureBiasX,
            stance.physical[1] - kFixtureBiasY, stance_yaw - kFixtureBiasYaw - kFinalHeadingMarginRad);
        json stages = json::array();
        stages.push_back(fixture_stage("arrival_heading" + suffix, "clear", arrival_start, arrival_goal));
        stages.push_back(fixture_stage("dock", "dock", arrival_goal, dock_goal));
        stages.push_back(fixture_stage("final_heading_margin", "dock", dock_goal, final_goal));
        item["stages"] = stages;
        cases.push_back(item);
      }
    }
  }
  doc["cases"] = cases;
  return doc;
}

GridView test_grid(
  const std::string & label, const unsigned int width, const unsigned int height,
  const double origin_x, const double origin_y)
{
  GridView grid;
  grid.label = label;
  grid.id = label;
  grid.frame_id = "map";
  grid.stamp_ns = 1;
  grid.width = width;
  grid.height = height;
  grid.resolution = 0.05;
  grid.origin_x = origin_x;
  grid.origin_y = origin_y;
  grid.costs.assign(static_cast<std::size_t>(width) * height, 0);
  return grid;
}

// Cell containing world (x, y); tests only use cell-center coordinates.
void set_world_cost(GridView & grid, const double x, const double y, const unsigned char cost)
{
  const auto column = static_cast<long long>(std::floor((x - grid.origin_x) / grid.resolution));
  const auto row = static_cast<long long>(std::floor((y - grid.origin_y) / grid.resolution));
  ensure(column >= 0 && row >= 0 && column < grid.width && row < grid.height,
    "test cell is outside the test grid");
  grid.costs[static_cast<std::size_t>(column) + static_cast<std::size_t>(row) * grid.width] = cost;
}

ReplayData old_checker_data(const GridView & grid, const Polygon & polygon)
{
  ReplayData data{};
  data.width = grid.width;
  data.height = grid.height;
  data.resolution = grid.resolution;
  data.origin_x = grid.origin_x;
  data.origin_y = grid.origin_y;
  data.frame_id = grid.frame_id;
  data.costs = grid.costs;
  for (const auto & vertex : polygon) {
    geometry_msgs::msg::Point point;
    point.x = vertex.first;
    point.y = vertex.second;
    data.footprint.push_back(point);
  }
  return data;
}

// The old permissive (perimeter-only) checker.
bool old_checker_accepts(
  const GridView & grid, const Polygon & polygon, const std::vector<PoseData> & path)
{
  const ReplayData data = old_checker_data(grid, polygon);
  auto costmap = makeCostmap(data);
  return checkWorldPath(data, costmap.get(), path, false).collision_free;
}

void expect_rejected(
  const json & doc, const std::function<void(json &)> & mutate, const std::string & fragment,
  const std::string & name)
{
  json copy = doc;
  mutate(copy);
  bool rejected = false;
  std::string why;
  try {
    validate_scene(copy);
  } catch (const SceneError & error) {
    rejected = true;
    why = error.what();
  }
  ensure(rejected, name + ": malformed scene was accepted");
  ensure(why.find(fragment) != std::string::npos,
    name + ": rejected for the wrong reason: " + why);
}

struct SelfTestResult
{
  std::size_t passed = 0;
  std::vector<std::string> failed;
};

void run_test(SelfTestResult & result, const std::string & name, const std::function<void()> & body)
{
  try {
    body();
    ++result.passed;
    std::cout << "[PASS] " << name << "\n";
  } catch (const std::exception & error) {
    result.failed.push_back(name);
    std::cout << "[FAIL] " << name << ": " << error.what() << "\n";
  }
}

constexpr PoseData kPlannerMapTestStart{-1.025, -0.025, 0.0};
constexpr PoseData kPlannerMapTestGoal{-0.525, -0.025, 0.0};
constexpr double kPlannerMapMarkerX = 5.925;
constexpr double kPlannerMapMarkerY = 4.925;

struct PlannerMapCell
{
  unsigned int x = 0;
  unsigned int y = 0;
  std::size_t index = 0;
};

PlannerMapCell planner_map_cell(
  const GridView & grid, const double world_x, const double world_y)
{
  ReplayData geometry{};
  geometry.width = grid.width;
  geometry.height = grid.height;
  geometry.resolution = grid.resolution;
  geometry.origin_x = grid.origin_x;
  geometry.origin_y = grid.origin_y;
  geometry.costs = grid.costs;
  auto map = makeCostmap(geometry);
  unsigned int mx = 0;
  unsigned int my = 0;
  ensure(map->worldToMap(world_x, world_y, mx, my),
    grid.label + ": planner-map test coordinate is outside the captured grid");
  double center_x = 0.0;
  double center_y = 0.0;
  map->mapToWorld(mx, my, center_x, center_y);
  ensure(std::abs(center_x - world_x) <= 1.0e-6 && std::abs(center_y - world_y) <= 1.0e-6,
    grid.label + ": planner-map test coordinate is not a cell center");
  return PlannerMapCell{mx, my, static_cast<std::size_t>(mx) +
    static_cast<std::size_t>(my) * grid.width};
}

Scene make_planner_map_test_scene(const bool block_start, const bool add_phase_markers)
{
  Scene scene = validate_scene(make_fixture_scene());
  GridView & clear = scene.grids[kClearGlobal];
  GridView & dock = scene.grids[kDockGlobal];
  const PlannerMapCell clear_start = planner_map_cell(
    clear, kPlannerMapTestStart.x, kPlannerMapTestStart.y);
  const PlannerMapCell dock_start = planner_map_cell(
    dock, kPlannerMapTestStart.x, kPlannerMapTestStart.y);
  const PlannerMapCell clear_marker = planner_map_cell(
    clear, kPlannerMapMarkerX, kPlannerMapMarkerY);
  const PlannerMapCell dock_marker = planner_map_cell(
    dock, kPlannerMapMarkerX, kPlannerMapMarkerY);
  ensure(clear_start.index == dock_start.index && clear_marker.index == dock_marker.index,
    "clear and dock fixture grids do not share the expected test-cell geometry");
  ensure(clear_start.index != clear_marker.index,
    "planner-map phase marker is not away from the start route");
  if (block_start) {
    clear.costs[clear_start.index] = kInscribedCost;
    dock.costs[dock_start.index] = kInscribedCost;
  }
  if (add_phase_markers) {
    clear.costs[clear_marker.index] = 17;
    dock.costs[dock_marker.index] = 23;
  }
  return scene;
}

std::vector<unsigned char> live_costmap_bytes(nav2_costmap_2d::Costmap2D & map)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
  const std::size_t cell_count = static_cast<std::size_t>(map.getSizeInCellsX()) *
    static_cast<std::size_t>(map.getSizeInCellsY());
  const unsigned char * costs = map.getCharMap();
  ensure(costs != nullptr, "live costmap has no byte storage");
  return std::vector<unsigned char>(costs, costs + cell_count);
}

void set_live_costmap_cell(
  PlanningFixture & fixture, const PlannerMapCell & cell, const unsigned char cost)
{
  nav2_costmap_2d::Costmap2D * map = fixture.costmap_ros->getCostmap();
  ensure(map != nullptr, "planner-map test fixture has no live costmap");
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
  map->setCost(cell.x, cell.y, cost);
}

class RecordingGlobalPlanner final : public nav2_core::GlobalPlanner
{
public:
  RecordingGlobalPlanner(
    std::shared_ptr<nav2_core::GlobalPlanner> delegate, nav2_costmap_2d::Costmap2D * map)
  : delegate_(std::move(delegate)), map_(map)
  {
    ensure(delegate_ != nullptr && map_ != nullptr,
      "planner-map recorder requires a real plugin and live costmap");
  }

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    delegate_->configure(parent, std::move(name), std::move(tf), std::move(costmap_ros));
  }

  void cleanup() override
  {
    delegate_->cleanup();
  }

  void activate() override
  {
    delegate_->activate();
  }

  void deactivate() override
  {
    delegate_->deactivate();
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    entry_costs.push_back(live_costmap_bytes(*map_));
    return delegate_->createPlan(start, goal);
  }

  std::vector<std::vector<unsigned char>> entry_costs;

private:
  std::shared_ptr<nav2_core::GlobalPlanner> delegate_;
  nav2_costmap_2d::Costmap2D * map_;
};

std::shared_ptr<RecordingGlobalPlanner> attach_planner_map_recorder(PlanningFixture & fixture)
{
  auto recorder = std::make_shared<RecordingGlobalPlanner>(
    fixture.plugin, fixture.costmap_ros->getCostmap());
  fixture.plugin = recorder;
  return recorder;
}

void run_planner_map_tests(SelfTestResult & result)
{
  const auto start = stamped_pose(kPlannerMapTestStart);
  const auto goal = stamped_pose(kPlannerMapTestGoal);

  run_test(result, "planner_map_classifies_before_real_navfn_mutation", [&]() {
      const Scene scene = make_planner_map_test_scene(true, false);
      auto fixture = make_fixture(scene, kClearGlobal, "planner_map_order");
      const PlannerMapCell start_cell = planner_map_cell(
        scene.grids[kClearGlobal], kPlannerMapTestStart.x, kPlannerMapTestStart.y);
      auto recorder = attach_planner_map_recorder(*fixture);
      ensure(fixture->data.costs[start_cell.index] == kInscribedCost,
        "captured start center is not inscribed cost 253");
      auto independent_map = makeCostmap(fixture->data);
      ensure(independent_map->getCost(start_cell.x, start_cell.y) == kInscribedCost,
        "independent captured map did not preserve start cost 253");
      const auto captured_segment = amr_navigation::make_precision_segment(
        start, goal, *independent_map, fixture->padded_footprint);
      ensure(captured_segment.poses.empty(),
        "direct precision helper accepted the captured blocked start center");

      const PlannedStage planned = plan_fixture_stage(*fixture, start, goal);
      ensure(!planned.path.poses.empty(), "real Navfn fallback returned an empty path");
      ensure(recorder->entry_costs.size() == 1 &&
        recorder->entry_costs.front() == fixture->data.costs &&
        recorder->entry_costs.front()[start_cell.index] == kInscribedCost,
        "real plugin did not receive the captured start cost at createPlan entry");
      const auto after_plugin = live_costmap_bytes(*fixture->costmap_ros->getCostmap());
      ensure(after_plugin[start_cell.index] == 0,
        "real Navfn fallback did not clear the live start cell after planning");
      auto * live_map = fixture->costmap_ros->getCostmap();
      nav_msgs::msg::Path post_call_segment;
      {
        std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*live_map->getMutex());
        post_call_segment = amr_navigation::make_precision_segment(
          start, goal, *live_map, fixture->padded_footprint);
      }
      ensure(!post_call_segment.poses.empty(),
        "post-call direct helper did not witness the plugin's start-cell mutation");
      ensure(planned.routing == "navfn_fallback",
        "routing did not preserve the pre-plugin fallback classification");
    });

  run_test(result, "planner_map_restores_every_phase_call", [&]() {
      const Scene scene = make_planner_map_test_scene(true, true);
      auto clear = make_fixture(scene, kClearGlobal, "planner_map_restore_clear");
      auto dock = make_fixture(scene, kDockGlobal, "planner_map_restore_dock");
      auto clear_recorder = attach_planner_map_recorder(*clear);
      auto dock_recorder = attach_planner_map_recorder(*dock);
      const PlannerMapCell clear_start = planner_map_cell(
        scene.grids[kClearGlobal], kPlannerMapTestStart.x, kPlannerMapTestStart.y);
      const PlannerMapCell dock_start = planner_map_cell(
        scene.grids[kDockGlobal], kPlannerMapTestStart.x, kPlannerMapTestStart.y);
      const PlannerMapCell clear_marker = planner_map_cell(
        scene.grids[kClearGlobal], kPlannerMapMarkerX, kPlannerMapMarkerY);
      const PlannerMapCell dock_marker = planner_map_cell(
        scene.grids[kDockGlobal], kPlannerMapMarkerX, kPlannerMapMarkerY);
      ensure(clear->data.costs[clear_marker.index] == 17 &&
        dock->data.costs[dock_marker.index] == 23 && clear->data.costs != dock->data.costs,
        "phase fixtures do not retain their distinct captured marker costs");

      for (std::size_t call = 0; call < 4; ++call) {
        const bool use_dock = (call % 2) == 1;
        PlanningFixture & fixture = use_dock ? *dock : *clear;
        const auto & recorder = use_dock ? dock_recorder : clear_recorder;
        const PlannerMapCell & start_cell = use_dock ? dock_start : clear_start;
        const PlannerMapCell & marker_cell = use_dock ? dock_marker : clear_marker;
        if (call == 2 || call == 3) {
          set_live_costmap_cell(fixture, marker_cell, 211);
        }
        const PlannedStage planned = plan_fixture_stage(fixture, start, goal);
        ensure(!planned.path.poses.empty(),
          "phase planning call did not return the real plugin path");
        const std::size_t phase_call = call / 2;
        ensure(recorder->entry_costs.size() == phase_call + 1 &&
          recorder->entry_costs.back() == fixture.data.costs &&
          recorder->entry_costs.back()[start_cell.index] == kInscribedCost &&
          recorder->entry_costs.back()[marker_cell.index] == fixture.data.costs[marker_cell.index],
          fixture.label + ": complete createPlan entry bytes were not restored from capture");
        if (call == 0 || call == 1) {
          const auto after_first_call = live_costmap_bytes(*fixture.costmap_ros->getCostmap());
          ensure(after_first_call[start_cell.index] == 0,
            fixture.label + ": first real fallback did not clear the live start cell");
        }
      }
    });

  run_test(result, "planner_map_keeps_captured_collision_data_immutable", [&]() {
      Scene scene = make_planner_map_test_scene(true, false);
      std::array<std::vector<unsigned char>, kExpectedCaptures> scene_costs_before;
      for (std::size_t index = 0; index < scene.grids.size(); ++index) {
        scene_costs_before[index] = scene.grids[index].costs;
      }
      auto fixture = make_fixture(scene, kClearGlobal, "planner_map_immutable");
      const std::vector<unsigned char> fixture_costs_before = fixture->data.costs;
      const PlannerMapCell start_cell = planner_map_cell(
        scene.grids[kClearGlobal], kPlannerMapTestStart.x, kPlannerMapTestStart.y);
      auto recorder = attach_planner_map_recorder(*fixture);
      const PlannedStage planned = plan_fixture_stage(*fixture, start, goal);
      ensure(!planned.path.poses.empty(), "real plugin fallback returned an empty path");
      ensure(recorder->entry_costs.size() == 1 &&
        recorder->entry_costs.front() == fixture_costs_before,
        "real plugin did not receive the fixture's captured map bytes");
      const auto live_after_plugin = live_costmap_bytes(*fixture->costmap_ros->getCostmap());
      ensure(live_after_plugin[start_cell.index] == 0,
        "real Navfn fallback did not clear the live start cell after planning");
      ensure(fixture->data.costs == fixture_costs_before,
        "fixture's original captured costs changed after plugin fallback");
      for (std::size_t index = 0; index < scene.grids.size(); ++index) {
        ensure(scene.grids[index].costs == scene_costs_before[index],
          scene.grids[index].label + ": captured scene grid bytes changed after plugin fallback");
      }
      const auto verdict = strict_check_path(
        scene.grids[kClearGlobal], fixture->footprint.actual,
        {{kPlannerMapTestStart.x, kPlannerMapTestStart.y, kPlannerMapTestStart.yaw}},
        "planner_map_captured_blocked_start");
      ensure(!verdict.ok && verdict.failure.reason == "center_cost_inscribed_or_worse" &&
        verdict.failure.cost == kInscribedCost && verdict.failure.cell_x == start_cell.x &&
        verdict.failure.cell_y == start_cell.y,
        "strict captured-grid check did not reject the original blocked start cell 253");
    });

  run_test(result, "planner_map_clear_positive_uses_real_direct_plugin", [&]() {
      const Scene scene = make_planner_map_test_scene(false, false);
      for (const auto & grid : scene.grids) {
        ensure(std::all_of(grid.costs.begin(), grid.costs.end(),
          [](const unsigned char cost) {return cost == 0;}),
          grid.label + ": positive planner-map scene is not all free");
      }
      auto clear = make_fixture(scene, kClearGlobal, "planner_map_positive_clear");
      auto dock = make_fixture(scene, kDockGlobal, "planner_map_positive_dock");
      auto clear_recorder = attach_planner_map_recorder(*clear);
      auto dock_recorder = attach_planner_map_recorder(*dock);
      const std::array<std::pair<PlanningFixture *,
        std::shared_ptr<RecordingGlobalPlanner>>, 2> phase_fixtures{{
          {clear.get(), clear_recorder}, {dock.get(), dock_recorder}}};
      for (const auto & item : phase_fixtures)
      {
        const PlannedStage planned = plan_fixture_stage(*item.first, start, goal);
        ensure(!planned.path.poses.empty() && planned.routing == "direct_precision",
          item.first->label + ": clear real plugin call was not classified direct_precision");
        ensure(item.second->entry_costs.size() == 1 &&
          item.second->entry_costs.front() == item.first->data.costs,
          item.first->label + ": real plugin did not receive the captured all-free bytes");
        const auto poses = poses_from_path(planned.path, "map", "planner-map positive path");
        ensure(!poses.empty(), item.first->label + ": real plugin poses were not validated");
      }
    });
}

Polygon test_rectangle(
  const double max_x, const double max_y, const double min_x, const double min_y)
{
  return Polygon{{max_x, max_y}, {max_x, min_y}, {min_x, min_y}, {min_x, max_y}};
}

void expect_footprint_rejected(
  const Polygon & actual, const Polygon & nominal, const std::vector<std::string> & fragments,
  const std::string & name)
{
  bool rejected = false;
  std::string why;
  try {
    compare_plugin_footprint("clear", actual, nominal, kPluginSceneConsistencyAllowance);
  } catch (const SceneError & error) {
    rejected = true;
    why = error.what();
  }
  ensure(rejected, name + ": footprint was accepted");
  for (const auto & fragment : fragments) {
    ensure(why.find(fragment) != std::string::npos,
      name + ": proof lacks '" + fragment + "': " + why);
  }
}

// Footprint-focused groups: run by --self-test-footprint and, unchanged, by --self-test.
// They reuse make_fixture, compare_plugin_footprint, conservative_envelope and the strict
// checker; none runs replay_scene, the smoother or the 100-case roster.
void run_footprint_tests(SelfTestResult & result)
{
  const Polygon nominal{{0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}, {-0.61, 0.41}};
  const json fixture = make_fixture_scene();

  run_test(result, "footprint_production_float_parse_accepted_in_both_phases_with_envelope", [&]() {
      const Scene scene = validate_scene(fixture);
      const auto clear = make_fixture(scene, kClearGlobal, "clear");
      const auto dock = make_fixture(scene, kDockGlobal, "dock");
      for (const auto * item : {clear.get(), dock.get()}) {
        // Float-rounded 0.6/0.4 differ from the decimal scene by ~2e-8 m: more than the
        // 1e-9 nominal tolerance (the old false failure), far inside the 0.02 m allowance.
        ensure(item->footprint.max_abs_delta > kPaddingTolerance &&
          item->footprint.max_abs_delta <= kPluginSceneConsistencyAllowance,
          item->label + ": float-rounding delta " + metres(item->footprint.max_abs_delta) +
          " is not in (1e-9, 0.02] m");
        ensure(item->footprint.signed_delta.size() == 4 && item->footprint.actual.size() == 4,
          item->label + ": phase footprint evidence is incomplete");
      }
      const Polygon envelope = conservative_envelope(
        {scene.padded_polygon, clear->footprint.actual, dock->footprint.actual});
      for (const Polygon * polygon : std::vector<const Polygon *>{
          &scene.padded_polygon, &clear->footprint.actual, &dock->footprint.actual})
      {
        ensure(polygon_encloses(envelope, *polygon), "envelope does not enclose an input polygon");
      }
      ensure(envelope[0].first >= clear->footprint.actual[0].first &&
        envelope[0].first >= scene.padded_polygon[0].first &&
        envelope[0].second >= dock->footprint.actual[0].second &&
        envelope[2].first <= clear->footprint.actual[2].first &&
        envelope[2].second <= scene.padded_polygon[2].second, "envelope bound was shrunk");
      const json evidence = footprint_evidence_json(
        scene.padded_polygon, clear->footprint, dock->footprint, envelope);
      ensure(evidence.at("phases").at("clear").at("signed_vertex_deltas_m").size() == 4 &&
        evidence.at("plugin_scene_consistency_allowance_m") == 0.02 &&
        evidence.at("conservative_strict_polygon").size() == 4,
        "footprint evidence block is incomplete");
    });

  run_test(result, "footprint_allowance_boundary_is_0_02_m_per_coordinate", [&]() {
      const auto widened = [](const double extra) {
          return test_rectangle(0.61 + extra, 0.41, -0.61, -0.41);
        };
      const auto inside = compare_plugin_footprint(
        "clear", widened(0.019), nominal, kPluginSceneConsistencyAllowance);
      ensure(std::abs(inside.max_abs_delta - 0.019) < 1.0e-12 &&
        inside.signed_delta[0].first > 0.0 && inside.signed_delta[3].first == 0.0,
        "0.019 m enlarged bound was not accepted with its signed deltas");
      // 0.61 + 0.02 - 0.61 is 0.020000000000000018 in binary; the 1e-12 noise term admits
      // exactly 0.02 m only. It is not a wider allowance: 0.02 m + 1e-9 m must fail.
      compare_plugin_footprint("clear", widened(0.02), nominal, kPluginSceneConsistencyAllowance);
      compare_plugin_footprint(
        "dock", test_rectangle(0.61, 0.41, -0.61 - 0.02, -0.41), nominal,
        kPluginSceneConsistencyAllowance);
      ensure(kAllowanceBoundaryNoise < 1.0e-9, "boundary noise term exceeds numerical noise");
      expect_footprint_rejected(widened(0.0201), nominal,
        {"phase clear", "vertex 0", "coordinate x", "actual 0.63", "expected " + metres(nominal[0].first), "signed delta",
          "0.02"}, "0.0201 m");
      expect_footprint_rejected(widened(0.02 + 1.0e-9), nominal, {"vertex 0", "coordinate x"},
        "0.02 m + 1e-9 m");
      expect_footprint_rejected(test_rectangle(0.61, 0.41, -0.61, -0.41 - 0.0201), nominal,
        {"vertex 1", "coordinate y", "signed delta -0.02"}, "-0.0201 m on y");
    });

  run_test(result, "footprint_malformed_nonfinite_count_order_and_shape_still_reject", [&]() {
      const double nan = std::numeric_limits<double>::quiet_NaN();
      const double inf = std::numeric_limits<double>::infinity();
      expect_footprint_rejected(test_rectangle(nan, 0.41, -0.61, -0.41), nominal,
        {"phase clear", "vertex 0", "not finite"}, "nan vertex");
      expect_footprint_rejected(test_rectangle(0.61, 0.41, -0.61, -inf), nominal,
        {"vertex 1", "not finite"}, "inf vertex");
      expect_footprint_rejected(Polygon{{0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}}, nominal,
        {"vertex count 3"}, "three vertices");
      Polygon five = nominal;
      five.emplace_back(0.0, 0.41);
      expect_footprint_rejected(five, nominal, {"vertex count 5"}, "five vertices");
      expect_footprint_rejected(Polygon{nominal[3], nominal[2], nominal[1], nominal[0]}, nominal,
        {"axis-aligned"}, "reversed vertex order");
      expect_footprint_rejected(Polygon{nominal[1], nominal[2], nominal[3], nominal[0]}, nominal,
        {"axis-aligned"}, "rotated vertex order");
      Polygon skewed = nominal;
      skewed[1].first += 0.01;  // inside the allowance, but no longer a rectangle
      expect_footprint_rejected(skewed, nominal, {"axis-aligned"}, "skewed rectangle");
      bool degenerate = false;
      try {
        conservative_envelope({test_rectangle(0.0, 0.0, 0.0, 0.0)});
      } catch (const SceneError &) {
        degenerate = true;
      }
      ensure(degenerate, "degenerate envelope was accepted");
      bool nonfinite = false;
      try {
        conservative_envelope({nominal, test_rectangle(nan, 0.41, -0.61, -0.41)});
      } catch (const SceneError &) {
        nonfinite = true;
      }
      ensure(nonfinite, "nonfinite envelope input was accepted");
    });

  run_test(result, "footprint_one_envelope_covers_clear_dock_and_nominal", [&]() {
      const Polygon clear = test_rectangle(0.61 + 0.019, 0.41, -0.61 - 0.006, -0.41);
      const Polygon dock = test_rectangle(0.61, 0.41 + 0.015, -0.61, -0.41 - 0.012);
      const auto clear_checked =
        compare_plugin_footprint("clear", clear, nominal, kPluginSceneConsistencyAllowance);
      const auto dock_checked =
        compare_plugin_footprint("dock", dock, nominal, kPluginSceneConsistencyAllowance);
      const Polygon envelope = conservative_envelope({nominal, clear_checked.actual, dock_checked.actual});
      ensure(envelope == test_rectangle(0.61 + 0.019, 0.41 + 0.015, -0.61 - 0.006, -0.41 - 0.012),
        "envelope is not the one rectangle with the largest bound on each side");
      for (const Polygon * polygon : {&nominal, &clear, &dock}) {
        for (const auto & vertex : *polygon) {
          ensure(vertex.first >= envelope[2].first && vertex.first <= envelope[0].first &&
            vertex.second >= envelope[2].second && vertex.second <= envelope[0].second,
            "an input vertex lies outside the envelope");
        }
      }
      ensure(envelope[0].first > nominal[0].first && envelope[3].second > nominal[3].second,
        "envelope did not grow beyond nominal where an actual polygon is larger");
    });

  run_test(result, "footprint_allowance_cannot_hide_a_collision_in_the_enlarged_envelope", [&]() {
      // Pose x = 0.035: the nominal +x edge is 0.645 m (cell column 52, which ends at 0.65);
      // the 0.019 m enlarged edge is 0.664 m (column 53). Column 53 holds the cell under test.
      const Polygon actual = test_rectangle(0.61 + 0.019, 0.41, -0.61, -0.41);
      const Polygon envelope = conservative_envelope(
        {nominal, compare_plugin_footprint(
            "clear", actual, nominal, kPluginSceneConsistencyAllowance).actual});
      const std::vector<PoseData> path{{0.035, 0.0, 0.0}};
      for (const unsigned char cost : {kLethalCost, kUnknownCost}) {
        GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
        set_world_cost(grid, 0.675, 0.025, cost);
        const auto nominal_verdict = strict_check_path(grid, nominal, path, "nominal");
        ensure(nominal_verdict.ok, "the nominal polygon should clear the cell outside 0.645 m");
        const auto verdict = strict_check_path(grid, envelope, path, "envelope");
        ensure(!verdict.ok && verdict.failure.role == "footprint_cell" &&
          verdict.failure.reason == (cost == kUnknownCost ? "unknown_cell" : "lethal_cell") &&
          verdict.failure.cell_x == 53 && verdict.failure.cell_y == 40 &&
          verdict.failure.cost == cost, "conservative strict checker missed the enclosed cell");
      }
      // 253 is not lethal away from the center: unchanged cost gate.
      GridView inscribed = test_grid("test", 100, 100, -2.0, -2.0);
      set_world_cost(inscribed, 0.675, 0.025, kInscribedCost);
      ensure(strict_check_path(inscribed, envelope, path, "inscribed").ok,
        "off-center cost 253 changed behaviour");
      // Outside-map rejection through the same checker: the nominal edge is 2.985 m and the
      // enlarged edge 3.004 m, with the map ending at 3.0 m.
      const GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
      const std::vector<PoseData> edge{{2.375, 0.0, 0.0}};
      ensure(strict_check_path(grid, nominal, edge, "nominal").ok, "nominal polygon should fit");
      const auto outside = strict_check_path(grid, envelope, edge, "envelope");
      ensure(!outside.ok && outside.failure.reason == "outside_map" &&
        outside.failure.role == "footprint_vertex", "enlarged envelope was clipped to the map");
    });

  run_test(result, "footprint_nominal_scene_shape_and_stance_gates_remain_strict", [&]() {
      ensure(validate_scene(fixture).padded_polygon.size() == 4, "nominal fixture scene was rejected");
      // Within the 0.02 m plugin allowance, but the scene's own shape gate stays at 1e-9.
      expect_rejected(fixture, [](json & d) {d["config"]["padded_footprint"][0][0] = 0.6100001;},
        "padded footprint", "padded shape 1e-7 m off");
      expect_rejected(fixture, [](json & d) {d["config"]["padded_footprint"][0][0] = 0.629;},
        "padded footprint", "padded shape 0.019 m off");
      expect_rejected(fixture, [](json & d) {d["config"]["footprint_padding"] = 0.02;},
        "0.01 m", "padding");
      expect_rejected(fixture, [](json & d) {d["registry"]["selected_slot"][0] = -4.1000001;},
        "final_placement_stance()", "stance 1e-7 m off");
    });
}

// Straight stage path with shortest-arc yaw interpolation (clear in the all-free fixture grids).
std::vector<PoseData> interpolated_stage_path(const StageData & stage, const int count)
{
  std::vector<PoseData> path;
  const double delta_yaw = wrap_rad(stage.goal.yaw - stage.start.yaw);
  for (int index = 0; index < count; ++index) {
    const double t = static_cast<double>(index) / static_cast<double>(count - 1);
    path.push_back(PoseData{
      stage.start.x + t * (stage.goal.x - stage.start.x),
      stage.start.y + t * (stage.goal.y - stage.start.y), wrap_rad(stage.start.yaw + t * delta_yaw)});
  }
  return path;
}

// Smoother-completion groups: run by --self-test-smoother-completion and, unchanged, by
// --self-test. They exercise consume_smoothed_stage (the seam run_stage uses after the real
// planner and smoother calls) plus one real configured SimpleSmoother call; none runs
// replay_scene or the 100-case roster.
void run_smoother_completion_tests(
  SelfTestResult & result, const std::shared_ptr<nav2_util::LifecycleNode> & smoother_node)
{
  const json fixture = make_fixture_scene();
  const Polygon rectangle{{0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}, {-0.61, 0.41}};

  // Result as the backend builds it. "success" is the helper's composite (completion, old
  // perimeter collision, endpoint compatibility), never an action code; it is varied freely.
  const auto helper_result = [](const json & completed, const bool composite_success) {
      json out = json::object();
      if (!completed.is_discarded()) {
        out["smoother_completed"] = completed;
      }
      out["smoother_budget_s"] = kSmootherBudgetSeconds;
      out["success"] = composite_success;
      return out;
    };
  const json missing = json::value_t::discarded;

  // Runs the seam on a fresh context and returns {accepted, record, failure}.
  struct Outcome
  {
    bool accepted = false;
    json record;
    json failure;
    Totals totals;
  };
  const auto consume = [&](
    const Scene & scene, const json & smoother, const std::vector<PoseData> & raw,
    const std::vector<PoseData> & smoothed) {
      ReplayContext ctx{scene, {}, smoother_node, {}, scene.padded_polygon};
      Outcome outcome;
      const CaseData & c = scene.cases[0];
      outcome.accepted = consume_smoothed_stage(
        ctx, c, 0, c.stages[1], 1, "direct_precision", "pre_dock.global", smoother, raw, smoothed,
        outcome.record, outcome.failure);
      outcome.totals = ctx.totals;
      return outcome;
    };
  const auto expect_closed = [&](
    const Outcome & outcome, const std::string & reason, const std::string & name) {
      ensure(!outcome.accepted, name + ": incomplete smoothing was admitted");
      ensure(outcome.failure.at("category") == "smoother" && outcome.failure.at("reason") == reason,
        name + ": wrong rejection: " + outcome.failure.dump());
      ensure(outcome.failure.contains("case_id") && outcome.failure.contains("stage_name") &&
        outcome.failure.contains("stage_provenance") && outcome.failure.contains("smoother_completed_present") &&
        outcome.failure.at("smoother_budget_expected_s") == kSmootherBudgetSeconds,
        name + ": failure proof is incomplete: " + outcome.failure.dump());
      ensure(outcome.totals.strict_samples == 0 && outcome.totals.strict_cells == 0,
        name + ": strict sweeps ran before the completion gate");
      ensure(outcome.record.is_null() && outcome.totals.by_routing.empty() &&
        outcome.totals.stages == 0 && outcome.totals.cases == 0 && outcome.totals.by_stage.empty() &&
        outcome.totals.by_kind.empty(), name + ": a successful record/acceptance was left behind");
    };

  run_test(result, "smoother_completion_false_is_rejected_before_strict_sweeps", [&]() {
      const Scene scene = validate_scene(fixture);
      const auto path = interpolated_stage_path(scene.cases[0].stages[1], 12);
      // The geometry alone is clear: strict sweeps would have accepted this path.
      const auto accepted = consume(scene, helper_result(true, true), path, path);
      ensure(accepted.accepted && accepted.totals.strict_samples > 0,
        "baseline: the clear finite path should pass when completion is true");
      // Composite success true must not mask completion false.
      for (const bool composite : {true, false}) {
        const auto outcome = consume(scene, helper_result(false, composite), path, path);
        expect_closed(outcome, "smoother_incomplete", "completed=false");
        ensure(outcome.failure.at("smoother_completed_present") == true &&
          outcome.failure.at("smoother_completed_observed") == false &&
          outcome.failure.at("smoother_budget_observed_s") == kSmootherBudgetSeconds &&
          outcome.totals.smoother_incomplete == 1, "incomplete proof/count is wrong");
      }
    });

  run_test(result, "smoother_completion_missing_or_malformed_fails_closed", [&]() {
      const Scene scene = validate_scene(fixture);
      const auto path = interpolated_stage_path(scene.cases[0].stages[1], 12);
      expect_closed(consume(scene, helper_result(missing, true), path, path),
        "smoother_completion_missing", "missing key");
      expect_closed(consume(scene, json(nullptr), path, path),
        "smoother_result_missing_or_not_object", "null result");
      expect_closed(consume(scene, json::array({true}), path, path),
        "smoother_result_missing_or_not_object", "array result");
      expect_closed(consume(scene, json("true"), path, path),
        "smoother_result_missing_or_not_object", "string result");
      for (const json & bad : {json(nullptr), json("true"), json(1), json(1.0), json::array(),
          json::object()})
      {
        expect_closed(consume(scene, helper_result(bad, true), path, path),
          "smoother_completion_not_boolean", "completion " + bad.dump());
      }
    });

  run_test(result, "smoother_completion_true_does_not_bypass_other_strict_gates", [&]() {
      const Scene scene = validate_scene(fixture);
      const auto path = interpolated_stage_path(scene.cases[0].stages[1], 12);
      // Completion alone is not acceptance; the helper composite success is not consulted.
      const auto composite_false = consume(scene, helper_result(true, false), path, path);
      ensure(composite_false.accepted && composite_false.record.at("smoother_completed") == true &&
        composite_false.totals.by_routing.at("direct_precision") == 1 &&
        composite_false.totals.strict_samples > 0 && composite_false.totals.strict_cells > 0 &&
        composite_false.failure.is_null(), "completion=true with a clear path was not accepted");
      json no_success = helper_result(true, true);
      no_success.erase("success");
      ensure(consume(scene, no_success, path, path).accepted, "absent composite success blocked a completed result");

      const auto empty = consume(scene, helper_result(true, true), path, {});
      ensure(!empty.accepted && empty.failure.at("reason") == "empty_smoothed_path" &&
        empty.totals.strict_samples == 0, "empty smoothed path was admitted");
      json wrong_budget = helper_result(true, true);
      wrong_budget["smoother_budget_s"] = 2;
      const auto budget = consume(scene, wrong_budget, path, path);
      ensure(!budget.accepted && budget.failure.at("reason") == "smoother_budget_is_not_1_s" &&
        budget.failure.at("smoother_budget_observed_s") == 2 && budget.totals.strict_samples == 0,
        "wrong smoother budget was admitted");
      json no_budget = helper_result(true, true);
      no_budget.erase("smoother_budget_s");
      ensure(!consume(scene, no_budget, path, path).accepted, "missing smoother budget was admitted");

      Scene blocked = scene;
      const PoseData middle = path[path.size() / 2];
      set_world_cost(blocked.grids[kClearGlobal], middle.x, middle.y, kLethalCost);
      const auto strict = consume(blocked, helper_result(true, true), path, path);
      ensure(!strict.accepted && strict.failure.at("category") == "clearance" &&
        strict.failure.at("routing") == "direct_precision" && strict.record.is_null() &&
        strict.totals.by_routing.empty(), "completion=true bypassed the strict collision gate");
    });

  run_test(result, "smoother_completion_real_configured_smoother_result_feeds_the_seam", [&]() {
      const Scene scene = validate_scene(fixture);
      const StageData & stage = scene.cases[0].stages[1];
      const auto raw = interpolated_stage_path(stage, 20);
      const GridView grid = test_grid("test", 240, 200, -6.0, -5.0);
      ReplayData data = old_checker_data(grid, rectangle);
      data.frame_id = "map";
      data.recorded_plan = json{{"frame_id", "map"}, {"poses", json::array()}};
      data.start = stage.start;
      data.goal = stage.goal;
      RunOptions options;
      options.allow_unknown = false;
      std::vector<PoseData> smoothed;
      const json actual = runConfiguredSmoother(
        data, options, smoother_node, raw, "plugin_raw_path", kSmootherBudgetSeconds, &smoothed);
      ensure(actual.is_object() && actual.contains("smoother_completed") &&
        actual.at("smoother_completed").is_boolean() && actual.at("smoother_completed").get<bool>() &&
        actual.at("smoother_budget_s") == kSmootherBudgetSeconds && !smoothed.empty(),
        "real smoother did not return explicit completion=true with 1 s and a nonempty path: " +
        actual.dump());
      ensure(kDefaultSmootherSeconds == 2, "backend default smoother budget changed from 2 s");
      const auto outcome = consume(scene, actual, raw, smoothed);
      ensure(outcome.accepted && outcome.failure.is_null() &&
        outcome.record.at("smoother_completed") == true &&
        outcome.record.at("smoothed_pose_count") == smoothed.size() &&
        outcome.totals.strict_samples > 0 && outcome.totals.by_routing.at("direct_precision") == 1,
        "actual smoother result was not consumed by the seam");
    });
}

int run_smoother_completion_self_test(const std::shared_ptr<nav2_util::LifecycleNode> & smoother_node)
{
  SelfTestResult result;
  run_smoother_completion_tests(result, smoother_node);
  std::cout << "native45_smoother_completion_self_test (smoother-completion groups only; not the "
            << "full native45_clearance_contract, captured replay or action-server execution): "
            << result.passed << " passed, " << result.failed.size() << " failed\n";
  return result.failed.empty() ? 0 : 1;
}

int run_footprint_self_test()
{
  SelfTestResult result;
  run_footprint_tests(result);
  std::cout << "native45_footprint_self_test (footprint groups only; not the full "
            << "native45_clearance_contract): " << result.passed << " passed, "
            << result.failed.size() << " failed\n";
  return result.failed.empty() ? 0 : 1;
}

int run_planner_map_self_test()
{
  SelfTestResult result;
  run_planner_map_tests(result);
  std::cout << "native45_planner_map_self_test (planner-map groups only; not the full "
            << "native45_clearance_contract, captured replay or smoother execution): "
            << result.passed << " passed, " << result.failed.size() << " failed\n";
  return result.failed.empty() ? 0 : 1;
}

int run_self_test(const std::shared_ptr<nav2_util::LifecycleNode> & smoother_node)
{
  SelfTestResult result;
  const Polygon rectangle{{0.61, 0.41}, {0.61, -0.41}, {-0.61, -0.41}, {-0.61, 0.41}};

  run_test(result, "strict_rejects_empty_path_where_old_checker_accepts", [&]() {
      const GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
      ensure(old_checker_accepts(grid, rectangle, {}), "old checker no longer accepts an empty path");
      const auto verdict = strict_check_path(grid, rectangle, {}, "empty");
      ensure(!verdict.ok && verdict.failure.reason == "empty_path", "strict checker accepted an empty path");
    });

  run_test(result, "strict_rejects_lethal_and_unknown_interior_where_old_checker_accepts", [&]() {
      for (const unsigned char cost : {kLethalCost, kUnknownCost}) {
        GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
        // Interior of the padded rectangle; the center cell and the perimeter stay clear.
        set_world_cost(grid, 0.225, 0.125, cost);
        const std::vector<PoseData> path{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
        ensure(old_checker_accepts(grid, rectangle, path),
          "old perimeter-only checker no longer accepts the known-broken interior baseline");
        const auto verdict = strict_check_path(grid, rectangle, path, "interior");
        ensure(!verdict.ok && verdict.failure.role == "footprint_cell" &&
          verdict.failure.reason == (cost == kUnknownCost ? "unknown_cell" : "lethal_cell") &&
          verdict.failure.cell_x == 44 && verdict.failure.cell_y == 42 &&
          verdict.failure.cost == cost, "strict checker missed the interior cell");
      }
    });

  run_test(result, "strict_rejects_intermediate_turn_collision_with_clear_endpoints", [&]() {
      GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
      set_world_cost(grid, 0.675, 0.175, kLethalCost);
      ensure(strict_check_path(grid, rectangle, {{0.0, 0.0, 0.0}}, "start").ok,
        "start endpoint should be clear");
      ensure(strict_check_path(grid, rectangle, {{0.0, 0.0, M_PI_2}}, "end").ok,
        "terminal endpoint should be clear");
      const auto verdict = strict_check_path(
        grid, rectangle, {{0.0, 0.0, 0.0}, {0.0, 0.0, M_PI_2}}, "turn");
      ensure(!verdict.ok && verdict.failure.reason == "lethal_cell" &&
        verdict.failure.cell_x == 53 && verdict.failure.cell_y == 43 &&
        verdict.failure.sample_index >= 1, "intermediate swept collision was not found");
      const auto sweep = strict_check_heading_sweep(grid, rectangle, 0.0, 0.0, 0.0, M_PI_2, "sweep");
      ensure(!sweep.ok && sweep.failure.reason == "lethal_cell",
        "explicit heading sweep missed the intermediate collision");
    });

  run_test(result, "strict_rejects_outside_map_polygon_without_clipping", [&]() {
      const GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
      const auto verdict = strict_check_path(grid, rectangle, {{2.6, 0.0, 0.0}}, "edge");
      ensure(!verdict.ok && verdict.failure.reason == "outside_map" &&
        verdict.failure.role == "footprint_vertex", "polygon extending outside the map was clipped");
      const auto center = strict_check_path(grid, rectangle, {{5.0, 0.0, 0.0}}, "far");
      ensure(!center.ok && center.failure.reason == "outside_map" && center.failure.role == "center",
        "center outside the map was accepted");
      GridView local = test_grid("test.local", 100, 100, -0.3, -1.0);
      local.frame_id = "odom";
      local.has_transform = true;
      local.map_to_odom_x = 1.0;
      local.map_to_odom_y = 2.0;
      local.map_to_odom_yaw = M_PI_2;
      const PoseData in_grid = to_grid_frame(local, {1.0, 2.5, M_PI_2 + 0.3});
      ensure(std::abs(in_grid.x - 0.5) < 1.0e-12 && std::abs(in_grid.y) < 1.0e-12 &&
        std::abs(in_grid.yaw - 0.3) < 1.0e-12, "map<-odom transform was not inverted");
    });

  run_test(result, "strict_accepts_clear_path_and_perimeter_inscribed_cost", [&]() {
      GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
      // 253 inside the footprint but away from the center is allowed.
      set_world_cost(grid, 0.225, 0.125, kInscribedCost);
      const std::vector<PoseData> path{{0.025, 0.025, 0.0}, {0.325, 0.025, 0.0}, {0.325, 0.025, 0.5}};
      const auto accepted = strict_check_path(grid, rectangle, path, "clear");
      ensure(accepted.ok && accepted.stats.samples > 10 && accepted.stats.cells > 100,
        "clear path with an off-center 253 cell was rejected");
      for (const unsigned char cost : {kInscribedCost, kLethalCost, kUnknownCost}) {
        GridView blocked = test_grid("test", 100, 100, -2.0, -2.0);
        set_world_cost(blocked, 0.025, 0.025, cost);
        const auto verdict = strict_check_path(blocked, rectangle, {{0.025, 0.025, 0.0}}, "center");
        ensure(!verdict.ok && verdict.failure.reason == "center_cost_inscribed_or_worse" &&
          verdict.failure.role == "center", "center cost >= 253 was accepted");
      }
    });

  run_test(result, "strict_and_path_conversion_reject_malformed_poses", [&]() {
      const GridView grid = test_grid("test", 100, 100, -2.0, -2.0);
      const double nan = std::numeric_limits<double>::quiet_NaN();
      ensure(strict_check_path(grid, rectangle, {{0.0, 0.0, 0.0}, {nan, 0.0, 0.0}}, "nan").failure.reason ==
        "nonfinite_pose", "nonfinite path pose accepted");
      ensure(strict_check_heading_sweep(grid, rectangle, 0.0, 0.0, 0.0,
        std::numeric_limits<double>::infinity(), "inf").failure.reason == "nonfinite_pose",
        "nonfinite sweep yaw accepted");
      ensure(strict_check_path(grid, {{0.1, 0.1}, {0.2, 0.2}}, {{0.0, 0.0, 0.0}}, "poly").failure.reason ==
        "bad_polygon", "degenerate polygon accepted");
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      geometry_msgs::msg::PoseStamped pose = stamped_pose({0.0, 0.0, 0.0});
      const auto rejects = [&](const nav_msgs::msg::Path & candidate, const std::string & name) {
          bool rejected = false;
          try {
            poses_from_path(candidate, "map", "test");
          } catch (const SceneError &) {
            rejected = true;
          }
          ensure(rejected, name + " was accepted");
        };
      rejects(path, "empty path");
      path.poses.push_back(pose);
      ensure(poses_from_path(path, "map", "test").size() == 1, "valid path was rejected");
      auto wrong_frame = path;
      wrong_frame.header.frame_id = "odom";
      rejects(wrong_frame, "wrong path frame");
      auto wrong_pose_frame = path;
      wrong_pose_frame.poses[0].header.frame_id = "odom";
      rejects(wrong_pose_frame, "wrong pose frame");
      auto nonfinite = path;
      nonfinite.poses[0].pose.position.x = nan;
      rejects(nonfinite, "nonfinite position");
      auto unnormalized = path;
      unnormalized.poses[0].pose.orientation.w = 2.0;
      rejects(unnormalized, "unnormalized quaternion");
      auto tilted = path;
      tilted.poses[0].pose.orientation.x = 0.5;
      tilted.poses[0].pose.orientation.w = std::sqrt(0.75);
      rejects(tilted, "non-planar quaternion");
    });

  const json fixture = make_fixture_scene();
  run_test(result, "scene_schema_roster_and_stance_validation", [&]() {
      const Scene scene = validate_scene(fixture);
      ensure(scene.cases.size() == 100 && scene.excluded_repairs == 0 && scene.center_slot,
        "valid fixture scene did not validate to the 4/16/80 roster");
      ensure(matches_current_padded_shape(scene.padded_polygon), "padded shape is not +/-0.61 x +/-0.41");
      const auto grid_global = [](json & doc, const std::string & name) -> json & {
          return doc["scenes"][name]["grids"]["global"];
        };
      const auto grid_local = [](json & doc, const std::string & name) -> json & {
          return doc["scenes"][name]["grids"]["local"];
        };
      const auto cases = [](json & doc) -> json::array_t & {
          return doc["cases"].get_ref<json::array_t &>();
        };
      const double nan = std::numeric_limits<double>::quiet_NaN();
      expect_rejected(fixture, [](json & d) {d["schema"] = "other";}, "unsupported schema", "schema");
      expect_rejected(fixture, [&](json & d) {grid_global(d, "clear")["frame_id"] = "odom";},
        "wrong frame_id", "global frame");
      expect_rejected(fixture, [&](json & d) {grid_local(d, "clear")["frame_id"] = "map";},
        "wrong frame_id", "local frame");
      expect_rejected(fixture, [&](json & d) {grid_local(d, "clear")["map_to_odom"] = nullptr;},
        "map_to_odom", "missing local transform");
      expect_rejected(fixture, [&](json & d) {grid_local(d, "pre_dock")["map_to_odom"]["x"] = nan;},
        "not a finite number", "nonfinite transform");
      expect_rejected(fixture, [&](json & d) {grid_local(d, "clear")["map_to_odom"]["stamp_ns"] = 1;},
        "acquisition stamp", "transform stamp");
      expect_rejected(fixture, [&](json & d) {grid_global(d, "clear")["map_to_odom"] = json::object();},
        "must not carry map_to_odom", "global transform");
      expect_rejected(fixture, [&](json & d) {
          grid_global(d, "clear")["data"].get_ref<json::array_t &>().pop_back();},
        "raw cost length", "short raw costs");
      expect_rejected(fixture, [&](json & d) {grid_global(d, "pre_dock")["data"][0] = 256;},
        "outside 0..255", "cost 256");
      expect_rejected(fixture, [&](json & d) {grid_global(d, "pre_dock")["width"] = 0;},
        "malformed grid size", "zero width");
      expect_rejected(fixture, [&](json & d) {grid_global(d, "clear")["resolution"] = nan;},
        "not a finite number", "nonfinite resolution");
      expect_rejected(fixture, [&](json & d) {cases(d).pop_back();}, "roster", "missing case");
      expect_rejected(fixture, [&](json & d) {cases(d)[1]["id"] = cases(d)[0]["id"];},
        "duplicate case id", "duplicate id");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["kind"] = "bias_sensitivity";},
        "roster kind", "wrong kind");
      expect_rejected(fixture, [&](json & d) {
          cases(d)[0]["admitted"] = false;
          cases(d)[0]["admission_reason"] = "registered_approach_distance_exceeded";
          cases(d)[0]["stages"] = json::array();},
        "mandatory nominal case was excluded", "excluded nominal");
      expect_rejected(fixture, [&](json & d) {
          cases(d)[1]["admitted"] = false;
          cases(d)[1]["admission_reason"] = "unexplained";
          cases(d)[1]["stages"] = json::array();},
        "exact admission reason", "inexact repair exclusion");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][0]["name"] = "dock";},
        "stage names", "stage names");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][1]["start"]["x"] = 0.0;},
        "stage continuity", "stage continuity");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][0]["goal"].erase("yaw");},
        "missing key 'yaw'", "missing pose key");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][0]["goal"]["x"] = nan;},
        "not a finite number", "nonfinite stage pose");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][0]["phase"] = "dock";},
        "wrong phase", "wrong phase");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][1]["goal"]["x"] = -3.3;},
        "dock goal", "dock goal");
      expect_rejected(fixture, [&](json & d) {cases(d)[0]["stages"][2]["goal"]["yaw"] = 3.0;},
        "final heading goal", "final heading margin");
      expect_rejected(fixture, [&](json & d) {d["config"]["smoother_budget_s"] = 2;},
        "1 s budget", "2 s budget");
      expect_rejected(fixture, [&](json & d) {d["config"]["simple_smoother"]["w_data"] = 0.3;},
        "unchanged SimpleSmoother", "smoother parameters");
      expect_rejected(fixture, [&](json & d) {d["config"]["padded_footprint"][0][0] = 0.62;},
        "padded footprint", "padded shape");
      expect_rejected(fixture, [&](json & d) {d["config"]["footprint_padding"] = 0.02;},
        "0.01 m", "padding");
      expect_rejected(fixture, [&](json & d) {d["registry"]["selected_slot"][0] = -4.1000001;},
        "final_placement_stance()", "stance disagreement");
      expect_rejected(fixture, [&](json & d) {d["runtime_properties"]["observer_settling"] = "exercised";},
        "must stay not_exercised", "runtime property");
      expect_rejected(fixture, [&](json & d) {d["planning_grid_by_phase"]["dock"] = "clear.global";},
        "planning_grid mapping", "planning grid mapping");
      expect_rejected(fixture, [&](json & d) {d["source_constants"]["kFinalHeadingGoalMargin"] = 0.05;},
        "kFinalHeadingGoalMargin", "heading margin constant");
      // A repair exclusion with an exact reason is accepted and counted.
      json excluded = fixture;
      cases(excluded)[1]["admitted"] = false;
      cases(excluded)[1]["admission_reason"] = "repair_did_not_converge";
      cases(excluded)[1]["stages"] = json::array();
      ensure(validate_scene(excluded).excluded_repairs == 1, "exact repair exclusion was not counted");
    });

  run_test(result, "explicit_one_second_smoother_selection_and_old_default", [&]() {
      static_assert(kSmootherBudgetSeconds == 1, "production mission budget is 1 s");
      ensure(kDefaultSmootherSeconds == 2, "backend default smoother budget changed from 2 s");
      const GridView grid = test_grid("test", 240, 200, -6.0, -5.0);
      ReplayData data = old_checker_data(grid, rectangle);
      data.frame_id = "map";
      data.recorded_plan = json{{"frame_id", "map"}, {"poses", json::array()}};
      std::vector<PoseData> path;
      for (int index = 0; index <= 12; ++index) {
        path.push_back(PoseData{-1.0 + 0.05 * index, 0.0, 0.0});
      }
      RunOptions options;
      options.allow_unknown = false;
      std::vector<PoseData> smoothed;
      const auto defaulted = runConfiguredSmoother(data, options, smoother_node, path, "default");
      ensure(defaulted.at("smoother_budget_s").get<int>() == 2, "default smoother budget is not 2 s");
      const auto explicit_one = runConfiguredSmoother(
        data, options, smoother_node, path, "explicit", kSmootherBudgetSeconds, &smoothed);
      ensure(explicit_one.at("smoother_budget_s").get<int>() == 1, "explicit 1 s was not applied");
      ensure(smoothed.size() == path.size(), "smoothed pose vector was not exposed");
      bool rejected = false;
      try {
        runConfiguredSmoother(data, options, smoother_node, path, "zero", 0);
      } catch (const std::runtime_error &) {
        rejected = true;
      }
      ensure(rejected, "non-positive smoother budget accepted");
    });

  run_test(result, "plugin_replay_passes_fixture_and_reports_exercised_counts", [&]() {
      const Scene scene = validate_scene(fixture);
      const json report = replay_scene(scene, smoother_node, "<fixture>");
      ensure(report.at("status") == "PASS", "fixture replay failed: " + report.at("first_failure").dump());
      const json & exercised = report.at("exercised");
      ensure(exercised.at("cases_exercised") == 100 && exercised.at("stages_exercised") == 300 &&
        exercised.at("cases_by_kind").at("nominal") == 4 &&
        exercised.at("cases_by_kind").at("bias_sensitivity") == 16 &&
        exercised.at("cases_by_kind").at("synthetic_repair") == 80,
        "exercised counts do not cover the 4/16/80 roster");
      ensure(exercised.at("routing").value("direct_precision", 0) == 300,
        "free-space fixture stages did not all route through the direct precision segment");
      ensure(exercised.at("smoother_budget_s") == 1 && report.at("plugin").at("class") == kPluginClass &&
        !report.at("plugin").at("library_path").get<std::string>().empty() &&
        report.at("plugin").at("padded_footprint_consistent_within_allowance") == true &&
        report.at("plugin").at("footprint").at("plugin_scene_consistency_allowance_m") ==
        kPluginSceneConsistencyAllowance,
        "plugin identity/budget/footprint consistency not reported");
      ensure(report.at("runtime_properties").at("observer_settling") == "not_exercised",
        "runtime properties were not preserved");
    });

  run_test(result, "plugin_replay_stops_at_first_interior_failure_with_proof", [&]() {
      json doc = fixture;
      Scene probe = validate_scene(doc);
      const GridView & grid = probe.grids[kClearGlobal];
      // Interior of the first arrival-heading stage footprint (0.2, 0.1 m off the pose); the
      // center cell and perimeter stay clear, so the plugin cannot see it.
      const StageData & first = probe.cases[0].stages[0];
      const double x = first.start.x + 0.2;
      const double y = first.start.y + 0.1;
      const auto column = static_cast<long long>(std::floor((x - grid.origin_x) / grid.resolution));
      const auto row = static_cast<long long>(std::floor((y - grid.origin_y) / grid.resolution));
      doc["scenes"]["clear"]["grids"]["global"]["data"][static_cast<std::size_t>(column) +
        static_cast<std::size_t>(row) * grid.width] = 254;
      const Scene scene = validate_scene(doc);
      const json report = replay_scene(scene, smoother_node, "<fixture>");
      ensure(report.at("status") == "FAIL", "interior obstacle did not fail the replay");
      const json & failure = report.at("first_failure");
      ensure(failure.at("category") == "clearance" && failure.at("reason") == "lethal_cell" &&
        failure.at("role") == "footprint_cell" && failure.at("grid") == "clear.global" &&
        failure.at("case_id") == scene.cases[0].id && failure.at("stage_name") == scene.cases[0].stages[0].name &&
        failure.at("path") == "raw_plugin_path" && failure.at("cell").at("x") == column &&
        failure.at("cell").at("y") == row && failure.contains("pose_map") && failure.contains("pose_grid"),
        "first-failure proof is incomplete: " + failure.dump());
      ensure(report.at("exercised").at("cases_exercised") == 0 &&
        report.at("exercised").at("stages_exercised") == 0, "replay continued after the first failure");
    });

  run_footprint_tests(result);
  run_smoother_completion_tests(result, smoother_node);
  run_planner_map_tests(result);

  std::cout << "native45_clearance_contract: " << result.passed << " passed, "
            << result.failed.size() << " failed\n";
  return result.failed.empty() ? 0 : 1;
}

}  // namespace native45

int main(int argc, char ** argv)
{
  std::string scene_path;
  std::string output_path;
  bool self_test = false;
  bool footprint_self_test = false;
  bool smoother_completion_self_test = false;
  bool planner_map_self_test = false;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    if (arg == "--self-test") {
      self_test = true;
    } else if (arg == "--self-test-footprint") {
      footprint_self_test = true;
    } else if (arg == "--self-test-smoother-completion") {
      smoother_completion_self_test = true;
    } else if (arg == "--self-test-planner-map") {
      planner_map_self_test = true;
    } else if ((arg == "--scene" || arg == "--output") && index + 1 < argc) {
      (arg == "--scene" ? scene_path : output_path) = argv[++index];
    } else {
      std::cerr << "usage: production_precision_replay --scene SCENE.json --output REPORT.json | "
                << "--self-test | --self-test-footprint | --self-test-smoother-completion | "
                << "--self-test-planner-map\n";
      return 1;
    }
  }
  const bool any_self_test =
    self_test || footprint_self_test || smoother_completion_self_test || planner_map_self_test;
  if ((static_cast<int>(self_test) + static_cast<int>(footprint_self_test) +
    static_cast<int>(smoother_completion_self_test) + static_cast<int>(planner_map_self_test) > 1) ||
    any_self_test == (!scene_path.empty() || !output_path.empty()) ||
    (!any_self_test && (scene_path.empty() || output_path.empty())))
  {
    std::cerr << "usage: production_precision_replay --scene SCENE.json --output REPORT.json | "
              << "--self-test | --self-test-footprint | --self-test-smoother-completion | "
              << "--self-test-planner-map\n";
    return 1;
  }
  if (!any_self_test && std::filesystem::exists(output_path)) {
    std::cerr << "refusing to overwrite existing report: " << output_path << "\n";
    return 1;
  }

  rclcpp::init(0, nullptr);
  int status = 1;
  try {
    auto smoother_node = makeNode();
    if (planner_map_self_test) {
      status = native45::run_planner_map_self_test();
    } else if (smoother_completion_self_test) {
      status = native45::run_smoother_completion_self_test(smoother_node);
    } else if (footprint_self_test) {
      status = native45::run_footprint_self_test();
    } else if (self_test) {
      status = native45::run_self_test(smoother_node);
    } else {
      json report;
      try {
        const json doc = native45::load_json_file(scene_path);
        const native45::Scene scene = native45::validate_scene(doc);
        report = native45::replay_scene(scene, smoother_node, scene_path);
      } catch (const native45::SceneError & error) {
        report = native45::failure_report(scene_path, "scene_validation", error.what());
      } catch (const std::exception & error) {
        report = native45::failure_report(scene_path, "internal_error", error.what());
      }
      native45::write_report(output_path, report);
      const bool pass = report.at("status") == "PASS";
      const json & exercised = report.at("exercised");
      std::cout << "production_precision_replay " << report.at("status").get<std::string>()
                << ": cases=" << exercised.value("cases_exercised", 0)
                << " stages=" << exercised.value("stages_exercised", 0) << " report=" << output_path
                << "\n";
      if (!pass) {
        std::cout << "first failure: " << report.at("first_failure").value("reason", std::string("?"))
                  << "\n";
      }
      status = pass ? 0 : 2;
    }
    smoother_node.reset();
  } catch (const std::exception & error) {
    std::cerr << "production_precision_replay failed: " << error.what() << "\n";
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}

import fs from "node:fs/promises";
import path from "node:path";
import { pathToFileURL } from "node:url";
import { Presentation, PresentationFile } from "@oai/artifact-tool";

const workspaceDir = "/home/pete/amr_ws";
const SKILL_DIR = "/home/pete/.codex/plugins/cache/openai-primary-runtime/presentations/26.904.11930/skills/presentations";
const TMP_DIR = path.join(workspaceDir, ".codex-presentation-build", "ros2-full-report");
const FINAL_PPTX = path.join(workspaceDir, "deliverables", "ROS2_Software_Architecture_10min_v2.pptx");
const RUNTIME_PYTHON = "/home/pete/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3.12";
const FONT = "DejaVu Sans";

const { makeNativeBulletParagraphs, finalizePresentation } = await import(
  pathToFileURL(path.join(SKILL_DIR, "container_tools", "artifact_tool_utils.mjs")).href,
);

await fs.mkdir(TMP_DIR, { recursive: true });
await fs.mkdir(path.dirname(FINAL_PPTX), { recursive: true });

const C = {
  white: "#FFFFFF",
  navy: "#102A43",
  blue: "#0077B6",
  cyan: "#00A6D6",
  lightBlue: "#EAF6FC",
  pale: "#F5F8FA",
  line: "#C9D6E2",
  gray: "#536575",
  darkGray: "#334E68",
  green: "#18794E",
  lightGreen: "#EAF7F0",
  amber: "#B96B12",
  lightAmber: "#FFF4E5",
  red: "#B42318",
};

const presentation = Presentation.create({ slideSize: { width: 1280, height: 720 } });

function addText(slide, text, left, top, width, height, opts = {}) {
  const shape = slide.shapes.add({
    geometry: "textbox",
    position: { left, top, width, height },
    fill: opts.fill ?? "none",
    line: { fill: "none", width: 0 },
  });
  shape.text = text;
  shape.text.style = {
    typeface: FONT,
    fontSize: opts.fontSize ?? 24,
    bold: opts.bold ?? false,
    color: opts.color ?? C.navy,
    alignment: opts.alignment ?? "left",
    verticalAlignment: opts.verticalAlignment ?? "top",
    autoFit: opts.autoFit ?? "none",
    lineSpacing: opts.lineSpacing ?? 1.0,
    insets: opts.insets ?? { top: 4, right: 4, bottom: 4, left: 4 },
  };
  return shape;
}

function addBox(slide, text, left, top, width, height, opts = {}) {
  const shape = slide.shapes.add({
    geometry: opts.geometry ?? "rect",
    position: { left, top, width, height },
    fill: opts.fill ?? C.white,
    line: { style: "solid", fill: opts.line ?? C.line, width: opts.lineWidth ?? 1.5 },
    borderRadius: opts.radius ?? 10,
  });
  shape.text = text;
  shape.text.style = {
    typeface: FONT,
    fontSize: opts.fontSize ?? 23,
    bold: opts.bold ?? false,
    color: opts.color ?? C.navy,
    alignment: opts.alignment ?? "center",
    verticalAlignment: opts.verticalAlignment ?? "middle",
    autoFit: opts.autoFit ?? "shrinkText",
    lineSpacing: opts.lineSpacing ?? 0.95,
    insets: opts.insets ?? { top: 8, right: 10, bottom: 8, left: 10 },
  };
  return shape;
}

function addBullets(slide, items, left, top, width, height, opts = {}) {
  const shape = slide.shapes.add({
    geometry: "textbox",
    position: { left, top, width, height },
    fill: "none",
    line: { fill: "none", width: 0 },
  });
  shape.text = makeNativeBulletParagraphs(items, {
    marginLeftPoints: opts.marginLeftPoints ?? 18,
    hangingPoints: opts.hangingPoints ?? 9,
    spaceAfterPoints: opts.spaceAfterPoints ?? 6,
  });
  shape.text.style = {
    typeface: FONT,
    fontSize: opts.fontSize ?? 24,
    color: opts.color ?? C.darkGray,
    autoFit: opts.autoFit ?? "shrinkText",
    verticalAlignment: "top",
    lineSpacing: opts.lineSpacing ?? 1.0,
    insets: { top: 2, right: 4, bottom: 2, left: 4 },
  };
  return shape;
}

function connect(slide, from, to, opts = {}) {
  return slide.shapes.connect(from, to, {
    kind: opts.kind ?? "straight",
    fromSide: opts.fromSide ?? "right",
    toSide: opts.toSide ?? "left",
    line: { style: opts.style ?? "solid", fill: opts.color ?? C.blue, width: opts.width ?? 2.5 },
    tail: { type: opts.head ?? "triangle", width: "sm", length: "sm" },
  });
}

function baseSlide(title, number, subtitle = "") {
  const slide = presentation.slides.add();
  slide.background.fill = C.white;
  slide.shapes.add({
    geometry: "rect",
    position: { left: 0, top: 0, width: 1280, height: 12 },
    fill: C.blue,
    line: { fill: "none", width: 0 },
  });
  addText(slide, title, 64, 34, 1060, 62, { fontSize: 42, bold: true, color: C.navy });
  if (subtitle) addText(slide, subtitle, 66, 92, 1060, 34, { fontSize: 21, color: C.gray });
  addText(slide, String(number).padStart(2, "0"), 1170, 42, 50, 30, {
    fontSize: 18, bold: true, color: C.blue, alignment: "right",
  });
  return slide;
}

function setNotes(slide, script, sources) {
  slide.speakerNotes.textFrame.setText(
    `${script}\n\nSources:\n${sources.map((s) => `- ${s}`).join("\n")}`,
  );
  slide.speakerNotes.setVisible(true);
}

// Slide 1: cover
{
  const slide = presentation.slides.add();
  slide.background.fill = C.white;
  slide.shapes.add({
    geometry: "rect",
    position: { left: 0, top: 0, width: 22, height: 720 },
    fill: C.blue,
    line: { fill: "none", width: 0 },
  });
  addText(slide, "ROS 2 Software Architecture", 92, 105, 1020, 90, {
    fontSize: 58, bold: true, color: C.navy,
  });
  addText(slide, "Autonomous mobile manipulator software stack", 96, 205, 820, 45, {
    fontSize: 28, color: C.blue,
  });
  addText(slide, "ROS 2 Humble  |  Gazebo Harmonic  |  Nav2  |  MoveIt", 96, 270, 930, 40, {
    fontSize: 23, color: C.darkGray,
  });
  addText(slide, "Software scope only", 96, 326, 300, 36, {
    fontSize: 20, bold: true, color: C.gray,
  });
  const labels = ["Sensing", "State", "Planning", "Authority", "Motion"];
  const boxes = labels.map((label, i) => addBox(slide, label, 96 + i * 220, 505, 170, 58, {
    fill: i === 3 ? C.lightBlue : C.pale,
    line: i === 3 ? C.blue : C.line,
    fontSize: 22,
    bold: i === 3,
    color: i === 3 ? C.blue : C.navy,
  }));
  for (let i = 0; i < boxes.length - 1; i += 1) connect(slide, boxes[i], boxes[i + 1]);
  addText(slide, "Typed interfaces connect intent to controlled execution", 96, 595, 1060, 40, {
    fontSize: 24, color: C.gray,
  });
  setNotes(
    slide,
    "Open by defining the software scope. The project uses ROS 2 as a typed distributed graph. Navigation and manipulation request work, while one gated command path decides whether base motion reaches the simulator. Hardware and mechanical design are outside this presentation.",
    ["docs/ROS2_SOFTWARE_PRESENTATION_REPORT.md", "src/README.md"],
  );
}

// Slide 2: package structure
{
  const slide = baseSlide("System structure and package ownership", 2, "17 ROS 2 packages grouped by software responsibility");
  const layers = [
    ["Contracts + policy", "amr_interfaces  |  amr_bringup", C.lightBlue],
    ["Model + simulation boundary", "amr_description  |  amr_simulation", C.pale],
    ["Adapters + state", "amr_sensor_adapters  |  amr_base_adapter  |  amr_localization  |  amr_perception", C.lightBlue],
    ["Mapping + navigation", "amr_slam  |  amr_navigation  |  amr_mpc_controller  |  amr_mission", C.pale],
    ["Authority + application", "amr_control  |  amr_health  |  amr_exploration  |  amr_factory  |  amr_manipulation", C.lightBlue],
  ];
  layers.forEach(([name, packages, fill], i) => {
    const y = 145 + i * 92;
    addBox(slide, name, 70, y, 255, 68, { fill: C.navy, line: C.navy, color: C.white, fontSize: 22, bold: true });
    addBox(slide, packages, 342, y, 610, 68, { fill, line: C.line, color: C.navy, fontSize: 20, alignment: "left" });
  });
  addText(slide, "External stacks", 990, 142, 220, 34, { fontSize: 24, bold: true, color: C.blue });
  addBullets(slide, ["Nav2", "robot_localization", "SLAM Toolbox + AMCL", "MoveIt", "AprilTag ROS", "ros_gz_bridge"], 986, 188, 230, 280, {
    fontSize: 20, spaceAfterPoints: 5,
  });
  addBox(slide, "Legacy source\namr_machine_controller\nnot an active package", 982, 500, 238, 98, {
    fill: C.lightAmber, line: C.amber, color: C.amber, fontSize: 18,
  });
  setNotes(
    slide,
    "Use the layers to connect package names to responsibilities. Packages either define project code and interfaces or configure external ROS 2 components. The orphan amr_machine_controller source has no package manifest or CMake entry, so it does not belong to the active graph.",
    ["src/README.md", "src/amr_bringup/config/interface_ownership.yaml", "docs/ROS2_SOFTWARE_PRESENTATION_REPORT.md sections 4, 5, and 12"],
  );
}

// Slide 3: interfaces
{
  const slide = baseSlide("ROS 2 interfaces and communication", 3, "Names, types, QoS and ownership form the public contract");
  const specs = [
    ["TOPIC", "Continuous stream", "/amr/localization/\nodometry", "nav_msgs/Odometry"],
    ["SERVICE", "Bounded request / response", "/amr/factory/\nset_operation_mode", "amr_interfaces/\nSetOperationMode"],
    ["ACTION", "Goal + feedback + result + cancel", "/amr/factory/\nrun_sequence", "amr_interfaces/RunSequence"],
    ["LIFECYCLE", "Controlled startup", "configure -> inactive -> active", "adapters, estimation, Nav2"],
  ];
  const cards = specs.map(([label, purpose, name, type], i) => {
    const x = 65 + i * 300;
    const box = addBox(slide, "", x, 152, 270, 215, { fill: i % 2 === 0 ? C.lightBlue : C.pale, line: C.line });
    addText(slide, label, x + 18, 170, 230, 30, { fontSize: 20, bold: true, color: C.blue, alignment: "center" });
    addText(slide, purpose, x + 18, 210, 230, 46, { fontSize: 20, bold: true, color: C.navy, alignment: "center", verticalAlignment: "middle" });
    addText(slide, name, x + 18, 264, 230, 58, { fontSize: 16, color: C.darkGray, alignment: "center", verticalAlignment: "middle", autoFit: "shrinkText" });
    addText(slide, type, x + 18, 320, 230, 38, { fontSize: 15, color: C.gray, alignment: "center", verticalAlignment: "middle", autoFit: "shrinkText" });
    return box;
  });
  addText(slide, "DDS discovery + compatible QoS", 430, 392, 420, 34, { fontSize: 25, bold: true, color: C.blue, alignment: "center" });
  slide.shapes.add({ geometry: "line", position: { left: 90, top: 438, width: 1100, height: 0 }, fill: "none", line: { style: "solid", fill: C.blue, width: 2 } });
  addText(slide, "Project status", 72, 468, 210, 32, { fontSize: 22, bold: true, color: C.navy });
  addText(slide, "BaseStatus  |  HealthStatus  |  ManipulatorStatus  |  FactoryStatus", 72, 506, 1110, 36, { fontSize: 21, color: C.darkGray });
  addText(slide, "Standard data", 72, 558, 210, 32, { fontSize: 22, bold: true, color: C.navy });
  addText(slide, "Twist  |  TwistStamped  |  Odometry  |  LaserScan  |  PointCloud2  |  Imu  |  OccupancyGrid  |  Path", 72, 596, 1120, 54, { fontSize: 19, color: C.darkGray, autoFit: "shrinkText" });
  setNotes(
    slide,
    "Explain why the project uses each interface style. Topics carry streams, services handle short requests, and actions retain state for long-running work. DDS matches endpoint names and types, while QoS controls reliability, durability, queue depth, deadline and lifespan. Status messages also carry boot IDs, sequences and validity so consumers can reject stale evidence.",
    ["src/amr_interfaces", "src/amr_interfaces/include/amr_interfaces/qos_profiles.hpp", "src/amr_bringup/config/interface_ownership.yaml"],
  );
}

// Slide 4: simulation and visualization
{
  const slide = baseSlide("Gazebo, ros_gz_bridge, RViz2 and adapters", 4, "Simulation-specific endpoints become stable project interfaces");
  const gazebo = addBox(slide, "GAZEBO HARMONIC\nworld + sensors + joints + odometry + base endpoint", 65, 200, 245, 130, {
    fill: C.navy, line: C.navy, color: C.white, fontSize: 22, bold: true,
  });
  const bridge = addBox(slide, "ros_gz_bridge\nmessage conversion", 360, 215, 200, 100, {
    fill: C.lightBlue, line: C.blue, color: C.blue, fontSize: 22, bold: true,
  });
  const adapters = addBox(slide, "Lifecycle adapters\n/amr/sensors/*\n/amr/base/*", 620, 195, 210, 140, {
    fill: C.pale, line: C.line, color: C.navy, fontSize: 22, bold: true,
  });
  const consumers = addBox(slide, "Consumers\nlocalization | perception\nSLAM | Nav2 | AprilTag", 900, 195, 300, 140, {
    fill: C.lightBlue, line: C.blue, color: C.navy, fontSize: 21, bold: true,
  });
  connect(slide, gazebo, bridge);
  connect(slide, bridge, adapters);
  connect(slide, adapters, consumers);
  const rviz = addBox(slide, "RViz2\nmap | TF | RobotModel | scans | clouds | paths\nvisualization only", 835, 455, 365, 120, {
    fill: C.white, line: C.cyan, color: C.navy, fontSize: 21, bold: true,
  });
  connect(slide, consumers, rviz, { kind: "elbow", fromSide: "bottom", toSide: "top", color: C.cyan, style: "dashed" });
  addBox(slide, "Product camera adapter\nexact image / CameraInfo timestamp match\n640 x 480", 360, 450, 380, 125, {
    fill: C.pale, line: C.line, color: C.darkGray, fontSize: 20,
  });
  addText(slide, "Factory acceptance runs headless; sensors.rviz is optional", 66, 624, 1110, 32, { fontSize: 20, color: C.gray });
  setNotes(
    slide,
    "Gazebo publishes native simulation endpoints. ros_gz_bridge converts them into ROS 2 messages. The project adapters then create stable names under /amr so downstream nodes do not depend on the simulator's native names. RViz2 subscribes to graph data for inspection and has no accepted velocity authority.",
    ["src/amr_simulation/launch/amr_simulation.launch.py", "src/amr_simulation/rviz/sensors.rviz", "src/amr_sensor_adapters", "src/amr_factory/launch/factory_localization.launch.py"],
  );
}

// Slide 5: localization and TF
{
  const slide = baseSlide("Localization and TF ownership", 5, "Sensor fusion estimates motion; TF connects global, local and sensor frames");
  addText(slide, "ESTIMATION", 70, 140, 220, 30, { fontSize: 20, bold: true, color: C.blue });
  const joints = addBox(slide, "wheel joint states", 70, 190, 200, 62, { fill: C.pale, fontSize: 20 });
  const wheel = addBox(slide, "wheel_odometry_node\n/amr/localization/\nwheel_odometry", 330, 170, 275, 102, { fill: C.lightBlue, line: C.blue, fontSize: 18, bold: true });
  const imu = addBox(slide, "IMU\n/amr/sensors/imu/\ndata_raw", 330, 300, 275, 92, { fill: C.pale, fontSize: 18 });
  const ekf = addBox(slide, "robot_localization EKF\n30 Hz | two-dimensional mode", 665, 225, 255, 105, { fill: C.navy, line: C.navy, color: C.white, fontSize: 21, bold: true });
  const odomOut = addBox(slide, "/amr/localization/\nodometry\nodom -> base_footprint", 980, 220, 230, 115, { fill: C.lightGreen, line: C.green, color: C.green, fontSize: 18, bold: true });
  connect(slide, joints, wheel);
  connect(slide, wheel, ekf);
  connect(slide, imu, ekf, { kind: "elbow", fromSide: "right", toSide: "bottom" });
  connect(slide, ekf, odomOut, { color: C.green });

  addText(slide, "TF TREE", 70, 438, 220, 30, { fontSize: 20, bold: true, color: C.blue });
  const map = addBox(slide, "map", 70, 492, 130, 54, { fill: C.navy, line: C.navy, color: C.white, fontSize: 22, bold: true });
  const odom = addBox(slide, "odom", 270, 492, 130, 54, { fill: C.lightBlue, line: C.blue, fontSize: 22, bold: true });
  const footprint = addBox(slide, "base_footprint", 470, 492, 190, 54, { fill: C.pale, line: C.line, fontSize: 20, bold: true });
  const base = addBox(slide, "base_link", 730, 492, 150, 54, { fill: C.pale, line: C.line, fontSize: 21, bold: true });
  const sensors = addBox(slide, "lidars | IMU | camera | arm", 950, 492, 260, 54, { fill: C.lightBlue, line: C.blue, fontSize: 19, bold: true });
  connect(slide, map, odom);
  connect(slide, odom, footprint);
  connect(slide, footprint, base);
  connect(slide, base, sensors);
  addText(slide, "SLAM Toolbox (mapping)\nor AMCL (factory)", 95, 560, 280, 62, { fontSize: 17, color: C.gray, alignment: "center" });
  addText(slide, "EKF", 505, 560, 120, 32, { fontSize: 18, color: C.green, bold: true, alignment: "center" });
  addText(slide, "robot_state_publisher (static tree)", 740, 560, 440, 32, { fontSize: 18, color: C.gray, alignment: "center" });
  setNotes(
    slide,
    "Wheel odometry integrates named wheel joint positions, while the EKF fuses wheel velocity and IMU information. The EKF is the only owner of odom to base_footprint. Mapping mode uses SLAM Toolbox for map to odom; factory localization uses AMCL. robot_state_publisher creates the fixed body and sensor tree from URDF.",
    ["src/amr_localization/src/wheel_odometry_node.cpp", "src/amr_localization/config/ekf.yaml", "src/amr_description/urdf/amr.urdf.xacro", "src/amr_bringup/config/interface_ownership.yaml"],
  );
}

// Slide 6: Nav2
{
  const slide = baseSlide("Nav2 navigation pipeline", 6, "Mission intent becomes a bounded controller request");
  const mission = addBox(slide, "Mission supervisor\nvalidates planar goal", 55, 235, 210, 105, { fill: C.navy, line: C.navy, color: C.white, fontSize: 20, bold: true });
  const planner = addBox(slide, "NavFn planner\nglobal path", 295, 235, 210, 105, { fill: C.lightBlue, line: C.blue, fontSize: 20, bold: true });
  const smoother = addBox(slide, "Simple Smoother\nrefined path", 535, 235, 210, 105, { fill: C.pale, line: C.line, fontSize: 20, bold: true });
  const controller = addBox(slide, "RPP controller\npath following", 775, 235, 210, 105, { fill: C.lightBlue, line: C.blue, fontSize: 20, bold: true });
  const command = addBox(slide, "/amr/mpc/\ncmd_vel\ngeometry_msgs/Twist", 1015, 235, 210, 105, { fill: C.lightGreen, line: C.green, color: C.green, fontSize: 17, bold: true });
  connect(slide, mission, planner);
  connect(slide, planner, smoother);
  connect(slide, smoother, controller);
  connect(slide, controller, command, { color: C.green });
  const mapInput = addBox(slide, "Map + global costmap", 280, 145, 245, 58, { fill: C.white, line: C.cyan, color: C.blue, fontSize: 19, bold: true });
  const cloudInput = addBox(slide, "front + rear PointCloud2", 755, 145, 250, 58, { fill: C.white, line: C.cyan, color: C.blue, fontSize: 19, bold: true });
  connect(slide, mapInput, planner, { kind: "elbow", fromSide: "bottom", toSide: "top", color: C.cyan });
  connect(slide, cloudInput, controller, { kind: "elbow", fromSide: "bottom", toSide: "top", color: C.cyan });
  addText(slide, "Current controller profile", 72, 430, 340, 34, { fontSize: 25, bold: true, color: C.navy });
  const metrics = [
    ["0.5 m/s", "desired speed"],
    ["0.3–0.9 m", "lookahead range"],
    ["1.5 s", "lookahead time"],
    ["1.2 x 0.8 m", "software footprint"],
  ];
  metrics.forEach(([value, label], i) => {
    const x = 72 + i * 290;
    addText(slide, value, x, 490, 250, 48, { fontSize: 31, bold: true, color: C.blue, alignment: "center" });
    addText(slide, label, x, 540, 250, 34, { fontSize: 19, color: C.gray, alignment: "center" });
  });
  addText(slide, "amr_mpc_controller is compatibility naming; the active algorithm is Regulated Pure Pursuit", 72, 620, 1110, 34, { fontSize: 20, color: C.amber });
  setNotes(
    slide,
    "The mission supervisor exposes the project navigation actions and coordinates Nav2 clients. NavFn computes the path, Simple Smoother refines it, and RPP produces the upstream Twist request. Costmaps combine the map, robot footprint and front/rear perception data. This is a request path, not the final authority path.",
    ["src/amr_mission/src/mission_supervisor_node.cpp", "src/amr_navigation/config/planner.yaml", "src/amr_mpc_controller/config/controller.yaml"],
  );
}

// Slide 7: MoveIt and factory
{
  const slide = baseSlide("MoveIt and factory orchestration", 7, "Factory actions compose navigation and manipulation into a product cycle");
  const operator = addBox(slide, "Operator / CLI", 55, 215, 175, 86, { fill: C.pale, fontSize: 21, bold: true });
  const factory = addBox(slide, "Factory supervisor\nregistry + queue + status", 280, 200, 225, 116, { fill: C.navy, line: C.navy, color: C.white, fontSize: 21, bold: true });
  const mission = addBox(slide, "Mission + Nav2\nstation navigation", 555, 200, 200, 116, { fill: C.lightBlue, line: C.blue, fontSize: 21, bold: true });
  const cycle = addBox(slide, "Cycle supervisor\nExecuteProductCycle", 805, 200, 210, 116, { fill: C.pale, line: C.line, fontSize: 20, bold: true });
  const moveit = addBox(slide, "MoveIt move_group\narm planning + execution", 1065, 200, 170, 116, { fill: C.lightBlue, line: C.blue, fontSize: 18, bold: true });
  connect(slide, operator, factory);
  connect(slide, factory, mission);
  connect(slide, factory, cycle, { kind: "elbow", fromSide: "bottom", toSide: "bottom" });
  connect(slide, cycle, moveit);
  addText(slide, "Public factory interfaces", 70, 386, 410, 38, { fontSize: 24, bold: true, color: C.navy });
  addText(slide, "TransportProduct  |  RunSequence  |  NavigateStation  |  SetOperationMode  |  stop / cancel", 70, 430, 1120, 38, { fontSize: 19, color: C.darkGray, autoFit: "shrinkText" });
  addText(slide, "Evidence gates", 70, 500, 270, 38, { fontSize: 24, bold: true, color: C.navy });
  addText(slide, "tag identity + five stable observations + contact freshness + pose tolerance + attachment state", 70, 542, 1120, 38, { fontSize: 19, color: C.darkGray, autoFit: "shrinkText" });
  addBox(slide, "Autonomous scope\nProduct 101 + Product 102", 70, 610, 360, 70, { fill: C.lightGreen, line: C.green, color: C.green, fontSize: 21, bold: true });
  addBox(slide, "Product 103 remains disabled", 850, 610, 365, 70, { fill: C.lightAmber, line: C.amber, color: C.amber, fontSize: 21, bold: true });
  setNotes(
    slide,
    "The factory supervisor loads the product and station registries, validates them and owns the high-level action results. Navigation reaches stations through the mission boundary. The cycle supervisor coordinates the manipulation stage, and MoveIt plans arm trajectories. Evidence gates prevent a cycle from accepting inconsistent tag, contact, pose or attachment data.",
    ["src/amr_factory/src/factory_supervisor_node.cpp", "src/amr_factory/config/products.yaml", "src/amr_factory/config/stations.yaml", "src/amr_manipulation/scripts/cycle_manipulation_supervisor.py", "src/amr_manipulation/launch/move_group.launch.py"],
  );
}

// Slide 8: authority
{
  const slide = baseSlide("Command authority and fail-closed motion", 8, "Every layer must prove that a velocity request remains admissible");
  const nav = addBox(slide, "Nav2 request\nTwist", 45, 235, 155, 92, { fill: C.pale, fontSize: 20, bold: true });
  const arb = addBox(slide, "COMMAND ARBITRATION\nfreshness | planar validity\nmanipulator permission\nbase readiness | limits", 230, 190, 290, 180, { fill: C.navy, line: C.navy, color: C.white, fontSize: 19, bold: true });
  const stamped = addBox(slide, "/amr/control/\ncmd_vel\nTwistStamped", 550, 230, 220, 102, { fill: C.lightGreen, line: C.green, color: C.green, fontSize: 18, bold: true });
  const adapter = addBox(slide, "Base adapter\nrevalidate + zero stale", 800, 225, 210, 112, { fill: C.lightBlue, line: C.blue, fontSize: 19, bold: true });
  const plant = addBox(slide, "Bridge + Gazebo\nwatchdog expiry", 1040, 225, 190, 112, { fill: C.pale, line: C.line, fontSize: 18, bold: true });
  connect(slide, nav, arb);
  connect(slide, arb, stamped, { color: C.green });
  connect(slide, stamped, adapter, { color: C.green });
  connect(slide, adapter, plant);
  addText(slide, "Sole project-owned velocity publisher", 245, 390, 275, 34, { fontSize: 18, bold: true, color: C.blue, alignment: "center" });
  const metrics = [
    ["20 Hz", "arbitration output"],
    ["200 ms", "command timeout + lifespan"],
    ["0.5 m/s", "linear limit"],
    ["0.4 rad/s", "angular limit"],
    ["0.025 m/s", "nominal change per tick"],
  ];
  metrics.forEach(([value, label], i) => {
    const x = 55 + i * 240;
    addText(slide, value, x, 495, 215, 48, { fontSize: 28, bold: true, color: C.blue, alignment: "center" });
    addText(slide, label, x, 545, 215, 42, { fontSize: 17, color: C.gray, alignment: "center", autoFit: "shrinkText" });
  });
  addBox(slide, "Missing, stale, invalid or contradictory evidence stops or rejects motion", 125, 620, 1030, 58, { fill: C.lightAmber, line: C.amber, color: C.amber, fontSize: 22, bold: true });
  setNotes(
    slide,
    "Separate planning from authority. Nav2 publishes an upstream request. The arbitration node checks freshness, semantics, interlocks and rate limits, then publishes the only project-owned accepted base velocity. The base adapter performs another finite and planar check. The Gazebo watchdog provides independent native command expiry. Health status only reports observations and cannot enable motion.",
    ["src/amr_control/src/command_arbitration_node.cpp", "src/amr_base_adapter/src/base_adapter_node.cpp", "src/amr_simulation/src/command_watchdog_system.cpp", "src/amr_interfaces/include/amr_interfaces/qos_profiles.hpp"],
  );
}

// Slide 9: calculations
{
  const slide = baseSlide("Theory and project calculations", 9, "Current software parameters explain odometry, timing and navigation geometry");
  addBox(slide, "DIFFERENTIAL DRIVE", 70, 145, 530, 58, { fill: C.navy, line: C.navy, color: C.white, fontSize: 22, bold: true });
  addText(slide, "v = r/2 (omega_L + omega_R)\nomega = r/L (omega_R - omega_L)", 92, 235, 490, 98, { fontSize: 29, bold: true, color: C.navy, alignment: "center", verticalAlignment: "middle" });
  addText(slide, "r = 0.1128 m     L = 0.566 m", 115, 340, 445, 40, { fontSize: 23, color: C.blue, alignment: "center" });
  addBox(slide, "Equal +1 rad wheel motion\nDelta s = 0.1128 m\nDelta theta = 0", 85, 425, 230, 120, { fill: C.lightBlue, line: C.blue, fontSize: 19, bold: true });
  addBox(slide, "Opposite +/-1 rad motion\nDelta s = 0\nDelta theta = 0.3986 rad", 350, 425, 230, 120, { fill: C.pale, line: C.line, fontSize: 19, bold: true });
  addText(slide, "Source: diff_drive.hpp", 120, 580, 430, 30, { fontSize: 17, color: C.gray, alignment: "center" });

  addText(slide, "NAVIGATION GEOMETRY", 675, 145, 490, 40, { fontSize: 25, bold: true, color: C.navy });
  addBox(slide, "100 x 100 cells", 675, 215, 210, 76, { fill: C.lightBlue, line: C.blue, color: C.blue, fontSize: 26, bold: true });
  addText(slide, "5 m local costmap / 0.05 m resolution", 900, 225, 305, 55, { fontSize: 19, color: C.darkGray, verticalAlignment: "middle" });
  addBox(slide, "11 cells", 675, 325, 210, 76, { fill: C.pale, line: C.line, color: C.navy, fontSize: 26, bold: true });
  addText(slide, "0.55 m inflation / 0.05 m resolution", 900, 335, 305, 55, { fontSize: 19, color: C.darkGray, verticalAlignment: "middle" });
  addBox(slide, "0.75 m", 675, 435, 210, 76, { fill: C.lightGreen, line: C.green, color: C.green, fontSize: 26, bold: true });
  addText(slide, "RPP: 0.5 m/s x 1.5 s lookahead time", 900, 445, 305, 55, { fontSize: 19, color: C.darkGray, verticalAlignment: "middle" });
  addBox(slide, "0.025 m/s per tick", 675, 545, 210, 76, { fill: C.lightAmber, line: C.amber, color: C.amber, fontSize: 22, bold: true });
  addText(slide, "0.5 m/s² acceleration x 0.05 s at 20 Hz", 900, 555, 305, 55, { fontSize: 19, color: C.darkGray, verticalAlignment: "middle" });
  setNotes(
    slide,
    "Connect each equation to a software component. Differential-drive midpoint integration produces wheel odometry. Costmap resolution determines memory geometry and inflation width. RPP scales lookahead with speed. The arbitration timer and acceleration limit determine the nominal per-tick command change. These are explanatory calculations from current parameters, not new thresholds.",
    ["src/amr_localization/include/amr_localization/diff_drive.hpp", "src/amr_navigation/config/planner.yaml", "src/amr_mpc_controller/config/controller.yaml", "src/amr_control/src/command_arbitration_node.cpp"],
  );
}

// Slide 10: state and limitations
{
  const slide = baseSlide("Current software state", 10, "Implemented source and remaining runtime evidence are separate claims");
  addText(slide, "IMPLEMENTED IN SOURCE", 80, 145, 480, 38, { fontSize: 24, bold: true, color: C.green });
  addText(slide, "PENDING EVIDENCE", 710, 145, 460, 38, { fontSize: 24, bold: true, color: C.amber });
  addBox(slide, "", 70, 195, 535, 335, { fill: C.lightGreen, line: C.green });
  addBullets(slide, [
    "Typed public interfaces and ownership registry",
    "Adapters, localization, perception and TF model",
    "Nav2 RPP pipeline and command arbitration",
    "Factory cycles for Product 101 and 102",
    "Fail-closed freshness and interlock gates",
  ], 92, 220, 490, 280, { fontSize: 23, color: C.navy, spaceAfterPoints: 10 });
  addBox(slide, "", 685, 195, 525, 335, { fill: C.lightAmber, line: C.amber });
  addBullets(slide, [
    "Fresh Phase 15 mapping runtime proof",
    "Live per-edge TF ownership and freshness proof",
    "Quantitative map-quality acceptance",
    "Candidate-map promotion and canonical replacement",
    "Hardware and functional-safety acceptance",
  ], 710, 220, 475, 280, { fontSize: 23, color: C.navy, spaceAfterPoints: 10 });
  addBox(slide, "Contract gap: /amr/mission/navigate_to_pose_retreat exists in source but is absent from the ownership registry", 135, 552, 1010, 58, { fill: C.pale, line: C.line, color: C.darkGray, fontSize: 19, bold: true });
  addText(slide, "Typed interfaces + explicit ownership + gated motion", 190, 625, 900, 46, { fontSize: 27, bold: true, color: C.blue, alignment: "center", verticalAlignment: "middle" });
  setNotes(
    slide,
    "Close by distinguishing implementation from runtime proof. The source tree contains the architecture and current Product 101/102 factory scope. Phase 15 still needs a fresh runtime mapping run, per-edge TF evidence, map-quality acceptance and explicit promotion. The ownership registry also needs the source-defined retreat action. Do not claim hardware or functional-safety acceptance.",
    ["SESSION_HANDOFF.md", "docs/PHASE_15_FACTORY_SLAM_COMMISSIONING.md", "src/amr_bringup/config/interface_ownership.yaml", "src/amr_mission/src/mission_supervisor_node.cpp"],
  );
}

const requirements = {
  explicitTotalSlideCount: 10,
  requiredNativeTableOwnerSlides: [],
  requiredNativeChartOwnerSlides: [],
};
const fontPolicy = { basis: "design", families: [FONT] };
const stagingDir = path.join(workspaceDir, ".codex-finalizer");
await fs.mkdir(stagingDir, { recursive: true });
const candidatePath = path.join(stagingDir, "ros2-software-architecture-candidate.pptx");
await (await PresentationFile.exportPptx(presentation)).save(candidatePath);

const result = await finalizePresentation({
  ...requirements,
  workspaceDir,
  candidatePath,
  finalPath: FINAL_PPTX,
  pythonExecutable: RUNTIME_PYTHON,
  integrityValidatorPath: path.join(SKILL_DIR, "container_tools", "inspect_presentation_package_integrity.py"),
  layoutValidatorPath: path.join(SKILL_DIR, "container_tools", "inspect_presentation_layout_geometry.py"),
  layoutArgs: [
    "--expected-slide-size-emu", "12192000,6858000",
    "--validate-bullet-geometry",
    "--validate-heading-fit",
  ],
  requiredNativeTableOwnerSlides: [],
  fontPolicy,
  verifyArtifactToolImport: true,
  receiptPath: path.join(stagingDir, "ROS2_Software_Architecture_10min_v2.validation.json"),
});

console.log(JSON.stringify({ final: FINAL_PPTX, result }, null, 2));

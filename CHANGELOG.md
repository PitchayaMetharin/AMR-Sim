# CHANGELOG.md

All significant project changes must be recorded here.

## Unreleased

### Added
- Phase-gated engineering workflow.
- Requirement to stop after every phase and await approval.
- `PROJECT_STATUS.md`, `CHANGELOG.md`, and `TODO.md` repository records.
- MPC selected as the local path-tracking controller.
- Dual SICK MRS1000 perception architecture.
- Phase 0 requirements and architecture baseline.
- Comprehensive robot parameter register covering geometry, mass, running gear,
  sensors, motion limits, control, compute, PLC, safety, networking, power,
  environment, and validation targets.
- Full-project acceptance framework and hardware evidence gate.
- Phase 0 BOM quality and architecture-alignment review.
- C++ selected as the primary production implementation language.
- Two SICK MRS1000 lines added to the BOM and subsequently resolved to the exact
  MRS1104C-111011 / 1081208 ordering code and official electrical values.
- Initial payload, motion, PLC/ROS responsibility, mission, endurance,
  availability, and navigation-accuracy requirements.
- Phase 0 software baseline with inspected workstation evidence.
- Simulation-only safety scope and official standards reference set.
- Staged simulation risk-reduction plan separating current environment checks,
  later robot-model tests, optional payload escalation, and prohibited physical
  safety claims.
- Phase 0 environment and integration test report covering host capacity,
  C++17/ament, ROS 2/Fast DDS, Gazebo transport/control/GUI, RViz rendering,
  SDFormat inertia, required packages, and remaining blockers.
- Root-level new-session handoff containing phase state, frozen requirements,
  installed environment, validation evidence, deferred inputs, and exact next
  actions.

### Changed
- Project-agent role changed from teaching assistant to lead robotics engineer.
- Navigation LiDAR changed from SICK LMS151 to 2 × SICK MRS1000.
- Local controller changed from Regulated Pure Pursuit to MPC.
- Mechanical CAD removed from Codex scope.
- Project status advanced to Phase 0 review, with unresolved values explicitly
  prevented from becoming implementation assumptions.
- BOM, power-budget, communication-matrix, and design-note content updated to
  remove the obsolete LMS151/outdoorScan3 sensing plan.
- LiDAR selection frozen to 2 × SICK MRS1104C-111011, order number 1081208.
- Software baseline frozen to ROS 2 Humble, Ubuntu 22.04, and C++17 minimum.
- Unloaded mass resolved to approximately 30 kg.
- Caster quantity changed from two to four TENTE LEVINA 5370PJP100P62 units.
- Provisional nominal drive-wheel radius set to 0.127 m from the verified
  manufacturer-stated 10-inch diameter.
- Initial fleet integration deferred; local single-robot mission interfaces
  retained with future integration boundaries.
- Current project scope changed to laptop-based simulation only; all listed
  physical hardware is conceptual future-reference equipment.
- Gazebo Harmonic selected as the Phase 6 simulator.
- Default and initially rated simulated payload confirmed as 50 kg, with
  approximately 80 kg initial total moving mass.
- Payload retained as a manual Xacro/launch parameter before model spawn;
  live in-session adjustment is not required.
- A 300 kg payload retained only as an optional future simulation stress case
  and physical design target, not the current rating.
- Payload configuration required to maintain consistent mass,
  center-of-gravity, collision, and inertia properties.
- Gazebo Harmonic 8.14.0 installation verified through a headless empty-world
  smoke test; the explicit Humble/Harmonic integration was subsequently
  installed and validated.
- C++17/ament build and runtime, Fast DDS 6/6 delivery, Gazebo world control,
  Gazebo GUI/OGRE2, RViz OpenGL 4.6, and analytical inertia checks passed.
- Exact `ros-humble-ros-gzharmonic` installation path dry-run completed with
  no removals or upgrades; joint-state publisher package gaps recorded.
- Installed the explicit Humble/Harmonic integration and joint-state publisher
  packages after user authorization.
- Verified end-to-end Gazebo-to-ROS `/clock` delivery plus LaserScan,
  PointCloud2, and IMU bridge mappings.
- Confirmed that Phase 6 will use parameterized primitive geometry and does not
  depend on completed mechanical CAD.
- Unloaded mass tolerance set to ±5 kg around the 30 kg nominal value.
- Thailand selected as the future jurisdiction, with ISO 3691-4:2023,
  ISO 12100:2010, ISO 13849-1:2023, and IEC 60204-1:2016+A1:2021 as conceptual
  international references.
- PL d, Category 3 recorded only as a provisional conceptual target; no
  compliance claim is made.

### Removed
- SICK outdoorScan3 safety LiDAR.
- PROFIsafe field-switching requirements tied specifically to outdoorScan3.
- Teaching-mode behavior.

### Safety Note
- SLAM, Nav2 costmaps, and standard LiDAR obstacle detection must not be described as a certified personnel-safety system.

### Resolved Requirement Conflict
- The current default and initially rated simulated payload is 50 kg, giving
  approximately 80 kg total moving mass at nominal unloaded mass. The user may
  manually override the payload before later simulation runs. A 300 kg case is
  optional and unvalidated.

### Verification Note
- The current BOM still contains shifted supplier/shop/price fields in later
  rows. Correction is deferred because there is no current procurement.

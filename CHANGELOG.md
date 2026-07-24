# CHANGELOG.md

All significant project changes must be recorded here.

## Unreleased

### Added
- Phase-gated engineering workflow.
- Requirement to stop after every phase and await approval.
- `PROJECT_STATUS.md`, `CHANGELOG.md`, and `TODO.md` repository records.
- MPC selected as the local path-tracking controller.
- Dual SICK MRS1000 perception architecture.

### Changed
- Project-agent role changed from teaching assistant to lead robotics engineer.
- Navigation LiDAR changed from SICK LMS151 to 2 × SICK MRS1000.
- Local controller changed from Regulated Pure Pursuit to MPC.
- Mechanical CAD removed from Codex scope.

### Removed
- SICK outdoorScan3 safety LiDAR.
- PROFIsafe field-switching requirements tied specifically to outdoorScan3.
- Teaching-mode behavior.

### Safety Note
- SLAM, Nav2 costmaps, and standard LiDAR obstacle detection must not be described as a certified personnel-safety system.

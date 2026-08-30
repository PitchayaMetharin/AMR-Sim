# Phase 8 Test Report

## Result

Phase 8 simulation-only LiDAR perception implementation and validation passed.
It was approved and closed by the user; no local commit is authorized.

## Automated validation

The complete workspace built successfully with all nine packages. The complete
test result reported 54 tests, zero errors, zero failures, and zero skips.
This includes PointCloud2 layout unit tests, perception ownership/launch
contract tests, and the reusable live fault-acceptance script contract test.

## Live Gazebo validation

In a headless Gazebo launch, the two nominal pipelines published independent
fresh clouds with their source frames preserved:

| Output | Frame | Layout |
| --- | --- | --- |
| `/amr/perception/front_lidar/points` | `front_lidar_link` | 4 x 720 PointCloud2 with XYZ fields |
| `/amr/perception/rear_lidar/points` | `rear_lidar_link` | 4 x 720 PointCloud2 with XYZ fields |

The reusable `perception_fault_acceptance` ran against a separate active
lifecycle instance using the Gazebo simulation clock. It passed after proving
that malformed-layout, one-second-stale, and backward-time clouds produced no
output, while one fresh cloud was forwarded exactly once.

The acceptance harness was corrected to use simulation time after its initial
wall-clock timestamps were correctly rejected as future data. This was a test
harness defect, not an accepted pipeline input or a change to pipeline
authority.

## Scope boundary

This evidence validates only simulated PointCloud2 preparation and inhibition
at this boundary. It does not validate obstacle semantics, maps, localization,
navigation, motion, personnel protection, physical sensor behavior, or
functional safety.

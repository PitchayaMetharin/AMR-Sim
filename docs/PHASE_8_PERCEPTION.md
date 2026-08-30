# Phase 8 Perception

## Scope and authority

Phase 8 adds two independent, simulation-only LiDAR preparation pipelines.
Each consumes the corresponding adapted `PointCloud2` topic and publishes a
validated navigation-perception point cloud. The pipeline is neither a map,
localization, navigation, motion, nor permission authority. It publishes no
TF and cannot make personnel-safety claims.

## Input and output contract

| Pipeline | Input | Output | Frame rule |
| --- | --- | --- | --- |
| Front | `/amr/sensors/front_lidar/points` | `/amr/perception/front_lidar/points` | Preserve the accepted input frame and stamp. |
| Rear | `/amr/sensors/rear_lidar/points` | `/amr/perception/rear_lidar/points` | Preserve the accepted input frame and stamp. |

The inputs originate at the Phase 5 sensor-adapter boundary. A pipeline accepts
only a non-empty frame id, a non-zero stamp, valid PointCloud2 layout, and a
fresh message. Invalid, malformed, stale, or backward-time input is dropped;
it is never republished as usable perception. Freshness uses ROS time so that
the simulation clock is authoritative.

The output intentionally remains separate by sensor. Phase 9 SLAM or Phase 10
navigation may choose their own transform, aggregation, and obstacle policy
only when authorized. This phase does not invent a fused obstacle model,
sensor noise model, occupancy semantics, or safety behavior.

## Completion checks

1. Build the new package and run its unit and contract tests.
2. Verify the ownership registry has a unique publisher for each output.
3. In Gazebo, verify both pipelines publish fresh frame-preserving clouds.
4. Verify malformed, stale, and backward-time inputs do not produce output.


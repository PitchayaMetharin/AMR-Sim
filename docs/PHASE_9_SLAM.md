# Phase 9 SLAM

## Scope

Phase 9 configures online asynchronous SLAM Toolbox for simulation mapping.
It consumes the front adapted LaserScan and the existing `odom ->
base_footprint` transform, publishes the occupancy map, and is the sole owner
of `map -> odom`. It publishes no motion command and makes no safety,
localization-accuracy, navigation, or physical-sensor claim.

The front scan is used directly because Phase 8's deliberately separate
outputs are PointCloud2-only. No Phase 9 transform, aggregation, or obstacle
policy is invented for the rear sensor.

## Completion checks

1. Build the configured package and run its contract tests.
2. Verify `slam_toolbox` is the unique `map -> odom` publisher.
3. In Gazebo, verify a map and `map -> odom` are published from the front scan.
4. Verify no command or permission topic is created by the package.

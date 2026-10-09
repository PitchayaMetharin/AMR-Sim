# TF extrapolation fault: diagnosis summary (Opus high, 2026-10-09)

Full report and scripts: .ros_logs/hospital_explore_20261009_14/tfdiag/ (stalls_run14.txt, lookup_wait_run14.txt, growth_run14.txt, lag_run14.txt, rtf_run14.txt, scan_feed_run14.txt, run07_ceres.txt).

## Symptom
Runs 07 and 14: the controller's TF lookup fails ("extrapolation into the future", a 0.09 s gap). Path following aborts and the explorer FAULT-latches.

## Verdict
The original root hypothesis ("Ceres loop-closure optimisation stalls SLAM") is FALSIFIED for run 14.
- Run 14 has 1 Ceres solve and 42 map->odom stamp stalls longer than 0.3 s. Only 3 of the stalls are within 1.5 s after a solve.
- 39 of 42 stalls are phase-locked at ~1 Hz to /map publishes (map_update_interval 1.0).
  - Stall end comes 0.125 s (median) after a /map publish; the random-time baseline is 0.497 s.
  - 65 of 66 long stalls contain a /map publish.
- Proposed mechanism: updateMap() holds smapper_mutex_ while it rebuilds the grid from all processed scans. This blocks the scan callback, so publishTransformLoop keeps republishing map->odom stamped (last scan stamp + transform_timeout 1.0).
- Stall length grows with the number of processed scans, about 1.4 ms per scan:
  - 0.1 s at 188 scans, 0.4 s at 264, 0.8 s at 567, 0.9 s at 618.
  - There are no stalls while the robot is stationary, since async mode skips addScan.
  - The grid size plateaued from 180 s on.
- Failure condition reproduces: lag = sim_now - (map->odom stamp - transform_timeout).
  - A lookup fails when lag > 1.0 + RPP transform_tolerance 0.3.
  - Lag is normally 0.13-0.19 s and peaked at 1.43 s at the fault.
  - In 16 controller-active waits the wait was 0.03-0.283 s; the fault wait was 0.418 s.
- Ruled out:
  - RTF drop (0.997-1.005 near the fault);
  - odom stamped ahead (-0.01 s);
  - raw scan feed gaps.
- NOT excluded: the merged scan topic (/amr/sensors/merged_lidar/scan, the SLAM input) has 0 messages in the bag. This is an observation gap; possibly a wrong topic name or QoS.
- Run 07 (no bag): 3 Ceres solves 0.02-1.17 s before the fault, while 18 solves elsewhere did not fault.
  - The proposed reading is rebuild plus loop-closure solve together; this is unproven.
  - Open inconsistency: why run 07 did not fail earlier on long drives.

## Config (src/amr_slam/config/mapper.yaml, shared with factory mapping, so any change must be an overlay)

| Parameter | Workspace | Upstream default |
|---|---|---|
| map_update_interval | 1.0 | 5.0 |
| minimum_time_interval | 0.1 | 0.5 |
| minimum_travel_distance | 0.1 | 0.5 |
| minimum_travel_heading | 0.1 | 0.5 |
| transform_timeout | 1.0 (pinned by test_slam_contract.py:19; coupled to frontier_explorer.py:41) | 0.2 |

## Remedies proposed (exploration-only overlay)
1. Fewer processed scans: minimum_time_interval / travel_distance / travel_heading at 0.3-0.5. This is the best single lever.
2. Rarer rebuilds: map_update_interval 2.0. It must stay well below the explorer map_timeout_sec 3.0.
3. Optional: fewer loop closures.
4. transform_timeout to 2.0: hides the stall; it is a contract change.
5. RPP transform_tolerance to 0.5: margin only. It is not durable, and it blocks the 20 Hz control loop for up to the tolerance.

Recommended: 1 + 2. Verify with the robot moving that max lag stays below ~0.6 s at more than 600 processed scans.

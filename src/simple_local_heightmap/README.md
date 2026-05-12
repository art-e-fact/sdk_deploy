# simple_local_heightmap

This package contains:

- `local_heightmap_node`: builds a local `grid_map_msgs/GridMap` elevation map.
- `rail_detector_node`: parses a rail pair from the local height map with a multi-slice detector and publishes RViz markers.
- `rail_target_follower_node`: converts detected rail geometry and target distance into `/cmd_vel` commands.

## local_heightmap_node

The local heightmap keeps one elevation value per grid cell and ages out cells using the
time since they were last observed. To clear transient hits faster in front of the robot,
the node can apply a shorter timeout inside a rectangle defined in the robot frame.

### Parameters

- `stale_time_sec` (`1.0`): default timeout used outside the front fast-clear rectangle
- `front_clear_enabled` (`false`): enables a robot-frame rectangle with a shorter timeout
- `front_clear_length` (`3.5`): length of the fast-clear rectangle in meters
- `front_clear_width` (`1.0`): width of the fast-clear rectangle in meters
- `front_clear_offset_x` (`0.25`): forward offset from the robot origin to the start of the rectangle
- `front_stale_time_sec` (`0.75`): timeout used for cells inside the fast-clear rectangle

The front-clear overlay is published on `/local_heightmap/front_clear_markers` as a `visualization_msgs/MarkerArray` for RViz debugging.

## rail_detector_node

The detector assumes:

- the robot pose is available on an odometry topic
- the robot is between the two rails
- the robot heading is roughly aligned with the rail tangent locally

It first sweeps the center cross-section around the robot heading, keeps the angle with the smallest valid gauge error, then uses that orientation for the slices ahead and behind. Each slice groups neighboring cells with similar rail-like height, pairs one left group with one right group, and contributes a midpoint to the fitted centerline.

### Parameters

- `heightmap_topic` (`/local_heightmap`): input `grid_map_msgs/GridMap`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot position and heading
- `marker_topic` (`/rail_detector/markers`): output `visualization_msgs/MarkerArray`
- `center_offset_topic` (`/rail_detector/center_offset`): output `std_msgs/Float32` with signed rail-center offset in meters
- `tangent_yaw_topic` (`/rail_detector/tangent_yaw`): output `std_msgs/Float32` with detected rail tangent yaw in radians
- `target_distance_topic` (`/rail_detector/target_distance`): output `std_msgs/Float32` with target distance in meters, or `-1.0` when no target is detected
- `track_gauge` (`1.067`): expected distance between rails in meters
- `rail_width` (`0.15`): expected lateral width of one rail in meters
- `gauge_tolerance` (`0.40`): maximum allowed gauge error when pairing left and right rail groups
- `angle_sweep_deg` (`40.0`): total half-range of the center-slice angle search around the robot heading
- `angle_step_deg` (`5.0`): spacing between tested center-slice angles in degrees
- `min_rail_height` (`0.05`): minimum elevation above the slice center to accept a rail hit
- `max_rail_height` (`0.30`): maximum allowed rail height above the local slice baseline
- `max_rail_height_difference` (`0.08`): maximum allowed height mismatch between the two rails in one slice
- `forward_span` (`2.6`): total span covered by the slice set along the robot heading
- `num_slices` (`15`): number of lateral slices used by the parser
- `lateral_search_width` (`1.8`): half-width of each lateral slice in meters
- `follow_target_lookahead` (`8.0`): forward distance along the detected rail center checked for a follow target
- `follow_target_kernel_size` (`0.35`): width of the center sample window used to measure a follow target
- `follow_target_sample_step` (`0.10`): distance between follow-target samples along the rail center
- `follow_target_min_height` (`0.10`): minimum height above the detected rail height to treat a center blob as a follow target
- `follow_target_max_height` (`2.2`): maximum allowed height above the detected rail height for a follow target

After the rail centerline is detected, the follow target detector samples independently along that centerline and keeps the first center measurement that rises enough above the detected rail height.

The slice grouping step uses the height-map resolution to decide when neighboring samples are similar enough to stay in the same group, so there is no extra grouping threshold parameter.

### Outputs

- slice profile markers for each sampled cross-section
- sphere markers for left and right rail hits plus fitted midpoints
- a centerline marker showing the local rail tangent
- follow target candidate markers inside the rail corridor and a highlighted nearest follow target
- a text marker showing the signed robot offset from the fitted rail center and the nearest follow target distance
- `/rail_detector/center_offset` (`std_msgs/Float32`) with the signed rail-center offset, or `NaN` if the centerline is invalid
- `/rail_detector/tangent_yaw` (`std_msgs/Float32`) with the detected rail tangent yaw, or `NaN` if the centerline is invalid
- `/rail_detector/target_distance` (`std_msgs/Float32`) with the nearest follow-target distance, or `-1.0` if no target is detected

### Example

```bash
ros2 run simple_local_heightmap rail_detector_node --ros-args \
  -p heightmap_topic:=/local_heightmap \
  -p odom_topic:=/odom \
  -p marker_topic:=/rail_detector/markers
```

## rail_target_follower_node

The follower consumes the detector outputs and odometry, then publishes `geometry_msgs/Twist`
commands that keep the robot near the rail center while stopping at a configured distance from
the nearest detected follow target.

The forward speed uses a simple distance ramp:

- stop when the target is inside `follow_distance` or inside the deadband just beyond it
- command `min_linear_x` just outside the deadband so the robot reliably starts walking
- ramp smoothly up to `max_linear_x` once the distance error reaches `distance_error_for_max_speed`

If the rail line becomes invalid, the target disappears, or any input becomes stale, the follower
publishes a zero `Twist`.

### Parameters

- `cmd_vel_topic` (`/cmd_vel`): output `geometry_msgs/Twist`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot yaw and body-frame conversion
- `center_offset_topic` (`/rail_detector/center_offset`): input signed rail-center offset topic
- `tangent_yaw_topic` (`/rail_detector/tangent_yaw`): input rail tangent yaw topic
- `target_distance_topic` (`/rail_detector/target_distance`): input follow-target distance topic
- `control_rate_hz` (`15.0`): control loop rate used to publish `cmd_vel`
- `stale_timeout_sec` (`0.5`): maximum wall-time age accepted for detector and odometry inputs
- `follow_distance` (`1.5`): desired stop distance to the follow target
- `target_distance_deadband` (`0.1`): extra no-motion margin beyond `follow_distance`
- `min_linear_x` (`0.4`): minimum forward command that reliably starts locomotion
- `max_linear_x` (`0.55`): maximum forward body-frame speed command
- `distance_error_for_max_speed` (`1.5`): distance error at which forward speed reaches `max_linear_x`
- `max_linear_y` (`0.4`): maximum lateral centering speed command
- `max_angular_z` (`0.5`): maximum yaw-rate command
- `k_center` (`1.0`): gain that converts rail-center offset into lateral correction speed
- `k_heading` (`1.2`): gain that converts rail tangent yaw error into angular speed

### Example

```bash
ros2 run simple_local_heightmap rail_target_follower_node --ros-args \
  -p center_offset_topic:=/rail_detector/center_offset \
  -p tangent_yaw_topic:=/rail_detector/tangent_yaw \
  -p target_distance_topic:=/rail_detector/target_distance \
  -p follow_distance:=1.5 \
  -p min_linear_x:=0.4 \
  -p max_linear_x:=0.55 \
  -p distance_error_for_max_speed:=1.5
```
# simple_local_heightmap

This package contains:

- `local_heightmap_node`: builds a local `grid_map_msgs/GridMap` elevation map.
- `rail_detector_node`: parses a rail pair from the local height map with a multi-slice detector and publishes RViz markers.

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
- `track_gauge` (`1.067`): expected distance between rails in meters
- `rail_width` (`0.15`): expected lateral width of one rail in meters
- `gauge_tolerance` (`0.40`): maximum allowed gauge error when pairing left and right rail groups
- `angle_sweep_deg` (`20.0`): total half-range of the center-slice angle search around the robot heading
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

### Example

```bash
ros2 run simple_local_heightmap rail_detector_node --ros-args \
  -p heightmap_topic:=/local_heightmap \
  -p odom_topic:=/odom \
  -p marker_topic:=/rail_detector/markers
```
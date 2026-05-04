# simple_local_heightmap

This package contains:

- `local_heightmap_node`: builds a local `grid_map_msgs/GridMap` elevation map.
- `rail_detector_node`: parses a rail pair from the local height map with a multi-slice detector and publishes RViz markers.

## rail_detector_node

The detector assumes:

- the robot pose is available on an odometry topic
- the robot is between the two rails
- the robot heading is roughly aligned with the rail tangent locally

It samples several short cross-sections across the robot heading, groups neighboring cells with similar rail-like height, pairs one left group with one right group, and fits a centerline through the detected slice midpoints.

### Parameters

- `heightmap_topic` (`/local_heightmap`): input `grid_map_msgs/GridMap`
- `odom_topic` (`/odom`): input `nav_msgs/Odometry` used for robot position and heading
- `marker_topic` (`/rail_detector/markers`): output `visualization_msgs/MarkerArray`
- `track_gauge` (`1.067`): expected distance between rails in meters
- `rail_width` (`0.15`): expected lateral width of one rail in meters
- `gauge_tolerance` (`0.40`): maximum allowed gauge error when pairing left and right rail groups
- `min_rail_height` (`0.05`): minimum elevation above the slice center to accept a rail hit
- `max_rail_height` (`0.30`): maximum allowed rail height above the local slice baseline
- `max_rail_height_difference` (`0.08`): maximum allowed height mismatch between the two rails in one slice
- `forward_span` (`2.6`): total span covered by the slice set along the robot heading
- `num_slices` (`15`): number of lateral slices used by the parser
- `lateral_search_width` (`1.8`): half-width of each lateral slice in meters

The slice grouping step uses the height-map resolution to decide when neighboring samples are similar enough to stay in the same group, so there is no extra grouping threshold parameter.

### Outputs

- slice profile markers for each sampled cross-section
- sphere markers for left and right rail hits plus fitted midpoints
- a centerline marker showing the local rail tangent
- a text marker showing the signed robot offset from the fitted rail center

### Example

```bash
ros2 run simple_local_heightmap rail_detector_node --ros-args \
  -p heightmap_topic:=/local_heightmap \
  -p odom_topic:=/odom \
  -p marker_topic:=/rail_detector/markers
```
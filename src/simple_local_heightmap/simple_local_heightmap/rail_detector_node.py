import math

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class RailDetectorNode(Node):
    def __init__(self):
        super().__init__('rail_detector_node')

        self.heightmap_topic = self.declare_parameter(
            'heightmap_topic', '/local_heightmap'
        ).value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.marker_topic = self.declare_parameter(
            'marker_topic', '/rail_detector/markers'
        ).value
        self.track_gauge = float(self.declare_parameter('track_gauge', 1.067).value)
        self.rail_width = float(self.declare_parameter('rail_width', 0.15).value)
        self.gauge_tolerance = float(self.declare_parameter('gauge_tolerance', 0.40).value)
        self.angle_sweep_deg = float(self.declare_parameter('angle_sweep_deg', 40.0).value)
        self.angle_step_deg = float(self.declare_parameter('angle_step_deg', 5.0).value)
        self.min_rail_height = float(self.declare_parameter('min_rail_height', 0.05).value)
        self.max_rail_height = float(self.declare_parameter('max_rail_height', 0.30).value)
        self.max_rail_height_difference = float(
            self.declare_parameter('max_rail_height_difference', 0.08).value
        )
        self.forward_span = float(self.declare_parameter('forward_span', 2.6).value)
        self.num_slices = max(3, int(self.declare_parameter('num_slices', 15).value))
        self.lateral_search_width = float(
            self.declare_parameter('lateral_search_width', 1.8).value
        )

        self.latest_odom = None

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(GridMap, self.heightmap_topic, self.heightmap_callback, 10)

        self.get_logger().info(
            f'Parsing rails from {self.heightmap_topic} with {self.num_slices} slices '
            f'and publishing markers on {self.marker_topic}'
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def heightmap_callback(self, msg):
        if self.latest_odom is None:
            return

        grid = self._decode_grid_map(msg)
        if grid is None:
            return

        detection = self._detect_rails(grid, self.latest_odom)
        self.marker_pub.publish(self._build_markers(msg.header.frame_id, msg.header.stamp, detection))

    def _decode_grid_map(self, msg):
        """Decode the published GridMap message back into a 2D elevation array."""
        if 'elevation' not in msg.layers:
            self.get_logger().warn('GridMap does not contain an elevation layer', throttle_duration_sec=2.0)
            return None

        layer = msg.data[msg.layers.index('elevation')]
        if len(layer.layout.dim) < 2:
            self.get_logger().warn('GridMap elevation layout is missing dimensions', throttle_duration_sec=2.0)
            return None

        width = int(layer.layout.dim[0].size)
        height = int(layer.layout.dim[1].size)
        flat = np.asarray(layer.data, dtype=np.float32)
        if flat.size != width * height:
            self.get_logger().warn('GridMap elevation data size does not match its layout', throttle_duration_sec=2.0)
            return None

        # Undo the flip used by local_heightmap_node before publishing the GridMap layer.
        elevation = np.flip(flat.reshape(height, width), axis=(0, 1))
        resolution = float(msg.info.resolution)
        center = np.array([
            float(msg.info.pose.position.x),
            float(msg.info.pose.position.y),
        ], dtype=np.float32)
        return {
            'elevation': elevation,
            'resolution': resolution,
            'center': center,
            'length_x': float(msg.info.length_x),
            'length_y': float(msg.info.length_y),
            'width': width,
            'height': height,
        }

    def _detect_rails(self, grid, odom):
        """Extract rail hits from several slices after aligning the center slice orientation."""
        robot_xy = np.array([
            float(odom.pose.pose.position.x),
            float(odom.pose.pose.position.y),
        ], dtype=np.float32)
        yaw = self._yaw_from_quaternion(odom.pose.pose.orientation)

        forward_offsets = np.linspace(
            -0.5 * self.forward_span, 0.5 * self.forward_span, self.num_slices, dtype=np.float32
        )
        sample_count = max(31, int(round(2.0 * self.lateral_search_width / grid['resolution'])) + 1)
        lateral_offsets = np.linspace(
            -self.lateral_search_width, self.lateral_search_width, sample_count, dtype=np.float32
        )
        forward, lateral = self._find_best_slice_axes(robot_xy, yaw, lateral_offsets, grid)

        slices = []
        midpoints = []
        hits = []

        for forward_offset in forward_offsets:
            xy, z, baseline = self._sample_slice(
                robot_xy,
                forward,
                lateral,
                forward_offset,
                lateral_offsets,
                grid,
            )

            pair = self._find_rail_pair(
                xy,
                lateral_offsets,
                z,
                baseline=baseline,
                resolution=grid['resolution'],
            )

            slice_result = {'xy': xy, 'z': z, 'left': None, 'right': None, 'midpoint': None}
            if pair is not None:
                left_point, right_point = pair
                midpoint = 0.5 * (left_point + right_point)
                slice_result['left'] = left_point
                slice_result['right'] = right_point
                slice_result['midpoint'] = midpoint
                midpoints.append(midpoint)
                hits.extend([left_point, right_point])

            slices.append(slice_result)

        line = self._fit_centerline(robot_xy, forward, midpoints)
        return {
            'robot_xy': robot_xy,
            'forward': forward,
            'slices': slices,
            'midpoints': np.asarray(midpoints, dtype=np.float32),
            'hits': np.asarray(hits, dtype=np.float32),
            'line': line,
        }

    def _find_best_slice_axes(self, robot_xy, yaw, lateral_offsets, grid):
        """Sweep the center slice angle and keep the valid orientation with the smallest gauge error."""
        best = None
        if self.angle_step_deg > 0.0 and self.angle_sweep_deg > 0.0:
            angle_offsets_deg = np.arange(
                -self.angle_sweep_deg,
                self.angle_sweep_deg + 0.5 * self.angle_step_deg,
                self.angle_step_deg,
                dtype=np.float32,
            )
        else:
            angle_offsets_deg = np.array([0.0], dtype=np.float32)

        for angle_offset_deg in angle_offsets_deg:
            forward, lateral = self._slice_axes(yaw + math.radians(float(angle_offset_deg)))
            xy, z, baseline = self._sample_slice(
                robot_xy,
                forward,
                lateral,
                0.0,
                lateral_offsets,
                grid,
            )
            pair = self._find_rail_pair(
                xy,
                lateral_offsets,
                z,
                baseline=baseline,
                resolution=grid['resolution'],
            )
            if pair is None:
                continue

            left_point, right_point = pair
            gauge_error = abs(float(np.linalg.norm(left_point[:2] - right_point[:2])) - self.track_gauge)
            candidate = (gauge_error, abs(float(angle_offset_deg)), forward, lateral)
            if best is None or candidate[:2] < best[:2]:
                best = candidate

        if best is not None:
            return best[2], best[3]
        return self._slice_axes(yaw)

    def _sample_slice(self, robot_xy, forward, lateral, forward_offset, lateral_offsets, grid):
        """Sample one cross-section and estimate its local center baseline."""
        slice_center = robot_xy + forward_offset * forward
        xy = slice_center + lateral_offsets[:, None] * lateral[None, :]
        z = self._sample_grid(grid, xy)
        center_mask = np.abs(lateral_offsets) <= min(0.25, 0.25 * self.track_gauge)
        baseline = self._safe_nanmedian(z[center_mask])
        if not math.isfinite(baseline):
            baseline = self._safe_nanmedian(z)
        return xy, z, baseline

    def _sample_grid(self, grid, xy):
        """Sample the elevation grid at world XY points with nearest-neighbor lookup."""
        x_min = grid['center'][0] - 0.5 * grid['length_x']
        y_min = grid['center'][1] - 0.5 * grid['length_y']
        coords = np.empty_like(xy)
        coords[:, 0] = (xy[:, 0] - x_min) / grid['resolution'] - 0.5
        coords[:, 1] = (xy[:, 1] - y_min) / grid['resolution'] - 0.5
        cols = np.rint(coords[:, 0]).astype(np.int32)
        rows = np.rint(coords[:, 1]).astype(np.int32)
        inside = (
            (cols >= 0)
            & (cols < grid['width'])
            & (rows >= 0)
            & (rows < grid['height'])
        )

        z = np.full(len(xy), np.nan, dtype=np.float32)
        z[inside] = grid['elevation'][rows[inside], cols[inside]]
        return z

    def _find_rail_pair(self, xy, lateral_offsets, z, baseline, resolution):
        """Group one slice into rail-like blobs and return the best left/right pair."""
        if not math.isfinite(baseline):
            return None

        smooth = self._smooth_profile(z)
        in_height_band = (
            np.isfinite(smooth)
            & (smooth >= baseline + self.min_rail_height)
            & (smooth <= baseline + self.max_rail_height)
        )
        groups = self._extract_slice_groups(
            xy,
            lateral_offsets,
            z,
            smooth,
            in_height_band,
            resolution,
        )
        if not groups:
            return None

        width_tolerance = max(2.0 * resolution, 0.5 * self.rail_width)
        left_groups = [
            group for group in groups
            if group['offset'] > 0.0 and abs(group['width'] - self.rail_width) <= width_tolerance
        ]
        right_groups = [
            group for group in groups
            if group['offset'] < 0.0 and abs(group['width'] - self.rail_width) <= width_tolerance
        ]
        if not left_groups or not right_groups:
            return None

        best_pair = None
        best_score = float('inf')
        for left_group in left_groups:
            for right_group in right_groups:
                measured_gauge = float(left_group['offset'] - right_group['offset'])
                gauge_error = abs(measured_gauge - self.track_gauge)
                height_error = abs(left_group['point'][2] - right_group['point'][2])
                width_error = (
                    abs(left_group['width'] - self.rail_width)
                    + abs(right_group['width'] - self.rail_width)
                )
                if (
                    gauge_error > self.gauge_tolerance
                    or height_error > self.max_rail_height_difference
                ):
                    continue

                score = gauge_error + height_error + 0.5 * width_error
                if score < best_score:
                    best_score = score
                    best_pair = (left_group['point'], right_group['point'])

        return best_pair

    def _extract_slice_groups(self, xy, lateral_offsets, z, smooth, mask, resolution):
        """Split one slice into contiguous height-consistent groups."""
        groups = []
        group_height_step = max(resolution, 0.5 * self.min_rail_height)
        start = None
        prev = None

        for index in range(len(smooth)):
            if not mask[index]:
                if start is not None:
                    groups.append(self._build_slice_group(xy, lateral_offsets, z, start, prev, resolution))
                    start = None
                    prev = None
                continue

            if start is None:
                start = prev = index
                continue

            if abs(float(smooth[index] - smooth[prev])) <= group_height_step:
                prev = index
                continue

            groups.append(self._build_slice_group(xy, lateral_offsets, z, start, prev, resolution))
            start = prev = index

        if start is not None:
            groups.append(self._build_slice_group(xy, lateral_offsets, z, start, prev, resolution))

        return [group for group in groups if group is not None]

    def _build_slice_group(self, xy, lateral_offsets, z, start, end, resolution):
        """Summarize one contiguous group by its center, width, and representative height."""
        segment = slice(start, end + 1)
        segment_xy = xy[segment]
        segment_z = z[segment]
        valid = np.isfinite(segment_z)
        if not np.any(valid):
            return None

        point = np.empty(3, dtype=np.float32)
        point[:2] = np.mean(segment_xy[valid], axis=0)
        point[2] = float(np.nanmedian(segment_z[valid]))
        return {
            'point': point,
            'offset': float(np.mean(lateral_offsets[segment])),
            'width': float(abs(lateral_offsets[end] - lateral_offsets[start]) + resolution),
        }

    def _smooth_profile(self, z):
        """Apply a tiny 1D smoothing kernel while preserving missing samples."""
        finite = np.isfinite(z)
        weights = finite.astype(np.float32)
        values = np.where(finite, z, 0.0).astype(np.float32)
        kernel = np.array([1.0, 2.0, 1.0], dtype=np.float32)
        numer = np.convolve(values, kernel, mode='same')
        denom = np.convolve(weights, kernel, mode='same')
        smooth = np.full(len(z), np.nan, dtype=np.float32)
        valid = denom > 0.0
        smooth[valid] = numer[valid] / denom[valid]
        return smooth

    @staticmethod
    def _slice_axes(yaw):
        """Convert a slice yaw into forward and lateral unit vectors."""
        forward = np.array([math.cos(yaw), math.sin(yaw)], dtype=np.float32)
        lateral = np.array([-math.sin(yaw), math.cos(yaw)], dtype=np.float32)
        return forward, lateral

    def _fit_centerline(self, robot_xy, forward, midpoints):
        """Fit a short centerline and derive the robot's signed lateral offset."""
        if len(midpoints) == 0:
            return None

        xy = np.asarray(midpoints, dtype=np.float32)[:, :2]
        center = np.mean(xy, axis=0)
        tangent = np.asarray(forward, dtype=np.float32)
        if len(xy) >= 2:
            # PCA gives the dominant local rail direction through the slice midpoints.
            _, _, vh = np.linalg.svd(xy - center[None, :], full_matrices=False)
            tangent = vh[0].astype(np.float32)
        if np.dot(tangent, forward) < 0.0:
            tangent = -tangent

        normal = np.array([-tangent[1], tangent[0]], dtype=np.float32)
        signed_offset = float(np.dot(robot_xy - center, normal))
        return {
            'center': center,
            'tangent': tangent,
            'signed_offset': signed_offset,
            'yaw': math.atan2(float(tangent[1]), float(tangent[0])),
        }

    def _build_markers(self, frame_id, stamp, detection):
        """Visualize slice samples, accepted rail hits, and the fitted centerline."""
        msg = MarkerArray()
        msg.markers.append(self._delete_all_marker(frame_id, stamp))

        for index, slice_result in enumerate(detection['slices']):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'slice_profiles'
            marker.id = index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.015
            marker.color.r = 0.2
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker.color.a = 0.75
            marker.points = [
                self._point(x, y, z + 0.01)
                for (x, y), z in zip(slice_result['xy'], slice_result['z'])
                if math.isfinite(float(z))
            ]
            if marker.points:
                msg.markers.append(marker)

        if len(detection['hits']) != 0:
            msg.markers.append(
                self._sphere_list_marker(
                    frame_id,
                    stamp,
                    namespace='rail_hits',
                    marker_id=100,
                    points=detection['hits'],
                    rgb=(0.5, 0.2, 0.95),
                    scale=0.08,
                )
            )

        if len(detection['midpoints']) != 0:
            msg.markers.append(
                self._sphere_list_marker(
                    frame_id,
                    stamp,
                    namespace='center_samples',
                    marker_id=101,
                    points=detection['midpoints'],
                    rgb=(1.0, 0.9, 0.1),
                    scale=0.10,
                )
            )

        if detection['line'] is not None:
            line = detection['line']
            z_level = self._marker_height(detection['midpoints'], detection['hits'])
            start = line['center'] - 0.8 * self.forward_span * line['tangent']
            end = line['center'] + 0.8 * self.forward_span * line['tangent']
            msg.markers.append(
                self._line_marker(
                    frame_id,
                    stamp,
                    namespace='centerline',
                    marker_id=200,
                    start=np.array([start[0], start[1], z_level + 0.04], dtype=np.float32),
                    end=np.array([end[0], end[1], z_level + 0.04], dtype=np.float32),
                    rgb=(0.1, 1.0, 0.2),
                    width=0.05,
                )
            )
            text = (
                f'offset={line["signed_offset"]:+.2f} m\\n'
                f'heading={math.degrees(line["yaw"]):.1f} deg'
            )
        else:
            text = 'rail parse incomplete'

        robot = detection['robot_xy']
        msg.markers.append(
            self._text_marker(
                frame_id,
                stamp,
                namespace='summary',
                marker_id=300,
                x=float(robot[0]),
                y=float(robot[1]),
                z=self._marker_height(detection['midpoints'], detection['hits']) + 0.25,
                text=text,
            )
        )
        return msg

    @staticmethod
    def _safe_nanmedian(values):
        finite = np.asarray(values, dtype=np.float32)
        finite = finite[np.isfinite(finite)]
        if len(finite) == 0:
            return float('nan')
        return float(np.median(finite))

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _delete_all_marker(frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    @staticmethod
    def _point(x, y, z):
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _sphere_list_marker(self, frame_id, stamp, namespace, marker_id, points, rgb, scale):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 0.95
        marker.points = [self._point(x, y, z + 0.02) for x, y, z in np.asarray(points)]
        return marker

    def _line_marker(self, frame_id, stamp, namespace, marker_id, start, end, rgb, width):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r, marker.color.g, marker.color.b = rgb
        marker.color.a = 0.95
        marker.points = [self._point(*start), self._point(*end)]
        return marker

    def _text_marker(self, frame_id, stamp, namespace, marker_id, x, y, z, text):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.text = text
        return marker

    @staticmethod
    def _marker_height(midpoints, hits):
        if len(midpoints) != 0:
            return float(np.nanmedian(midpoints[:, 2]))
        if len(hits) != 0:
            return float(np.nanmedian(hits[:, 2]))
        return 0.0


def main(args=None):
    rclpy.init(args=args)
    node = RailDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
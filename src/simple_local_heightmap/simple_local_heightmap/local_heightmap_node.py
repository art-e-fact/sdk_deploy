import math

import numpy as np
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped
from grid_map_msgs.msg import GridMap
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Header, MultiArrayDimension
from tf2_ros import Buffer, TransformException, TransformListener


class LocalHeightmapNode(Node):
    def __init__(self):
        super().__init__('local_heightmap_node')

        self.cloud_topic = self.declare_parameter('cloud_topic', '/mid360/points').value
        self.pose_topic = self.declare_parameter(
            'pose_with_covariance_topic', '/pose_with_covariance'
        ).value
        self.output_topic = self.declare_parameter('heightmap_topic', '/local_heightmap').value
        self.debug_topic = self.declare_parameter(
            'debug_cloud_topic', '/local_heightmap/debug_points'
        ).value
        self.map_frame = self.declare_parameter('map_frame', 'base_link').value
        self.resolution = float(self.declare_parameter('resolution', 0.025).value)
        self.length_x = float(self.declare_parameter('length_x', 8.0).value)
        self.length_y = float(self.declare_parameter('length_y', 8.0).value)
        self.min_z = float(self.declare_parameter('min_z', -1.0).value)
        self.max_z = float(self.declare_parameter('max_z', 2.0).value)
        self.min_range = float(self.declare_parameter('min_range', 0.1).value)
        self.max_range = float(self.declare_parameter('max_range', 12.0).value)
        self.max_pose_variance = float(
            self.declare_parameter('max_pose_variance', 0.0).value
        )

        self.width = max(1, int(round(self.length_x / self.resolution)))
        self.height = max(1, int(round(self.length_y / self.resolution)))
        self.length_x = self.width * self.resolution
        self.length_y = self.height * self.resolution
        self.latest_pose_variance = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_pub = self.create_publisher(GridMap, self.output_topic, 10)
        self.debug_pub = self.create_publisher(PointCloud2, self.debug_topic, 10)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )

        self.get_logger().info(
            f'Publishing {self.length_x:.2f} x {self.length_y:.2f} m height maps at '
            f'{self.resolution:.3f} m/cell from {self.cloud_topic}'
        )

    def pose_callback(self, msg):
        cov = msg.pose.covariance
        self.latest_pose_variance = max(cov[0], cov[7], cov[35])

    def cloud_callback(self, msg):
        if self.max_pose_variance > 0.0:
            if self.latest_pose_variance > self.max_pose_variance:
                return

        points = self._cloud_to_array(msg)
        if len(points) == 0:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, msg.header.stamp
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}', throttle_duration_sec=2.0)
            return

        points = self._filter_points(self._transform_points(points, transform))
        if len(points) == 0:
            return

        heightmap = self._rasterize(points)
        stamp = msg.header.stamp
        self.map_pub.publish(self._to_grid_map(heightmap, stamp))
        self.debug_pub.publish(self._to_debug_cloud(heightmap, stamp))

    def _cloud_to_array(self, msg):
        points = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if isinstance(points, np.ndarray) and points.dtype.names:
            return np.column_stack((points['x'], points['y'], points['z'])).astype(np.float32)
        return np.asarray(list(points), dtype=np.float32).reshape(-1, 3)

    def _filter_points(self, points):
        ranges = np.linalg.norm(points, axis=1)
        valid = (
            (ranges >= self.min_range)
            & (ranges <= self.max_range)
            & (points[:, 2] >= self.min_z)
            & (points[:, 2] <= self.max_z)
        )
        return points[valid]

    def _rasterize(self, points):
        x_min = -0.5 * self.length_x
        y_min = -0.5 * self.length_y
        cols = np.floor((points[:, 0] - x_min) / self.resolution).astype(np.int32)
        rows = np.floor((points[:, 1] - y_min) / self.resolution).astype(np.int32)
        inside = (cols >= 0) & (cols < self.width) & (rows >= 0) & (rows < self.height)

        heightmap = np.full((self.height, self.width), np.nan, dtype=np.float32)
        rows = rows[inside]
        cols = cols[inside]
        zs = points[inside, 2]
        if len(zs) == 0:
            return heightmap

        # Median per cell keeps one robust surface estimate from each scan.
        linear = rows * self.width + cols
        unique_cells, inverse = np.unique(linear, return_inverse=True)
        for index, cell in enumerate(unique_cells):
            heightmap.flat[cell] = np.median(zs[inverse == index])
        return heightmap

    def _to_grid_map(self, heightmap, stamp):
        msg = GridMap()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.length_x = self.length_x
        msg.info.length_y = self.length_y
        msg.info.pose.orientation.w = 1.0
        msg.layers = ['elevation']
        msg.basic_layers = ['elevation']
        msg.data = [self._to_multi_array(heightmap)]
        return msg

    def _to_multi_array(self, heightmap):
        data = Float32MultiArray()
        data.layout.dim = [
            MultiArrayDimension(
                label='column_index', size=self.width, stride=self.width * self.height
            ),
            MultiArrayDimension(label='row_index', size=self.height, stride=self.height),
        ]
        data.data = np.flip(heightmap, axis=(0, 1)).reshape(-1).astype(np.float32).tolist()
        return data

    def _to_debug_cloud(self, heightmap, stamp):
        rows, cols = np.nonzero(np.isfinite(heightmap))
        x = (cols + 0.5) * self.resolution - 0.5 * self.length_x
        y = (rows + 0.5) * self.resolution - 0.5 * self.length_y
        points = np.column_stack((x, y, heightmap[rows, cols])).astype(np.float32)
        header = Header(stamp=stamp, frame_id=self.map_frame)
        return pc2.create_cloud_xyz32(header, points.tolist())

    @staticmethod
    def _transform_points(points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rot = LocalHeightmapNode._quaternion_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        offset = np.array([translation.x, translation.y, translation.z], dtype=np.float32)
        return (rot @ points.T).T + offset

    @staticmethod
    def _quaternion_matrix(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3, dtype=np.float32)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float32)


def main(args=None):
    rclpy.init(args=args)
    node = LocalHeightmapNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

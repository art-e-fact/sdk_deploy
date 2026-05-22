"""2D LiDAR for Newton using custom SensorTiledCamera rays."""

import math
import newton
import numpy as np
from newton._src.sensors.sensor_tiled_camera import SensorTiledCamera
from sensor_msgs.msg import LaserScan

from sensors.common.lidar import (
    LIDAR_FRAME_ID,
    LIDAR_FREQUENCY_HZ,
    LIDAR_SITE_NAME,
    LIDAR_TOPIC,
    NUM_RAYS,
    RANGE_MAX,
    RANGE_MIN,
    planar_lidar_dirs,
)
from sensors.common.transforms import make_transform, quat_from_matrix
from sensors.newton.geometry import camera_transforms, find_builder_shape_index, find_site_index, site_local_pose, site_world_pose
from sensors.newton.ray_buffers import rays_from_dirs

MAX_ALL_INF_DEBUG_LOGS = 5


class NewtonLidarSensor:
    @staticmethod
    def init_visuals(builder):
        site_index = find_builder_shape_index(builder, LIDAR_SITE_NAME)
        builder.shape_type[site_index] = newton.GeoType.CYLINDER
        builder.shape_scale[site_index] = (0.025, 0.01, 0.0)
        builder.shape_color[site_index] = (0.1, 0.1, 0.1)
        builder.shape_flags[site_index] |= int(newton.ShapeFlags.VISIBLE)

    def __init__(self, model, node, enabled: bool = False):
        self.model = model
        self.node = node
        self.enabled = enabled
        self.site_index = -1

        if not enabled:
            node.get_logger().info("[INFO] Newton LiDAR disabled")
            return

        self.site_index = find_site_index(model, LIDAR_SITE_NAME)
        self.body_id = int(model.shape_body.numpy()[self.site_index])
        self.shape_body = model.shape_body.numpy()
        self.local_dirs = planar_lidar_dirs(NUM_RAYS)
        self.rays = rays_from_dirs(self.local_dirs)
        self.sensor = SensorTiledCamera(model, load_textures=False)
        self.depth_image = self.sensor.utils.create_depth_image_output(NUM_RAYS, 1)
        self.shape_index_image = self.sensor.utils.create_shape_index_image_output(NUM_RAYS, 1)

        self.pub = node.create_publisher(LaserScan, LIDAR_TOPIC, 10)
        self.angle_increment = 2.0 * math.pi / NUM_RAYS
        self.scan_time = 1.0 / LIDAR_FREQUENCY_HZ
        self.time_increment = self.scan_time / (NUM_RAYS - 1)
        self._all_inf_debug_logs = 0
        node.get_logger().info(f"[INFO] Newton LiDAR initialized ({NUM_RAYS} rays @ {LIDAR_FREQUENCY_HZ} Hz)")

    def update(self, state, timestamp: float):
        if not self.enabled:
            return

        site_pos, site_rot, _ = site_world_pose(self.model, state, self.site_index)
        transforms = camera_transforms(site_pos, site_rot, self.model.world_count)
        self.sensor.update(
            state,
            transforms,
            self.rays,
            depth_image=self.depth_image,
            shape_index_image=self.shape_index_image,
        )

        ranges = self.depth_image.numpy()[0, 0, 0].astype(np.float32)
        shape_indices = self.shape_index_image.numpy()[0, 0, 0]
        raw_hit_count = int(np.count_nonzero(ranges > 0.0))

        ranges[ranges <= 0.0] = float("inf")
        ranges[(ranges < RANGE_MIN) | (ranges > RANGE_MAX)] = float("inf")
        range_hit_count = int(np.count_nonzero(ranges < float("inf")))

        # SensorTiledCamera has no bodyexclude option, so filter hits on the mounting body.
        valid_shape = shape_indices < len(self.shape_body)
        hit_bodies = np.full(shape_indices.shape, -9999, dtype=np.int32)
        hit_bodies[valid_shape] = self.shape_body[shape_indices[valid_shape].astype(np.int64)]
        ranges[hit_bodies == self.body_id] = float("inf")
        self_filtered_count = int(np.count_nonzero(ranges < float("inf")))

        if self._all_inf_debug_logs < MAX_ALL_INF_DEBUG_LOGS and not np.any(ranges < float("inf")):
            self._all_inf_debug_logs += 1
            self.node.get_logger().warn(
                "Newton LiDAR produced all inf ranges "
                f"(raw_hits={raw_hit_count}, in_range={range_hit_count}, "
                f"after_self_filter={self_filtered_count}, site_z={site_pos[2]:.3f})"
            )

        msg = LaserScan()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = LIDAR_FRAME_ID
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = self.angle_increment
        msg.time_increment = self.time_increment
        msg.scan_time = self.scan_time
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges.astype(np.float32).tolist()
        self.pub.publish(msg)

    def get_static_transforms(self, stamp):
        if not self.enabled or self.site_index < 0:
            return []
        site_pos_local, site_rot = site_local_pose(self.model, self.site_index)
        return [make_transform(stamp, "base_link", LIDAR_FRAME_ID, site_pos_local, quat_from_matrix(site_rot))]

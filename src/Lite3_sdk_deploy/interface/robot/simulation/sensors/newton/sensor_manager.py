"""Orchestrates Newton sensors at their own publish rates."""

from dataclasses import dataclass

from tf2_ros import StaticTransformBroadcaster

from sensors.common.camera import DEPTH_FREQUENCY_HZ
from sensors.common.lidar import LIDAR_FREQUENCY_HZ, MID360_FREQUENCY_HZ
from sensors.newton.bvh import NewtonBvh
from sensors.newton.depth_sensor import NewtonDepthSensor
from sensors.newton.lidar_sensor import NewtonLidarSensor
from sensors.newton.mid360_lidar_sensor import NewtonMid360LidarSensor


@dataclass
class NewtonSensorOptions:
    enable_lidar: bool = False
    enable_mid360: bool = False
    enable_depth: bool = False
    enable_color: bool = False
    enable_pointcloud: bool = False
    mid360_downsample: int = 1


class NewtonSensorManager:
    def __init__(self, model, state, node, dt: float, options: NewtonSensorOptions):
        self.model = model
        self.node = node
        self.static_tf_broadcaster = StaticTransformBroadcaster(node)

        self.lidar = NewtonLidarSensor(model, node, enabled=options.enable_lidar)
        self.mid360 = NewtonMid360LidarSensor(
            model,
            node,
            enabled=options.enable_mid360,
            downsample=options.mid360_downsample,
        )
        self.depth = NewtonDepthSensor(
            model,
            node,
            enable_depth=options.enable_depth,
            enable_color=options.enable_color,
            enable_pointcloud=options.enable_pointcloud,
        )

        self.lidar_step_interval = max(1, int(1.0 / (LIDAR_FREQUENCY_HZ * dt)))
        self.mid360_step_interval = max(1, int(1.0 / (MID360_FREQUENCY_HZ * dt)))
        self.depth_step_interval = max(1, int(1.0 / (DEPTH_FREQUENCY_HZ * dt)))
        self.bvh = NewtonBvh(model, state) if self.enabled else None
        self._publish_static_transforms()

    @property
    def enabled(self) -> bool:
        return self.lidar.enabled or self.mid360.enabled or self.depth.enabled

    def update(self, state, step_count: int, timestamp: float):
        if not self.enabled:
            return

        due_lidar = self.lidar.enabled and step_count % self.lidar_step_interval == 0
        due_mid360 = self.mid360.enabled and step_count % self.mid360_step_interval == 0
        due_depth = self.depth.enabled and step_count % self.depth_step_interval == 0
        if not (due_lidar or due_mid360 or due_depth):
            return

        # Refit once for all due sensors; sensor rendering then sees the same world pose.
        self.bvh.refit(state)
        if due_lidar:
            self.lidar.update(state, timestamp)
        if due_mid360:
            self.mid360.update(state, timestamp)
        if due_depth:
            self.depth.update(state, timestamp)

    def _publish_static_transforms(self):
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        transforms.extend(self.lidar.get_static_transforms(stamp))
        transforms.extend(self.mid360.get_static_transforms(stamp))
        transforms.extend(self.depth.get_static_transforms(stamp))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

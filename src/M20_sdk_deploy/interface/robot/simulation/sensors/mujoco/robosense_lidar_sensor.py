"""
CPU-only RoboSense 96-line LiDAR simulation using MuJoCo ray casting.

The Lynx M20 carries two 96-line units (360 deg x 90 deg FOV, ~860,000 pts/s
combined). Each simulated unit casts a mechanical-spinner ray grid
(channels x columns) from its mount site and publishes
sensor_msgs/msg/PointCloud2 in the sensor frame.

Mount sites (from the Lynx M20 hardware manual, body frame):
    front: ( 0.32028, 0, -0.013) m   -> site "lidar_front_site"
    rear:  (-0.32028, 0, -0.013) m   -> site "lidar_back_site" (yawed 180 deg)
"""

import mujoco
import numpy as np
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from sensor_msgs.msg import PointCloud2, PointField

# Full-rate spec: 96 channels x 448 columns @ 10 Hz ~= 430k pts/s per unit.
DEFAULT_CHANNELS = 96
DEFAULT_COLUMNS = 448
# Cast every Nth column to keep CPU ray casting near real-time (the stair
# scene has ~2200 geoms). Set to 1 for the full ~430k pts/s per unit if you
# have CPU headroom; 8 keeps all 96 lines at 6.4 deg azimuth resolution.
DEFAULT_COLUMN_DOWNSAMPLE = 8
DEFAULT_FREQUENCY_HZ = 10.0
DEFAULT_RANGE_MIN = 0.1
DEFAULT_RANGE_MAX = 30.0
# 90 deg vertical FOV, assumed symmetric about the sensor's horizontal plane.
DEFAULT_V_FOV_DEG = (-45.0, 45.0)

_XYZ_POINT_STEP = 12


def _make_xyz_pointcloud(points: np.ndarray, stamp, frame_id: str) -> PointCloud2:
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = _XYZ_POINT_STEP
    msg.row_step = _XYZ_POINT_STEP * msg.width
    msg.data = points.astype(np.float32, copy=False).tobytes()
    msg.is_dense = True
    return msg


class RobosenseLidarSensor:
    """One spinning 96-line LiDAR attached to a MuJoCo site."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node,
        site_name: str,
        topic: str,
        frame_id: str,
        channels: int = DEFAULT_CHANNELS,
        columns: int = DEFAULT_COLUMNS,
        column_downsample: int = DEFAULT_COLUMN_DOWNSAMPLE,
        v_fov_deg=DEFAULT_V_FOV_DEG,
        range_min: float = DEFAULT_RANGE_MIN,
        range_max: float = DEFAULT_RANGE_MAX,
        frequency_hz: float = DEFAULT_FREQUENCY_HZ,
    ):
        self.model = model
        self.data = data
        self.node = node
        self.frame_id = frame_id
        self.range_min = float(range_min)
        self.range_max = float(range_max)
        self.frequency_hz = float(frequency_hz)

        self.site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.site_id < 0:
            raise ValueError(f"Site '{site_name}' not found in model")
        self.body_id = model.site_bodyid[self.site_id]
        self.geomgroup = np.ones(mujoco.mjNGROUP, dtype=np.uint8)

        # Precompute the spinner ray grid in the sensor frame:
        # columns sweep 360 deg azimuth, channels span the vertical FOV.
        cols = max(1, columns // max(1, int(column_downsample)))
        theta = np.linspace(-np.pi, np.pi, cols, endpoint=False, dtype=np.float64)
        phi = np.deg2rad(
            np.linspace(v_fov_deg[0], v_fov_deg[1], channels, dtype=np.float64)
        )
        theta_grid, phi_grid = np.meshgrid(theta, phi)
        theta_flat = theta_grid.ravel()
        phi_flat = phi_grid.ravel()
        cos_phi = np.cos(phi_flat)
        self.local_dirs = np.column_stack((
            cos_phi * np.cos(theta_flat),
            cos_phi * np.sin(theta_flat),
            np.sin(phi_flat),
        ))

        nray = len(self.local_dirs)
        self.distances = np.empty(nray, dtype=np.float64)
        self.geom_ids = np.empty(nray, dtype=np.int32)
        self.pub = node.create_publisher(PointCloud2, topic, 10)
        node.get_logger().info(
            f"[INFO] RoboSense LiDAR '{frame_id}' initialized "
            f"({channels}x{cols} = {nray} rays @ {self.frequency_hz} Hz -> {topic})"
        )

    def update(self, stamp):
        site_pos = self.data.site_xpos[self.site_id]
        site_rot = self.data.site_xmat[self.site_id].reshape(3, 3)
        world_dirs = self.local_dirs @ site_rot.T

        self.distances.fill(self.range_max)
        self.geom_ids.fill(-1)
        mujoco.mj_multiRay(
            self.model,
            self.data,
            pnt=site_pos,
            vec=world_dirs.ravel(),
            geomgroup=self.geomgroup,
            flg_static=1,
            bodyexclude=self.body_id,
            geomid=self.geom_ids,
            dist=self.distances,
            normal=None,
            nray=len(self.local_dirs),
            cutoff=self.range_max,
        )

        valid = (
            (self.geom_ids != -1)
            & (self.distances >= self.range_min)
            & (self.distances <= self.range_max)
        )
        points = (self.local_dirs[valid] * self.distances[valid, None]).astype(np.float32)
        self.pub.publish(_make_xyz_pointcloud(points, stamp, self.frame_id))

    def get_static_transform(self, stamp, parent_frame: str = "base_link") -> TransformStamped:
        """base_link -> sensor frame, from the site's model-local pose."""
        pos = self.model.site_pos[self.site_id]
        quat_wxyz = self.model.site_quat[self.site_id]

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = self.frame_id
        transform.transform.translation = Vector3(
            x=float(pos[0]), y=float(pos[1]), z=float(pos[2])
        )
        transform.transform.rotation = Quaternion(
            x=float(quat_wxyz[1]),
            y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]),
            w=float(quat_wxyz[0]),
        )
        return transform

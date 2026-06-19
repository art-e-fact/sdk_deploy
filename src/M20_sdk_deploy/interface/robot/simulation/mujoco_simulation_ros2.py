"""
 * @file mujoco_simulation.py
 * @brief simulation in mujoco
 * @author Bo (Percy) Peng
 * @version 1.0
 * @date 2025-11-05
 *
 * @copyright Copyright (c) 2025  DeepRobotics
"""

import os
import time
import tempfile
import argparse
from pathlib import Path
from xml.sax.saxutils import quoteattr
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer

from sensors.mujoco.lidar_sensor import LidarSensor
from sensors.mujoco.depth_sensor import DepthSensor
from sensors.mujoco.mid360_lidar_sensor import Mid360LidarSensor
from simulation_config import SimulationConfig

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from drdds.msg import ImuData, JointsData, JointsDataCmd, MetaType, ImuDataValue, JointsDataValue, JointData, JointDataCmd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster



MODEL_NAME = "M20"
# Get the directory of the current Python file
CURRENT_DIR = Path(__file__).resolve().parent

# Define the XML path relative to the Python file
XML_PATH = CURRENT_DIR / ".." / ".." / ".." / "M20_description" / "m20_mjcf" / "mjcf" / "M20_stair.xml"

# Convert to absolute path as string
XML_PATH = str(XML_PATH.resolve())
USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 50

# Calibaration parameters (for sim-to-real consistency)
JOINT_DIR = np.array([1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1], dtype=np.float32)
POS_OFFSET_DEG = np.array([-25, 229, 160, 0, 25, -131, -200, 0, -25, -229, -160, 0, 25, 131, 200, 0], dtype=np.float32)
POS_OFFSET_RAD = POS_OFFSET_DEG / 180.0 * np.pi

JOINT_INIT = {
    "M20": np.array([-0.438, -1.16, 2.76, 0,
                     0.438, -1.16, 2.76, 0,
                     -0.438, 1.16, -2.76, 0,
                     0.438, 1.16, -2.76, 0], dtype=np.float32),
}


def _add_ground_plane(spec: mujoco.MjSpec) -> None:
    """Add a simple ground plane when no scene is provided."""
    tex = spec.add_texture()
    tex.name = "grid_tex"
    tex.type = mujoco.mjtTexture.mjTEX_2D
    tex.builtin = mujoco.mjtBuiltin.mjBUILTIN_CHECKER
    tex.rgb1 = [0.1, 0.2, 0.3]
    tex.rgb2 = [0.2, 0.3, 0.4]
    tex.width = 300
    tex.height = 300
    mat = spec.add_material()
    mat.name = "grid_mat"
    mat.texrepeat = [5, 5]
    mat.set_texture(tex, mujoco.mjtTextureRole.mjTEXROLE_RGB)
    geom = spec.worldbody.add_geom()
    geom.type = mujoco.mjtGeom.mjGEOM_PLANE
    geom.size = [50.0, 50.0, 0.05]
    geom.set_material(mat)


def _build_static_spec(scene_path: str | None, robot_xml_path: str) -> mujoco.MjSpec:
    """Merge scene + robot into a single MjSpec, or load robot-only with a ground plane."""
    if not scene_path:
        spec = mujoco.MjSpec.from_file(robot_xml_path)
        _add_ground_plane(spec)
        return spec

    # Write the temp file in the robot XML's directory so relative meshdir
    # paths (e.g. ../meshes/) resolve correctly.
    robot_dir = os.path.dirname(os.path.abspath(robot_xml_path))
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False,
                                     encoding="utf-8", dir=robot_dir) as merged_file:
        merged_file.write(
            '<mujoco model="M20_static_scene">\n'
            f'  <include file={quoteattr(os.path.abspath(robot_xml_path))}/>\n'
            f'  <include file={quoteattr(os.path.abspath(scene_path))}/>\n'
            '</mujoco>\n'
        )
        merged_path = merged_file.name

    try:
        return mujoco.MjSpec.from_file(merged_path)
    finally:
        try:
            os.unlink(merged_path)
        except OSError:
            pass


def _as_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


class MuJoCoSimulationNode(Node):
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_path: str = XML_PATH,
                 config: SimulationConfig | None = None):

        super().__init__('mujoco_simulation')

        config = config or SimulationConfig.load()
        if xml_path != XML_PATH:
            config = config.with_overrides({"scene": xml_path})

        self.declare_parameter('headless', config.headless)
        self.declare_parameter('enable_lidar', config.sensors.lidar_2d.enabled)
        self.declare_parameter('enable_mid360', config.sensors.mid360.enabled)
        self.declare_parameter('enable_depth', config.sensors.realsense.enable_depth)
        self.declare_parameter('enable_color', config.sensors.realsense.enable_color)
        self.declare_parameter('enable_pointcloud', config.sensors.realsense.enable_pointcloud)

        config = config.with_overrides({
            "headless": _as_bool(self.get_parameter('headless').value, config.headless),
            "sensors.lidar_2d.enabled": _as_bool(self.get_parameter('enable_lidar').value, config.sensors.lidar_2d.enabled),
            "sensors.mid360.enabled": _as_bool(self.get_parameter('enable_mid360').value, config.sensors.mid360.enabled),
            "sensors.realsense.enable_depth": _as_bool(self.get_parameter('enable_depth').value, config.sensors.realsense.enable_depth),
            "sensors.realsense.enable_color": _as_bool(self.get_parameter('enable_color').value, config.sensors.realsense.enable_color),
            "sensors.realsense.enable_pointcloud": _as_bool(self.get_parameter('enable_pointcloud').value, config.sensors.realsense.enable_pointcloud),
        })

        headless = config.headless
        enable_lidar = config.sensors.lidar_2d.enabled
        enable_mid360 = config.sensors.mid360.enabled
        enable_depth = config.sensors.realsense.enable_depth
        enable_color = config.sensors.realsense.enable_color
        use_viewer = USE_VIEWER and (not headless)

        # Load model via MjSpec so sensors can attach before compile
        robot_xml_path = config.resolved_robot_description()
        scene_path = config.resolved_scene()
        if not os.path.isfile(robot_xml_path):
            raise FileNotFoundError(f"Cannot find robot MJCF: {robot_xml_path}")

        spec = _build_static_spec(scene_path, robot_xml_path)
        if scene_path:
            self.get_logger().info(f"[INFO] Using scene: {scene_path}")
        else:
            self.get_logger().info("[INFO] No scene configured; using robot model with generated floor")

        # Sensor-driven MjSpec mutations (must happen before compile)
        if enable_depth or enable_color:
            DepthSensor.init_visuals(spec, config.sensors.realsense)
            self.get_logger().info("[INFO] D435i model attached via mjSpec")
        if enable_lidar:
            LidarSensor.init_visuals(spec, config.sensors.lidar_2d)
        if enable_mid360:
            Mid360LidarSensor.init_visuals(spec, config.sensors.mid360)
            self.get_logger().info("[INFO] Mid360 model attached via mjSpec")

        self.model = spec.compile()
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..15
        self.dof_num = len(self.actuator_ids)
        assert self.dof_num == 16, "Expected 16 DOF for M20"

        # 初始化站立姿态
        self._set_initial_pose(model_key)

        # 缓存
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0

        self.get_logger().info(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # ROS Publishers
        self.imu_pub = self.create_publisher(ImuData, '/IMU_DATA', 200)
        self.joints_pub = self.create_publisher(JointsData, '/JOINTS_DATA', 200)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 50)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        # TF Broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ROS Subscriber
        self.cmd_sub = self.create_subscription(
            JointsDataCmd,
            '/JOINTS_CMD',
            self._cmd_callback,
            50
        )

        # 可视化
        self.viewer = None
        if use_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        else:
            self.get_logger().info("[INFO] Running MuJoCo in headless mode (viewer disabled)")

        # Sensors
        self.lidar = LidarSensor(self.model, self.data, self, self.viewer, config=config.sensors.lidar_2d)
        self.lidar_step_interval = max(1, int(1.0 / (config.sensors.lidar_2d.frequency_hz * DT)))

        self.mid360 = Mid360LidarSensor(self.model, self.data, self, self.viewer, config=config.sensors.mid360)
        self.mid360_step_interval = max(1, int(1.0 / (config.sensors.mid360.frequency_hz * DT)))

        self.depth = DepthSensor(self.model, self.data, self, self.viewer, config=config.sensors.realsense)
        self.depth_step_interval = max(1, int(1.0 / (config.sensors.realsense.frequency_hz * DT)))

        # Publish initial clock + static TFs
        self._publish_clock(self._make_sim_stamp())
        self._publish_static_transforms()

    def _set_initial_pose(self, key: str):
        """关节位置设置为与 PyBullet 脚本一致的初始角度"""
        qpos0 = self.data.qpos.copy()
        qpos0[7:7 + self.dof_num] = JOINT_INIT[key]  # ,3-6 basequat，0-2 basepos
        qpos0[:3] = np.array([0, 0, 0.2])
        qpos0[3:7] = np.array([1, 0, 0, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

    def _cmd_callback(self, msg: JointsDataCmd):
        """Convert received (published) positions/velocities to internal (raw)"""
        if len(msg.data.joints_data) != 16:
            self.get_logger().warn("Received JointsDataCmd with incorrect number of joints")
            return

        pub_pos = np.zeros(self.dof_num, dtype=np.float32)
        pub_vel = np.zeros(self.dof_num, dtype=np.float32)
        for i in range(self.dof_num):
            joint_cmd = msg.data.joints_data[i]
            self.kp_cmd[i] = joint_cmd.kp
            self.kd_cmd[i] = joint_cmd.kd
            pub_pos[i] = joint_cmd.position
            pub_vel[i] = joint_cmd.velocity
            self.tau_ff[i] = joint_cmd.torque  # tau_ff no processing

        # Convert: raw = published * dir + offset_rad
        self.pos_cmd.flat = pub_pos * JOINT_DIR + POS_OFFSET_RAD
        self.vel_cmd.flat = pub_vel * JOINT_DIR

    @staticmethod
    def _stamp_from_seconds(timestamp: float) -> Time:
        stamp = Time()
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def _make_sim_stamp(self, timestamp: float | None = None) -> Time:
        if timestamp is None:
            timestamp = self.timestamp
        return self._stamp_from_seconds(timestamp)

    def _publish_clock(self, stamp: Time):
        clock_msg = Clock()
        clock_msg.clock = stamp
        self.clock_pub.publish(clock_msg)

    def _publish_static_transforms(self):
        stamp = self._make_sim_stamp()
        transforms = []
        transforms.extend(self.lidar.get_static_transforms(stamp))
        transforms.extend(self.mid360.get_static_transforms(stamp))
        transforms.extend(self.depth.get_static_transforms(stamp))
        if transforms:
            self.static_tf_broadcaster.sendTransform(transforms)

    def _publish_odom_and_tf(self, stamp: Time, publish_odom: bool = True):
        pos = self.data.qpos[0:3]
        quat_wxyz = self.data.qpos[3:7]
        linvel = self.data.qvel[0:3]
        angvel = self.data.qvel[3:6]

        R = Rotation.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        body_linvel = R.inv().apply(linvel)
        body_angvel = R.inv().apply(angvel)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation = Vector3(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        t.transform.rotation = Quaternion(
            x=float(quat_wxyz[1]), y=float(quat_wxyz[2]),
            z=float(quat_wxyz[3]), w=float(quat_wxyz[0])
        )
        self.tf_broadcaster.sendTransform(t)

        if not publish_odom:
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])
        odom_msg.pose.pose.orientation.x = float(quat_wxyz[1])
        odom_msg.pose.pose.orientation.y = float(quat_wxyz[2])
        odom_msg.pose.pose.orientation.z = float(quat_wxyz[3])
        odom_msg.pose.pose.orientation.w = float(quat_wxyz[0])
        odom_msg.twist.twist.linear.x = float(body_linvel[0])
        odom_msg.twist.twist.linear.y = float(body_linvel[1])
        odom_msg.twist.twist.linear.z = float(body_linvel[2])
        odom_msg.twist.twist.angular.x = float(body_angvel[0])
        odom_msg.twist.twist.angular.y = float(body_angvel[1])
        odom_msg.twist.twist.angular.z = float(body_angvel[2])
        self.odom_pub.publish(odom_msg)

    def start(self):
        # 主模拟循环
        step = 0
        last_time = time.time()
        while rclpy.ok():
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1
                # 控制律
                self._apply_joint_torque()
                # 模拟一步
                mujoco.mj_step(self.model, self.data)

                self.timestamp = step * DT
                stamp = self._make_sim_stamp(self.timestamp)
                self._publish_clock(stamp)

                # TF (every step for sensor lookups)
                self._publish_odom_and_tf(stamp, publish_odom=False)

                # 采样 & 发送观测 (every 5 steps for 200 Hz)
                if step % 5 == 0:
                    self._publish_robot_state(stamp)
                    self._publish_odom_and_tf(stamp)

                # LiDAR
                if step % self.lidar_step_interval == 0:
                    self.lidar.update(stamp)

                # Mid360
                if step % self.mid360_step_interval == 0:
                    self.mid360.update(stamp)

                # Depth camera
                if step % self.depth_step_interval == 0:
                    self.depth.update(stamp)

                # 可视化
                if self.viewer and step % RENDER_INTERVAL == 0:
                    self.lidar.visualize()
                    self.viewer.sync()

            # Handle ROS callbacks
            rclpy.spin_once(self, timeout_sec=0.0)

    def _apply_joint_torque(self):
        # 当前关节状态
        q = self.data.qpos[7:7 + self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[6:6 + self.dof_num].reshape(-1, 1)
        self.input_tq = (
                self.kp_cmd * (self.pos_cmd - q) +
                self.kd_cmd * (self.vel_cmd - dq) +
                self.tau_ff
        )

        # 写入 control 缓冲区
        self.data.ctrl[:] = self.input_tq.flatten()

    # --------------------------------------------------------
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # 防止数值漂移导致 |t2|>1
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    # --------------------------------------------------------

    def _publish_robot_state(self, stamp: Time):
        # ----- IMU -----
        q_world = self.data.sensordata[:4]  # quaternion (w, x, y, z) in MuJoCo convention
        rpy_rad = self.quaternion_to_euler(q_world)  # returns [roll, pitch, yaw] in radians

        # Convert to degrees
        rpy_deg = [angle * (180.0 / 3.141592653589793) for angle in rpy_rad]

        body_acc = self.data.sensordata[4:7]
        angvel_b = self.data.sensordata[7:10]  # body frame

        imu_msg = ImuData()
        imu_msg.header = MetaType()
        imu_msg.header.frame_id = 0
        imu_msg.header.stamp = stamp
        imu_msg.data = ImuDataValue()
        imu_msg.data.roll = float(rpy_deg[0])
        imu_msg.data.pitch = float(rpy_deg[1])
        imu_msg.data.yaw = float(rpy_deg[2])
        imu_msg.data.omega_x = float(angvel_b[0])
        imu_msg.data.omega_y = float(angvel_b[1])
        imu_msg.data.omega_z = float(angvel_b[2])
        imu_msg.data.acc_x = float(body_acc[0])
        imu_msg.data.acc_y = float(body_acc[1])
        imu_msg.data.acc_z = float(body_acc[2])
        self.imu_pub.publish(imu_msg)

        # ----- 关节 -----
        q = self.data.qpos[7:7 + self.dof_num]
        dq = self.data.qvel[6:6 + self.dof_num]
        tau = self.input_tq.flatten()

        # Convert raw to published: published = (raw - offset_rad) * dir
        pub_pos = (q - POS_OFFSET_RAD) * JOINT_DIR
        pub_vel = dq * JOINT_DIR
        pub_tau = tau * JOINT_DIR  # Torque also needs direction flip
        
        joints_msg = JointsData()
        joints_msg.header = MetaType()
        joints_msg.header.frame_id = 0
        joints_msg.header.stamp = stamp
        joints_msg.data = JointsDataValue()
        joints_msg.data.joints_data = [JointData() for _ in range(self.dof_num)]
        for i in range(self.dof_num):
            joint = joints_msg.data.joints_data[i]
            joint.name = [32, 32, 32, 32]  # Dummy name (four spaces)
            joint.data_id = 0  # Dummy
            joint.status_word = 1  # Normal
            joint.position = float(pub_pos[i])
            joint.torque = float(pub_tau[i])
            joint.velocity = float(pub_vel[i])
            joint.motion_temp = 40.0  # Dummy normal temp
            joint.driver_temp = 45.0  # Dummy normal temp
        self.joints_pub.publish(joints_msg)


def run_mujoco(config: SimulationConfig, ros_args: list[str] | None = None):
    rclpy.init(args=ros_args)
    sim_node = MuJoCoSimulationNode(config=config)
    try:
        sim_node.start()
    finally:
        sim_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    np.set_printoptions(precision=4, suppress=True)
    parser = argparse.ArgumentParser(description="Run M20 MuJoCo ROS2 simulation")
    parser.add_argument("--config", default=None, help="Path to simulation YAML config")
    args, ros_args = parser.parse_known_args()

    config = SimulationConfig.load(args.config)
    errors = config.validate()
    if errors:
        raise SystemExit("Invalid simulation config:\n- " + "\n- ".join(errors))
    run_mujoco(config, ros_args)


if __name__ == "__main__":
    main()
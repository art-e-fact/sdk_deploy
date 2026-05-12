"""
Optional MuJoCo follow camera video recorder.

This module renders an offscreen third-person RGB view that follows the robot
and writes frames to a video file when enabled by the simulation node.
"""

import math
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np


FOLLOW_CAMERA_FPS = 20.0
FOLLOW_CAMERA_WIDTH = 480
FOLLOW_CAMERA_HEIGHT = 320
FOLLOW_CAMERA_DISTANCE_M = 4.0
FOLLOW_CAMERA_ELEVATION_DEG = -18.0
FOLLOW_CAMERA_AZIMUTH_OFFSET_DEG = 60.0
FOLLOW_CAMERA_TARGET_HEIGHT_M = 0.55
FOLLOW_CAMERA_SMOOTHING = 0.08
FOLLOW_CAMERA_QUALITY = 8


class FollowCameraRecorder:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        node,
        enabled: bool = False,
        video_path: str = "",
    ):
        self.model = model
        self.data = data
        self.node = node
        self.enabled = bool(enabled)
        self.video_path = str(video_path).strip()
        self.fps = FOLLOW_CAMERA_FPS
        self.closed = True
        self.frame_count = 0
        self._lookat = None

        if not self.enabled:
            node.get_logger().info("[INFO] Follow camera recorder disabled")
            self.renderer = None
            self.writer = None
            self.camera = None
            return

        if not self.video_path:
            raise ValueError("follow_camera_video_path must be set when enable_follow_camera is true")

        output_path = Path(self.video_path).expanduser()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.video_path = str(output_path)

        self.renderer = mujoco.Renderer(
            model,
            height=FOLLOW_CAMERA_HEIGHT,
            width=FOLLOW_CAMERA_WIDTH,
        )
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self.camera)
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = FOLLOW_CAMERA_DISTANCE_M
        self.camera.elevation = FOLLOW_CAMERA_ELEVATION_DEG

        self.writer = imageio.get_writer(
            self.video_path,
            fps=self.fps,
            codec="libx264",
            quality=FOLLOW_CAMERA_QUALITY,
            macro_block_size=16,
        )
        self.closed = False
        node.get_logger().info(
            f"[INFO] Follow camera recording enabled: {self.video_path} "
            f"({FOLLOW_CAMERA_WIDTH}x{FOLLOW_CAMERA_HEIGHT} @ {self.fps:.1f} Hz)"
        )

    def update(self):
        if not self.enabled or self.closed:
            return

        self._update_camera()
        self.renderer.update_scene(self.data, camera=self.camera)
        frame_rgb = self.renderer.render()
        self.writer.append_data(frame_rgb)
        self.frame_count += 1

    def close(self):
        if not self.enabled or self.closed:
            return

        self.closed = True
        try:
            self.writer.close()
        finally:
            self.renderer.close()

        self.node.get_logger().info(
            f"[INFO] Follow camera video saved: {self.video_path} ({self.frame_count} frames)"
        )

    def _update_camera(self):
        target = np.array(self.data.qpos[0:3], dtype=np.float64)
        target[2] += FOLLOW_CAMERA_TARGET_HEIGHT_M
        if self._lookat is None:
            self._lookat = target
        else:
            alpha = float(np.clip(FOLLOW_CAMERA_SMOOTHING, 0.0, 1.0))
            self._lookat = (1.0 - alpha) * self._lookat + alpha * target

        self.camera.lookat[:] = self._lookat
        self.camera.distance = FOLLOW_CAMERA_DISTANCE_M
        self.camera.elevation = FOLLOW_CAMERA_ELEVATION_DEG
        self.camera.azimuth = self._robot_yaw_deg() + FOLLOW_CAMERA_AZIMUTH_OFFSET_DEG

    def _robot_yaw_deg(self) -> float:
        w, x, y, z = self.data.qpos[3:7]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

"""
In-process ONNX policy runner for the Lite3 MuJoCo simulation.

Mirrors the deployed C++ policy runner (`lite3_policy_runner.hpp`) so the
policy can be exercised end-to-end inside the same MuJoCo process that is
used elsewhere for SDK deployment. Designed for sim-to-sim debugging: all
the quantities that typically cause Isaac-Lab vs MuJoCo gaps are exposed
as dataclass fields so they can be tweaked quickly during testing.

Observation layout (must match training):
    [base_ang_vel * ang_vel_scale,                  # 3
     projected_gravity,                             # 3
     velocity_command,                              # 3  (vx, vy, wz, unscaled)
     (joint_pos - joint_default) * joint_pos_scale, # 12
     joint_vel * joint_vel_scale,                   # 12
     last_action]                                   # 12  (raw policy output)
Total: 45.

Actions are converted to per-joint position targets with:
    target = action * action_scale + joint_default
A torque is then applied via `tau = kp * (target - q) + kd * (0 - qd)`.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Sequence

import mujoco
import numpy as np
import onnxruntime as ort

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


# Default joint order used both for Isaac Lab training AND the C++ deploy
# runner. The MuJoCo joint IDs are resolved by name so that this order does
# not have to match the MJCF author order.
DEFAULT_JOINT_ORDER: List[str] = [
    "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
    "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
    "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
    "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
]

# Per-joint action scale matching `Lite3PolicyRunner::action_scale_robot`
# and the Isaac-Lab training config
# (`self.actions.joint_pos.scale = {".*_HipX_joint": 0.125, ...: 0.25}`).
DEFAULT_ACTION_SCALE: List[float] = [
    0.125, 0.25, 0.25,
    0.125, 0.25, 0.25,
    0.125, 0.25, 0.25,
    0.125, 0.25, 0.25,
]

# Default standing pose used as the policy offset
# (`dof_default_eigen_policy` in the C++ runner).
DEFAULT_JOINT_POS: List[float] = [
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
    0.0, -0.8, 1.6,
]


@dataclass
class PolicyConfig:
    """All knobs that typically need to change when debugging sim-to-sim."""

    onnx_path: str = ""

    # --- control loop ---
    # Sim step is 0.001 s, policy trained at 50 Hz -> 20 sim steps / inference
    decimation: int = 20
    # Extra delay (in policy steps) between action computation and being
    # applied. 0 matches an instant actuator; Isaac's DelayedPDActuatorCfg
    # used min_delay=0, max_delay=1 during training. Try 0 or 1.
    action_delay_steps: int = 0

    # --- observation scales (must match training cfg) ---
    ang_vel_scale: float = 0.25
    joint_pos_scale: float = 1.0
    joint_vel_scale: float = 0.05
    # Linear/angular command scales: Isaac applies no scale on commands,
    # they are passed raw to the policy. Expose for experimentation.
    cmd_lin_scale: float = 1.0
    cmd_ang_scale: float = 1.0

    # --- action scales (must match training cfg) ---
    action_scale: Sequence[float] = field(default_factory=lambda: list(DEFAULT_ACTION_SCALE))
    # Optional symmetric clip on the raw policy output before scaling.
    # Isaac training used clip=(-100, 100) for this env, so effectively off.
    action_clip: float | None = None

    # --- PD gains (match `Lite3PolicyRunner` and training actuators) ---
    kp: float = 30.0
    kd: float = 1.0

    # --- standing default pose ---
    joint_default: Sequence[float] = field(default_factory=lambda: list(DEFAULT_JOINT_POS))

    # --- joint ordering (ONNX expected order) ---
    joint_order: Sequence[str] = field(default_factory=lambda: list(DEFAULT_JOINT_ORDER))

    # --- command source ---
    cmd_topic: str = "/cmd_vel"
    cmd_timeout_sec: float = 0.5  # zero the command if stale

    # --- body used as IMU / root. MuJoCo free joint expected on this body. ---
    base_body_name: str = "TORSO"

    # --- behaviour ---
    # Warm-up: number of initial sim steps to hold the default pose before
    # letting the policy drive the robot. Helps avoid large initial deltas
    # that can saturate the action and trip the robot on spawn.
    warmup_steps: int = 500

    # Verbose per-inference debug prints (set True when comparing Isaac/MuJoCo
    # traces side-by-side). Spammy at 50 Hz.
    debug_print: bool = False


class PolicyRunner:
    """Runs an ONNX locomotion policy against a MuJoCo model/data pair.

    Usage inside the simulation loop:

        runner = PolicyRunner(node, model, data, PolicyConfig(onnx_path=...))
        for step in range(...):
            runner.update(step)  # run inference on decimation boundary
            kp, kd, pos_cmd, vel_cmd, tau_ff = runner.get_joint_command()
            # then apply PD as usual
            mujoco.mj_step(model, data)
    """

    def __init__(
        self,
        node: Node,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: PolicyConfig,
    ) -> None:
        self.node = node
        self.model = model
        self.data = data
        self.cfg = config

        if not config.onnx_path:
            raise ValueError("PolicyConfig.onnx_path is required")

        self._dof = len(config.joint_order)
        if self._dof != 12:
            raise ValueError(f"Expected 12 joints in order, got {self._dof}")
        if len(config.action_scale) != self._dof:
            raise ValueError("action_scale length must match joint_order")
        if len(config.joint_default) != self._dof:
            raise ValueError("joint_default length must match joint_order")

        # Resolve joint-name -> (qpos index, qvel index) for the free-base
        # model. Each hinge joint contributes one qpos/qvel slot.
        self._qpos_idx, self._qvel_idx = self._resolve_joint_indices(model, config.joint_order)

        # Resolve actuator-name -> ctrl index for commanding torques in the
        # caller's order. Assumes actuators are named "<joint>_ctrl" (as in
        # Lite3.xml). Falls back to matching the actuator's `trnid` if not.
        self._ctrl_idx = self._resolve_actuator_indices(model, config.joint_order)

        # Resolve base body id
        self._base_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, config.base_body_name
        )
        if self._base_id < 0:
            raise ValueError(f"Body '{config.base_body_name}' not in model")

        # Pre-allocate arrays reused every inference
        self._joint_default = np.asarray(config.joint_default, dtype=np.float32)
        self._action_scale = np.asarray(config.action_scale, dtype=np.float32)
        self._last_action = np.zeros(self._dof, dtype=np.float32)  # obs feedback
        self._current_action = np.zeros(self._dof, dtype=np.float32)

        # Delay buffer holds past raw actions; applied action is popped
        # `action_delay_steps` steps after inference.
        buf_len = max(1, config.action_delay_steps + 1)
        self._action_buf: deque[np.ndarray] = deque(
            [np.zeros(self._dof, dtype=np.float32) for _ in range(buf_len)],
            maxlen=buf_len,
        )

        # Cached joint command (what _apply_joint_torque in the node reads)
        self._pos_cmd = self._joint_default.copy().reshape(-1, 1)
        self._vel_cmd = np.zeros((self._dof, 1), dtype=np.float32)
        self._tau_ff = np.zeros((self._dof, 1), dtype=np.float32)
        self._kp = np.full((self._dof, 1), config.kp, dtype=np.float32)
        self._kd = np.full((self._dof, 1), config.kd, dtype=np.float32)

        # Latest velocity command from /cmd_vel
        self._cmd_vel = np.zeros(3, dtype=np.float32)
        self._last_cmd_wall_time = 0.0

        # ONNX session
        so = ort.SessionOptions()
        so.intra_op_num_threads = 2
        self._session = ort.InferenceSession(
            config.onnx_path,
            sess_options=so,
            providers=["CPUExecutionProvider"],
        )
        inputs = self._session.get_inputs()
        outputs = self._session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise RuntimeError("Expected a single-input/single-output ONNX policy")
        self._input_name = inputs[0].name
        self._output_name = outputs[0].name

        # Gravity direction in world frame (policy expects projected gravity
        # in body frame). MuJoCo stores gravity as model.opt.gravity.
        g = np.asarray(model.opt.gravity, dtype=np.float32)
        g_norm = np.linalg.norm(g)
        self._gravity_world = (g / g_norm) if g_norm > 1e-6 else np.array([0, 0, -1], np.float32)

        # ROS subscription for velocity commands. BEST_EFFORT + KEEP_LAST(1)
        # because we only ever care about the freshest teleop command and
        # must not let the transport queue stale twists behind live ones.
        cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._cmd_sub = node.create_subscription(
            Twist, config.cmd_topic, self._cmd_cb, cmd_qos
        )

        self._step_count = 0

        node.get_logger().info(
            "[PolicyRunner] Loaded '%s' | decimation=%d | action_delay=%d | "
            "kp=%.1f kd=%.2f | ang_vel_scale=%.3f joint_vel_scale=%.3f"
            % (
                config.onnx_path,
                config.decimation,
                config.action_delay_steps,
                config.kp,
                config.kd,
                config.ang_vel_scale,
                config.joint_vel_scale,
            )
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _cmd_cb(self, msg: Twist) -> None:
        print(f"cmd_cb: {msg}")
        self._cmd_vel[0] = float(msg.linear.x) * self.cfg.cmd_lin_scale
        self._cmd_vel[1] = float(msg.linear.y) * self.cfg.cmd_lin_scale
        self._cmd_vel[2] = float(msg.angular.z) * self.cfg.cmd_ang_scale
        self._last_cmd_wall_time = _monotonic()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _resolve_joint_indices(
        model: mujoco.MjModel, names: Sequence[str]
    ) -> tuple[List[int], List[int]]:
        qpos_idx: List[int] = []
        qvel_idx: List[int] = []
        for name in names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"Joint '{name}' not found in MuJoCo model")
            qpos_idx.append(int(model.jnt_qposadr[jid]))
            qvel_idx.append(int(model.jnt_dofadr[jid]))
        return qpos_idx, qvel_idx

    @staticmethod
    def _resolve_actuator_indices(
        model: mujoco.MjModel, joint_names: Sequence[str]
    ) -> List[int]:
        # Map joint id -> actuator id via `actuator_trnid`.
        joint_to_act: Dict[int, int] = {}
        for a in range(model.nu):
            jid = int(model.actuator_trnid[a, 0])
            if jid >= 0:
                joint_to_act[jid] = a
        ctrl_idx: List[int] = []
        for name in joint_names:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid not in joint_to_act:
                raise ValueError(
                    f"No actuator actuates joint '{name}' in the MuJoCo model"
                )
            ctrl_idx.append(joint_to_act[jid])
        return ctrl_idx

    @property
    def ctrl_indices(self) -> List[int]:
        """Ctrl indices corresponding to `joint_order` (for custom wiring)."""
        return list(self._ctrl_idx)

    # ------------------------------------------------------------------
    # State extraction
    # ------------------------------------------------------------------
    def _read_joint_state(self) -> tuple[np.ndarray, np.ndarray]:
        q = np.asarray(
            [self.data.qpos[i] for i in self._qpos_idx], dtype=np.float32
        )
        dq = np.asarray(
            [self.data.qvel[i] for i in self._qvel_idx], dtype=np.float32
        )
        return q, dq

    def _read_base_state(self) -> tuple[np.ndarray, np.ndarray]:
        """Return (body-frame ang vel, body-frame projected gravity)."""
        # xmat is a row-major 3x3 rotation (world_R_body).
        R_wb = np.asarray(self.data.xmat[self._base_id], dtype=np.float32).reshape(3, 3)
        # body-frame angular velocity. MuJoCo exposes it directly on cvel
        # (spatial velocity) for the body; using qvel is free-joint specific.
        # We use cvel to be robust:
        # cvel[:3] = angular velocity in world frame expressed at the body
        # origin, cvel[3:] = linear. Transform angular to body frame.
        cvel = np.asarray(self.data.cvel[self._base_id], dtype=np.float32)
        ang_vel_world = cvel[:3]
        ang_vel_body = R_wb.T @ ang_vel_world
        projected_gravity = R_wb.T @ self._gravity_world
        return ang_vel_body.astype(np.float32), projected_gravity.astype(np.float32)

    def _get_command(self) -> np.ndarray:
        # Zero the command if /cmd_vel is stale (acts as a safety).
        if self._last_cmd_wall_time > 0.0:
            if (_monotonic() - self._last_cmd_wall_time) > self.cfg.cmd_timeout_sec:
                return np.zeros(3, dtype=np.float32)
        return self._cmd_vel.copy()

    # ------------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------------
    def _infer(self) -> np.ndarray:
        q, dq = self._read_joint_state()
        ang_vel_b, proj_g = self._read_base_state()
        cmd = self._get_command()

        obs = np.empty(3 + 3 + 3 + self._dof * 3, dtype=np.float32)
        obs[0:3] = ang_vel_b * self.cfg.ang_vel_scale
        obs[3:6] = proj_g
        obs[6:9] = cmd
        obs[9:9 + self._dof] = (q - self._joint_default) * self.cfg.joint_pos_scale
        obs[9 + self._dof:9 + 2 * self._dof] = dq * self.cfg.joint_vel_scale
        obs[9 + 2 * self._dof:] = self._last_action

        out = self._session.run(
            [self._output_name],
            {self._input_name: obs[None, :]},
        )[0]
        action = np.asarray(out, dtype=np.float32).reshape(-1)
        if self.cfg.action_clip is not None:
            np.clip(action, -self.cfg.action_clip, self.cfg.action_clip, out=action)

        # Update last_action with the freshly computed action (before delay);
        # this matches how Isaac feeds `actions` as obs (the value used by
        # the policy to produce the target, regardless of actuation delay).
        self._last_action = action.copy()

        if self.cfg.debug_print:
            self.node.get_logger().info(
                f"[PolicyRunner] obs(ang)={obs[0:3]} gproj={obs[3:6]} "
                f"cmd={obs[6:9]} act={action[:3]}..."
            )
        return action

    # ------------------------------------------------------------------
    # Main entry points
    # ------------------------------------------------------------------
    def update(self, step: int) -> None:
        """Called every sim step; runs inference on decimation boundaries."""
        self._step_count = step

        # During warm-up we hold the standing pose to let the robot settle.
        if step < self.cfg.warmup_steps:
            self._pos_cmd[:, 0] = self._joint_default
            return

        # Run inference on the decimation boundary (measured from the end
        # of warm-up for a clean 50 Hz cadence).
        phase = (step - self.cfg.warmup_steps) % self.cfg.decimation
        if phase == 0:
            action = self._infer()
            # Push into delay buffer; the applied action is read off the
            # other end.
            self._action_buf.append(action)
            applied = self._action_buf[0]
            target = applied * self._action_scale + self._joint_default
            self._pos_cmd[:, 0] = target
            self._current_action = applied

    def get_joint_command(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Return (kp, kd, pos_cmd, vel_cmd, tau_ff) each shape (12, 1).

        The arrays are ordered according to `cfg.joint_order` (= ONNX order),
        NOT necessarily MuJoCo's internal order. Use `ctrl_indices` to wire
        them into `data.ctrl`.
        """
        return self._kp, self._kd, self._pos_cmd, self._vel_cmd, self._tau_ff

    def apply_torque(self) -> None:
        """Compute and write PD torques directly into `data.ctrl`.

        Convenience path used when the simulation node delegates actuation
        entirely to the policy runner. Mirrors
        `MuJoCoSimulationNode._apply_joint_torque` but indexed by
        `joint_order`.
        """
        q, dq = self._read_joint_state()
        # broadcast PD in policy order
        tau = (
            self._kp[:, 0] * (self._pos_cmd[:, 0] - q)
            + self._kd[:, 0] * (self._vel_cmd[:, 0] - dq)
            + self._tau_ff[:, 0]
        )
        for i, cidx in enumerate(self._ctrl_idx):
            self.data.ctrl[cidx] = float(tau[i])


def _monotonic() -> float:
    # Imported lazily to keep the top of the module focused on the public API.
    import time as _t

    return _t.monotonic()

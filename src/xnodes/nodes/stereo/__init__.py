"""Stereo anchor TF publisher with smoothing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from transforms3d.quaternions import mat2quat, quat2mat

from ...core import ema_quat


# helpers so we can use transforms3d which works with pixi/ros
def quat_xyzw_to_matrix4(q):
    x, y, z, w = q
    R = quat2mat([w, x, y, z])
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    return T


def matrix4_to_quat_xyzw(T):
    w, x, y, z = mat2quat(T[:3, :3])
    return np.array([x, y, z, w], dtype=float)


@dataclass
class Config:
    anchor: str
    floaters: List[str]
    at: str
    ft: List[str]
    world: str = "world"
    smooth: float = 10.0


class PoseEMA:
    """EMA for pose translation and rotation."""

    def __init__(self, alpha: float):
        self.alpha = float(np.clip(alpha, 0.0, 1.0))
        self._q: Optional[np.ndarray] = None
        self._t: Optional[np.ndarray] = None

    def update(self, pose: PoseStamped) -> np.ndarray:
        pos = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ], dtype=float)
        q_meas = np.array(
            [
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
            ],
            dtype=float,
        )

        if self._t is None or self.alpha >= 1.0:
            self._t = pos
        elif self.alpha > 0.0:
            self._t = (1.0 - self.alpha) * self._t + self.alpha * pos

        if self._q is None or self.alpha >= 1.0:
            self._q = q_meas
        elif self.alpha > 0.0:
            self._q = ema_quat(self._q, q_meas, self.alpha)

        if self._t is None or self._q is None:
            raise RuntimeError("EMA has not been initialized")

        q_xyzw = np.array([self._q[1], self._q[2], self._q[3], self._q[0]], dtype=float)
        T = quat_xyzw_to_matrix4(q_xyzw)
        T[:3, 3] = self._t
        return T


def mat_to_tf(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = T[:3, 3]
    q = matrix4_to_quat_xyzw(T)
    msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
    return msg


class StereoAnchorNode(Node):
    """Publishes smoothed TFs from anchor/at/ft poses."""

    def __init__(self) -> None:
        super().__init__("stereo_anchor_node")

        self.cfg = Config(
            anchor=self.declare_parameter("anchor", "anchor").value,
            floaters=self.declare_parameter("floaters", ["cam_low", "cam_side"]).value,
            at=self.declare_parameter("at", "/anchor_tracker/pose").value,
            ft=self.declare_parameter("ft", ["/cam_low/pose", "/cam_side/pose"]).value,
            world=self.declare_parameter("world", "world").value,
            smooth=float(self.declare_parameter("smooth", 10.0).value),
        )

        if len(self.cfg.floaters) != len(self.cfg.ft):
            self.get_logger().error("floaters and ft lists must be same length")
            raise RuntimeError("Config mismatch")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        alpha = 1.0 / max(self.cfg.smooth, 1.0)
        self._at_ema = PoseEMA(alpha)
        self._ft_emas: Dict[str, PoseEMA] = {f: PoseEMA(alpha) for f in self.cfg.floaters}

        self.latest_at: Optional[np.ndarray] = None
        self.latest_ft: Dict[str, np.ndarray] = {}

        self.create_subscription(PoseStamped, self.cfg.at, self._on_at, 10)
        for floater, topic in zip(self.cfg.floaters, self.cfg.ft):
            self.create_subscription(PoseStamped, topic, self._make_ft_cb(floater), 10)

        self.timer = self.create_timer(1.0 / 30.0, self._tick)

    def _on_at(self, msg: PoseStamped) -> None:
        self.latest_at = self._at_ema.update(msg)

    def _make_ft_cb(self, floater_name: str):
        ema = self._ft_emas[floater_name]

        def cb(msg: PoseStamped) -> None:
            self.latest_ft[floater_name] = ema.update(msg)

        return cb

    def _tick(self) -> None:
        if self.latest_at is None:
            return

        stamp = self.get_clock().now().to_msg()

        try:
            t_anchor = self.tf_buffer.lookup_transform(self.cfg.world, self.cfg.anchor, rclpy.time.Time())
            T_world_anchor = self._tf_to_mat(t_anchor)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(
                f"Waiting for TF {self.cfg.world}->{self.cfg.anchor}: {e}",
                throttle_duration_sec=5.0,
            )
            return

        T_anchor_at = self.latest_at

        for floater, T_at_ft in list(self.latest_ft.items()):
            T_world_ft = T_world_anchor @ T_anchor_at @ T_at_ft
            tf_msg = mat_to_tf(T_world_ft, self.cfg.world, floater, stamp)
            self.tf_broadcaster.sendTransform(tf_msg)

    @staticmethod
    def _tf_to_mat(t: TransformStamped) -> np.ndarray:
        q = [t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        T = quat_xyzw_to_matrix4(q)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        return T


def main() -> None:
    rclpy.init()
    node = StereoAnchorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


__all__ = ["Config", "PoseEMA", "StereoAnchorNode", "main"]

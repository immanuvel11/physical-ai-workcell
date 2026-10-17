"""Small rigid-body helpers (numpy only, no tf_transformations dependency)."""
import math

import numpy as np
from geometry_msgs.msg import Quaternion


def yaw_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def quat_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tool_down_quat(yaw):
    """Orientation of a top-down tool (z axis pointing at the table) rotated by yaw: Rz(yaw) * Rx(pi)."""
    return Quaternion(x=math.cos(yaw / 2), y=math.sin(yaw / 2), z=0.0, w=0.0)


def wrap_half_pi(a):
    """Wrap an angle to [-pi/2, pi/2): a parallel gripper is symmetric under a 180 deg turn."""
    return (a + math.pi / 2) % math.pi - math.pi / 2


def transform_points(tf, pts):
    """Apply a geometry_msgs/TransformStamped to an (N, 3) array."""
    t, q = tf.transform.translation, tf.transform.rotation
    return pts @ quat_mat(q).T + np.array([t.x, t.y, t.z])


def point_in_rect(px, py, cx, cy, yaw, sx, sy, margin=0.0):
    """Is (px, py) inside the yawed rectangle centred at (cx, cy) with side lengths sx, sy?"""
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = px - cx, py - cy
    lx, ly = c * dx - s * dy, s * dx + c * dy
    return abs(lx) <= sx / 2 + margin and abs(ly) <= sy / 2 + margin

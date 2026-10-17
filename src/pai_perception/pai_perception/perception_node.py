"""Perception node.

    /perception/refresh (std_srvs/Trigger): segment the next camera frame and publish
    /perception/detections (pai_interfaces/SceneGraph, objects only) + /perception/debug_image.

backend:=geometric  colour + shape names from geometry only (offline, ~0.1 s)
backend:=claude     same geometry, plus Claude vision names each segment ("bolt", "coffee mug") and
                    can re-categorise it; results are cached per segment so unchanged objects are free.
"""
import json
import math
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from pai_core.claude import ClaudeClient, image_block
from pai_core.geometry import quat_mat, yaw_quat
from pai_interfaces.msg import DetectedObject, SceneGraph
from pai_perception.segment import annotate, backproject, flatten, robot_mask, segment, shape_word

ROBOT_CHAIN = ['panda_link0', 'panda_link1', 'panda_link2', 'panda_link3', 'panda_link4', 'panda_link5',
               'panda_link6', 'panda_link7', 'panda_hand', 'panda_tcp']
LABEL_SCHEMA = {
    'type': 'object',
    'properties': {'segments': {'type': 'array', 'items': {
        'type': 'object',
        'properties': {
            'index': {'type': 'integer'},
            'name': {'type': 'string', 'description': 'short common noun phrase, e.g. "bolt", "coffee mug"'},
            'category': {'type': 'string', 'enum': ['part', 'receptacle', 'obstacle']},
        },
        'required': ['index', 'name', 'category'], 'additionalProperties': False}}},
    'required': ['segments'], 'additionalProperties': False,
}
LABEL_SYSTEM = (
    'You label objects for a robot workcell. The image is a top-down camera view of a table. '
    'Numbered boxes mark segments found by a depth sensor; their measured colour, shape and size are listed. '
    'For every segment give a short everyday name (what a factory worker would call it) and a category: '
    'part = a small item the robot may move; receptacle = bin/tray/container that holds parts; '
    'obstacle = something the robot should not handle (cups, tools, hands, cables). '
    'Use the measured size, not only appearance. Never invent segments that are not listed.')


def snake(text):
    return re.sub(r'[^a-z0-9]+', '_', text.lower()).strip('_')


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        self.declare_parameter('backend', 'geometric')
        self.declare_parameter('table_bounds', [-0.22, 0.92, -0.67, 0.67])
        self.cb = ReentrantCallbackGroup()
        self.rgb = self.depth = self.K = None
        self.frame_stamp = 0.0
        self.label_cache = {}    # (colour, shape, x_cm//3, y_cm//3) -> (name, category, time)
        self.lock = threading.Lock()
        self.create_subscription(Image, '/camera/image', self._on_rgb, qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(Image, '/camera/depth_image', self._on_depth, qos_profile_sensor_data, callback_group=self.cb)
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_info, qos_profile_sensor_data, callback_group=self.cb)
        self.pub = self.create_publisher(SceneGraph, '/perception/detections', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 1)
        self.create_service(Trigger, '/perception/refresh', self._on_refresh, callback_group=self.cb)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        backend = self.get_parameter('backend').value
        self.claude = ClaudeClient(self.get_logger()) if backend == 'claude' else None
        if self.claude is not None and not self.claude.available:
            self.get_logger().warn(f'Claude labelling unavailable ({self.claude.reason}); using geometric names')
        self.last_latency = 0.0
        self.get_logger().info(f'Perception ready (backend: {backend})')

    def _on_rgb(self, m):
        self.rgb = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        self.frame_stamp = time.monotonic()

    def _on_depth(self, m):
        self.depth = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)

    def _on_info(self, m):
        self.K = np.array(m.k).reshape(3, 3)

    def _on_refresh(self, _req, res):
        t0 = time.monotonic()
        while self.frame_stamp < t0 + 0.05:          # use a frame captured after the request
            if time.monotonic() - t0 > 5.0:
                res.success, res.message = False, 'no camera frames'
                return res
            time.sleep(0.02)
        with self.lock:
            try:
                sg, summary = self.detect()
            except Exception as e:
                self.get_logger().error(f'perception failed: {e!r}')
                res.success, res.message = False, f'perception failed: {e!r}'
                return res
        self.pub.publish(sg)
        self.last_latency = time.monotonic() - t0
        res.success = True
        res.message = json.dumps({'objects': len(sg.objects), 'latency_s': round(self.last_latency, 2),
                                  'summary': summary})
        return res

    def detect(self):
        rgb, depth, K = self.rgb.copy(), self.depth.copy(), self.K
        cam = self.tf.lookup_transform('world', 'camera_optical_frame', Time()).transform
        R = quat_mat(cam.rotation)
        t = np.array([cam.translation.x, cam.translation.y, cam.translation.z])
        valid = np.isfinite(depth) & (depth > 0.1)
        xyz = backproject(np.where(valid, depth, 0.0), K, R, t)

        pts = []
        for f in ROBOT_CHAIN:
            p = self.tf.lookup_transform('world', f, Time()).transform.translation
            pts.append(np.array([p.x, p.y, p.z]))
        # radii: big near the base, 12 cm around wrist/hand (the hand body is 22 cm long)
        capsules = [(pts[i], pts[i + 1], 0.12 if i < 2 or i >= 6 else 0.09) for i in range(len(pts) - 1)]
        robot = robot_mask(xyz, capsules, valid)

        segs = segment(rgb, xyz, valid, tuple(self.get_parameter('table_bounds').value), robot)
        flat = flatten(segs)
        names = [f'{s.color} {shape_word(s)}' for s in flat]
        categories = [s.category for s in flat]
        if self.claude is not None and self.claude.available and flat:
            names, categories = self._claude_labels(rgb, segs, flat, names, categories)

        sg = SceneGraph()
        sg.header.stamp = self.get_clock().now().to_msg()
        sg.header.frame_id = 'world'
        for s, name, cat in zip(flat, names, categories):
            o = DetectedObject()
            o.label = name
            o.color, o.shape, o.category = s.color, s.shape, cat
            o.id = snake(name if name.startswith(s.color) else f'{s.color} {name.split()[-1]}')
            o.pose.position.x, o.pose.position.y, o.pose.position.z = (float(v) for v in s.center)
            o.pose.orientation = yaw_quat(s.yaw)
            o.size = Vector3(x=float(s.size[0]), y=float(s.size[1]), z=float(s.size[2]))
            o.confidence = float(min(1.0, s.pixels / 150.0))
            o.graspable = cat == 'part' and s.graspable
            sg.objects.append(o)

        dbg = annotate(rgb, segs, names)
        msg = Image(height=dbg.shape[0], width=dbg.shape[1], encoding='bgr8', step=dbg.shape[1] * 3,
                    data=dbg.tobytes())
        msg.header = sg.header
        self.debug_pub.publish(msg)
        return sg, [f'{o.id} ({o.category})' for o in sg.objects]

    def _claude_labels(self, rgb, segs, flat, names, categories):
        now = time.monotonic()
        keys = [(s.color, s.shape, round(s.center[0] * 100) // 3, round(s.center[1] * 100) // 3) for s in flat]
        missing = [i for i, k in enumerate(keys) if k not in self.label_cache or now - self.label_cache[k][2] > 300]
        if missing:
            ok, jpg = cv2.imencode('.jpg', annotate(rgb, segs, [str(i + 1) for i in range(len(flat))]),
                                   [cv2.IMWRITE_JPEG_QUALITY, 85])
            listing = '\n'.join(
                f'{i + 1}: colour {s.color}, {s.shape}, {s.size[0] * 100:.1f} x {s.size[1] * 100:.1f} cm footprint, '
                f'{s.size[2] * 100:.1f} cm tall, geometric guess: {s.category}' for i, s in enumerate(flat))
            data, err = self.claude.json_call(
                LABEL_SYSTEM, [image_block(jpg.tobytes()), {'type': 'text', 'text': f'Segments:\n{listing}'}],
                LABEL_SCHEMA, effort='low')
            if data is None:
                self.get_logger().warn(f'Claude labelling failed ({err}); using geometric names')
            else:
                for item in data['segments']:
                    i = item['index'] - 1
                    if 0 <= i < len(flat):
                        self.label_cache[keys[i]] = (item['name'].strip().lower(), item['category'], now)
                self.get_logger().info(f'Claude labelled {len(data["segments"])} segments '
                                       f'in {self.claude.last_latency:.1f}s')
        for i, k in enumerate(keys):
            if k in self.label_cache:
                name, cat, _ = self.label_cache[k]
                names[i] = name if name.startswith(flat[i].color) else f'{flat[i].color} {name}'
                # geometry has the last word on what can physically be grasped or is a receptacle
                if flat[i].category == 'receptacle' or cat == 'receptacle':
                    cat = flat[i].category
                categories[i] = cat
        return names, categories


def main():
    rclpy.init()
    node = PerceptionNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

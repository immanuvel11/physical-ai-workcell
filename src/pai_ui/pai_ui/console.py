"""Operator console: a small web UI served at http://localhost:8080 (local only).

GET  /                 the page
GET  /api/state        task status, plan, scene graph, safety (JSON)
GET  /api/camera.jpg   latest camera frame;  /api/perception.jpg  latest annotated perception frame
POST /api/run          {"instruction": "..."}      POST /api/cancel
POST /api/estop        POST /api/reset             POST /api/walk_in   (simulated co-worker)
"""
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from pai_interfaces.action import RunInstruction
from pai_interfaces.msg import SafetyStatus, SceneGraph, TaskStatus

PAGE = os.path.join(os.path.dirname(__file__), 'index.html')


class Console(Node):
    def __init__(self):
        super().__init__('operator_console')
        self.declare_parameter('port', 8080)
        self.state = {'task': {}, 'safety': {}, 'scene': {}, 'last_result': None, 'log': []}
        self.jpeg = {'camera': b'', 'perception': b''}
        self.goal_handle = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TaskStatus, '/pai/task_status', self._on_task, 10)
        self.create_subscription(SafetyStatus, '/safety/status', self._on_safety, 10)
        self.create_subscription(SceneGraph, '/scene_graph', self._on_scene, latched)
        self.create_subscription(Image, '/camera/image', lambda m: self._on_image(m, 'camera'), qos_profile_sensor_data)
        self.create_subscription(Image, '/perception/debug_image', lambda m: self._on_image(m, 'perception'), 1)
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', latched)
        self.reset_cli = self.create_client(Trigger, '/safety/reset')
        self.walk_cli = self.create_client(Trigger, '/safety/human_sim/walk_in')
        self.run_cli = ActionClient(self, RunInstruction, '/pai/run_instruction')
        self._frame_count = 0

    def _on_task(self, m):
        self.state['task'] = {'instruction': m.instruction, 'state': m.state, 'planner': m.planner,
                              'plan': json.loads(m.plan_json) if m.plan_json else None,
                              'step': m.current_step, 'message': m.message}

    def _on_safety(self, m):
        self.state['safety'] = {'state': m.state, 'speed_scale': round(m.speed_scale, 2),
                                'distance': None if m.min_human_distance == float('inf') else round(m.min_human_distance, 2),
                                'reason': m.reason}

    def _on_scene(self, m):
        self.state['scene'] = {
            'held': m.held_object,
            'objects': [{'id': o.id, 'label': o.label, 'category': o.category,
                         'xyz': [round(o.pose.position.x, 3), round(o.pose.position.y, 3), round(o.pose.position.z, 3)],
                         'size_cm': [round(o.size.x * 100, 1), round(o.size.y * 100, 1), round(o.size.z * 100, 1)]}
                        for o in m.objects],
            'relations': [f'{r.subject} {r.predicate} {r.object}' for r in m.relations if r.predicate != 'near']}

    def _on_image(self, m, key):
        if key == 'camera':
            self._frame_count += 1
            if self._frame_count % 3:          # ~3 fps is plenty for the page
                return
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        if m.encoding == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            self.jpeg[key] = buf.tobytes()

    def log(self, text):
        self.state['log'] = (self.state['log'] + [text])[-12:]

    def run(self, instruction):
        if not self.run_cli.server_is_ready():
            return False, 'executor not running'
        fut = self.run_cli.send_goal_async(RunInstruction.Goal(instruction=instruction))

        def accepted(f):
            gh = f.result()
            if not gh.accepted:
                self.log('busy: instruction rejected')
                return
            self.goal_handle = gh
            gh.get_result_async().add_done_callback(self._done)
        fut.add_done_callback(accepted)
        self.log(f'> {instruction}')
        return True, 'sent'

    def _done(self, f):
        r = f.result().result
        self.state['last_result'] = {'success': r.success, 'summary': r.summary, 'planning_time': round(r.planning_time, 2),
                                     'time_to_first_motion': round(r.time_to_first_motion, 1),
                                     'total_time': round(r.total_time, 1), 'replans': r.replans}
        self.log(('OK  ' if r.success else 'NOT DONE  ') + r.summary)
        self.goal_handle = None


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/index'):
                with open(PAGE, 'rb') as f:
                    return self._send(200, f.read(), 'text/html; charset=utf-8')
            if self.path.startswith('/api/state'):
                return self._send(200, json.dumps(node.state).encode())
            for key in ('camera', 'perception'):
                if self.path.startswith(f'/api/{key}.jpg'):
                    return self._send(200, node.jpeg[key], 'image/jpeg') if node.jpeg[key] else self._send(404, b'')
            self._send(404, b'{}')

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0) or 0)
            body = json.loads(self.rfile.read(n) or b'{}') if n else {}
            if self.path == '/api/run':
                text = str(body.get('instruction', '')).strip()[:500]
                ok, msg = node.run(text) if text else (False, 'empty instruction')
                return self._send(200 if ok else 400, json.dumps({'ok': ok, 'message': msg}).encode())
            if self.path == '/api/estop':
                node.estop_pub.publish(Bool(data=True))
                node.log('E-STOP pressed')
            elif self.path == '/api/reset':
                node.estop_pub.publish(Bool(data=False))
                node.reset_cli.call_async(Trigger.Request())
                node.log('e-stop reset')
            elif self.path == '/api/walk_in':
                node.walk_cli.call_async(Trigger.Request())
                node.log('simulated co-worker walking in')
            elif self.path == '/api/cancel' and node.goal_handle is not None:
                node.goal_handle.cancel_goal_async()
                node.log('cancel requested')
            self._send(200, b'{"ok": true}')
    return Handler


def main():
    rclpy.init()
    node = Console()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    port = node.get_parameter('port').value
    server = ThreadingHTTPServer(('127.0.0.1', port), make_handler(node))
    node.get_logger().info(f'Operator console on http://localhost:{port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.try_shutdown()

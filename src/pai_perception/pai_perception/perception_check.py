"""Phase 3 acceptance test: perception + world model against Gazebo ground truth.

Needs bringup.launch.py (with skills) + perception_node + world_model running.
Checks: every object found with the right colour/shape/category, position error, novel objects,
lighting change, id stability, latency.
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from pai_core.ros_utils import wait_future
from pai_core.sim import SimControl
from pai_interfaces.action import MoveTo
from pai_interfaces.msg import SafetyStatus


class PerceptionCheck(Node):
    def __init__(self, scene):
        super().__init__('perception_check')
        self.scene = scene
        self.gz = {}
        self.results = []
        self.errors = []
        self.create_subscription(TFMessage, '/world/workcell/dynamic_pose/info', self._on_gz, 10)
        self.safety = self.create_publisher(SafetyStatus, '/safety/status', 10)
        self.create_timer(0.1, lambda: self.safety.publish(SafetyStatus(state='NORMAL', speed_scale=1.0, reason='check')))
        self.refresh_cli = self.create_client(Trigger, '/world_model/refresh')
        self.move_to = ActionClient(self, MoveTo, '/pai/skills/move_to')
        self.sim = SimControl(self)

    def _on_gz(self, msg):
        for t in msg.transforms:
            self.gz[t.child_frame_id] = t.transform.translation

    def record(self, name, ok, detail=''):
        self.results.append((name, ok, detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name} {detail}')
        return ok

    def refresh(self):
        t0 = time.monotonic()
        res = wait_future(self.refresh_cli.call_async(Trigger.Request()), 120.0)
        dt = time.monotonic() - t0
        if not res or not res.success:
            self.record('world model refresh', False, res.message if res else 'timeout')
            return None, dt
        return json.loads(res.message), dt

    def truth(self, include_parked=False):
        """Ground-truth objects: (name, category, shape, colour, x, y)."""
        out = []
        for f in self.scene['fixtures']:
            out.append((f['name'], 'receptacle', f['type'], f['name'].split('_')[0] if f['type'] == 'bin' else 'gray',
                        f['pose'][0], f['pose'][1]))
        for key, cat in (('objects', 'part'), ('obstacles', 'obstacle')):
            for o in self.scene[key]:
                p = self.gz.get(o['name'])
                if p is None or (o.get('parked') and not include_parked) or p.z < -0.3:
                    continue
                color = 'white' if o['name'] == 'coffee_cup' else o['name'].split('_')[0]
                out.append((o['name'], cat, o['shape'], color, p.x, p.y))
        return out

    def compare(self, scene, label, truth):
        objs = scene['objects']
        ok_all = True
        for name, cat, shape, color, x, y in truth:
            near = [o for o in objs if math.hypot(o['position_m'][0] - x, o['position_m'][1] - y) < 0.04]
            if not near:
                ok_all &= self.record(f'{label}: {name} detected', False, 'nothing within 4 cm')
                continue
            o = min(near, key=lambda o: math.hypot(o['position_m'][0] - x, o['position_m'][1] - y))
            err = math.hypot(o['position_m'][0] - x, o['position_m'][1] - y)
            self.errors.append(err)
            good = o['color'] == color and o['shape'] == shape and o['category'] == cat
            ok_all &= self.record(f'{label}: {name}', good,
                                  f'-> {o["id"]} ({o["color"]} {o["shape"]}, {o["category"]}, err {err * 1000:.0f} mm)')
        extra = len(objs) - len(truth)
        ok_all &= self.record(f'{label}: no phantom objects', extra == 0, f'({len(objs)} detected, {len(truth)} real)')
        return ok_all


def main():
    rclpy.init()
    with open(f"{get_package_share_directory('pai_bringup')}/config/scene_default.yaml") as f:
        scene = yaml.safe_load(f)
    n = PerceptionCheck(scene)
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    n.move_to.wait_for_server(timeout_sec=60)
    n.refresh_cli.wait_for_service(timeout_sec=60)
    time.sleep(3)
    h = wait_future(n.move_to.send_goal_async(MoveTo.Goal(named_target='camera_clear')), 20)
    wait_future(h.get_result_async(), 60)

    s1, dt1 = n.refresh()
    if s1:
        n.compare(s1, 'default scene', n.truth())
        n.record('perception latency < 2 s', dt1 < 2.0, f'({dt1:.2f}s)')
        s1b, _ = n.refresh()
        ids1, ids2 = sorted(o['id'] for o in s1['objects']), sorted(o['id'] for o in s1b['objects'])
        n.record('ids stable across refreshes', ids1 == ids2, str(ids1))

    # Novel objects the code has never been told about: bring two pool objects onto the table.
    n.sim.set_pose('orange_box', 0.40, -0.05, 0.02, 0.6)
    n.sim.set_pose('purple_cylinder', 0.66, -0.22, 0.03, 0.0)
    time.sleep(1.5)
    s2, _ = n.refresh()
    if s2:
        n.compare(s2, 'novel objects', n.truth(include_parked=True))

    # Parts inside receptacles, including the same colour as the bin (blue bolt in the blue bin).
    n.sim.set_pose('blue_bolt', 0.29, 0.44, 0.041, 0.0)
    n.sim.set_pose('green_nut', 0.56, 0.46, 0.016, 0.0)
    n.sim.set_pose('red_cube', 0.42, -0.45, 0.046, 0.3)
    time.sleep(1.5)
    s_in, _ = n.refresh()
    if s_in:
        n.compare(s_in, 'parts in bins / on tray', n.truth(include_parked=True))
        rel = s_in['relations']
        for part, holder in (('blue_bolt', 'blue_bin'), ('green_nut', 'green_bin'), ('red_cube', 'kit_tray')):
            p = n.gz[part]
            det = min(s_in['objects'], key=lambda o: math.hypot(o['position_m'][0] - p.x, o['position_m'][1] - p.y))
            hid = {'blue_bin': 'blue_bin', 'green_bin': 'green_bin', 'kit_tray': 'gray_tray'}[holder]
            n.record(f'relation: {part} in/on {holder}', any(r in rel for r in (f'{det["id"]} in {hid}', f'{det["id"]} on {hid}')),
                     f'({det["id"]})')

    # Lighting change: dimmer, low warm-ish angle.
    lit = n.sim.set_light([0.7, -0.5, -0.5], intensity=0.45)
    n.record('lighting changed', lit)
    time.sleep(1.0)
    s3, _ = n.refresh()
    if s3:
        n.compare(s3, 'dim side lighting', n.truth(include_parked=True))
    n.sim.set_light([-0.4, 0.2, -0.9], intensity=0.9)

    errs = np.array(n.errors) * 1000
    passed = sum(ok for _, ok, _ in n.results)
    print(f'\n===== Phase 3: {passed}/{len(n.results)} checks passed =====')
    if len(errs):
        print(f'  position error: mean {errs.mean():.1f} mm, max {errs.max():.1f} mm over {len(errs)} detections')
    for name, ok, detail in n.results:
        print(f'  {"PASS" if ok else "FAIL"}  {name} {detail}')
    rclpy.shutdown()
    sys.exit(0 if passed == len(n.results) else 1)

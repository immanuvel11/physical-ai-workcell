"""Phase 2 acceptance test: pick & place real parts using a ground-truth scene graph.

Needs bringup.launch.py with perception:=false (this test supplies the scene graph itself). Publishes the YAML scene as /scene_graph and a
NORMAL safety heartbeat (the real supervisor arrives in Phase 5), then checks Gazebo's
ground-truth poses to see where the parts actually ended up.
"""
import math
import sys
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from moveit_msgs.srv import ApplyPlanningScene

from pai_core.geometry import point_in_rect
from pai_core.planning_scene import scene_diff
from pai_core.ros_utils import wait_future
from pai_core.scene import scene_graph_from_yaml
from pai_interfaces.action import MoveTo, Pick, Place
from pai_interfaces.msg import SafetyStatus, SceneGraph

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class SkillsCheck(Node):
    def __init__(self, scene):
        super().__init__('skills_check')
        self.scene = scene
        self.gz = {}
        self.results = []
        self.create_subscription(TFMessage, '/world/workcell/dynamic_pose/info', self._on_gz, 10)
        self.scene_pub = self.create_publisher(SceneGraph, '/scene_graph', LATCHED)
        self.safety_pub = self.create_publisher(SafetyStatus, '/safety/status', 10)
        self.create_timer(0.1, lambda: self.safety_pub.publish(
            SafetyStatus(state='NORMAL', speed_scale=1.0, min_human_distance=math.inf, reason='skills_check')))
        self.pick = ActionClient(self, Pick, '/pai/skills/pick')
        self.place = ActionClient(self, Place, '/pai/skills/place')
        self.move_to = ActionClient(self, MoveTo, '/pai/skills/move_to')
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.held = ''
        self.scene_ids = set()

    def _on_gz(self, msg):
        for t in msg.transforms:
            self.gz[t.child_frame_id] = t.transform.translation

    def publish_scene(self):
        """Ground-truth scene graph + matching MoveIt collision objects (the world model's job from Phase 3)."""
        sg = scene_graph_from_yaml(self.scene, self.gz)
        sg.held_object = self.held
        self.scene_pub.publish(sg)
        ps, self.scene_ids = scene_diff(sg.objects, exclude={self.held}, previous_ids=self.scene_ids)
        wait_future(self.apply.call_async(ApplyPlanningScene.Request(scene=ps)), 5.0)

    def call(self, client, goal, label):
        t0 = time.monotonic()
        handle = wait_future(client.send_goal_async(goal), 20.0)
        if handle is None or not handle.accepted:
            return self.record(label, False, 'goal not accepted')
        res = wait_future(handle.get_result_async(), 180.0)
        ok = res is not None and res.result.success
        msg = res.result.message if res else 'timeout'
        return self.record(label, ok, f'({msg}, {time.monotonic() - t0:.1f}s)')

    def record(self, name, ok, detail=''):
        self.results.append((name, ok, detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name} {detail}')
        return ok

    def check_inside(self, part, fixture_name):
        f = next(f for f in self.scene['fixtures'] if f['name'] == fixture_name)
        p = self.gz.get(part)
        x, y, _, yaw = f['pose']
        ok = p is not None and point_in_rect(p.x, p.y, x, y, yaw, f['size'][0], f['size'][1]) and p.z < 0.12
        where = f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})' if p else 'unknown'
        return self.record(f'{part} is now in/on {fixture_name} (Gazebo ground truth)', ok, where)


def main():
    rclpy.init()
    with open(f"{get_package_share_directory('pai_bringup')}/config/scene_default.yaml") as f:
        scene = yaml.safe_load(f)
    node = SkillsCheck(scene)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    for c in (node.pick, node.place, node.move_to):
        if not c.wait_for_server(timeout_sec=60.0):
            print('skill server not available'); sys.exit(2)
    if node.count_publishers('/scene_graph') > 1 or any(n == 'world_model' for n in node.get_node_names()):
        print('This test publishes its own ground-truth /scene_graph and must run without the world model:\n'
              '  ros2 launch pai_bringup bringup.launch.py perception:=false executor:=false')
        sys.exit(2)
    time.sleep(4.0)  # gz poses arrive; grasp joints released by the server
    node.publish_scene()
    node.call(node.move_to, MoveTo.Goal(named_target='ready'), 'move_to ready')

    for part, target in (('red_cube', 'blue_bin'), ('blue_bolt', 'kit_tray'), ('yellow_block', 'kit_tray')):
        node.publish_scene()
        time.sleep(0.5)
        if node.call(node.pick, Pick.Goal(object_id=part), f'pick {part}'):
            node.held = part
            if node.call(node.place, Place.Goal(target_id=target), f'place {part} -> {target}'):
                node.held = ''
        time.sleep(1.5)
        node.check_inside(part, target)

    node.publish_scene()
    node.call(node.pick, Pick.Goal(object_id='coffee_cup'), 'pick coffee_cup (should be refused: too wide)')
    node.results[-1] = (node.results[-1][0], not node.results[-1][1], node.results[-1][2])
    node.call(node.move_to, MoveTo.Goal(named_target='ready'), 'move_to ready')

    passed = sum(ok for _, ok, _ in node.results)
    print(f'\n===== Phase 2: {passed}/{len(node.results)} checks passed =====')
    for name, ok, detail in node.results:
        print(f'  {"PASS" if ok else "FAIL"}  {name} {detail}')
    rclpy.shutdown()
    sys.exit(0 if passed == len(node.results) else 1)

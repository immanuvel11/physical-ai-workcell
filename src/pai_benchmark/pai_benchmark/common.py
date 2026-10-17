"""Test/benchmark harness: ground truth from Gazebo, scene resets, goal scoring, task runner."""
import math
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from pai_core.geometry import point_in_rect
from pai_core.ros_utils import wait_future
from pai_core.sim import SimControl
from pai_interfaces.action import MoveTo, RunInstruction
from pai_interfaces.msg import SafetyStatus, TaskStatus

ARM = [f'panda_joint{i}' for i in range(1, 8)]


def load_scene(path=None):
    path = path or f"{get_package_share_directory('pai_bringup')}/config/scene_default.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def object_height(o):
    return o['size'][2] if o['shape'] == 'box' else o['size'][1]


class Harness(Node):
    def __init__(self, name, scene=None):
        super().__init__(name)
        self.scene = scene or load_scene()
        self.fixtures = {f['name']: f for f in self.scene['fixtures']}
        self.objects = {o['name']: o for o in self.scene['objects'] + self.scene.get('obstacles', [])}
        self.gz = {}
        self.safety = None
        self.safety_log = []          # (t, state)
        self.task = None
        self.joint_vel = {}
        self.vel_log = []             # (t, max |arm joint velocity|)
        self.lock = threading.Lock()
        self.create_subscription(TFMessage, '/world/workcell/dynamic_pose/info', self._on_gz, 10)
        self.create_subscription(SafetyStatus, '/safety/status', self._on_safety, 10)
        self.create_subscription(TaskStatus, '/pai/task_status', lambda m: setattr(self, 'task', m), 10)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.estop_pub = self.create_publisher(Bool, '/safety/estop', 10)
        self.reset_cli = self.create_client(Trigger, '/safety/reset')
        self.walk_cli = self.create_client(Trigger, '/safety/human_sim/walk_in')
        self.release_cli = self.create_client(Trigger, '/pai/skills/release_all')
        self.run_cli = ActionClient(self, RunInstruction, '/pai/run_instruction')
        self.move_cli = ActionClient(self, MoveTo, '/pai/skills/move_to')
        self.sim = SimControl(self)
        self.results = []

    # ------------------------------------------------------------------ inputs
    def _on_gz(self, msg):
        for t in msg.transforms:
            self.gz[t.child_frame_id] = t.transform.translation

    def _on_safety(self, msg):
        if self.safety is None or msg.state != self.safety.state:
            self.safety_log.append((time.monotonic(), msg.state))
        self.safety = msg

    def _on_joints(self, msg):
        # Speed from position differences: the simulator's velocity field jitters while the arm holds.
        p = dict(zip(msg.name, msg.position))
        if not all(j in p for j in ARM):
            return
        now = time.monotonic()
        prev = getattr(self, '_last_pos', None)
        if prev is not None and now - prev[0] > 0.02:
            speed = max(abs(p[j] - prev[1][j]) for j in ARM) / (now - prev[0])
            self.vel_log.append((now, speed))
            if len(self.vel_log) > 20000:
                self.vel_log = self.vel_log[-10000:]
        if prev is None or now - prev[0] > 0.02:
            self._last_pos = (now, p)

    # ------------------------------------------------------------------ ground truth
    def where(self, part):
        """Fixture the part is in/on according to Gazebo ('table', a fixture name, 'off_table', or None)."""
        p = self.gz.get(part)
        if p is None:
            return None
        if p.z < -0.3:
            return 'off_table'
        for name, f in self.fixtures.items():
            x, y, _, yaw = f['pose']
            if point_in_rect(p.x, p.y, x, y, yaw, f['size'][0], f['size'][1]) and p.z < f['size'][2] + 0.06:
                return name
        return 'table'

    def goals_met(self, goals):
        """goals: [(part, fixture)]. Returns (all_met, [per-goal bool])."""
        per = [self.where(p) == f for p, f in goals]
        return all(per), per

    # ------------------------------------------------------------------ scene control
    def place_objects(self, layout, settle=1.5):
        """layout: {name: (x, y, yaw)} on the table; every other movable object is parked."""
        # Teleporting a part that is still welded to the hand would yank the arm: release first.
        wait_future(self.release_cli.call_async(Trigger.Request()), 10)
        parked_i = 0
        for name, o in self.objects.items():
            if name == 'coffee_cup' and name not in layout:
                layout = dict(layout, coffee_cup=tuple(o['pose']))
            h = object_height(o)
            if name in layout:
                x, y, yaw = layout[name]
                self.sim.set_pose(name, x, y, h / 2 + 0.002, yaw)
            else:
                self.sim.set_pose(name, 2.0 + 0.15 * (parked_i % 6), 1.6 + 0.15 * (parked_i // 6), -0.75 + h / 2 + 0.002)
                parked_i += 1
        time.sleep(settle)

    def home(self):
        h = wait_future(self.move_cli.send_goal_async(MoveTo.Goal(named_target='camera_clear')), 20)
        if h and h.accepted:
            wait_future(h.get_result_async(), 90)

    # ------------------------------------------------------------------ tasks
    def run(self, instruction, timeout=600.0, on_feedback=None):
        """Send an instruction to the executor; returns the RunInstruction result (or None)."""
        handle = wait_future(self.run_cli.send_goal_async(RunInstruction.Goal(instruction=instruction),
                                                          feedback_callback=on_feedback), 30)
        if handle is None or not handle.accepted:
            return None
        res = wait_future(handle.get_result_async(), timeout)
        return res.result if res else None

    def record(self, name, ok, detail=''):
        self.results.append((name, bool(ok), detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name} {detail}')
        return ok

    def max_velocity_between(self, t0, t1):
        vs = [v for t, v in self.vel_log if t0 <= t <= t1]
        return max(vs) if vs else math.nan

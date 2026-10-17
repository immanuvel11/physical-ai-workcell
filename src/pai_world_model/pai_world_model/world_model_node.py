"""World model: the single shared picture of the workcell.

* /world_model/refresh (Trigger): asks perception for a fresh frame, folds it into the tracks,
  publishes /scene_graph (latched) and syncs the MoveIt planning scene.
* Track ids stay stable across refreshes (same colour+shape, nearest position), and a part the
  skill server just placed keeps its id even though it moved far.
* The object in the gripper (from /pai/manipulation_state) stays in the graph as held_object and
  is left out of the collision world; so is the part currently being grasped.
"""
import json
import math
import threading
import time

import rclpy
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from pai_core.planning_scene import scene_diff
from pai_core.ros_utils import wait_future
from pai_core.scene import compute_relations, scene_to_dict
from pai_interfaces.msg import Relation, SceneGraph

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MATCH_DIST = 0.10       # same object if it moved less than this between refreshes
ROBOT_CLEARANCE = 0.10  # objects closer than this to a robot link are never added to the collision world
ROBOT_LINKS = ['panda_link3', 'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7', 'panda_hand', 'panda_tcp']
PLACED_DIST = 0.10      # a placed part is matched near its release point


class WorldModel(Node):
    def __init__(self):
        super().__init__('world_model')
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.tracks = {}              # id -> DetectedObject
        self.manip = {'target': '', 'held': '', 'placed': {}}
        self.pending_placed = None
        self.stale = set()            # placed parts whose pose is unknown until the next refresh
        self.scene_ids = set()
        self.detections = None
        self.detection_count = 0
        self.create_subscription(SceneGraph, '/perception/detections', self._on_detections, 10, callback_group=self.cb)
        self.create_subscription(String, '/pai/manipulation_state', self._on_manip, LATCHED, callback_group=self.cb)
        self.pub = self.create_publisher(SceneGraph, '/scene_graph', LATCHED)
        self.perception = self.create_client(Trigger, '/perception/refresh', callback_group=self.cb)
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.cb)
        self.create_service(Trigger, '/world_model/refresh', self._on_refresh, callback_group=self.cb)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.get_logger().info('World model ready')

    # ------------------------------------------------------------------ inputs
    def _on_detections(self, msg):
        self.detections = msg
        self.detection_count += 1

    def _on_manip(self, msg):
        state = json.loads(msg.data)
        with self.lock:
            if state.get('placed') and state['placed'] != self.manip.get('placed'):
                self.pending_placed = state['placed']
                self.stale.add(state['placed'].get('id', ''))
            self.manip = state
            self._publish()

    def _on_refresh(self, _req, res):
        if not self.perception.wait_for_service(timeout_sec=5.0):
            res.success, res.message = False, 'perception service not available'
            return res
        count = self.detection_count
        out = wait_future(self.perception.call_async(Trigger.Request()), 120.0)
        if out is None or not out.success:
            res.success, res.message = False, f'perception failed: {out.message if out else "timeout"}'
            return res
        end = time.monotonic() + 5.0
        while self.detection_count == count and time.monotonic() < end:
            time.sleep(0.02)
        with self.lock:
            self._integrate(self.detections)
            sg = self._publish()
        res.success = True
        res.message = json.dumps(scene_to_dict(sg))
        return res

    # ------------------------------------------------------------------ tracking
    def _integrate(self, det):
        held = self.manip.get('held', '')
        old = dict(self.tracks)
        new = {}
        bad = self._overlapping_robot(det.objects)          # perception artefacts on the robot itself
        unmatched = [d for d in det.objects if d.id not in bad]

        # 1) A part that was just placed keeps its id (it moved far, so plain matching would lose it).
        placed = self.pending_placed
        if placed and placed.get('id') in old:
            prev = old[placed['id']]
            cands = [d for d in unmatched if d.color == prev.color and d.category != 'receptacle']
            if cands:
                best = min(cands, key=lambda d: math.hypot(d.pose.position.x - placed['x'], d.pose.position.y - placed['y']))
                if math.hypot(best.pose.position.x - placed['x'], best.pose.position.y - placed['y']) < PLACED_DIST:
                    best.id = placed['id']
                    new[best.id] = best
                    unmatched.remove(best)
                    old.pop(placed['id'])
        self.pending_placed = None
        self.stale = set()

        # 2) Everything else: greedy nearest match on same colour + shape family.
        pairs = []
        for d in unmatched:
            for tid, t in old.items():
                if tid == held or t.color != d.color or (t.category == 'receptacle') != (d.category == 'receptacle'):
                    continue
                dist = math.hypot(d.pose.position.x - t.pose.position.x, d.pose.position.y - t.pose.position.y)
                if dist < MATCH_DIST:
                    pairs.append((dist, tid, d))
        for dist, tid, d in sorted(pairs, key=lambda p: p[0]):
            if tid in old and d in unmatched:
                d.id = tid
                new[tid] = d
                old.pop(tid)
                unmatched.remove(d)

        # 3) New objects get fresh, unique, human-readable ids.
        for d in unmatched:
            base, i = d.id, 2
            while d.id in new or d.id in old or d.id == held:
                d.id = f'{base}_{i}'
                i += 1
            new[d.id] = d

        # 4) The held part is hidden by the robot filter: keep it.
        if held and held in self.tracks:
            new[held] = self.tracks[held]
        self.tracks = new

    def _publish(self):
        sg = SceneGraph()
        sg.header.stamp = self.get_clock().now().to_msg()
        sg.header.frame_id = 'world'
        sg.objects = sorted(self.tracks.values(), key=lambda o: (o.category != 'receptacle', o.id))
        sg.held_object = self.manip.get('held', '')
        visible = [o for o in sg.objects if o.id != sg.held_object]
        sg.relations = [Relation(subject=s, predicate=p, object=o) for s, p, o in compute_relations(visible)]
        self.pub.publish(sg)
        exclude = {self.manip.get('held', ''), self.manip.get('target', '')} | self.stale
        ps, self.scene_ids = scene_diff(sg.objects, exclude=exclude, previous_ids=self.scene_ids)
        if self.apply.service_is_ready():
            self.apply.call_async(ApplyPlanningScene.Request(scene=ps))
        return sg


    def _overlapping_robot(self, objects):
        """Ids of fresh detections sitting where the robot is (perception artefacts): putting them in the
        collision world would make every start state 'in collision' and deadlock the planner."""
        links = []
        for f in ROBOT_LINKS:
            try:
                t = self.tf.lookup_transform('world', f, Time()).transform.translation
                links.append((t.x, t.y, t.z))
            except Exception:
                pass
        bad = set()
        for o in objects:
            if o.category == 'receptacle':
                continue
            p = o.pose.position
            if any(math.dist((p.x, p.y, p.z), l) < ROBOT_CLEARANCE + max(o.size.x, o.size.y) / 2 for l in links):
                bad.add(o.id)
        if bad:
            self.get_logger().warn(f'ignoring objects overlapping the robot: {sorted(bad)}')
        return bad


def main():
    rclpy.init()
    node = WorldModel()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

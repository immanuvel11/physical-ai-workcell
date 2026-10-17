"""Skill action servers: /pai/skills/pick, /pai/skills/place, /pai/skills/move_to.

Every arm motion goes through `_run_motion`, which
  * waits while the safety supervisor says STOP / ESTOP (protective stop, then auto-resume),
  * scales velocity by the supervisor's speed_scale,
  * cancels and re-plans from the current state if the allowed speed drops mid-motion.
Targets are checked against a workspace envelope before anything moves.
"""
import json
import math
import subprocess
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, BoundingVolume, CollisionObject, Constraints,
                             JointConstraint, MotionPlanRequest, MoveItErrorCodes, OrientationConstraint,
                             PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from pai_core.geometry import quat_yaw, tool_down_quat, wrap_half_pi
from pai_core.ros_utils import wait_future
from pai_core.scene import find, is_within, support_surface_z, top_z
from pai_interfaces.action import MoveTo, Pick, Place
from pai_interfaces.msg import SafetyStatus, SceneGraph

ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
NAMED_TARGETS = {
    'ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    'camera_clear': [0.0, -1.4, 0.0, -2.9, 0.0, 1.6, 0.785],
}
GRIPPER_OPEN = 0.035          # per finger (m); kept off the 0.04 joint limit
MAX_GRASP_WIDTH = 0.064       # narrow side of a part must be below this to fit between the fingers
FINGER_DEPTH = 0.045          # usable finger length below the hand
HAND_CLEARANCE = 0.03         # keep the TCP at least rim - this, so the hand body stays above bin walls
HAND_LENGTH = 0.22            # Panda hand body: long along the finger axis ...
HAND_WIDTH = 0.08             # ... and narrow across it
HAND_BOTTOM = 0.038           # hand body starts this far above the TCP (measured with MoveIt)
MIN_TRANSIT_Z = 0.18          # free-space moves end at least this high; final approach is a straight line
EMPTY_GRIP = 0.003            # finger position below this after closing => nothing in the hand
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
MOVEIT_ERRORS = {
    99999: 'no feasible motion found', -1: 'planning failed', -2: 'invalid motion plan',
    -3: 'plan invalidated by an environment change', -4: 'controller failed to follow the motion',
    -6: 'motion timed out', -7: 'motion preempted', -10: 'start state in collision',
    -12: 'goal in collision', -31: 'no inverse-kinematics solution (out of reach)',
}


class SkillError(Exception):
    pass


class SkillServer(Node):
    def __init__(self):
        super().__init__('skill_server')
        self.declare_parameter('max_velocity', 0.35)
        self.declare_parameter('max_acceleration', 0.35)
        self.declare_parameter('approach_height', 0.12)
        self.declare_parameter('lift_height', 0.15)
        self.declare_parameter('require_safety', True)   # no supervisor heartbeat => do not move
        self.declare_parameter('safety_pause_timeout', 120.0)
        self.declare_parameter('grasp_objects', [''])    # gz models that have a sim grasp joint
        # Workspace envelope for the tool centre point: x_min, x_max, y_min, y_max, z_min, z_max
        self.declare_parameter('workspace', [0.15, 0.85, -0.62, 0.62, 0.003, 0.65])

        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.scene = SceneGraph()
        self.safety = None
        self.safety_stamp = 0.0
        self.stop_epoch = 0
        self.joints = {}
        self.gz_poses = {}
        self.held = None                       # dict describing the object in the hand

        self.create_subscription(SceneGraph, '/scene_graph', self._on_scene, LATCHED, callback_group=self.cb)
        self.create_subscription(SafetyStatus, '/safety/status', self._on_safety, 10, callback_group=self.cb)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10, callback_group=self.cb)
        self.create_subscription(TFMessage, '/world/workcell/dynamic_pose/info', self._on_gz_poses, 10,
                                 callback_group=self.cb)
        self.state_pub = self.create_publisher(String, '/pai/manipulation_state', LATCHED)
        self.grasp_pubs = {}
        for name in [n for n in self.get_parameter('grasp_objects').value if n]:
            self.grasp_pubs[name] = (self.create_publisher(Empty, f'/pai/grasp/{name}/attach', 10),
                                     self.create_publisher(Empty, f'/pai/grasp/{name}/detach', 10))

        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.move_group = ActionClient(self, MoveGroup, '/move_action', callback_group=self.cb)
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=self.cb)
        self.gripper = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd', callback_group=self.cb)
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=self.cb)
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=self.cb)

        for name, action, fn in (('pick', Pick, self._pick), ('place', Place, self._place),
                                 ('move_to', MoveTo, self._move_to)):
            ActionServer(self, action, f'/pai/skills/{name}', execute_callback=self._wrap(fn, action),
                         goal_callback=self._accept_one, cancel_callback=lambda _: CancelResponse.ACCEPT,
                         callback_group=self.cb)
        self._publish_state()
        # DetachableJoints start ATTACHED in Gazebo Fortress. Release everything, but only once the
        # bridge is subscribed (a detach published earlier is silently lost), and repeat it a few times.
        self._release_rounds = {name: 0 for name in self.grasp_pubs}
        self.sim_ready = not self.grasp_pubs     # real hardware: nothing to release
        self._robot_up_since = None
        self.create_service(Trigger, '/pai/skills/release_all', self._on_release_all, callback_group=self.cb)
        self._startup_timer = self.create_timer(1.0, self._release_all_at_startup, callback_group=self.cb)
        self.get_logger().info(f'Skill server ready ({len(self.grasp_pubs)} sim-grasp objects)')

    # ------------------------------------------------------------------ inputs
    def _on_scene(self, msg):
        self.scene = msg

    def _on_safety(self, msg):
        if msg.state in ('STOP', 'ESTOP') and (self.safety is None or self.safety.state not in ('STOP', 'ESTOP')):
            self.stop_epoch += 1          # every protective stop / e-stop gets a number
        self.safety, self.safety_stamp = msg, time.monotonic()

    def _on_joints(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def _on_gz_poses(self, msg):
        for t in msg.transforms:
            if t.child_frame_id in self.grasp_pubs:
                self.gz_poses[t.child_frame_id] = t.transform.translation

    def _release_all_at_startup(self):
        # Wait until the robot (and with it the grasp plugins) is really running in Gazebo:
        # detach messages sent before the plugin subscribes are silently dropped.
        if not self.joints:
            return
        if self._robot_up_since is None:
            self._robot_up_since = time.monotonic()
        if time.monotonic() - self._robot_up_since < 2.0:
            return
        for name, (_, detach) in self.grasp_pubs.items():
            if self._release_rounds[name] < 5 and detach.get_subscription_count() > 0:
                detach.publish(Empty())
                self._release_rounds[name] += 1
        if all(n >= 5 for n in self._release_rounds.values()):
            self._startup_timer.cancel()
            time.sleep(0.5)                  # let the last detach messages land before anything moves
            self.sim_ready = True
            self.get_logger().info('all simulated grasp joints released')

    def _on_release_all(self, _req, res):
        """Reset service for test harnesses (between trials only): drop whatever is in the hand and
        make sure no part is welded to it."""
        if self.held:
            held, self.held = self.held, None
            self._scene_detach(held['id'])
            self._gripper(GRIPPER_OPEN, lambda _s: None, 'reset: opening gripper')
        for _ in range(3):
            self._release_unheld()
            time.sleep(0.1)
        self._publish_state()
        res.success, res.message = True, 'released'
        return res

    def _release_unheld(self):
        """Idempotent safety net: make sure no object except the held one is welded to the hand."""
        held_model = self.held.get('gz_model') if self.held else None
        for name, (_, detach) in self.grasp_pubs.items():
            if name != held_model:
                detach.publish(Empty())
                self._gz_publish(f'/pai/grasp/{name}/detach')

    def _accept_one(self, _goal):
        return GoalResponse.REJECT if self.lock.locked() else GoalResponse.ACCEPT

    def _wrap(self, fn, action):
        def execute(goal_handle):
            with self.lock:
                action_result = action.Result()
                try:
                    # Never move while parts may still be welded to the hand (sim start-up).
                    end = time.monotonic() + 30.0
                    while not self.sim_ready:
                        if time.monotonic() > end:
                            raise SkillError('simulated grasp joints not released yet')
                        time.sleep(0.1)
                    fn(goal_handle, lambda stage: self._feedback(goal_handle, action, stage))
                    action_result.success, action_result.message = True, 'ok'
                    goal_handle.succeed()
                except SkillError as e:
                    action_result.success, action_result.message = False, str(e)
                    self.get_logger().warn(f'skill failed: {e}')
                    goal_handle.abort()
                except Exception as e:  # never leave a goal hanging
                    action_result.success, action_result.message = False, f'internal error: {e!r}'
                    self.get_logger().error(f'skill crashed: {e!r}')
                    goal_handle.abort()
                return action_result
        return execute

    def _feedback(self, goal_handle, action, stage):
        fb = action.Feedback()
        fb.stage = stage
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'  [{stage}]')

    def _publish_state(self, target='', placed=None):
        self.state_pub.publish(String(data=json.dumps({
            'target': target, 'held': self.held['id'] if self.held else '',
            'placed': placed or {}, 'stamp': time.time()})))

    # ------------------------------------------------------------------ safety
    def _allowed_scale(self):
        if self.safety is None or time.monotonic() - self.safety_stamp > 1.0:
            return 0.0 if self.get_parameter('require_safety').value else 1.0
        if self.safety.state in ('STOP', 'ESTOP'):
            return 0.0
        return max(0.0, min(1.0, self.safety.speed_scale))

    def _wait_until_safe(self, fb):
        if self._allowed_scale() > 0.0:
            return
        reason = self.safety.reason if self.safety else 'no safety heartbeat'
        fb(f'PAUSED by safety: {reason}')
        end = time.monotonic() + self.get_parameter('safety_pause_timeout').value
        while self._allowed_scale() <= 0.0:
            if time.monotonic() > end:
                raise SkillError('safety pause timed out')
            time.sleep(0.1)
        fb('resuming after safety pause')

    def _run_motion(self, label, send_fn, fb):
        """send_fn(velocity_scale) -> (action_client, goal). Retries after safety interruptions."""
        base = self.get_parameter('max_velocity').value
        for _ in range(6):
            self._wait_until_safe(fb)
            scale = self._allowed_scale()
            epoch = self.stop_epoch
            client, goal = send_fn(base * scale)
            handle = wait_future(client.send_goal_async(goal), 10.0)
            if handle is None or not handle.accepted:
                raise SkillError(f'{label}: goal rejected by MoveIt')
            result_future = handle.get_result_async()
            interrupted = False
            while not result_future.done():
                if self._allowed_scale() < scale - 1e-3:   # supervisor got stricter: stop now
                    wait_future(handle.cancel_goal_async(), 5.0)
                    wait_future(result_future, 10.0)
                    interrupted = True
                    break
                time.sleep(0.02)
            code = result_future.result().result.error_code.val
            if not interrupted and self.stop_epoch != epoch:
                # A protective stop / e-stop happened during this motion. Even a "success" may only mean the
                # supervisor's hold finished, not that we arrived: redo it (harmless if we did arrive).
                interrupted = True
            if interrupted:
                fb(f'{label}: interrupted by safety, re-planning from current state')
                continue
            if code == MoveItErrorCodes.SUCCESS:
                return
            raise SkillError(f'{label}: {MOVEIT_ERRORS.get(code, "motion failed")} (MoveIt error code {code})')
        raise SkillError(f'{label}: interrupted too many times')

    # ------------------------------------------------------------------ motion primitives
    def _check_workspace(self, x, y, z, what):
        x0, x1, y0, y1, z0, z1 = self.get_parameter('workspace').value
        if not (x0 <= x <= x1 and y0 <= y <= y1 and z0 <= z <= z1):
            raise SkillError(f'{what} ({x:.2f}, {y:.2f}, {z:.2f}) is outside the safe workspace')

    def _request(self, vel, planner='pilz'):
        r = MotionPlanRequest()
        r.group_name = 'panda_arm'
        if planner == 'pilz':     # deterministic point-to-point joint motion: short, predictable paths
            r.pipeline_id, r.planner_id = 'pilz_industrial_motion_planner', 'PTP'
        else:                     # sampling-based fallback that can route around obstacles
            r.pipeline_id, r.planner_id = 'ompl', ''
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = max(vel, 0.01)
        r.max_acceleration_scaling_factor = self.get_parameter('max_acceleration').value
        return r

    def _plan_with_fallback(self, label, make_send, fb):
        """Pilz PTP first (direct, predictable); if that is blocked, OMPL finds a way around."""
        try:
            self._run_motion(label, make_send('pilz'), fb)
        except SkillError as e:
            if 'MoveIt error code' not in str(e) or 'controller failed' in str(e):
                raise
            fb(f'{label}: direct motion not possible, planning around obstacles')
            self._run_motion(label, make_send('ompl'), fb)

    def _move_joints(self, positions, label, fb):
        def make_send(planner):
            return lambda vel: send(vel, planner)

        def send(vel, planner):
            g = MoveGroup.Goal()
            g.request = self._request(vel, planner)
            g.request.goal_constraints = [Constraints(joint_constraints=[
                JointConstraint(joint_name=j, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
                for j, p in zip(ARM_JOINTS, positions)])]
            g.planning_options.plan_only = False
            return self.move_group, g
        fb(label)
        self._plan_with_fallback(label, make_send, fb)

    def _move_tcp(self, x, y, z, yaw, label, fb):
        """Collision-aware free-space motion of the TCP to a top-down pose."""
        self._check_workspace(x, y, z, label)

        def make_send(planner):
            return lambda vel: send(vel, planner)

        def send(vel, planner):
            g = MoveGroup.Goal()
            g.request = self._request(vel, planner)
            target = Pose()
            target.position = Point(x=x, y=y, z=z)
            target.orientation.w = 1.0
            region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.004])],
                                    primitive_poses=[target])
            pc = PositionConstraint(link_name='panda_tcp', constraint_region=region, weight=1.0)
            pc.header.frame_id = 'world'
            oc = OrientationConstraint(link_name='panda_tcp', orientation=tool_down_quat(yaw), weight=1.0,
                                       absolute_x_axis_tolerance=0.03, absolute_y_axis_tolerance=0.03,
                                       absolute_z_axis_tolerance=0.05)
            oc.header.frame_id = 'world'
            g.request.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
            g.planning_options.plan_only = False
            # No MoveIt-side replanning: it would re-send motions while the supervisor holds the arm.
            # Retries are handled here, after the safety layer allows motion again.
            g.planning_options.replan = False
            return self.move_group, g
        fb(label)
        for attempt in range(2):
            self._plan_with_fallback(label, make_send, fb)
            try:
                # never carry on from the wrong place: the next straight-line move starts from here
                self._settle(x, y, z, label, tol=0.03, must_settle=True, timeout=5.0)
                return
            except SkillError:
                if attempt == 1:
                    raise SkillError(f'{label}: the arm did not reach the target (blocked?)')
                fb(f'{label}: arm did not arrive, retrying the motion')

    def _move_linear(self, x, y, z, yaw, label, fb, avoid_collisions=True, settle_tol=0.015, must_settle=False):
        """Straight-line TCP motion (approach / retreat). Re-computed from the current state on every retry.
        Only for (nearly) vertical moves: refusing a long sideways 'straight line' prevents sweeping across
        the table if the arm is not where it should be."""
        self._check_workspace(x, y, z, label)
        here, _ = self._tcp()
        if math.hypot(here.x - x, here.y - y) > 0.03:
            raise SkillError(f'{label}: refusing a sideways straight-line move '
                             f'({math.hypot(here.x - x, here.y - y) * 100:.0f} cm); the arm is not above the target')

        def send(vel):
            req = GetCartesianPath.Request()
            req.header.frame_id = 'world'
            req.start_state.is_diff = True
            req.group_name = 'panda_arm'
            req.link_name = 'panda_tcp'
            wp = Pose()
            wp.position = Point(x=x, y=y, z=z)
            wp.orientation = tool_down_quat(yaw)
            req.waypoints = [wp]
            req.max_step = 0.005
            req.jump_threshold = 0.0
            req.avoid_collisions = avoid_collisions
            req.max_velocity_scaling_factor = max(vel * 0.6, 0.01)   # approach/retreat slower than transit
            req.max_acceleration_scaling_factor = self.get_parameter('max_acceleration').value
            res = wait_future(self.cartesian.call_async(req), 10.0)
            if res is None or res.fraction < 0.98:
                frac = 'timeout' if res is None else f'{res.fraction:.0%}'
                raise SkillError(f'{label}: straight-line path blocked ({frac} feasible)')
            g = ExecuteTrajectory.Goal()
            g.trajectory = res.solution
            return self.execute, g
        fb(label)
        self._run_motion(label, send, fb)
        self._settle(x, y, z, label, settle_tol, must_settle)

    def _settle(self, x, y, z, label, tol, must_settle, timeout=3.0):
        """Wait until the TCP is really at (x, y, z): the simulated arm can lag its trajectory."""
        end = time.monotonic() + timeout
        while True:
            p, _ = self._tcp()
            err = math.dist((p.x, p.y, p.z), (x, y, z))
            if err <= tol:
                return
            if time.monotonic() > end:
                if must_settle:
                    raise SkillError(f'{label}: arm did not settle at the target ({err * 1000:.0f} mm off)')
                self.get_logger().warn(f'{label}: still {err * 1000:.0f} mm from target, continuing')
                return
            time.sleep(0.05)

    def _gripper(self, position, fb, label):
        fb(label)
        g = GripperCommand.Goal()
        g.command.position = position
        g.command.max_effort = 20.0
        handle = wait_future(self.gripper.send_goal_async(g), 10.0)
        if handle is None or not handle.accepted:
            raise SkillError('gripper command rejected')
        wait_future(handle.get_result_async(), 10.0)
        time.sleep(0.3)
        return self.joints.get('panda_finger_joint1', 0.0)

    def _tcp(self):
        try:
            t = self.tf.lookup_transform('world', 'panda_tcp', Time()).transform
        except Exception as e:
            raise SkillError(f'TCP pose unavailable: {e}')
        return t.translation, wrap_half_pi(_tool_yaw(t.rotation))

    # ------------------------------------------------------------------ planning scene
    def _apply(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        res = wait_future(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)), 5.0)
        if res is None or not res.success:
            self.get_logger().warn('planning scene update failed')

    def _scene_remove(self, object_id):
        co = CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
        co.header.frame_id = 'world'
        ps = PlanningScene()
        ps.world.collision_objects = [co]
        self._apply(ps)

    def _scene_attach(self, held):
        side = max(held['size'][0], held['size'][1])
        co = CollisionObject(id=held['id'], operation=CollisionObject.ADD)
        co.header.frame_id = 'panda_tcp'
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[side, side, held['size'][2]])]
        p = Pose()
        p.position.z = held['center_below_tcp']    # tool z points down
        p.orientation.w = 1.0
        co.primitive_poses = [p]
        aco = AttachedCollisionObject(link_name='panda_hand', object=co,
                                      touch_links=['panda_hand', 'panda_leftfinger', 'panda_rightfinger'])
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [aco]
        self._apply(ps)

    def _scene_detach(self, object_id):
        co = CollisionObject(id=object_id, operation=CollisionObject.REMOVE)
        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [AttachedCollisionObject(link_name='panda_hand', object=co)]
        ps.world.collision_objects = [CollisionObject(id=object_id, operation=CollisionObject.REMOVE)]
        ps.world.collision_objects[0].header.frame_id = 'world'
        self._apply(ps)

    # ------------------------------------------------------------------ simulated grasp
    def _sim_part_between_fingers(self, tcp, height):
        """Gazebo model whose centre is within 2.5 cm of the TCP axis and overlaps the finger span.
        Sim-only stand-in for the width/force sensing a real gripper uses to confirm a grasp."""
        best, best_d = None, 0.025
        for name, p in self.gz_poses.items():
            d = math.hypot(p.x - tcp.x, p.y - tcp.y)
            if d < best_d and abs(p.z - tcp.z) < height / 2 + FINGER_DEPTH:
                best, best_d = name, d
        return best

    @staticmethod
    def _gz_publish(topic):
        """Also publish straight on Gazebo transport (independent of the ROS bridge), fire-and-forget."""
        try:
            subprocess.Popen(['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty', '-p', ' '],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            pass

    def _sim_attach(self, name):
        # ROS->gz bridge delivery is not guaranteed; attach/detach are idempotent, so send a few, two ways.
        self._gz_publish(f'/pai/grasp/{name}/attach')
        for _ in range(3):
            self.grasp_pubs[name][0].publish(Empty())
            time.sleep(0.05)

    def _sim_detach(self, name):
        if name in self.grasp_pubs:
            self._gz_publish(f'/pai/grasp/{name}/detach')
            for _ in range(3):
                self.grasp_pubs[name][1].publish(Empty())
                time.sleep(0.05)

    def _sim_follows_tcp(self, name, timeout=1.0):
        """True if the gz model is still moving with the hand (i.e. welded)."""
        time.sleep(timeout)
        p = self.gz_poses.get(name)
        tcp, _ = self._tcp()
        return p is not None and math.hypot(p.x - tcp.x, p.y - tcp.y) < 0.03 and abs(p.z - tcp.z) < 0.06

    # ------------------------------------------------------------------ skills
    def _move_to(self, goal_handle, fb):
        name = goal_handle.request.named_target
        if name not in NAMED_TARGETS:
            raise SkillError(f'unknown named target "{name}" (known: {", ".join(NAMED_TARGETS)})')
        self._move_joints(NAMED_TARGETS[name], f'moving to {name}', fb)

    def _pick(self, goal_handle, fb):
        req = goal_handle.request
        if self.held:
            raise SkillError(f'already holding {self.held["id"]}')
        if req.object_id:
            obj = find(self.scene, req.object_id)
            if obj is None:
                raise SkillError(f'object "{req.object_id}" is not in the scene graph')
            if obj.category == 'receptacle':
                raise SkillError(f'"{obj.id}" is a receptacle, not a part')
            oid, pose, size, shape = obj.id, obj.pose, (obj.size.x, obj.size.y, obj.size.z), obj.shape
        else:  # explicit target (rule-based baseline: taught coordinates)
            oid, pose, size, shape = 'taught_part', req.pose, (req.size.x, req.size.y, req.size.z), 'box'
            # whatever the collision model has at the taught spot is what we are about to grasp
            near = [o for o in self.scene.objects if o.category != 'receptacle' and
                    math.hypot(o.pose.position.x - pose.position.x, o.pose.position.y - pose.position.y) < 0.04]
            if near:
                oid = near[0].id

        # Close the fingers across the narrow side of the part.
        obj_yaw = quat_yaw(pose.orientation)
        narrow = min(size[0], size[1])
        if narrow > MAX_GRASP_WIDTH:
            raise SkillError(f'{oid} is too wide to grasp ({narrow * 100:.1f} cm > {MAX_GRASP_WIDTH * 100:.1f} cm)')
        yaw = wrap_half_pi(obj_yaw + (math.pi / 2 if size[0] <= size[1] else 0.0))
        top = pose.position.z + size[2] / 2
        bottom = pose.position.z - size[2] / 2
        # Aim for ~2.5 cm of finger overlap. The finger collision mesh reaches ~8 mm below panda_tcp,
        # so keep the TCP >= 10 mm above the supporting surface.
        grasp_z = max(top - 0.025, bottom + 0.010)
        if top - grasp_z > FINGER_DEPTH:
            grasp_z = top - FINGER_DEPTH
        x, y = pose.position.x, pose.position.y
        # Inside a walled receptacle the hand body must stay above the rim: is the part reachable?
        if req.object_id:
            holder = next((r for r in self.scene.objects if r.category == 'receptacle' and r.shape == 'bin'
                           and is_within(obj, r)), None)
            if holder is not None:
                grasp_z = max(grasp_z, top_z(holder) - HAND_BOTTOM + 0.006)
                if top - grasp_z < 0.012:
                    raise SkillError(f'{oid} sits too deep inside {holder.id} for a top-down grasp '
                                     f'(the hand would hit the walls)')

        self._release_unheld()
        self._gripper(GRIPPER_OPEN, fb, 'opening gripper')
        # Equivalent grasps: a parallel gripper is symmetric under 180 deg, square parts and cylinders
        # also under 90 deg. They need very different wrist angles, so if one is out of reach, try another.
        if shape == 'cylinder':
            yaws = [yaw + k * math.pi / 6 for k in range(12)]            # round: any wrist angle grasps it
        elif abs(size[0] - size[1]) < 0.01:
            yaws = [yaw + k * math.pi / 2 for k in range(4)]             # square: every 90 deg
        else:
            yaws = [yaw, yaw + math.pi]
        # Best first: keep the 22 cm hand body away from tall neighbours (cups, bolts, bin walls).
        yaws.sort(key=lambda a: -self._hand_clearance(x, y, a, grasp_z + HAND_BOTTOM, oid))
        yaws = yaws[:4]
        pre_z = max(grasp_z + self.get_parameter('approach_height').value, MIN_TRANSIT_Z)
        for k, yaw in enumerate(yaws):
            try:
                # Transit with the part still in the collision model, so the path cannot run through it ...
                self._move_tcp(x, y, pre_z, yaw, 'moving above part', fb)
                # ... and only then let the fingers enter its space for the vertical approach.
                self._publish_state(target=oid)
                self._scene_remove(oid)
                self._move_linear(x, y, grasp_z, yaw, 'descending to grasp', fb, settle_tol=0.003, must_settle=True)
                break
            except SkillError as e:
                if k == len(yaws) - 1 or not ('blocked' in str(e) or 'MoveIt error' in str(e)):
                    raise
                fb(f'grasp approach {k + 1} not feasible ({e}); trying an equivalent grasp')
        tcp, tcp_yaw = self._tcp()
        self.get_logger().info(
            f'grasp {oid}: target ({x:.3f}, {y:.3f}, {grasp_z:.3f}, yaw {math.degrees(yaw):.0f}) '
            f'actual ({tcp.x:.3f}, {tcp.y:.3f}, {tcp.z:.3f}, yaw {math.degrees(tcp_yaw):.0f})')
        gz_model = None
        if self.grasp_pubs:
            # Simulation: finger-contact physics squeezes short, light parts out of the grasp, so
            # weld first (if the part really is between the fingers), then close the fingers.
            gz_model = self._sim_part_between_fingers(tcp, size[2])
            if gz_model is None:
                self._move_linear(x, y, grasp_z + self.get_parameter('lift_height').value, yaw, 'retreating', fb)
                self._publish_state()
                raise SkillError(f'grasp missed: no part between the fingers at the expected position of {oid}')
            self._sim_attach(gz_model)
            finger = self._gripper(max(narrow / 2 - 0.003, 0.0), fb, 'closing gripper')
        else:
            # Real gripper: close until the fingers stall on the part; fully closed means we missed.
            finger = self._gripper(0.001, fb, 'closing gripper')   # just off the 0.0 joint limit
            if finger < EMPTY_GRIP:
                self._gripper(GRIPPER_OPEN, fb, 'grasp missed: re-opening')
                self._move_linear(x, y, grasp_z + self.get_parameter('lift_height').value, yaw, 'retreating', fb)
                self._publish_state()
                raise SkillError(f'grasp missed: nothing between the fingers at the expected position of {oid}')
        self._gripper(finger, fb, 'holding')            # stop squeezing once the part is secured
        self.held = {'id': oid, 'size': size, 'shape': shape, 'gz_model': gz_model,
                     'below_tcp': grasp_z - bottom, 'center_below_tcp': grasp_z - pose.position.z}
        self._scene_attach(self.held)
        self._publish_state()
        self._move_linear(x, y, grasp_z + self.get_parameter('lift_height').value, yaw, 'lifting', fb)
        if gz_model and not self._sim_follows_tcp(gz_model, timeout=0.3):
            self._release(fb)
            raise SkillError(f'{oid} did not come up with the gripper')
        if not self.grasp_pubs and self.joints.get('panda_finger_joint1', 0.0) < EMPTY_GRIP:
            self._release(fb)
            raise SkillError(f'{oid} slipped out of the gripper while lifting')

    def _place(self, goal_handle, fb):
        req = goal_handle.request
        if not self.held:
            raise SkillError('not holding anything')
        held = self.held
        if req.target_id:
            target = find(self.scene, req.target_id)
            if target is None:
                raise SkillError(f'target "{req.target_id}" is not in the scene graph')
            if target.category != 'receptacle':
                raise SkillError(f'"{target.id}" is not a receptacle')
            surface = support_surface_z(target)
            rim = top_z(target)
        else:   # explicit (taught) place point
            surface, x, y, rim = float(req.surface_z), req.target_point.x, req.target_point.y, float(req.rim_z)
        _, yaw = self._tcp()
        release_z = surface + held['below_tcp'] + 0.012
        if (req.target_id and target.shape == 'bin') or (not req.target_id and rim > surface + 0.03):
            release_z = max(release_z, rim - HAND_CLEARANCE)   # part drops the last few cm
        candidates = self._free_spots(target, held, release_z, yaw) if req.target_id else [(x, y, yaw)]
        above_z = max(release_z, rim + held['below_tcp'] + 0.03, MIN_TRANSIT_Z - 0.08) + 0.08
        for i, (x, y, yaw) in enumerate(candidates[:5]):
            # Self-correction: if the straight-down approach to this spot is blocked, try the next best one.
            self._move_tcp(x, y, above_z, yaw, f'moving above {req.target_id or "target point"}', fb)
            try:
                self._move_linear(x, y, release_z, yaw, 'lowering part', fb, settle_tol=0.008)
                break
            except SkillError as e:
                if 'blocked' not in str(e) or i == min(len(candidates), 5) - 1:
                    raise
                fb(f'spot {i + 1} blocked, trying another placement')
        in_bin = bool(req.target_id and target.shape == 'bin') or (not req.target_id and rim > surface + 0.03)
        # in a bin, open only as far as needed: fully opened fingers can jam against the walls
        self._release(fb, opening=min(GRIPPER_OPEN, min(held['size'][0], held['size'][1]) / 2 + 0.01) if in_bin else GRIPPER_OPEN)
        self._publish_state(placed={'id': held['id'], 'x': x, 'y': y, 'target': req.target_id})
        self._move_linear(x, y, above_z, yaw, 'retreating', fb, avoid_collisions=False)
        model = held.get('gz_model')
        if model and self._sim_follows_tcp(model, timeout=0.2):
            self.get_logger().warn(f'{model} still welded after release: detaching again')
            self._sim_detach(model)
            self._move_linear(x, y, release_z + 0.01, yaw, 'putting part back down', fb, avoid_collisions=False)
            self._sim_detach(model)
            self._move_linear(x, y, above_z, yaw, 'retreating', fb, avoid_collisions=False)

    def _release(self, fb, opening=GRIPPER_OPEN):
        held, self.held = self.held, None
        if held.get('gz_model'):
            self._sim_detach(held['gz_model'])
        self._gripper(opening, fb, 'releasing')
        self._scene_detach(held['id'])

    def _hand_clearance(self, x, y, yaw, hand_z, ignore_id):
        """Smallest gap between the hand body (a HAND_LENGTH x HAND_WIDTH rectangle above hand_z, long
        axis along the fingers) and any object tall enough to reach it."""
        ay = (math.sin(yaw), -math.cos(yaw))
        gap = 1.0
        for o in self.scene.objects:
            # receptacles are left to MoveIt's exact collision check (a disc model fits them badly)
            if o.id == ignore_id or o.category == 'receptacle' or top_z(o) <= hand_z:
                continue
            r_o = max(o.size.x, o.size.y) / 2
            dx, dy = o.pose.position.x - x, o.pose.position.y - y
            along = abs(dx * ay[0] + dy * ay[1])
            across = abs(-dx * ay[1] + dy * ay[0])
            gap = min(gap, max(along - HAND_LENGTH / 2, across - HAND_WIDTH / 2) - r_o)
        return gap

    def _free_spots(self, target, held, release_z, current_yaw):
        """Candidate (x, y, yaw) placements inside the receptacle, best clearance first.

        Checks the held part's footprint against parts already there, and the hand body (a long
        rectangle starting HAND_BOTTOM above the TCP) against anything tall enough to reach it.
        Tries the current hand yaw and the yaw rotated by 90 degrees.
        """
        r = target.pose.position
        tyaw = quat_yaw(target.pose.orientation)
        wall = 0.006 if target.shape == 'bin' else 0.0
        half_fp = max(held['size'][0], held['size'][1]) / 2
        clearance = max(half_fp, 0.045 if target.shape == 'bin' else half_fp) + 0.01  # open fingers need room in bins
        ix = max(target.size.x / 2 - wall - clearance, 0.0)
        iy = max(target.size.y / 2 - wall - clearance, 0.0)
        contents = [o for o in self.scene.objects
                    if o.category != 'receptacle' and o.id != held['id'] and is_within(o, target)]
        hand_z = release_z + HAND_BOTTOM
        scored = []
        steps = [i / 4 for i in range(-4, 5)]
        for yaw in (current_yaw, wrap_half_pi(current_yaw + math.pi / 2)):
            ay = (math.sin(yaw), -math.cos(yaw))          # hand long axis (= finger axis) in the world
            for u in steps:
                for v in steps:
                    lx, ly = u * ix, v * iy
                    px = r.x + lx * math.cos(tyaw) - ly * math.sin(tyaw)
                    py = r.y + lx * math.sin(tyaw) + ly * math.cos(tyaw)
                    gap = 1.0
                    for o in contents:
                        dx, dy = o.pose.position.x - px, o.pose.position.y - py
                        r_o = max(o.size.x, o.size.y) / 2
                        gap = min(gap, math.hypot(dx, dy) - r_o - half_fp)
                        if top_z(o) > hand_z:                 # tall enough to hit the hand body
                            along = abs(dx * ay[0] + dy * ay[1])
                            across = abs(-dx * ay[1] + dy * ay[0])
                            gap = min(gap, max(along - HAND_LENGTH / 2, across - HAND_WIDTH / 2) - r_o)
                    score = min(gap, 0.1) - 0.01 * math.hypot(lx, ly) - (0.001 if yaw != current_yaw else 0.0)
                    if gap >= 0.0:
                        scored.append((score, (px, py, yaw)))
        if not scored:
            raise SkillError(f'no free space left in {target.id}')
        scored.sort(key=lambda c: -c[0])
        # keep candidates spread out: skip ones within 3 cm of a better candidate with the same yaw
        picked = []
        for _, c in scored:
            if all(math.hypot(c[0] - p[0], c[1] - p[1]) > 0.03 or c[2] != p[2] for p in picked):
                picked.append(c)
        return picked


def _tool_yaw(q):
    """Yaw of a top-down tool frame: inverse of geometry.tool_down_quat (q = Rz(yaw) * Rx(pi))."""
    return math.atan2(q.y, q.x) * 2 if abs(q.x) + abs(q.y) > 1e-6 else 0.0


def main():
    rclpy.init()
    node = SkillServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

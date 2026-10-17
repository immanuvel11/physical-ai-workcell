"""Safety supervisor: speed & separation monitoring + e-stop, published as /safety/status at 20 Hz.

Inputs
  /safety/humans  geometry_msgs/PoseArray  positions of people (world frame) from a person sensor.
                  In simulation: pai_safety/human_sim. On a real cell: a safety laser scanner or a
                  3D camera + person detector.
  /safety/estop   std_msgs/Bool            true = emergency stop (latched until /safety/reset)
  TF              robot link positions

Zones (separation = horizontal distance from a person to the nearest robot link, minus body radius)
  > slow_distance            NORMAL  speed_scale 1.0
  stop..slow_distance        SLOW    speed_scale slow_scale
  < stop_distance            STOP    protective stop, resumes automatically after resume_delay clear
  e-stop                     ESTOP   until reset
  no person-sensor data      STOP    fail-safe (sensor_timeout)

This is a software layer for the prototype. A production cell needs a safety-rated controller
(PL d / SIL 2) doing the same job in certified hardware.
"""
import math
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pai_interfaces.msg import SafetyStatus

ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
LINKS = ['panda_link2', 'panda_link3', 'panda_link4', 'panda_link5', 'panda_link6', 'panda_link7',
         'panda_hand', 'panda_tcp']


class Supervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('slow_distance', 0.90)
        self.declare_parameter('slow_scale', 0.3)
        self.declare_parameter('body_radius', 0.25)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('resume_delay', 1.0)
        self.humans = []
        self.humans_stamp = None
        self.estop = False
        self.state = 'STOP'
        self.clear_since = None
        self.events = 0
        self.create_subscription(PoseArray, '/safety/humans', self._on_humans, 10)
        # VOLATILE durability is compatible with every publisher (a TRANSIENT_LOCAL subscription would
        # silently ignore e-stops from ordinary publishers such as `ros2 topic pub`).
        self.create_subscription(Bool, '/safety/estop', self._on_estop, QoSProfile(depth=10))
        self.create_service(Trigger, '/safety/reset', self._on_reset)
        self.pub = self.create_publisher(SafetyStatus, '/safety/status', 10)
        # Direct line to the arm controller, independent of MoveIt and the skill server: a one-point
        # trajectory at the current joint positions replaces whatever is executing, so the arm brakes
        # and holds. Re-sent while stopped so nothing new can start.
        self.hold_pub = self.create_publisher(JointTrajectory, '/panda_arm_controller/joint_trajectory', 10)
        self.joints = {}
        self.create_subscription(JointState, '/joint_states', lambda m: self.joints.update(zip(m.name, m.position)), 10)
        self.last_hold = 0.0
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_timer(0.05, self._tick)
        self.get_logger().info('Safety supervisor running')

    def _on_humans(self, msg):
        self.humans = [(p.position.x, p.position.y) for p in msg.poses]
        self.humans_stamp = time.monotonic()

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().error('EMERGENCY STOP')
        self.estop = self.estop or msg.data

    def _on_reset(self, _req, res):
        self.estop = False
        res.success, res.message = True, 'e-stop reset'
        self.get_logger().warn('e-stop reset by operator')
        return res

    def _hold(self):
        point = JointTrajectoryPoint(positions=[float(self.joints[j]) for j in ARM_JOINTS],
                                     velocities=[0.0] * len(ARM_JOINTS), time_from_start=Duration(nanosec=100_000_000))
        self.hold_pub.publish(JointTrajectory(joint_names=ARM_JOINTS, points=[point]))

    def _robot_points(self):
        pts = []
        for f in LINKS:
            try:
                t = self.tf.lookup_transform('world', f, Time()).transform.translation
                pts.append((t.x, t.y))
            except Exception:
                pass
        return pts

    def _tick(self):
        p = self.get_parameter
        now = time.monotonic()
        robot = self._robot_points()
        d_min = math.inf
        for hx, hy in self.humans:
            for rx, ry in robot:
                d_min = min(d_min, math.hypot(hx - rx, hy - ry) - p('body_radius').value)
        d_min = max(d_min, 0.0)

        if self.estop:
            state, scale, reason = 'ESTOP', 0.0, 'emergency stop pressed'
        elif self.humans_stamp is None or now - self.humans_stamp > p('sensor_timeout').value:
            state, scale, reason = 'STOP', 0.0, 'person sensor silent (fail-safe)'
        elif not robot:
            state, scale, reason = 'STOP', 0.0, 'robot pose unknown'
        elif d_min < p('stop_distance').value:
            state, scale, reason = 'STOP', 0.0, f'person {d_min:.2f} m from the robot'
        elif d_min < p('slow_distance').value:
            state, scale, reason = 'SLOW', p('slow_scale').value, f'person {d_min:.2f} m from the robot'
        else:
            state, scale, reason = 'NORMAL', 1.0, 'clear'

        # Hysteresis: leave STOP only after the cause has been clear for resume_delay seconds.
        if self.state in ('STOP', 'ESTOP') and state in ('NORMAL', 'SLOW'):
            if self.clear_since is None:
                self.clear_since = now
            if now - self.clear_since < p('resume_delay').value:
                state, scale, reason = self.state, 0.0, f'waiting to resume ({reason})'
        else:
            self.clear_since = None
        if state != self.state:
            self.events += 1
            # (rclpy forbids one log call site with changing severities: use two call sites)
            if state == 'NORMAL':
                self.get_logger().info(f'safety {self.state} -> {state}: {reason}')
            else:
                self.get_logger().warn(f'safety {self.state} -> {state}: {reason}')
            self.state = state

        if state in ('STOP', 'ESTOP') and all(j in self.joints for j in ARM_JOINTS) and now - self.last_hold > 0.2:
            self._hold()
            self.last_hold = now
        msg = SafetyStatus(state=state, speed_scale=float(scale), min_human_distance=float(d_min), reason=reason)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

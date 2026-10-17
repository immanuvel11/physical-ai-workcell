"""Simulated co-worker + person sensor.

Moves the 'worker' mannequin in Gazebo and publishes its position on /safety/humans at 10 Hz,
standing in for a safety laser scanner. /safety/human_sim/walk_in (Trigger) plays a scripted
visit: walk up to the corner of the table next to the bins, reach in, wait, walk away.
"""
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from pai_core.sim import SimControl

FLOOR_Z = -0.75


class HumanSim(Node):
    def __init__(self):
        super().__init__('human_sim')
        self.declare_parameter('name', 'worker')
        self.declare_parameter('park', [2.2, -1.6])
        self.declare_parameter('speed', 0.8)
        # waypoints of the visit: (x, y, dwell seconds)
        # walk to the front-left corner of the table and reach in towards the bins for 8 s
        self.declare_parameter('visit', [1.4, 1.2, 0.0, 0.75, 0.62, 8.0])
        self.cb = ReentrantCallbackGroup()
        self.pos = list(self.get_parameter('park').value)
        self.busy = False
        self.sim = SimControl(self)
        self.pub = self.create_publisher(PoseArray, '/safety/humans', 10)
        self.create_timer(0.1, self._publish, callback_group=self.cb)
        self.create_service(Trigger, '/safety/human_sim/walk_in', self._on_walk_in, callback_group=self.cb)
        self.get_logger().info('Human simulator ready (call /safety/human_sim/walk_in)')

    def _publish(self):
        msg = PoseArray()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        p = Pose()
        p.position.x, p.position.y = self.pos
        p.orientation.w = 1.0
        msg.poses = [p]
        self.pub.publish(msg)

    def _on_walk_in(self, _req, res):
        if self.busy:
            res.success, res.message = False, 'already walking'
            return res
        threading.Thread(target=self._walk, daemon=True).start()
        res.success, res.message = True, 'worker is walking in'
        return res

    def _walk(self):
        self.busy = True
        v = self.get_parameter('visit').value
        stops = [(v[i], v[i + 1], v[i + 2]) for i in range(0, len(v), 3)]
        park = self.get_parameter('park').value
        for x, y, dwell in stops + [(stops[0][0], stops[0][1], 0.0), (park[0], park[1], 0.0)]:
            self._go_to(x, y)
            time.sleep(dwell)
        self.busy = False

    def _go_to(self, x, y, dt=0.1):
        sx, sy = self.pos
        dist = math.hypot(x - sx, y - sy)
        steps = max(1, int(dist / (self.get_parameter('speed').value * dt)))
        yaw = math.atan2(y - sy, x - sx)
        for k in range(1, steps + 1):
            self.pos = [sx + (x - sx) * k / steps, sy + (y - sy) * k / steps]
            self.sim.set_pose(self.get_parameter('name').value, self.pos[0], self.pos[1], FLOOR_Z, yaw + math.pi)
            time.sleep(dt)


def main():
    rclpy.init()
    node = HumanSim()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

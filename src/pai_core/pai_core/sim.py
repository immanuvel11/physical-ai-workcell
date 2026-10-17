"""Simulation control used by tests and the benchmark (never by the pipeline itself)."""
import math
import subprocess

from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose

from pai_core.geometry import yaw_quat
from pai_core.ros_utils import wait_future


class SimControl:
    def __init__(self, node, world='workcell'):
        self.node = node
        self.world = world
        self.set_pose_client = node.create_client(SetEntityPose, f'/world/{world}/set_pose')

    def set_pose(self, name, x, y, z, yaw=0.0, timeout=5.0):
        if not self.set_pose_client.wait_for_service(timeout_sec=timeout):
            return False
        req = SetEntityPose.Request()
        req.entity = Entity(name=name, type=Entity.MODEL)
        req.pose.position.x, req.pose.position.y, req.pose.position.z = float(x), float(y), float(z)
        req.pose.orientation = yaw_quat(yaw)
        res = wait_future(self.set_pose_client.call_async(req), timeout)
        return bool(res and res.success)

    def set_light(self, direction, intensity=0.9, name='sun'):
        """Change the directional light (Gazebo UserCommands light_config service)."""
        n = math.sqrt(sum(v * v for v in direction)) or 1.0
        dx, dy, dz = (v / n for v in direction)
        req = (f'name: "{name}", type: DIRECTIONAL, cast_shadows: true, '
               f'diffuse: {{r: {intensity}, g: {intensity}, b: {intensity}, a: 1}}, '
               f'specular: {{r: 0.2, g: 0.2, b: 0.2, a: 1}}, direction: {{x: {dx}, y: {dy}, z: {dz}}}')
        out = subprocess.run(['ign', 'service', '-s', f'/world/{self.world}/light_config',
                              '--reqtype', 'ignition.msgs.Light', '--reptype', 'ignition.msgs.Boolean',
                              '--timeout', '3000', '--req', req], capture_output=True, text=True, timeout=10)
        return 'true' in out.stdout

#!/usr/bin/env python3
"""Phase 1 acceptance test: robot, controllers, camera geometry, MoveIt motion, gripper.

Run while bringup.launch.py is up:
    ros2 run pai_bringup phase1_check.py
Exits 0 if every check passes.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest,
                             OrientationConstraint, PositionConstraint)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

ARM_JOINTS = [f'panda_joint{i}' for i in range(1, 8)]
CAMERA_CLEAR = [0.0, -1.4, 0.0, -2.9, 0.0, 1.6, 0.785]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
GRIPPER_OPEN = 0.035  # keep off the 0.04 joint limit (see panda_gz.urdf.xacro)
GRIPPER_CLOSED = 0.002  # and off the 0.0 limit
TCP_DOWN = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)  # tool z-axis pointing at the table


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Phase1Check(Node):
    def __init__(self):
        super().__init__('phase1_check', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.joint_state = None
        self.image = self.depth = self.info = None
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joint_state', m), 10)
        self.create_subscription(Image, '/camera/image', lambda m: setattr(self, 'image', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth_image', lambda m: setattr(self, 'depth', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera_info', lambda m: setattr(self, 'info', m), qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.gripper = ActionClient(self, GripperCommand, '/panda_hand_controller/gripper_cmd')
        self.list_ctrl = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.results = []

    # ---------------------------------------------------------------- helpers
    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if cond():
                return True
        return False

    def wait_future(self, future, timeout):
        return self.spin_until(future.done, timeout) and future.result()

    def record(self, name, ok, detail=''):
        self.results.append((name, ok, detail))
        self.get_logger().info(f'[{"PASS" if ok else "FAIL"}] {name} {detail}')
        return ok

    def lookup(self, target, source, timeout=5.0):
        tf = None

        def ready():
            nonlocal tf
            try:
                tf = self.tf_buffer.lookup_transform(target, source, Time())
                return True
            except Exception:
                return False
        self.spin_until(ready, timeout)
        return tf

    def move_joints(self, positions, label):
        goal = MoveGroup.Goal()
        goal.request = self.base_request()
        goal.request.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for j, p in zip(ARM_JOINTS, positions)])]
        return self.send_move(goal, label)

    def move_tcp(self, xyz, label):
        goal = MoveGroup.Goal()
        goal.request = self.base_request()
        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = xyz
        centre.orientation.w = 1.0
        region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.003])],
                                primitive_poses=[centre])
        pc = PositionConstraint(link_name='panda_tcp', constraint_region=region, weight=1.0)
        pc.header.frame_id = 'world'
        oc = OrientationConstraint(link_name='panda_tcp', orientation=TCP_DOWN, weight=1.0,
                                   absolute_x_axis_tolerance=0.02, absolute_y_axis_tolerance=0.02,
                                   absolute_z_axis_tolerance=3.14)  # any yaw is fine
        oc.header.frame_id = 'world'
        goal.request.goal_constraints = [Constraints(position_constraints=[pc], orientation_constraints=[oc])]
        return self.send_move(goal, label)

    def base_request(self):
        r = MotionPlanRequest()
        r.group_name = 'panda_arm'
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = 0.4
        r.max_acceleration_scaling_factor = 0.4
        return r

    def send_move(self, goal, label):
        goal.planning_options.plan_only = False
        t0 = time.monotonic()
        handle = self.wait_future(self.move_group.send_goal_async(goal), 10.0)
        if not handle or not handle.accepted:
            return self.record(label, False, 'goal rejected')
        result = self.wait_future(handle.get_result_async(), 60.0)
        code = result.result.error_code.val if result else None
        return self.record(label, code == 1, f'(moveit code {code}, {time.monotonic() - t0:.1f}s)')

    def gripper_to(self, position, label):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 20.0
        handle = self.wait_future(self.gripper.send_goal_async(goal), 10.0)
        if not handle or not handle.accepted:
            return self.record(label, False, 'goal rejected')
        self.wait_future(handle.get_result_async(), 15.0)
        self.spin_until(lambda: False, 0.5)
        actual = dict(zip(self.joint_state.name, self.joint_state.position))['panda_finger_joint1']
        return self.record(label, abs(actual - position) < 0.004, f'(finger at {actual:.4f} m, target {position})')

    # ----------------------------------------------------------------- checks
    def check_robot(self):
        ok = self.spin_until(lambda: self.joint_state is not None and len(self.joint_state.name) >= 9, 90.0)
        self.record('joint_states published (9 joints)', ok)
        if not self.list_ctrl.wait_for_service(timeout_sec=30.0):
            return self.record('controller_manager reachable', False)
        need = {'joint_state_broadcaster', 'panda_arm_controller', 'panda_hand_controller'}

        def all_active():
            res = self.wait_future(self.list_ctrl.call_async(ListControllers.Request()), 5.0)
            return res and need <= {c.name for c in res.controller if c.state == 'active'}
        end = time.monotonic() + 60.0
        while time.monotonic() < end and not all_active():
            time.sleep(1.0)
        return self.record('controllers active (jsb, arm, hand)', all_active())

    def check_camera(self, scene):
        ok = self.spin_until(lambda: None not in (self.image, self.depth, self.info), 30.0)
        if not self.record('camera rgb + depth + info streaming', ok):
            return False
        self.record('camera frame_id is camera_optical_frame', self.image.header.frame_id == 'camera_optical_frame',
                    f'(got "{self.image.header.frame_id}")')
        rgb = np.frombuffer(self.image.data, np.uint8).reshape(self.image.height, self.image.width, 3)
        depth = np.frombuffer(self.depth.data, np.float32).reshape(self.depth.height, self.depth.width)

        # Find the red cube purely by colour (just a geometry check; real perception comes in Phase 3).
        r, g, b = (rgb[..., i].astype(int) for i in range(3))
        mask = (r > g + 50) & (r > b + 50)  # relative test: robust to scene lighting
        if mask.sum() < 20:
            return self.record('red_cube visible to camera', False, f'({mask.sum()} red pixels)')
        vs, us = np.nonzero(mask)
        u, v = us.mean(), vs.mean()
        d = float(np.median(depth[mask]))
        k = self.info.k
        p_cam = np.array([(u - k[2]) * d / k[0], (v - k[5]) * d / k[4], d])

        tf = self.lookup('world', 'camera_optical_frame')
        if tf is None:
            return self.record('TF world -> camera_optical_frame', False)
        t, q = tf.transform.translation, tf.transform.rotation
        p_world = quat_to_mat(q) @ p_cam + np.array([t.x, t.y, t.z])

        cube = next(o for o in scene['objects'] if o['name'] == 'red_cube')
        truth = np.array([cube['pose'][0], cube['pose'][1], cube['size'][2]])  # top face centre
        err = np.linalg.norm(p_world[:2] - truth[:2])
        dz = abs(p_world[2] - truth[2])
        return self.record('camera->world projection of red_cube within 1.5 cm',
                           err < 0.015 and dz < 0.015,
                           f'(measured {np.round(p_world, 3).tolist()}, truth {truth.tolist()}, xy err {err * 1000:.1f} mm)')

    def check_motion(self, scene):
        if not self.move_group.wait_for_server(timeout_sec=60.0):
            return self.record('move_group action server', False)
        self.move_joints(CAMERA_CLEAR, 'MoveIt: joint goal camera_clear')
        cube = next(o for o in scene['objects'] if o['name'] == 'red_cube')
        target = (cube['pose'][0], cube['pose'][1], 0.20)
        if self.move_tcp(target, f'MoveIt: TCP pose goal above red_cube {target}'):
            self.spin_until(lambda: False, 1.0)  # let the arm settle
            tf = self.lookup('world', 'panda_tcp')
            p = tf.transform.translation
            err = math.dist((p.x, p.y, p.z), target)
            self.record('TCP reached target within 1 cm (in Gazebo)', err < 0.01, f'(error {err * 1000:.1f} mm)')
        # Several cycles: a single close/open can pass even when the fingers later get stuck.
        for i in range(1, 4):
            # 2 mm, not 0: a velocity-driven DART joint parked exactly on either limit can stick there
            self.gripper_to(GRIPPER_CLOSED, f'gripper close (cycle {i})')
            self.gripper_to(GRIPPER_OPEN, f'gripper open (cycle {i})')
        self.move_joints(READY, 'MoveIt: back to ready')


def main():
    rclpy.init()
    node = Phase1Check()
    with open(f"{get_package_share_directory('pai_bringup')}/config/scene_default.yaml") as f:
        scene = yaml.safe_load(f)
    node.check_robot()
    node.check_camera(scene)
    node.check_motion(scene)
    passed = sum(ok for _, ok, _ in node.results)
    print(f'\n===== Phase 1: {passed}/{len(node.results)} checks passed =====')
    for name, ok, detail in node.results:
        print(f'  {"PASS" if ok else "FAIL"}  {name} {detail}')
    rclpy.shutdown()
    sys.exit(0 if passed == len(node.results) else 1)


if __name__ == '__main__':
    main()

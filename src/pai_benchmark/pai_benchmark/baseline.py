"""Traditional rule-based robot cell, for comparison: /pai/baseline/run_instruction.

How cells are typically programmed today:
  * a handful of programs, each taught once at commissioning with fixed pick and place
    coordinates (here: the nominal layout in scene_default.yaml),
  * the operator selects a program (here: keyword match on the instruction text),
  * execution is open-loop: no camera, no check of where parts really are, no re-planning.
It uses exactly the same skills / motion planner as the AI pipeline, so the comparison isolates
task understanding and adaptation rather than motion quality.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Point, Pose, Vector3
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pai_core.geometry import yaw_quat
from pai_core.ros_utils import wait_future
from pai_interfaces.action import MoveTo, Pick, Place, RunInstruction

# Taught at commissioning (nominal layout). Pick: x, y, yaw, size; place: x, y, surface z, rim z.
TAUGHT_PICKS = {
    'red_cube': (0.45, 0.05, 0.3, (0.04, 0.04, 0.04)),
    'blue_bolt': (0.58, -0.12, 0.0, (0.03, 0.03, 0.07)),
    'yellow_block': (0.36, -0.16, 0.8, (0.06, 0.03, 0.03)),
    'green_nut': (0.62, 0.10, 0.0, (0.04, 0.04, 0.02)),
}
TAUGHT_PLACES = {
    'blue_bin': (0.30, 0.45, 0.008, 0.08),
    'green_bin': (0.55, 0.45, 0.008, 0.08),
    'tray_slot_1': (0.40, -0.45, 0.025, 0.0),
    'tray_slot_2': (0.51, -0.45, 0.025, 0.0),
}
PROGRAMS = {   # program -> [(taught part, taught place)]
    'P1 red cube to blue bin': [('red_cube', 'blue_bin')],
    'P2 sort by colour': [('blue_bolt', 'blue_bin'), ('green_nut', 'green_bin')],
    'P3 build kit': [('yellow_block', 'tray_slot_1'), ('blue_bolt', 'tray_slot_2')],
}


def select_program(text):
    t = text.lower()
    if all(w in t for w in ('red', 'cube', 'blue', 'bin')):
        return 'P1 red cube to blue bin'
    if 'sort' in t:
        return 'P2 sort by colour'
    if 'kit' in t and 'yellow' in t and 'bolt' in t:
        return 'P3 build kit'
    return None


class Baseline(Node):
    def __init__(self):
        super().__init__('baseline_cell')
        cb = ReentrantCallbackGroup()
        self.pick = ActionClient(self, Pick, '/pai/skills/pick', callback_group=cb)
        self.place = ActionClient(self, Place, '/pai/skills/place', callback_group=cb)
        self.move_to = ActionClient(self, MoveTo, '/pai/skills/move_to', callback_group=cb)
        ActionServer(self, RunInstruction, '/pai/baseline/run_instruction', execute_callback=self._execute,
                     callback_group=cb)
        self.get_logger().info('Rule-based baseline cell ready')

    def _call(self, client, goal):
        h = wait_future(client.send_goal_async(goal), 20)
        if h is None or not h.accepted:
            return False, 'skill unavailable'
        r = wait_future(h.get_result_async(), 300)
        return (r.result.success, r.result.message) if r else (False, 'timeout')

    def _fb(self, gh, state, msg, step=-1):
        gh.publish_feedback(RunInstruction.Feedback(state=state, current_step=step, message=msg))

    def _execute(self, gh):
        t0 = time.monotonic()
        result = RunInstruction.Result()
        program = select_program(gh.request.instruction)
        steps = []
        if program:
            for part, place in PROGRAMS[program]:
                steps += [{'skill': 'pick', 'object_id': part, 'target_id': '', 'named_target': '', 'why': 'taught'},
                          {'skill': 'place', 'object_id': '', 'target_id': place, 'named_target': '', 'why': 'taught'}]
        result.plan_json = json.dumps({'status': 'ok' if program else 'impossible', 'reasoning': program or '',
                                       'question': '', 'steps': steps})
        if program is None:
            result.success, result.summary = False, 'FAILED: no taught program matches this instruction'
            result.total_time = float(time.monotonic() - t0)
            gh.abort()
            return result
        result.time_to_first_motion = float(time.monotonic() - t0)
        ok, msg = True, ''
        for i, (part, place) in enumerate(PROGRAMS[program]):
            x, y, yaw, size = TAUGHT_PICKS[part]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = x, y, size[2] / 2
            pose.orientation = yaw_quat(yaw)
            self._fb(gh, 'EXECUTING', f'{program}: pick at taught pose of {part}', 2 * i)
            ok, msg = self._call(self.pick, Pick.Goal(pose=pose, size=Vector3(x=size[0], y=size[1], z=size[2])))
            if not ok:
                break
            px, py, surface, rim = TAUGHT_PLACES[place]
            self._fb(gh, 'EXECUTING', f'{program}: place at {place}', 2 * i + 1)
            ok, msg = self._call(self.place, Place.Goal(target_point=Point(x=px, y=py, z=0.0),
                                                        surface_z=surface, rim_z=rim))
            if not ok:
                break
        self._call(self.move_to, MoveTo.Goal(named_target='camera_clear'))
        result.success = ok        # open loop: "success" = every taught motion completed
        result.summary = f'{program}: ' + ('program completed' if ok else f'stopped: {msg}')
        result.total_time = float(time.monotonic() - t0)
        gh.succeed() if ok else gh.abort()
        return result


def main():
    rclpy.init()
    node = Baseline()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

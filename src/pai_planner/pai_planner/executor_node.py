"""Closed-loop task executor: /pai/run_instruction (pai_interfaces/RunInstruction).

    perceive -> plan -> for each step: execute skill -> verify with fresh perception
                    -> on failure or surprise: re-perceive, re-plan the remaining work (max N times)

planner:=auto    Claude when available, otherwise the offline rule planner
planner:=claude  Claude only (fails if unavailable)
planner:=rule    offline keyword planner (benchmark ablation)
"""
import json
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from pai_core.claude import ClaudeClient
from pai_core.ros_utils import wait_future
from pai_interfaces.action import MoveTo, Pick, Place, RunInstruction
from pai_interfaces.msg import SafetyStatus, TaskStatus
from pai_planner.planner import ClaudePlanner, RulePlanner, validate


class TaskFailed(Exception):
    pass


class Executor(Node):
    def __init__(self):
        super().__init__('executor')
        self.declare_parameter('planner', 'auto')
        self.declare_parameter('model', 'claude-opus-5-5')
        self.declare_parameter('effort', 'medium')
        self.declare_parameter('max_replans', 3)
        self.declare_parameter('log_dir', os.path.expanduser('~/.ros/pai/runs'))
        self.cb = ReentrantCallbackGroup()
        self.busy = threading.Lock()
        self.safety = None
        self.status = TaskStatus(state='IDLE')

        self.rule = RulePlanner()
        client = ClaudeClient(self.get_logger(), model=self.get_parameter('model').value)
        self.claude = ClaudePlanner(client, effort=self.get_parameter('effort').value)
        if not self.claude.available:
            self.get_logger().warn(f'Claude planner unavailable: {client.reason}')

        self.pick = ActionClient(self, Pick, '/pai/skills/pick', callback_group=self.cb)
        self.place = ActionClient(self, Place, '/pai/skills/place', callback_group=self.cb)
        self.move_to = ActionClient(self, MoveTo, '/pai/skills/move_to', callback_group=self.cb)
        self.refresh_cli = self.create_client(Trigger, '/world_model/refresh', callback_group=self.cb)
        self.status_pub = self.create_publisher(TaskStatus, '/pai/task_status', 10)
        self.create_subscription(SafetyStatus, '/safety/status', lambda m: setattr(self, 'safety', m), 10,
                                 callback_group=self.cb)
        self.create_timer(0.5, lambda: self.status_pub.publish(self.status), callback_group=self.cb)
        ActionServer(self, RunInstruction, '/pai/run_instruction', execute_callback=self._execute,
                     goal_callback=lambda _: GoalResponse.REJECT if self.busy.locked() else GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.cb)
        self.get_logger().info(f'Executor ready (planner: {self._planner_name()})')

    # ------------------------------------------------------------------ helpers
    @property
    def mode(self):
        # read on every task, so `ros2 param set /executor planner rule` switches at runtime
        return self.get_parameter('planner').value

    def _planner_name(self):
        if self.mode == 'rule' or (self.mode == 'auto' and not self.claude.available):
            return 'rule'
        return 'claude'

    def _set(self, gh, state, message='', step=-1, plan=None):
        self.status.state, self.status.message = state, message
        self.status.current_step = step
        if plan is not None:
            self.status.plan_json = json.dumps(plan)
        self.status.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(self.status)
        fb = RunInstruction.Feedback(state=state, current_step=step, message=message)
        gh.publish_feedback(fb)
        self.get_logger().info(f'[{state}] {message}')

    def _perceive(self):
        res = wait_future(self.refresh_cli.call_async(Trigger.Request()), 120.0)
        if res is None or not res.success:
            raise TaskFailed(f'perception failed: {res.message if res else "timeout"}')
        return json.loads(res.message)

    def _skill(self, client, goal, what, gh):
        handle = wait_future(client.send_goal_async(goal), 20.0)
        if handle is None or not handle.accepted:
            return False, f'{what}: skill server busy or unavailable'
        result_future = handle.get_result_async()
        while not result_future.done():
            if gh.is_cancel_requested:
                wait_future(handle.cancel_goal_async(), 5.0)
                raise TaskFailed('cancelled by operator')
            if self.safety is not None and self.safety.state in ('STOP', 'ESTOP') and self.status.state != 'PAUSED':
                self._set(gh, 'PAUSED', f'safety {self.safety.state}: {self.safety.reason}', self.status.current_step)
            elif self.status.state == 'PAUSED' and self.safety is not None and self.safety.state in ('NORMAL', 'SLOW'):
                self._set(gh, 'EXECUTING', 'resumed', self.status.current_step)
            time.sleep(0.05)
        r = result_future.result().result
        return r.success, r.message

    def _make_plan(self, instruction, scene, history, gh):
        """Returns (plan, planner_name, seconds)."""
        t0 = time.monotonic()
        name = self._planner_name()
        plan = None
        if name == 'claude':
            plan = self.claude.plan(instruction, scene, history)
            if plan is None:
                if self.mode == 'claude':
                    raise TaskFailed(f'Claude planner failed: {self.claude.last_error}')
                self.get_logger().warn(f'Claude planner failed ({self.claude.last_error}); using rule planner')
                name = 'rule'
        if plan is None:
            plan = self.rule.plan(instruction, scene, history)
            problems = validate(plan, scene)
            if problems:
                plan = {'status': 'impossible', 'reasoning': '; '.join(problems), 'question': '', 'steps': []}
        self.status.planner = name
        return plan, name, time.monotonic() - t0

    # ------------------------------------------------------------------ main loop
    def _execute(self, gh):
        with self.busy:
            req = gh.request
            result = RunInstruction.Result()
            t_start = time.monotonic()
            t_first_motion = None
            planning_time = 0.0
            replans = 0
            history = []
            plan = {}
            self.status = TaskStatus(instruction=req.instruction, state='PERCEIVING')
            try:
                self._set(gh, 'PERCEIVING', 'looking at the workcell')
                ok, msg = self._skill(self.move_to, MoveTo.Goal(named_target='camera_clear'), 'move_to', gh)
                if not ok:
                    raise TaskFailed(msg)
                scene = self._perceive()

                self._set(gh, 'PLANNING', f'planning with {self._planner_name()} planner')
                plan, planner, dt = self._make_plan(req.instruction, scene, history, gh)
                planning_time += dt
                self._set(gh, 'PLANNING', f'{planner}: {plan["reasoning"]}', plan=plan)
                if plan['status'] == 'clarify':
                    result.summary = f'NEEDS_CLARIFICATION: {plan["question"]}'
                    self._set(gh, 'NEEDS_CLARIFICATION', plan['question'])
                    return self._finish(gh, result, False, plan, planning_time, t_first_motion, t_start, replans)
                if plan['status'] == 'impossible':
                    raise TaskFailed(f'cannot do this: {plan["reasoning"]}')
                if req.dry_run:
                    result.summary = f'dry run: {len(plan["steps"])} steps'
                    return self._finish(gh, result, True, plan, planning_time, t_first_motion, t_start, replans)

                steps = list(plan['steps'])
                committed = [st['object_id'] for st in steps if st['skill'] == 'pick']   # promised deliveries
                i = 0
                while i < len(steps):
                    st = steps[i]
                    label = f'{st["skill"]} {st["object_id"] or st["target_id"] or st["named_target"]}'
                    self._set(gh, 'EXECUTING', f'step {i + 1}/{len(steps)}: {label}', i)
                    if t_first_motion is None:
                        t_first_motion = time.monotonic() - t_start
                    ok, msg = self._run_step(st, gh)
                    if ok and st['skill'] == 'place':
                        self._set(gh, 'VERIFYING', f'checking that {label} worked', i)
                        self._look(gh)
                        scene = self._perceive()
                        ok, msg = self._verify_place(steps, i, scene)
                    history.append({'step': st, 'result': 'ok' if ok else 'failed', 'detail': msg})
                    if ok:
                        i += 1
                        continue
                    # ---------------- self-correction: re-perceive and re-plan the remaining work
                    replans += 1
                    if replans > self.get_parameter('max_replans').value:
                        raise TaskFailed(f'giving up after {replans - 1} re-plans; last error: {msg}')
                    self._set(gh, 'REPLANNING', f'{label} failed ({msg}); re-perceiving and re-planning', i)
                    self._look(gh)
                    scene = self._perceive()
                    plan, planner, dt = self._make_plan(req.instruction, scene, history, gh)
                    planning_time += dt
                    self._set(gh, 'REPLANNING', f'{planner}: {plan["reasoning"]}', plan=plan)
                    if plan['status'] != 'ok':
                        raise TaskFailed(f'after a failure the planner says {plan["status"]}: '
                                         f'{plan["question"] or plan["reasoning"]}')
                    steps, i = list(plan['steps']), 0

                self._skill(self.move_to, MoveTo.Goal(named_target='camera_clear'), 'move_to', gh)
                done = sum(1 for h in history if h['result'] == 'ok' and h['step']['skill'] == 'place')
                if done < len(committed):
                    # e.g. a part fell off the table and vanished from view: re-planning then finds "nothing
                    # left to do". Never report that as success.
                    raise TaskFailed(f'only {done} of {len(committed)} part(s) delivered; lost track of a part '
                                     f'(planned: {", ".join(committed)})')
                result.summary = f'done: {done} part(s) placed, {replans} re-plan(s)'
                self._set(gh, 'SUCCEEDED', result.summary)
                return self._finish(gh, result, True, plan, planning_time, t_first_motion, t_start, replans)
            except Exception as e:   # TaskFailed or anything unexpected: always end the goal cleanly
                if not isinstance(e, TaskFailed):
                    self.get_logger().error(f'executor error: {e!r}')
                result.summary = f'FAILED: {e}'
                self._set(gh, 'FAILED', str(e))
                return self._finish(gh, result, False, plan, planning_time, t_first_motion, t_start, replans)

    def _look(self, gh):
        """Move the arm out of the camera view; perceiving with the arm in view is not trusted."""
        for _ in range(3):
            ok, msg = self._skill(self.move_to, MoveTo.Goal(named_target='camera_clear'), 'move_to', gh)
            if ok:
                return
            time.sleep(1.0)
        raise TaskFailed(f'cannot move the arm out of the camera view: {msg}')

    def _run_step(self, st, gh):
        if st['skill'] == 'pick':
            return self._skill(self.pick, Pick.Goal(object_id=st['object_id']), 'pick', gh)
        if st['skill'] == 'place':
            return self._skill(self.place, Place.Goal(target_id=st['target_id']), 'place', gh)
        return self._skill(self.move_to, MoveTo.Goal(named_target=st['named_target']), 'move_to', gh)

    @staticmethod
    def _verify_place(steps, i, scene):
        """The part picked just before this place must now be in/on the target."""
        picked = next((s['object_id'] for s in reversed(steps[:i]) if s['skill'] == 'pick'), None)
        target = steps[i]['target_id']
        if picked is None:
            return True, 'ok'
        rels = scene.get('relations', [])
        if f'{picked} in {target}' in rels or f'{picked} on {target}' in rels:
            return True, 'ok'
        return False, f'verification: {picked} is not in/on {target} after placing'

    def _finish(self, gh, result, success, plan, planning_time, t_first_motion, t_start, replans):
        result.success = success
        result.plan_json = json.dumps(plan)
        result.planning_time = float(planning_time)
        result.time_to_first_motion = float(t_first_motion or 0.0)
        result.total_time = float(time.monotonic() - t_start)
        result.replans = replans
        if gh.is_cancel_requested:
            gh.canceled()
        elif success:
            gh.succeed()
        else:
            gh.abort()
        self._log_run(gh.request.instruction, result)
        return result

    def _log_run(self, instruction, r):
        d = self.get_parameter('log_dir').value
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, 'runs.jsonl'), 'a') as f:
            f.write(json.dumps({'time': time.time(), 'instruction': instruction, 'planner': self.status.planner,
                                'success': r.success, 'summary': r.summary, 'planning_time': r.planning_time,
                                'time_to_first_motion': r.time_to_first_motion, 'total_time': r.total_time,
                                'replans': r.replans, 'plan': json.loads(r.plan_json or '{}')}) + '\n')


def main():
    rclpy.init()
    node = Executor()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()

"""Randomized benchmark: AI pipeline vs. the rule-based baseline, scored on Gazebo ground truth.

    ros2 run pai_benchmark benchmark --systems claude rule baseline --trials 3 --out ~/pai_results

Systems
  claude    the full pipeline with the Claude planner
  rule      the full pipeline with the offline keyword planner (ablation: perception + closed loop
            without language understanding)
  baseline  traditional cell: taught coordinates, keyword program selection, open loop
Conditions (cumulative variation, see CONDITIONS)
  nominal, pose, lighting, clutter, disturbance, novel
A trial succeeds when every goal holds in Gazebo afterwards AND no other part was disturbed.
"""
import argparse
import csv
import json
import math
import os
import random
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from pai_benchmark.common import Harness
from pai_core.ros_utils import wait_future
from pai_interfaces.action import RunInstruction
from rclpy.action import ActionClient

NOMINAL = {'red_cube': (0.45, 0.05, 0.3), 'blue_bolt': (0.58, -0.12, 0.0),
           'yellow_block': (0.36, -0.16, 0.8), 'green_nut': (0.62, 0.10, 0.0)}
TASKS = {
    'single': ('Put the red cube in the blue bin.', [('red_cube', 'blue_bin')]),
    'sort': ('Sort the parts by colour: blue parts go in the blue bin and green parts in the green bin.',
             [('blue_bolt', 'blue_bin'), ('green_nut', 'green_bin')]),
    'kit': ('Prepare a kit: place the yellow block and the blue bolt on the kit tray.',
            [('yellow_block', 'kit_tray'), ('blue_bolt', 'kit_tray')]),
}
# Unseen objects and free-form phrasing (never used while developing the planner prompts).
NOVEL_TASKS = {
    'novel_object': ('Drop the orange box into the green bin.', [('orange_box', 'green_bin')], ['orange_box']),
    'novel_pair': ('The purple cylinder and the cyan block belong on the kit tray.',
                   [('purple_cylinder', 'kit_tray'), ('cyan_block', 'kit_tray')], ['purple_cylinder', 'cyan_block']),
    'paraphrase': ('That crimson cube goes with the other blue things.', [('red_cube', 'blue_bin')], []),
    'paraphrase_kit': ('Load the bolt and the yellow bar onto the kitting tray, please.',
                       [('blue_bolt', 'kit_tray'), ('yellow_block', 'kit_tray')], []),
}
CONDITIONS = ['nominal', 'pose', 'lighting', 'clutter', 'disturbance', 'novel']
WORK_AREA = (0.32, 0.68, -0.26, 0.28)      # where parts are scattered (x0, x1, y0, y1)
DISTRACTORS = ['orange_box', 'cyan_block', 'red_cylinder', 'purple_cylinder']
DEFAULT_LIGHT = ([-0.4, 0.2, -0.9], 0.9)
ACTION = {'claude': '/pai/run_instruction', 'rule': '/pai/run_instruction', 'baseline': '/pai/baseline/run_instruction'}


def scatter(names, rng, avoid=(), min_gap=0.09, obstacle_gap=0.12):
    """Random non-overlapping (x, y, yaw) for each name inside WORK_AREA.
    `avoid` are obstacle centres (the cup); parts keep obstacle_gap from them."""
    x0, x1, y0, y1 = WORK_AREA
    placed = []
    out = {}
    for n in names:
        for _ in range(500):
            x, y = rng.uniform(x0, x1), rng.uniform(y0, y1)
            if all(math.hypot(x - px, y - py) >= min_gap for px, py in placed) and \
                    all(math.hypot(x - ax, y - ay) >= obstacle_gap for ax, ay in avoid):
                break
        placed.append((x, y))
        out[n] = (x, y, rng.uniform(-math.pi, math.pi))
    return out


def make_trial(condition, task, seed):
    """-> dict(instruction, goals, layout, light, disturb)."""
    rng = random.Random(f'{condition}/{task}/{seed}')
    if condition == 'novel':
        instruction, goals, extra = NOVEL_TASKS[task]
        names = list(NOMINAL) + extra
        layout = scatter(names, rng, avoid=[(0.50, 0.18)])
        layout['coffee_cup'] = (0.50, 0.18, 0.0)
        return dict(instruction=instruction, goals=goals, layout=layout, light=DEFAULT_LIGHT, disturb=None)
    instruction, goals = TASKS[task]
    trial = dict(instruction=instruction, goals=goals, light=DEFAULT_LIGHT, disturb=None)
    if condition == 'nominal':
        trial['layout'] = dict(NOMINAL, coffee_cup=(0.50, 0.18, 0.0))
        return trial
    cup = (0.50, 0.18)
    names = list(NOMINAL)
    if condition == 'clutter':
        cup = (rng.uniform(*WORK_AREA[:2]), rng.uniform(*WORK_AREA[2:]))
        names += rng.sample(DISTRACTORS, 2)
    layout = scatter(names, rng, avoid=[cup])
    layout['coffee_cup'] = (cup[0], cup[1], 0.0)
    trial['layout'] = layout
    if condition in ('lighting', 'clutter'):
        az, el = rng.uniform(-math.pi, math.pi), rng.uniform(0.35, 1.2)
        trial['light'] = ([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), -math.sin(el)],
                          rng.uniform(0.35, 1.0))
    if condition == 'disturbance':
        part = goals[0][0]
        trial['disturb'] = (part, scatter([part], rng, avoid=[(v[0], v[1]) for v in layout.values()])[part])
    return trial


class Bench(Harness):
    def __init__(self):
        super().__init__('benchmark')
        self.baseline_cli = ActionClient(self, RunInstruction, '/pai/baseline/run_instruction')
        self.wm_refresh = self.create_client(Trigger, '/world_model/refresh')

    def set_planner(self, name):
        subprocess.run(['ros2', 'param', 'set', '/executor', 'planner', name], capture_output=True, timeout=30)

    def run_trial(self, system, trial):
        self.place_objects(trial['layout'])
        self.sim.set_light(*trial['light'])
        self.home()
        wait_future(self.wm_refresh.call_async(Trigger.Request()), 60)   # same collision model for every system
        before = {n: (self.gz[n].x, self.gz[n].y) for n in self.objects if n in self.gz}
        moved = {'done': False, 'time': None}

        def on_feedback(fb):
            d = trial['disturb']
            if d and not moved['done'] and fb.feedback.state == 'EXECUTING' and 'pick' in fb.feedback.message:
                moved['done'] = True
                part, (x, y, yaw) = d
                h = self.objects[part]['size'][2] if self.objects[part]['shape'] == 'box' else self.objects[part]['size'][1]
                threading.Timer(1.5, lambda: self.sim.set_pose(part, x, y, h / 2 + 0.002, yaw)).start()
        self.safety_log.clear()
        client = self.baseline_cli if system == 'baseline' else self.run_cli
        t0 = time.monotonic()
        handle = wait_future(client.send_goal_async(RunInstruction.Goal(instruction=trial['instruction']),
                                                    feedback_callback=on_feedback), 30)
        res = wait_future(handle.get_result_async(), 900).result if handle and handle.accepted else None
        wall = time.monotonic() - t0
        time.sleep(1.0)
        met, per = self.goals_met(trial['goals'])
        goal_parts = {p for p, _ in trial['goals']}
        if trial['disturb']:
            before[trial['disturb'][0]] = trial['disturb'][1][:2]
        collateral = [n for n, (x, y) in before.items() if n not in goal_parts and n in self.gz
                      and math.hypot(self.gz[n].x - x, self.gz[n].y - y) > 0.03]
        plan = json.loads(res.plan_json) if res and res.plan_json else {}
        return {
            'system': system, 'success': int(met and not collateral), 'goals_met': sum(per), 'goals': len(per),
            'reported_success': int(bool(res and res.success)), 'collateral_moves': ';'.join(collateral),
            'clarified': int(bool(res and 'CLARIFICATION' in res.summary)),
            'planning_time': round(res.planning_time, 2) if res else '', 'time_to_first_motion':
            round(res.time_to_first_motion, 2) if res else '', 'total_time': round(res.total_time if res else wall, 1),
            'replans': res.replans if res else '', 'plan_steps': len(plan.get('steps', [])),
            'safety_stops': sum(1 for _, s in self.safety_log if s in ('STOP', 'ESTOP')),
            'summary': (res.summary if res else 'no result')[:160],
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--systems', nargs='+', default=['claude', 'rule', 'baseline'])
    ap.add_argument('--conditions', nargs='+', default=CONDITIONS)
    ap.add_argument('--trials', type=int, default=2)
    ap.add_argument('--out', default=os.path.expanduser('~/pai_results'))
    args, _ = ap.parse_known_args()
    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, 'benchmark.csv')

    rclpy.init()
    b = Bench()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(b)
    threading.Thread(target=ex.spin, daemon=True).start()
    for c in (b.run_cli, b.baseline_cli):
        if not c.wait_for_server(timeout_sec=90):
            sys.exit(f'action server {c._action_name} not available')
    time.sleep(3)

    fields = ['system', 'condition', 'task', 'trial', 'success', 'goals_met', 'goals', 'reported_success',
              'collateral_moves', 'clarified', 'planning_time', 'time_to_first_motion', 'total_time', 'replans',
              'plan_steps', 'safety_stops', 'instruction', 'summary']
    new_file = not os.path.exists(path)
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        if new_file:
            w.writeheader()
        for system in args.systems:
            if system in ('claude', 'rule'):
                b.set_planner(system)
            for cond in args.conditions:
                tasks = NOVEL_TASKS if cond == 'novel' else TASKS
                for task in tasks:
                    for k in range(args.trials):
                        trial = make_trial(cond, task, k)
                        row = b.run_trial(system, trial)
                        row.update(condition=cond, task=task, trial=k, instruction=trial['instruction'])
                        w.writerow(row)
                        f.flush()
                        print(f'{system:8s} {cond:11s} {task:14s} #{k}: {"OK  " if row["success"] else "FAIL"} '
                              f'{row["total_time"]:6}s  {row["summary"][:90]}', flush=True)
    b.sim.set_light(*DEFAULT_LIGHT)
    print(f'\nresults appended to {path}')
    rclpy.shutdown()

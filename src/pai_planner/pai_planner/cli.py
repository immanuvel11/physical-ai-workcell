"""Send an instruction from the terminal:  ros2 run pai_planner run_instruction "put the red cube in the blue bin"
Add --dry-run to only see the plan."""
import json
import sys

import rclpy
from rclpy.action import ActionClient

from pai_interfaces.action import RunInstruction


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    dry = '--dry-run' in args
    text = ' '.join(a for a in args if a != '--dry-run').strip()
    if not text:
        sys.exit(__doc__)
    rclpy.init()
    node = rclpy.create_node('run_instruction_cli')
    client = ActionClient(node, RunInstruction, '/pai/run_instruction')
    if not client.wait_for_server(timeout_sec=30.0):
        sys.exit('executor not running (/pai/run_instruction)')

    def on_feedback(fb):
        print(f'  [{fb.feedback.state}] {fb.feedback.message}', flush=True)

    goal_future = client.send_goal_async(RunInstruction.Goal(instruction=text, dry_run=dry), feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, goal_future)
    handle = goal_future.result()
    if not handle.accepted:
        sys.exit('executor busy: goal rejected')
    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    r = result_future.result().result
    plan = json.loads(r.plan_json or '{}')
    print('\nPlan:')
    for i, s in enumerate(plan.get('steps', []), 1):
        print(f'  {i}. {s["skill"]}({s["object_id"] or s["target_id"] or s["named_target"]})  - {s["why"]}')
    print(f'\n{"SUCCESS" if r.success else "NOT DONE"}: {r.summary}')
    print(f'planning {r.planning_time:.1f}s | first motion after {r.time_to_first_motion:.1f}s | '
          f'total {r.total_time:.1f}s | re-plans {r.replans}')
    rclpy.shutdown()
    sys.exit(0 if r.success else 1)

"""Phase 4 + 5 acceptance test of the whole pipeline (needs the full bringup running).

  A/B  instructions -> correct end state (Gazebo ground truth)
  C    disturbance: the part is moved while the robot reaches for it -> re-perceive, re-plan, succeed
  D/E  ambiguous / impossible instruction -> no motion, a question or an explanation
  F    a person walks up mid-task -> SLOW / STOP, arm actually halts, task resumes and completes
  G    e-stop mid-task -> arm halts until reset, then completes
"""
import sys
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from pai_benchmark.common import Harness
from pai_core.ros_utils import wait_future

DEFAULT_LAYOUT = {'red_cube': (0.45, 0.05, 0.3), 'blue_bolt': (0.58, -0.12, 0.0),
                  'yellow_block': (0.36, -0.16, 0.8), 'green_nut': (0.62, 0.10, 0.0)}


def main():
    rclpy.init()
    h = Harness('system_check')
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(h)
    threading.Thread(target=ex.spin, daemon=True).start()
    if not h.run_cli.wait_for_server(timeout_sec=90):
        sys.exit('executor not available')
    time.sleep(3)
    h.place_objects(DEFAULT_LAYOUT)
    h.home()

    # A: simple instruction
    r = h.run('Put the red cube in the blue bin.')
    met, _ = h.goals_met([('red_cube', 'blue_bin')])
    h.record('A: "Put the red cube in the blue bin"', r and r.success and met,
             f'({r.summary if r else "no result"}; {r.total_time:.0f}s)' if r else '')

    # B: two parts, synonyms ("bolt" is a cylinder, "kit tray")
    r = h.run('Place the yellow block and the blue bolt on the kit tray.')
    met, per = h.goals_met([('yellow_block', 'kit_tray'), ('blue_bolt', 'kit_tray')])
    h.record('B: two parts onto the kit tray', r and r.success and met, f'({r.summary if r else "-"}; per-goal {per})')

    # C: disturbance - move the nut while the arm is on its way to it
    moved = {'done': False}

    def disturb(fb):
        if not moved['done'] and fb.feedback.state == 'EXECUTING' and 'pick' in fb.feedback.message:
            moved['done'] = True
            # >= 12 cm from the cup: closer than that, the 22 cm hand often cannot reach down beside a tall
            # obstacle (documented limitation; the benchmark uses the same spacing)
            threading.Timer(1.5, lambda: h.sim.set_pose('green_nut', 0.38, -0.03, 0.011, 0.0)).start()
    r = h.run('Put the green nut in the green bin.', on_feedback=disturb)
    met, _ = h.goals_met([('green_nut', 'green_bin')])
    h.record('C: part moved mid-task -> recovered by re-planning', r and r.success and met and r.replans >= 1,
             f'({r.summary if r else "-"}; replans={r.replans if r else "-"})')

    # D: ambiguous
    r = h.run('Put the cylinder in the blue bin.')
    h.record('D: ambiguous instruction -> asks, does not move',
             r and not r.success and 'CLARIFICATION' in r.summary and r.time_to_first_motion == 0.0,
             f'({r.summary if r else "-"})')

    # E: not possible / not allowed
    r = h.run('Put the coffee cup in the green bin.')
    met = h.where('coffee_cup') == 'table'
    h.record('E: obstacle request -> refused, cup untouched', r and not r.success and met and r.time_to_first_motion == 0.0,
             f'({r.summary if r else "-"})')

    # F: a person walks up during a task
    h.place_objects(DEFAULT_LAYOUT)     # fresh nominal layout for the safety tests
    h.home()
    h.safety_log.clear()
    started = {'t': None}

    def walk(fb):
        if started['t'] is None and fb.feedback.state == 'EXECUTING':
            started['t'] = time.monotonic()
            wait_future(h.walk_cli.call_async(Trigger.Request()), 5)
    t0 = time.monotonic()
    r = h.run('Put the red cube in the green bin.', on_feedback=walk)
    states = [s for _, s in h.safety_log]
    stops = [(t, s) for t, s in h.safety_log if s == 'STOP']
    halted = True
    reaction = None
    for t_stop, _ in stops:
        t_end = next((t for t, s in h.safety_log if t > t_stop and s != 'STOP'), time.monotonic())
        if t_end - t_stop > 0.6:
            vmax = h.max_velocity_between(t_stop + 0.5, t_end)
            halted &= vmax < 0.02
            settle = next((t for t, v in h.vel_log if t > t_stop and v < 0.02), None)
            reaction = settle - t_stop if settle else None
    met, _ = h.goals_met([('red_cube', 'green_bin')])
    h.record('F: person nearby -> SLOW and STOP observed', 'SLOW' in states and 'STOP' in states, f'({states})')
    h.record('F: arm actually halted during STOP', bool(stops) and halted,
             f'(stop reaction {reaction:.2f}s)' if reaction is not None else '')
    h.record('F: task resumed and completed after the person left', r and r.success and met,
             f'({r.summary if r else "-"}; {time.monotonic() - t0:.0f}s)')

    # G: e-stop
    time.sleep(8)    # let the worker walk off
    h.safety_log.clear()
    pressed = {'t': None}

    def press(fb):
        if pressed['t'] is None and fb.feedback.state == 'EXECUTING':
            pressed['t'] = time.monotonic()

            def go():
                time.sleep(2.0)
                h.estop_pub.publish(Bool(data=True))
                time.sleep(4.0)
                pressed['released'] = time.monotonic()
                h.estop_pub.publish(Bool(data=False))
                wait_future(h.reset_cli.call_async(Trigger.Request()), 5)
            threading.Thread(target=go, daemon=True).start()
    r = h.run('Put the yellow block in the blue bin.', on_feedback=press)
    est = [t for t, s in h.safety_log if s == 'ESTOP']
    halted = bool(est) and h.max_velocity_between(est[0] + 0.5, pressed.get('released', est[0] + 3.5)) < 0.02
    met, _ = h.goals_met([('yellow_block', 'blue_bin')])
    h.record('G: e-stop halts the arm', halted)
    h.record('G: after reset the task completes', r and r.success and met, f'({r.summary if r else "-"})')

    passed = sum(ok for _, ok, _ in h.results)
    print(f'\n===== Phases 4+5: {passed}/{len(h.results)} checks passed =====')
    for name, ok, detail in h.results:
        print(f'  {"PASS" if ok else "FAIL"}  {name} {detail}')
    rclpy.shutdown()
    sys.exit(0 if passed == len(h.results) else 1)

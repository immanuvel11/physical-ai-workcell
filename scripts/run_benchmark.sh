#!/bin/bash
# Run the benchmark headless and write the report.
#
#   scripts/run_benchmark.sh                                   # default: rule + baseline, 2 trials
#   scripts/run_benchmark.sh --systems claude --trials 2       # add the Claude planner (needs an API key)
#   scripts/run_benchmark.sh --systems baseline --conditions novel
#
# Results are appended to docs/benchmark.csv; the report (docs/RESULTS.md + charts) is regenerated
# from everything in that file. Delete docs/benchmark.csv to start from scratch.
WS="$(cd "$(dirname "$0")/.." && pwd)"
LOG="$WS/log/benchmark_sim.log"
mkdir -p "$WS/log"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u     # only after the ROS setup files, which use unset variables
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ARGS=("$@")
[ ${#ARGS[@]} -eq 0 ] && ARGS=(--systems rule baseline --trials 2)

echo "starting simulator (headless), log: $LOG"
setsid ros2 launch pai_bringup bringup.launch.py headless:=true rviz:=false > "$LOG" 2>&1 < /dev/null &
LAUNCH_PID=$!
cleanup() {
  kill -INT -"$LAUNCH_PID" 2>/dev/null; sleep 6
  pkill -9 -f "^/opt/ros/humble/lib/|^ruby .*ign gazebo|^ign gazebo|^/usr/bin/python3 .*/install/pai_|^python3 .*/install/pai_" 2>/dev/null
}
trap cleanup EXIT
for _ in $(seq 1 60); do
  grep -q "all simulated grasp joints released" "$LOG" && grep -q "Executor ready" "$LOG" && break
  sleep 2
done
sleep 3

cd "$WS"
ros2 run pai_benchmark benchmark "${ARGS[@]}" --out docs
ros2 run pai_benchmark report --csv docs/benchmark.csv --out docs

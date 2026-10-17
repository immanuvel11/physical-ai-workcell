# 5-minute demo script

Setup (before the audience arrives):

```bash
cd physical_ai_ws && source install/setup.bash
ros2 launch pai_bringup bringup.launch.py rviz:=false
```

Open three windows side by side: the **Gazebo** GUI, the operator console at **http://localhost:8080**, and a
terminal. Close other apps; 7 GB RAM is enough but not by much.

| # | Say | Do | What they see |
|---|---|---|---|
| 1 | "One camera, one arm, nothing taught. It finds whatever is on the table." | Click **Put the red cube in the blue bin.** then **Run** | Console: PERCEIVING → PLANNING → plan with 2 steps → EXECUTING; scene graph fills with measured positions/sizes; the arm picks and places; VERIFYING → SUCCEEDED |
| 2 | "It plans multi-step tasks from plain language." | **Prepare a kit: place the yellow block and the blue bolt on the kit tray.** | 4-step plan; both parts end up on the tray side by side (free-spot search) |
| 3 | "It understands paraphrases it was never programmed for." *(needs the Claude key)* | **That crimson cube goes with the other blue things.** | Claude maps crimson → red cube and blue things → blue bin |
| 4 | "If the world changes while it works, it notices and re-plans." | Start **Put the green nut in the green bin.**; while the arm is moving, drag the nut elsewhere in Gazebo (or run the snippet below) | Grasp-missed → REPLANNING → re-perceives → picks the nut at its new place |
| 5 | "It asks instead of guessing." | **Put the cylinder in the blue bin.** | NEEDS_CLARIFICATION: "There are 2 cylinders … Which one?" No motion. |
| 6 | "It never touches things it shouldn't." | **Put the coffee cup in the green bin.** | Refused, no motion |
| 7 | "Safety: a person walks up." | Start any task, click **Simulate a co-worker walking up** | Banner turns SLOW (orange) then STOP (red); the arm halts; it resumes by itself once the person leaves |
| 8 | "E-stop." | During a task press **E-STOP**, then **Reset** | Arm halts immediately; the task continues after reset |
| 9 | "And it beats a traditionally programmed cell." | Show `docs/RESULTS.md` and the chart | Success by condition: the taught cell only works in the nominal layout |

Disturbance snippet (step 4), from a second terminal:

```bash
ros2 service call /world/workcell/set_pose ros_gz_interfaces/srv/SetEntityPose \
  "{entity: {name: green_nut, type: 2}, pose: {position: {x: 0.42, y: 0.24, z: 0.011}}}"
```

To reset the scene between demo runs, stop the launch (Ctrl-C) and start it again (about 30 s).

# Building the real robot (SO-101 version)

The simulation uses a Franka Panda, an industrial arm that cannot be 3D printed. The hardware
version uses the **SO-101**: an open-source, 3D-printable arm by TheRobotStudio (Apache-2.0), driven
by six Feetech STS3215 smart servos. Its design files are in [`SO-ARM100/`](SO-ARM100/): a trimmed
copy of https://github.com/TheRobotStudio/SO-ARM100 with only the SO-101 follower-arm files this
project uses (see `SO-ARM100/TRIMMED.md`).

> **Status:** the project's software still drives the simulated Panda. Porting it to the SO-101
> (robot model, MoveIt, servo driver, camera calibration) is the next phase; see the end of this file.

## 1. What you build

```
              depth camera (looking down, ~50 cm above the table)
                       │ USB 3
   ┌───────────────────┴──────────────────────────────┐
   │   [blue bin]                        [green bin]  │   printed workcell kit
   │          ▫ cube   ▭ block   ▮ bolt   ◎ nut        │   parts 12–24 cm in front of the arm
   │                    [grey kit tray]                │
   │                      ╔══════╗                     │
   │                      ║SO-101║  ← clamped to table │
   └──────────────────────╨──────╨─────────────────────┘
                             │ USB-C            │ DC power through an E-STOP switch
                           your PC          power supply
```

## 2. Shopping list (one arm)

Prices are from the official bill of materials in `SO-ARM100/README.md` (US links there).
They change; check before you order.

| Part | Qty | ~Price (US) | Notes |
|---|---|---|---|
| **Feetech STS3215 servo, 7.4 V, 1/345 gear (C001)** | 6 | $13.89 each | The motors. All six joints use the same model. |
| Waveshare serial bus servo control board | 1 | $10.60 | Connects the servo chain to the PC over USB. |
| Power supply (as listed in the BOM, 5 V for the 7.4 V servos) | 1 | $10 | |
| USB-C cable | 1 | $7 | |
| Table clamps | 2 | $5 | Hold the arm base to the table. |
| Small Phillips screwdrivers (#0, #1) | 1 set | $6 | |
| **Subtotal (official BOM)** | | **≈ $122** | |
| **Depth (RGB-D) camera** with a ROS 2 driver, e.g. Intel RealSense D435/D435i, D405, or an Orbbec camera | 1 | check current price | Replaces the simulated overhead camera. |
| Camera arm or tripod with a 1/4"-20 screw | 1 | | Holds the camera above the table. |
| **Latching emergency-stop switch** rated for the supply's DC current | 1 | | Wired in the servo power line: cuts motor power instantly, independent of software. |
| PLA+ filament: arm colour, plus red, yellow, blue, green, orange, grey | | | Colours matter: perception names parts by colour. |

**Stronger option:** the STS3215 also comes in a **12 V, 30 kg·cm** version (the 7.4 V one gives
16.5 kg·cm at 6 V). It needs a **12 V, 5 A or more** supply instead. The designers report the 7.4 V
version is sufficient. Either way, buy **all six in the same version**.

You do **not** need the "leader" arm (the second arm with mixed gearing, for steering by hand).
This project drives the arm autonomously. A leader arm is only useful later, to record
demonstrations for training learned skills.

## 3. 3D printing

**The arm** (from `SO-ARM100/STL/SO101/`):
- Easiest: one all-in-one plate file for your printer from `Follower/`
  (`Ender_Follower_SO101.stl`, `Prusa_Follower_SO101.stl` or `BambuLabA1mini_Follower_SO101.stl`).
- Or individual parts from `Individual/` (11 files, one of each; this is what print services need).
  The mounting plate there is the one for the Waveshare board in the shopping list.
- Recommended settings: PLA+, 0.4 mm nozzle, 0.2 mm layers, 15–20 % infill, supports on.
  Expect roughly 16 hours of printing.
- No printer? `SO-ARM100/3DPRINT.md` explains ordering prints from Craftcloud3D or PCBWay.

**The workcell kit** (from [`workcell_print/`](workcell_print/), designed for this project):

| File | Size (mm) | Colour |
|---|---|---|
| `blue_bin_80x80x35mm.stl` | 80 × 80 × 35 | blue |
| `green_bin_80x80x35mm.stl` | 80 × 80 × 35 | green |
| `grey_kit_tray_110x80x8mm.stl` | 110 × 80 × 8 | grey |
| `red_cube_25mm.stl` | 25 cube | red |
| `yellow_block_35x18x18mm.stl` | 35 × 18 × 18 | yellow |
| `blue_bolt_d16_h35mm.stl` | Ø16 × 35 | blue |
| `green_nut_d24_h12mm_hole10.stl` | Ø24 × 12, Ø10 hole | green |
| `orange_box_30x20x20mm.stl` | 30 × 20 × 20 | orange (the "unseen" part) |

No supports needed. To change any size, edit the numbers at the top of `workcell_print/make_stl.py`
and run `python3 make_stl.py`. `preview.png` shows the whole kit.

**The motors are in the design:** `SO-ARM100/STEP/SO101/SO101 Assembly.step` (open in SolidWorks or
Fusion 360) is the full arm, and the simulation model `SO-ARM100/Simulation/SO101/so101_new_calib.urdf`
includes the STS3215 servo bodies, so they appear in the simulator where they sit on the real arm.

## 4. Assembly and wiring

Follow the official step-by-step guide: https://huggingface.co/docs/lerobot/so101. Two things
that are easy to get wrong:
- Each servo needs its **own ID** (1–6). The guide sets them **before** assembly; do not skip it.
- Route the servo cables as shown, so the arm can rotate without pulling them.

```
 power supply ──► E-STOP (latching) ──► Waveshare board ──► servo 1 ─► servo 2 ─► … ─► servo 6 (gripper)
                                              │                (daisy-chained 3-pin servo cables)
                                              └── USB-C ──► PC
 depth camera ── USB 3 ──► PC
```

## 5. Safety with the real arm

- **Keep the hardware e-stop within reach** whenever the arm has power. The software e-stop and
  person detection in this project are not a substitute for cutting power.
- Start with low speeds; clear the table of anything not part of the test.
- The SO-101 is light and low-torque, but it can still pinch fingers at the gripper and joints.

## 6. Software port (next phase)

| Step | What | Needs hardware? |
|---|---|---|
| H1 | SO-101 model (with motors) in the Gazebo workcell, scaled-down scene, MoveIt set-up for 5 joints + gripper, skills adapted | No, simulation only |
| H2 | ros2_control driver for the Feetech servos, so the same launch drives the real arm | Yes: the arm |
| H3 | Real depth camera driver + calibration (where the camera is relative to the arm) | Yes: arm + camera |
| H4 | Full system test and a short benchmark on the real arm | Yes |

Perception, the world model, the planner, the executor and the safety logic carry over unchanged;
only the robot-specific pieces change.

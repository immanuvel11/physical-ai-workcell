# Trimmed copy

This folder is a reduced copy of the SO-ARM100 repository by TheRobotStudio
(https://github.com/TheRobotStudio/SO-ARM100, Apache-2.0, see `LICENSE`), keeping only what this
project needs for one SO-101 follower arm:

- `STL/SO101/Follower/`  print plates for Ender, Prusa and Bambu Lab A1 mini printers
- `STL/SO101/Individual/` the same parts one by one (for print services)
- `STEP/SO101/`           CAD files; `SO101 Assembly.step` is the whole arm with its motors
- `Simulation/SO101/`     the robot model (URDF + meshes) used by the simulator
- `README.md`, `3DPRINT.md`  the original bill of materials and print-service instructions

Removed: the older SO-100 version, the leader (hand-steered) arm, optional accessories, photos and
documentation images, and the git history. Images referenced in `README.md` are therefore missing.
To get everything back: `git clone https://github.com/TheRobotStudio/SO-ARM100.git`

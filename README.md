# utils
ROS pkg implementing utilities for the robot like TTS, Simulation tools, topological mapping or odometry estimation.

## Checkout
```
git clone --recursive https://github.com/MAPIRlab/utils.git
```
## Pull changes
If it's the first time you check-out a repo you need to use --init first:
```
git submodule update --init --recursive
```
For the next times:
```
git submodule update --recursive
```
## Sub-modules not yet in ROS2
The following pkgs included as submodules in the "utils" repository are not yet fully migrated to ROS2. You may need to add a COLCON_IGNORE file.
- Odometry/SRF
- Mapping/Sigma_FP
- HRI/people-detection

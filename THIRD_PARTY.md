# Sources and licenses

This is an extraction of the working obstacle-arena integration. Vendored sources
already contain the ROS 2 adaptations; do not overwrite them with upstream sources.
Pinned upstream revisions are in sources.lock.json. FAST-LIO modifications are
also recorded in patches/fast_lio_ros2.patch.

- FAST-LIO2: https://github.com/hku-mars/FAST_LIO (GPL-2.0).
- Livox driver: https://github.com/Livox-SDK/livox_ros_driver2; ROS 2 manifest selected.
- Livox SDK2: https://github.com/Livox-SDK/Livox-SDK2, built locally into .deps.
- Nav2: https://github.com/ros-navigation/navigation2 (installed ROS Humble packages,
  Apache-2.0); SmacPlanner2D and MPPI Omni.
- MuJoCo: https://github.com/google-deepmind/mujoco, Python wheel 3.3.7.

License notices are retained in LICENSES and vendor directories. The integration
license does not replace upstream licenses. The physics base
(`src/nav_stack/urdf/physics_base.urdf`) and the robocon arena are primitive
geometry models. MID360 scanning and mecanum traction are simplified simulation
models, not calibrated hardware replicas. Simulator ground truth is used only for
evaluation, not SLAM or navigation feedback. The upstream loop-closure backend
(`backend_node`/`pose_graph`/GTSAM) is not included; this fork keeps the
FAST-LIO2 + `fastlio_map` mapping path only.

MID-360 model update:
- The `br` exterior meshes in `src/nav_stack/models/br/meshes_sim/` are derived
  from Livox's official MID-360 STEP model (https://www.livoxtech.com/mid-360/downloads)
  and decimated for MuJoCo's triangle limit. The original STEP source is not bundled.
- Scan directions are converted from the official MIT-licensed
  https://github.com/Livox-SDK/livox_laser_simulation mid360.csv, preserving the
  plugin's Euler angle convention. The generated `scan_patterns/mid360_directions.npy`
  is committed; the CSV library is not (see regeneration below).
- MuJoCo raycasting, simulated IMU, timestamp batching, range/noise approximation
  and reduced ray budget remain integration code; this is not an unmodified
  official simulator or a calibrated sensor digital twin.

Robocon fork and slim repository:
- This repository is a robocon-field fork of https://github.com/WrenInk/slam_simulation
  (BSD-3-Clause integration code), with the robocon arena, `br` visual model,
  robocon launch/scripts and RViz additions. Upstream notices are retained.
- The upstream visible robot/arena and loop-closure backend were removed; the
  invisible four-wheel `physics_base.urdf` remains only to provide MuJoCo physics
  and TF, while `br` is shown in MuJoCo/RViz.
- To keep the repository small, these reference/generated assets are **not**
  committed: `third_party/livox_laser_simulation/` (official scan-mode CSV
  library, ~173 MB), `src/nav_stack/models/br/meshes/` (original br CAD meshes,
  ~43 MB), `src/nav_stack/meshes/` (upstream MID-360 visual meshes, ~14 MB),
  `third_party/mid360_official/` (official STEP source, ~7.7 MB) and
  `maps/prebuilt_senior/` (upstream prebuilt map).
- Everything required at runtime is committed: the generated MID-360 directions
  (`src/nav_stack/scan_patterns/`), decimated br meshes (`models/br/meshes_sim/`)
  and the robocon map (`maps/map.pcd/pgm/yaml`).
- Regeneration instructions are in `scripts/convert_mid360_pattern.py` and
  `tools/prep_br_meshes.py`.

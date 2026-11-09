# Asset provenance

- `geometry.json`: original project-authored four-room indoor inspection layout, metres in Gazebo/map coordinates. No Sandbox mesh/map has been copied.
- `inspection_world.sdf`: deterministically generated local static box collision/visual geometry, flat floor and coloured floor regions. Gazebo Harmonic systems use the plugin conventions verified in the installed TB3/TB4 worlds. No Fuel/network includes.
- `inspection_world.pgm` and `.yaml`: analytical ground-truth occupancy raster of those exact grid-aligned collision footprints. Not a SLAM recording or an executed navigation result. JSON syntax in `.yaml` is a valid YAML subset.
- `nav2_template.json`: configuration copied via YAML parsing from `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml` (Nav2 1.3.13; source SHA-256 `299f9002d6dcfbf661e532c99f32dda1766eab3afa3ab71a52378d80e7ef956f`). Nav2 configuration is distributed under the Navigation2 project's Apache-2.0 license. No system file was edited.
- `nav2_params.yaml`: deterministic JSON/YAML serialization of that template, setting only AMCL initial pose and `set_initial_pose` from the authored spawn; robot/controller/planner parameters preserved.
- Robot/bridge/meshes are still supplied by installed `nav2_minimal_tb3_sim`; they are not copied into this scene. System upgrades require re-audit.
- `manifest.json` pins the six executable/source resource files. This notice and the documentation illustration do not affect navigation geometry or asset identity.

Version 1.0.0: static geometry, map consistency, clearance and connectivity validated offline only. Gazebo sensing, AMCL convergence and Nav2 traversal await manual simulation acceptance.

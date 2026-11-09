# Attribution and release scope

This V1.0 handoff is prepared for a **private** GitHub repository. Making the
repository public requires the owner's separate confirmation. A project-wide
open-source license has not yet been selected; this notice does not grant a
new license over third-party material.

- `assets/scenes/inspection_world/v1/nav2_template.json` and the derived
  `nav2_params.yaml` originate from ROS Navigation2 `nav2_bringup` (Apache-2.0).
  Version, upstream configuration hash and modifications are recorded in the
  asset directory's `NOTICE.md`. Upstream: https://github.com/ros-navigation/navigation2
- The TurtleBot3 Waffle robot description in `assets/robot_description/` is a
  local RViz-oriented copy associated with the installed
  `nav2_minimal_tb3_sim` 1.0.1 resources (package declares Apache 2.0).
  Runtime meshes / robot resources remain external ROS package dependencies.
  Upstream: https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation
  and https://github.com/ROBOTIS-GIT/turtlebot3
- The authored Inspection World geometry, analytical occupancy map, sanitized
  replay and plots are described in `docs/STAGE62_RESULTS.md`. Static map
  validation and historical simulator evidence are separate claims.
- Published numerical replay data are sanitized derivatives of historical
  simulation observations, not newly executed tests or invented trajectories.
  Original reports, raw responses, experiment ownership tokens and machine
  configuration are excluded. Test fixtures explicitly marked synthetic are
  protocol / validation examples and are not experiment evidence.

Before any public release, confirm the overall project license and complete
the third-party notice/license review. No original Nav2/Gazebo runtime code or
formal Inspection World assets is altered by this delivery preparation.

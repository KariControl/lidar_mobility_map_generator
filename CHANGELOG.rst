Changelog for lidar_mobility_map_generator
============================================

0.11.0 (2026-09-05)
-------------------

* Publish the first public release under the product name
  ``LiDAR Mobility Map Generator`` and the ROS package name
  ``lidar_mobility_map_generator``.
* Provide a Vector Map beta workflow for automatic
  closed-course Lanelet2 generation from LiDAR SLAM results (tested with GLIM)
  or rosbag2, target-vehicle-bound map generation, visual review, contiguous
  speed-limit authoring, virtual stop lines, target-route selection,
  regeneration, and Autoware test staging.
* Export the complete accepted recorded trajectory into Lanelet2 without
  shortening it. Initial generation does not create a target route; the
  operator selects the complete trajectory or creates a target route in the
  GUI. Keep map generation, static validation, simulated route completion,
  production readiness, and deployment readiness as separate results.
* Provide a Navigation Map alpha workflow for occupancy-map and route-artifact
  generation, GUI/RViz review, and Nav2 Map Server/Route Server load-only
  checks. Planning, Action execution, control, and robot motion are outside this
  alpha claim.
* Add machine-readable generation and readiness reports, explicit platform
  dimensions and LiDAR-extrinsics inputs, and fail-closed handling when required
  evidence is missing.
* Add English and Japanese READMEs and operator manuals for installation,
  input preparation, map generation, editing, and output review.
* Localize the built-in complete-route names in the English Web GUI while
  preserving their stable stored values.
* Document successful Vector Map generation from Velodyne GLIM and rosbag2
  inputs without publishing images derived from the acquisition site.

Version 0.11.0 is the first public release. Vector Map remains a beta feature
for controlled, closed-course use; publication does not constitute completed
real-vehicle or real-robot safety validation or production acceptance. The
project does not claim that every declared Autoware full-route case passes, or
that generated artifacts are approved for production use.

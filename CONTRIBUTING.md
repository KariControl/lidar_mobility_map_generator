# Contributing

Thank you for helping improve LiDAR Mobility Map Generator.

Vector Map is a beta feature for controlled, closed-course use. Navigation Map
is alpha. Do not describe either feature as production-ready without matching
evidence, and keep user-facing claims consistent across the English and
Japanese documentation.

Use `DEVELOP` as the integration branch. The `main` branch is the official
public-release branch beginning with v0.11.0; merge only a reviewed release
commit that has passed [RELEASING.md](RELEASING.md), and tag that exact commit.

## Development workflow

Use Ubuntu 24.04 and ROS 2 Jazzy for the primary build:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select lidar_mobility_map_generator
colcon test --packages-select lidar_mobility_map_generator
colcon test-result --verbose
```

Keep changes focused and add a regression test for behavior changes. Do not
commit build products, generated maps, point clouds, rosbag files, validation
outputs, private-site paths, credentials, or vehicle data that you do not have
permission to redistribute.

Run the public-release audit before opening a pull request:

```bash
python3 tools/audit_public_release.py --mode source .
```

Before publishing a release, follow the owner approvals and clean-tree release
gate in [RELEASING.md](RELEASING.md).

## Safety-related changes

Describe which checks are offline file validation, simulation, or physical
testing. Never weaken obstacle, unknown-space, clearance, vehicle-geometry, or
trajectory-coverage checks merely to make a fixture pass. Changes involving a
real site or vehicle must state how dimensions, LiDAR extrinsics, input rights,
and test evidence were established.

Use the private process in [SECURITY.md](SECURITY.md) for vulnerabilities. Do
not put sensitive site data or exploit details in a public issue.

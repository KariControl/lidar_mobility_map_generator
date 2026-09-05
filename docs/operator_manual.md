# LiDAR Mobility Map Generator: Map Creation and Editing Operator Manual

**English** | [日本語](operator_manual_ja.md)

This manual explains how to create a map from data you have collected, then review and edit it
in the Web GUI and RViz2.

- **Vector map (Lanelet2, beta)**: a map for evaluating Autoware® in a controlled, closed-course environment
- **Navigation map (alpha)**: a 2D occupancy-grid map and routes to load into Nav2

This manual applies to v0.11.0, the first public release dated September 5, 2026.

The input can be either a point-cloud map and localization trajectory produced by LiDAR SLAM,
or point clouds and localization results recorded in rosbag2. For LiDAR SLAM input, map generation
has been verified with a point-cloud map and trajectory produced by
[GLIM](https://github.com/koide3/glim). This tool does not include SLAM or localization functions.

## 1. Prerequisites

### 1.1 Operating environment

- Ubuntu 24.04
- ROS™ 2 Jazzy
- A desktop environment capable of displaying RViz2
- Docker Engine and an Autoware environment that can be started with Docker when evaluating a
  vector map in Autoware
- The vehicle and sensor models to use with Autoware
- Docker and a Nav2 image when running the navigation-map load test

First, build the package.

```bash
export LMMG_WORKSPACE="$HOME/lmmg_ws"
mkdir -p "$LMMG_WORKSPACE/src"
# Place this repository at the following location:
# $LMMG_WORKSPACE/src/lidar_mobility_map_generator

source /opt/ros/jazzy/setup.bash
cd "$LMMG_WORKSPACE"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install \
  --packages-select lidar_mobility_map_generator
source "$LMMG_WORKSPACE/install/setup.bash"
```

After opening a new terminal, run the following before continuing. Environment variables do not
carry over to a new terminal, so set `LMMG_PROJECT`, `LMMG_CONFIG`, `LMMG_VEHICLE`, the output
directory, and the other variables introduced later in this manual again in that terminal.

```bash
export LMMG_WORKSPACE="$HOME/lmmg_ws"
source /opt/ros/jazzy/setup.bash
source "$LMMG_WORKSPACE/install/setup.bash"
```

### 1.2 Input data

Prepare either of the following input types.

#### LiDAR SLAM results

- Point-cloud map: PLY or PCD
- Localization trajectory: TUM format

For GLIM, you can use its point-cloud map and `traj_lidar.txt`. If you use another LiDAR SLAM
system, convert its point cloud and trajectory to the formats above. In the configuration file,
this input method is represented by `input.type: glim` and `input.glim.*`.

#### rosbag2

- Point clouds as `sensor_msgs/msg/PointCloud2`
- Localization results as `nav_msgs/msg/Odometry`, `geometry_msgs/msg/PoseStamped`,
  `nav_msgs/msg/Path`, or TF
- The required `/tf` and `/tf_static`

This tool cannot estimate a driving trajectory from a bag that contains only point clouds. Confirm
that correctly calculated localization estimates for map creation have already been recorded in
the bag.

Map generation from rosbag2 recordings made with Hesai, Velodyne, and Livox MID-360 LiDARs has
been verified using localization results from
[gicp_gnss_odom_localizer](https://github.com/KariControl/gicp_gnss_odom_localizer).
Vector Map generation from GLIM output created from Velodyne data has also been verified. Images
derived from the Velodyne acquisition site are not published in order to protect its location.

### 1.3 Coordinate frames and LiDAR mounting pose

Specify the reference point represented by the trajectory using the setting for the selected input
type.

- LiDAR SLAM input: `input.glim.trajectory_frame: sensor` or `base`
- rosbag2 input: `input.rosbag2.pose_reference_frame: sensor` or `base`

`sensor` means that the trajectory represents the LiDAR origin; `base` means that it represents
the vehicle or robot reference point. When you select `sensor`, `T_base_sensor`—the LiDAR position
and orientation as seen from the vehicle base frame—is required. For LiDAR SLAM input, set
`extrinsics.source: parameters`, then specify the translation in `extrinsics.translation` and the
rotation in `extrinsics.quaternion_xyzw`. For rosbag2 input, use `extrinsics.source: tf_static` when
the bag contains the required transform. To supply the transform from the configuration file, use
`extrinsics.source: parameters` and the two parameters above.

For a real vehicle or robot, use values that were actually measured. To record the measurements as
verified, set the following three fields in addition to the values themselves.

```yaml
extrinsics.calibration_source: measured
extrinsics.calibration_confidence: high
extrinsics.verified: true
```

Do not set `extrinsics.verified: true` while using estimated values or initial defaults.

### 1.4 Vehicle or robot dimensions

Generate a vector map using the dimensions of the vehicle that will actually drive on it. Copy the
`vehicle_info.param.yaml` used by Autoware into the working project's `config` directory, then
replace every field below with a measured value. Neither the file itself nor any component of the
path to it may be a symbolic link.

```yaml
/**:
  ros__parameters:
    wheel_radius: 0.0
    wheel_width: 0.0
    wheel_base: 0.0
    wheel_tread: 0.0
    front_overhang: 0.0
    rear_overhang: 0.0
    left_overhang: 0.0
    right_overhang: 0.0
    vehicle_height: 0.0
    max_steer_angle: 0.0
```

The unit of `max_steer_angle` is radians. Map generation can fail if the dimensions of the data
collection vehicle differ from those of the vehicle that will use the map.

#### Configuring the left and right Lanelet margins

Configure the parameters that determine the drivable-corridor boundaries in the vector map. The
vehicle dimensions used for the vector map are read from `vehicle_info.param.yaml`. The additional
clearance from the outside of the vehicle body to each Lanelet boundary is specified as
`robot.clearance_margin` in the map-generation configuration file.

```yaml
lidar_mobility_map_generator:
  ros__parameters:
    robot.clearance_margin: 0.30  # Add 30 cm of clearance on each side of the vehicle
```

`robot.clearance_margin` is the value for **one side**. Determine it by accounting for lateral
control error, localization and point-cloud-map alignment error, vehicle-dimension error, body
motion, and the physical separation required from obstacles. Set it to at least the following sum.
Use the maximum value verified for the target vehicle and operating conditions for every term, not
an average value.

```text
robot.clearance_margin
  >= maximum lateral control error
   + maximum localization and map-alignment error
   + vehicle-width measurement error and lateral body motion
   + required separation from obstacles
```

Use the following formula as a guide to the Lanelet width generated on a straight section.

```text
vehicle width + 2 x (robot.clearance_margin + 0.05 m)
```

The final 0.05 m is an internal guard that prevents interpolation between discretely calculated
boundaries from cutting into the vehicle envelope. On curves, the boundaries may be wider than the
formula above because they also cover the swept area of the vehicle's front and rear ends while
turning.

For example, with a vehicle width of 1.695 m and `robot.clearance_margin: 0.30`, the expected
Lanelet width on a straight section is approximately 2.395 m.

```text
1.695 + 2 x (0.30 + 0.05) = 2.395 m
```

Increasing the value also increases the area around the vehicle that is checked during
regeneration. If that area overlaps an obstacle or an unverified area, the tool does not widen the
Lanelet unconditionally; the map instead fails validation. Do not reduce the clearance merely to
make validation pass. Reconsider the input point cloud, the centerline to be driven, or the actual
passage.

After generation, check the following fields in `autoware_candidate_acceptance.json`.

| Field | What to check |
|---|---|
| `metrics.estimated_vehicle_width_m` | Vehicle width used for map generation |
| `metrics.estimated_lateral_margin_m` | Configured clearance on one side |
| `metrics.estimated_boundary_interpolation_guard_m` | Internal boundary-interpolation guard |
| `metrics.minimum_lanelet_width_m` | Minimum width of the generated Lanelets |

For a robot navigation map, specify the robot width, front and rear lengths, required clearance,
and turning model with the `robot.*` settings in the configuration file. For a robot capable of
turning in place, set `robot.allow_in_place_rotation: true`. For a car-like vehicle, set
`robot.allow_in_place_rotation: false` and specify a positive minimum turning radius in
`robot.minimum_turning_radius`.

To record measured vehicle or robot dimensions as verified, also add the following settings to the
configuration file.

```yaml
robot.dimensions_source: measured
robot.dimensions_confidence: high
robot.dimensions_verified: true
```

Using `robot.dimensions_verified: true` also requires `extrinsics.verified: true` from the preceding
section. You cannot mark only the body dimensions as verified while leaving the LiDAR mounting
position and orientation unverified.

## 2. Working directory and configuration files

Create the working directories and configuration file. First, create the directories with the
following commands.

```bash
export LMMG_PROJECT="$HOME/lmmg_project"
export LMMG_PACKAGE_SHARE="$(ros2 pkg prefix --share lidar_mobility_map_generator)"

mkdir -p \
  "$LMMG_PROJECT/input" \
  "$LMMG_PROJECT/config" \
  "$LMMG_PROJECT/output"
```

When using LiDAR SLAM results, copy the template as follows.

```bash
cp "$LMMG_PACKAGE_SHARE/config/glim.yaml" \
  "$LMMG_PROJECT/config/site_glim.yaml"
```

When using rosbag2, copy this template instead.

```bash
cp "$LMMG_PACKAGE_SHARE/config/rosbag2.yaml" \
  "$LMMG_PROJECT/config/site_rosbag2.yaml"
```

In the copied YAML file, configure the input files, topics, coordinate frames, LiDAR mounting pose,
and vehicle or robot dimensions. The `/data/...` paths in the samples are examples; always replace
them with the paths to the actual input data.

Select the map type with `output.target_mode`.

| Value | Map to create |
|---|---|
| `vector_map` | Lanelet2 vector map |
| `navigation_map` | Navigation map for Nav2 |
| `both` | Use only when both maps can share the same body reference point, external dimensions, and turning model |

In this manual, vector maps and navigation maps are generated in separate output directories. The
commands in Chapters 3 and 6 override `output.target_mode` with the value for the respective map.

## 3. Creating a vector map

### 3.1 Initial generation

Generate the vector map by specifying the configuration file, vehicle information, and output
directory. The following example uses the LiDAR SLAM configuration file.

```bash
export LMMG_CONFIG="$LMMG_PROJECT/config/site_glim.yaml"
export LMMG_VEHICLE="$LMMG_PROJECT/config/vehicle_info.param.yaml"
export LMMG_VECTOR_OUTPUT="$LMMG_PROJECT/output/vector_map"

# This is the default. Record that it has not been confirmed whether the data collection
# vehicle and the target vehicle are the same vehicle.
export LMMG_ACQUISITION_VEHICLE_IS_TARGET=false

ros2 run lidar_mobility_map_generator run_vector_map_workflow.sh \
  generate \
  "$LMMG_CONFIG" \
  "$LMMG_VEHICLE" \
  "$LMMG_VECTOR_OUTPUT"
```

`LMMG_ACQUISITION_VEHICLE_IS_TARGET` records whether the data collection vehicle and the target
vehicle that will drive on the map are the same vehicle. Its default value is `false`. The map can
still be generated when it remains `false`, but do not begin a driving test with that output.

Only when vehicle-management records or equivalent evidence confirm that they are the same vehicle,
set the variable as follows before generation. Do not set it to `true` merely because two different
vehicles have the same dimensions.

```bash
export LMMG_ACQUISITION_VEHICLE_IS_TARGET=true
```

When using rosbag2, change `LMMG_CONFIG` to the rosbag2 configuration file. When comparing LiDAR
SLAM and rosbag2 results, specify a separate output directory for each.

When generation finishes, the following message appears in the terminal and the command prompt
returns.

```text
LiDAR Mobility Map Generator: Vector Map output is ready under ...
```

If an error appears, do not use the output. Correct the input data or configuration and run the
same `generate` command again.

### 3.2 Reviewing in the GUI and RViz2

Open the Web GUI and RViz2 with the following command.

```bash
ros2 run lidar_mobility_map_generator run_vector_map_workflow.sh \
  review \
  "$LMMG_CONFIG" \
  "$LMMG_VEHICLE" \
  "$LMMG_VECTOR_OUTPUT"
```

If the browser does not open automatically, open the localhost URL shown in the terminal. While
`review` is running, the editing server runs in that terminal. To run other commands, open a new
terminal and set the environment variables from Sections 1.1, 2, and 3.1 again.

![Vector map editor](images/vector_map_editor_en.png)

Check the following in the GUI and RViz2.

1. Confirm that `Vector Map Beta (Lanelet2)` appears at the top of the page.
2. Under `Display layers`, enable `Recorded trajectory (raw)`. Confirm that this layer covers the
   same range as `Trajectory used to generate the map`.
3. Confirm that the point cloud, trajectory used to generate the map, and road centerline overlap
   in the same position.
4. Confirm that the left and right Lanelet boundaries are not reversed or crossed and are not
   interrupted partway through.
5. In RViz2, also confirm that the point cloud, trajectory, and Lanelet2 map appear at the same
   position in the `map` frame.

Pan the map by dragging with the middle or right mouse button and zoom with the wheel. Use the left
button to select items.

If anything is misaligned or a boundary is abnormal, do not begin editing. Press `Ctrl+C` to stop,
check the input data, coordinate frames, and LiDAR mounting position and orientation, then restart
from Section 3.1. When you only need to review the map, also press `Ctrl+C` to stop the editing
server and RViz2 when finished.

## 4. Editing a vector map

### 4.1 The difference between saving and regenerating

Edits saved in the GUI are not reflected in the Lanelet2 output until the vector map is
regenerated. The following table shows what each operation saves.

| Operation | Content saved | When it is reflected in Lanelet2 |
|---|---|---|
| Tools under `Lanelet road-shape editing` | Centerline Nodes and Edges (saved automatically after each operation) | After regeneration |
| `Save semantic features` | Speed limits, no-entry restrictions, position annotations, and similar settings | After regeneration |
| `Use this target route` | One target route to use | After regeneration |
| `Save stop line` | A stop line on the target route | After regeneration |
| `Use this centerline source` | The recorded trajectory or centerline edited in the GUI | After regeneration |
| Vector-map regeneration button | Lanelet2 output generated from the saved content | After the completion message appears |

After each save operation, confirm that the GUI displays a message indicating that the save has
completed. Do not start regeneration while any item has unsaved changes.

The `Target routes` section in the GUI records the order and extent of connected centerline
segments used for generation and static validation. It does not send a start or goal pose to the
Autoware Planning Simulator. Set the Autoware goal separately as described in Section 5.2.

### 4.2 Creating adjacent speed sections

This procedure creates a 0.90 m/s section followed immediately by a 0.30 m/s section, without a gap.

1. Select `Speed limit (custom span)`.
2. Enter `0.90` in `Speed m/s`.
3. Select the start and end of the first section in the direction of travel. Enter the speed before
   selecting the end.
4. Select the 0.90 m/s section you created and click `Apply to selected feature`.
5. With the same section still selected, click `Continue next speed span from this end`.
6. Enter `0.30` in `Speed m/s`, then select only the end of the second section.
7. Select the 0.30 m/s section you created and click `Apply to selected feature`.
8. Confirm that the message states that no base-speed section exists between the two speed settings.
9. Click `Save semantic features`.

Creating the two speed sections independently can leave a small gap at their boundary. Use
`Continue next speed span from this end` for adjacent speed sections. If the bottom of the page
warns about a gap of 0.5 m or less, or an overlap between sections with different speeds, correct
the affected section and save again.

After completing all edits, regenerate the vector map with the current target route.

### 4.3 Regenerating from the complete recorded trajectory

To use the entire recorded trajectory as both the road centerline and the target route, regenerate
the map as follows.

1. Under `Road centerline source`, select `Recorded trajectory`.
2. Click `Use this centerline source`.
3. Click `Generate and export the full-route vector map`.
4. When the GUI reports that vector-map regeneration has started, wait without operating the GUI.
5. After the editing server exits, regeneration continues in the terminal from which it was
   launched. Do not operate a browser tab that remains open.
6. Confirm that a completion message appears in the terminal and that the command prompt returns.
7. Close the remaining browser tab and reopen the GUI with the `review` command from Section 3.2.
8. Under `Vector map output`, confirm that the map can be regenerated with the target route
   `Complete recorded trajectory`.

This operation does not automatically create a shortened version of the recorded trajectory. It
stops if the trajectory is interrupted, branched, inconsistent with the recorded geometry, or
otherwise unusable for generation. If an error appears, close the browser page from before
regeneration and do not use the output.

### 4.4 Editing the road centerline manually

In the GUI, draw the Lanelet centerline over the point cloud. You do not need to draw the left and
right boundaries individually. During regeneration, the tool generates those boundaries using the
target vehicle's width, distances to its front and rear ends, and `robot.clearance_margin`. This is
not a function for automatically extracting a road centerline from a point cloud.

The road centerline and target route have different roles.

| Item | Role |
|---|---|
| Road centerline | Defines the road geometry from which Lanelets are generated |
| Target route | Specifies the order and extent of connected centerline segments used for generation and static validation |

Even if you select only part of the road centerline as the target route, other Lanelets that pass
static validation remain in the vector map. The target route is also not replayed automatically as
an Autoware Mission.

The editor can represent branches, and the target-route tools can select a path through them.
However, the v0.11.0 swept-footprint validator fails closed if any branch remains in the Lanelet
topology. Before regeneration, delete every unused branch. A release-ready target must be one
connected, one-way, unbranched chain from start to end. Acceptance of networks containing branches,
intersections, loops, or bidirectional traffic is out of scope.

#### Creating the road centerline

1. To modify the existing centerline, begin editing it directly. To return to its automatically
   generated state, click `Reset to generated Route`. To redraw it from an empty state, click
   `Start from blank (delete all Routes)`.
2. Click `Add Node`, then select the locations that will be the endpoints of the segments in order.
3. Click `Add Edge`, then select its start and end Nodes in order. To create a curve, select the
   start Node, select intermediate points while holding `Shift`, and finally select the end Node.
4. As needed, use `Move Node`, `Delete Node`, `Split Edge`, or `Delete Edge`. If the direction of
   travel is reversed, correct it with `Change Edge direction`.
5. Under `Road centerline source`, select `Road centerline edited in this GUI`, then click
   `Use this centerline source`.
6. If you edit the road centerline again, click `Use this centerline source` again. Also recheck the
   saved target route and stop lines against the current road centerline.

The height of each clicked position is set automatically from a nearby existing route, input
trajectory, or point cloud. A Node cannot be added where the height cannot be determined. Changes
to the road centerline are saved automatically after each operation, but regeneration is required
to reflect them in Lanelet2.

In the GUI, yellow represents the trajectory used to generate the map, green represents the road
centerline edited in the GUI, and orange represents a stop line. These visual elements are examples
of editing state; they do not demonstrate safety in a real environment or a successful Autoware
driving result.

During regeneration, the tool checks the vehicle envelope and minimum turning radius for every
manually edited segment, and validates overlap with obstacles and unverified areas. Do not treat a
location that cannot be verified in the input point cloud as a drivable road. Do not change obstacle
detection, vehicle dimensions, or the LiDAR mounting position and orientation to values that differ
from reality merely to pass static validation. If validation fails, review the road centerline or
input data.

#### Setting the target route

To use the entire road centerline, click `Generate from the complete edited road centerline`. This
selects the built-in target route `Complete edited road centerline`. After regeneration starts,
wait for the terminal's completion message as in Section 4.3, then reopen the GUI with the `review`
command.

To use only part of the road centerline as the target route, follow the procedure below. Only one
route can be registered as the target route in use. If another route is already in use, select it
in the list and click `Delete selected target route` before creating the new one.

1. Click `New target route`.
2. Enter a `Target-route name`.
3. Click `1. Select start`, then select the start from the displayed route points. A position in the
   middle of a line cannot be selected as the start.
4. If the route is not unique because of a branch, or if you need to specify the order of travel,
   use `2. Select via segment (optional)` to select the via segments in order. This can help while
   editing or disambiguating a route, but the v0.11.0 static validator cannot accept the output
   until every unused branch has been deleted from the road centerline.
5. Click `3. Select end`, then select the end from the displayed route points. A position in the
   middle of a line cannot be selected as the end.
6. Click `Check connectivity` and confirm that the route is connected from start to end.
7. Click `Use this target route`.
8. Click `Regenerate the vector map with the current target route`.
9. Wait for the completion message in the terminal, then reopen the GUI with the `review` command.
10. Under `Vector map output`, confirm that the selected target-route name appears.

A target route does not change the road geometry. Change the road geometry under
`Lanelet road-shape editing` in the GUI. If the route is disconnected or any segment fails static
validation, do not use the output; review the road centerline or input data.

### 4.5 Adding a stop line

Specify a stopping position on the target route under `Stop lines` in the GUI.

1. Select the target route to use from the list, then click `Use this target route`.
2. Click `New stop line`.
3. Enter a name and a positive value in `Width [m]`.
4. Click `Click on Edge`, then select the position on the target route.
5. Check the displayed route-segment number, distance from the start, and distance to the end. When
   running a driving test in Planning Simulator, select a position at least 10 m from both the start
   and the end.
6. Click `Save stop line`.
7. Click `Regenerate the vector map with the current target route`.
8. Wait for the completion message in the terminal, then reopen the GUI with the `review` command.

`Automatically align with the stop line (advanced)` under `How to set the speed-change position`
aligns a speed-section boundary with the stop line and may split a Lanelet in the middle of a
segment. It does not calculate the deceleration distance needed to stop.
Configure the deceleration section separately based on the stopping distance verified for the
target vehicle and the characteristics of Autoware's control, then verify it in a driving test.

### 4.6 If a regeneration button is disabled

Read the reason displayed near the button, then check the following items from top to bottom.

- `Regenerate the vector map with the current target route` becomes enabled after you select one
  target route and click `Use this target route`.
- If you edited the road centerline, select the centerline to use and click
  `Use this centerline source` again.
- When generating from the complete centerline, confirm that it has no interrupted segments,
  branches, or loops.
- Complete or cancel any section, area, or road-centerline operation in progress. If speed limits
  and other settings, the target route, or a stop line have unsaved changes, click the corresponding
  save button.
- To use a regeneration button, open the GUI with `run_vector_map_workflow.sh review` from
  Section 3.2.
- If the input data, configuration file, or vehicle information changed after generation, restart
  with `generate` from Section 3.1.
- If the GUI says to regenerate the map or reports that source data is stale, close the GUI,
  regenerate, and reopen it.

If the problem remains, find the first error that explains the failure in the terminal that
launched the GUI.
Do not change the vehicle dimensions or road geometry without evidence merely to remove an error.

## 5. Checking the vector-map output

### 5.1 Output files and static validation

The principal outputs are as follows.

| File/directory | Description |
|---|---|
| `lanelet2_map_closed_course_experimental.osm` | Generated Lanelet2 vector map |
| `autoware_closed_course_experimental_map/` | Complete map directory loaded by Autoware |
| `target_vehicle_info.param.yaml` | Copy of the vehicle information used for map generation |
| `autoware_candidate_acceptance.json` | Static-validation result for the selected centerline and Lanelet2 map |
| `route_body_passage_planning_report.json` | Vehicle-envelope passage-validation result for the recorded-trajectory candidate |
| `target_vehicle_map_binding.json` | Result of matching the vehicle information to the vehicle attributes in Lanelet2 |
| `acquisition_vehicle_target_contract.json` | Generation-time record of the input, configuration, vehicle information, and vehicle identity |
| `acquisition_vehicle_target_contract.sha256` | SHA-256 digest of the generation-time record |

![Lanelet2 review in RViz2](images/autoware_lanelet2_rviz2.png)

The image above shows this tool's RViz2 review display. It is not an Autoware driving screen.

For example, check `autoware_candidate_acceptance.json` as follows.

```bash
python3 - "$LMMG_VECTOR_OUTPUT/autoware_candidate_acceptance.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)

if (
    not isinstance(report, dict)
    or report.get("format_version") != 1
    or report.get("acceptance_scope") != "static_format_geometry_coverage_only"
):
    raise SystemExit("The static-validation report has an unexpected format or scope")

source = report.get("centerline_source")
if source not in ("recorded_trajectory", "edited_topology"):
    raise SystemExit("Cannot identify the road centerline used for the vector map")

if not (
    report.get("accepted") is True
    and report.get("errors") == []
    and report.get("counts", {}).get("synthetic_planning_support_lanelets") == 0
    and report.get("planning_support", {}).get("present") is False
):
    raise SystemExit("Static validation did not pass")

warnings = report.get("warnings")
if not isinstance(warnings, list):
    raise SystemExit("Cannot read the warning list")

print("Static validation: passed")
print("Road-centerline source:", source)
print("Validation coverage:", report.get("coverage_reference"))
print("Warning count:", len(warnings))
for warning in warnings:
    print("Warning:", warning)
PY
```

A `centerline_source` of `recorded_trajectory` means that the recorded trajectory was used;
`edited_topology` means that the road centerline edited in the GUI was used. If warnings are
present, inspect their content and affected segments, then either resolve them or record them as
items to verify in the subsequent evaluation. Do not begin a driving test with output that contains
a warning whose cause has not been identified.

The tool does not automatically add a drivable Lanelet centerline before the start or beyond the
end. The left and right boundaries may extend beyond the centerline's endpoints to keep the front
and rear of the vehicle within the map, but this does not extend the centerline or target route.

`accepted: true` indicates only that static validation passed. It does not indicate that Autoware
can plan a route, that the Planning Simulator can drive the route, or that the map is safe to use
with a real vehicle or robot.

### 5.2 Checking simulated driving with Docker-based Autoware

This section starts Planning Simulator with an official Autoware Docker image. This tool does not
include Autoware itself, a Docker image, a vehicle model, or a sensor model. First, follow the
[Autoware Docker installation instructions](https://docs.autoware.org/main/installation/autoware/docker-installation/)
to obtain the Autoware image you will use. Record the image digest, not only its tag, so that the
test can be reproduced.

This section runs a simulated vehicle in Planning Simulator. These are not instructions for
starting a real-vehicle interface. Record the Autoware version, Docker image digest, vehicle model,
and sensor model in the work log.

#### Verifying the generation-time contract

Before the driving test, verify that the generation-time input, configuration, and vehicle
information have not changed and that the data collection vehicle and target vehicle are the same
vehicle. The following command also verifies hashes of the input files, so it may take some time to
complete. Run it in a new terminal.

```bash
set -o pipefail

export LMMG_WORKSPACE="$HOME/lmmg_ws"
source /opt/ros/jazzy/setup.bash
source "$LMMG_WORKSPACE/install/setup.bash"

export LMMG_PROJECT="$HOME/lmmg_project"
export LMMG_CONFIG="$LMMG_PROJECT/config/site_glim.yaml"
export LMMG_VECTOR_OUTPUT="$LMMG_PROJECT/output/vector_map"
export LMMG_REPOSITORY="$LMMG_WORKSPACE/src/lidar_mobility_map_generator"
export LMMG_DATASET="${LMMG_VECTOR_OUTPUT##*/}"

python3 "$LMMG_REPOSITORY/scripts/generation_calibration_contract.py" \
  verify-direct \
  --expected-dataset "$LMMG_DATASET" \
  --expected-map-type vector_map \
  --contract "$LMMG_VECTOR_OUTPUT/acquisition_vehicle_target_contract.json" \
  --sha256 "$LMMG_VECTOR_OUTPUT/acquisition_vehicle_target_contract.sha256" \
  --generator-parameters "$LMMG_CONFIG" \
  --target-vehicle-info "$LMMG_VECTOR_OUTPUT/target_vehicle_info.param.yaml" \
  --verify-inputs \
  --format tsv |
  awk -F '\t' '
    $1 == "ACQUISITION_VEHICLE_IS_TARGET" && $2 == "true" { confirmed = 1 }
    END {
      if (!confirmed) {
        print "Cannot confirm that the data collection vehicle and target vehicle are identical" > "/dev/stderr"
        exit 1
      }
      print "Generation-time contract: verified"
    }
  '
```

If the command fails, or if the map was generated with
`LMMG_ACQUISITION_VEHICLE_IS_TARGET=false`, do not begin the driving test. Simply changing the
setting to `true` does not alter the contract of existing output. Only if the conditions in
Section 3.1 are satisfied, set it to `true` and regenerate the map.

#### Checking the map and vehicle model

Run the following in the same terminal. Set `AUTOWARE_IMAGE` to a locally available Autoware
Universe image in `@sha256:...` form. Change `AUTOWARE_VEHICLE_MODEL` and
`AUTOWARE_SENSOR_MODEL` to suit your Autoware environment.

```bash
export AUTOWARE_DATA="$HOME/autoware_data"
export AUTOWARE_IMAGE="ghcr.io/autowarefoundation/autoware:universe-jazzy@sha256:<64 hexadecimal digits>"

# These model names are included in the standard Autoware image.
export AUTOWARE_VEHICLE_MODEL="sample_vehicle"
export AUTOWARE_SENSOR_MODEL="sample_sensor_kit"

export LMMG_AUTOWARE_MAP="$LMMG_VECTOR_OUTPUT/autoware_closed_course_experimental_map"
export LMMG_AUTOWARE_MAP="$(realpath "$LMMG_AUTOWARE_MAP")"

docker image inspect "$AUTOWARE_IMAGE" >/dev/null || {
  echo "The Autoware Docker image is not available locally" >&2
  exit 1
}
test -d "$AUTOWARE_DATA/ml_models" || {
  echo "Autoware ML models were not found: $AUTOWARE_DATA/ml_models" >&2
  exit 1
}
for FILE in lanelet2_map.osm pointcloud_map.pcd map_projector_info.yaml
do
  test -f "$LMMG_AUTOWARE_MAP/$FILE" || {
    echo "Map file not found: $LMMG_AUTOWARE_MAP/$FILE" >&2
    exit 1
  }
done
```

`AUTOWARE_VEHICLE_MODEL` is the name of a vehicle model installed in Autoware as a ROS 2 package;
it is not a path to `vehicle_info.param.yaml`. The `sample_vehicle` and `sample_sensor_kit` values
above are an example using Autoware's standard models. To use different models, prepare an
Autoware image containing both `${AUTOWARE_VEHICLE_MODEL}_description` and
`${AUTOWARE_SENSOR_MODEL}_description`, then pin that image by digest.

Use the following command to check both packages and extract the `vehicle_info.param.yaml` used by
Autoware from the container. It then compares the sole `ros__parameters` mapping in its entirety
with the `target_vehicle_info.param.yaml` saved during map generation.

```bash
(
  set -euo pipefail
  LMMG_AUTOWARE_PREFLIGHT="$(mktemp -d)"
  trap 'rm -rf -- "$LMMG_AUTOWARE_PREFLIGHT"' EXIT

  docker run --rm --pull never \
    --network none \
    -e HOST_UID="$(id -u)" \
    -e HOST_GID="$(id -g)" \
    -e AUTOWARE_VEHICLE_MODEL="$AUTOWARE_VEHICLE_MODEL" \
    -e AUTOWARE_SENSOR_MODEL="$AUTOWARE_SENSOR_MODEL" \
    -v "$LMMG_AUTOWARE_PREFLIGHT:/lmmg_preflight:rw" \
    "$AUTOWARE_IMAGE" \
    bash -lc '
      set -euo pipefail
      source /opt/autoware/setup.bash
      vehicle_package="${AUTOWARE_VEHICLE_MODEL}_description"
      sensor_package="${AUTOWARE_SENSOR_MODEL}_description"
      vehicle_share="$(ros2 pkg prefix --share "$vehicle_package")"
      sensor_share="$(ros2 pkg prefix --share "$sensor_package")"
      test -d "$vehicle_share"
      test -d "$sensor_share"
      vehicle_info="$vehicle_share/config/vehicle_info.param.yaml"
      test -f "$vehicle_info"
      cp -- "$vehicle_info" /lmmg_preflight/autoware_vehicle_info.param.yaml
      printf "Vehicle description: %s\n" "$vehicle_share"
      printf "Sensor description: %s\n" "$sensor_share"
      printf "Autoware vehicle information: %s\n" "$vehicle_info"
    '

  python3 - \
    "$LMMG_VECTOR_OUTPUT/target_vehicle_info.param.yaml" \
    "$LMMG_AUTOWARE_PREFLIGHT/autoware_vehicle_info.param.yaml" <<'PY'
import json
import sys
import yaml


def ros_parameters(path, label):
    with open(path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    blocks = []

    def collect(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key == "ros__parameters":
                    if not isinstance(child, dict):
                        raise SystemExit(f"{label}: ros__parameters is not a mapping")
                    blocks.append(child)
                else:
                    collect(child)
        elif isinstance(value, list):
            for child in value:
                collect(child)

    collect(document)
    if len(blocks) != 1:
        raise SystemExit(
            f"{label}: ros__parameters is not unique ({len(blocks)} mappings)"
        )
    return blocks[0]


def canonical(value):
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )


generated = ros_parameters(sys.argv[1], "Vehicle information used for map generation")
autoware = ros_parameters(sys.argv[2], "Vehicle information in the Autoware image")
if canonical(generated) != canonical(autoware):
    raise SystemExit(
        "The complete ros__parameters mappings for map generation and Autoware do not match"
    )

print("Complete ros__parameters mappings for map generation and Autoware: match")
PY
)
```

If package resolution or comparison fails, do not start Planning Simulator. If the standard
`sample_vehicle` does not match the vehicle information used for map generation, do not continue
the driving test with that model name. Switch to an image containing the target vehicle's
description package.

#### Starting Planning Simulator

Run the following in the same terminal. This example is for an Autoware Universe image that does
not use an NVIDIA GPU. The map and ML models are mounted into the container read-only.

```bash
xhost +local:docker
docker run --rm -it --pull never \
  --network host \
  -e DISPLAY="$DISPLAY" \
  -e HOST_UID="$(id -u)" \
  -e HOST_GID="$(id -g)" \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$LMMG_AUTOWARE_MAP:/autoware_map:ro" \
  -v "$AUTOWARE_DATA/ml_models:/autoware_data/ml_models:ro" \
  "$AUTOWARE_IMAGE" \
  bash -lc "source /opt/autoware/setup.bash && \
    ros2 launch autoware_launch planning_simulator.launch.xml \
      map_path:=/autoware_map \
      data_path:=/autoware_data/ml_models \
      vehicle_model:=$AUTOWARE_VEHICLE_MODEL \
      sensor_model:=$AUTOWARE_SENSOR_MODEL"
```

When using an NVIDIA GPU, select a `universe-cuda` image compatible with the Autoware version and
add the GPU-related options by following the official Docker installation instructions. Do not mix
the image's ROS 2 distribution, Autoware version, or CUDA/non-CUDA configuration.

If startup fails, identify the failed component and the earliest error that explains the failure.
Later errors may be consequences of an earlier loading failure. Before continuing, confirm in RViz2
that the point cloud and Lanelets are displayed and overlap in the same position.

#### Setting the start and goal poses in RViz2

1. Click `2D Pose Estimate` in the RViz2 toolbar. If the active RViz2 configuration supports it,
   you can also select the tool with `P`.
2. On a Lanelet centerline near the start of the target route, press the left mouse button, drag in
   the direction of travel, and release. Do not select a left or right boundary or a position
   outside the Lanelet.
3. Wait until the vehicle pose has been set and the vehicle appears at the correct position and
   orientation relative to the point cloud and Lanelets.
4. Click `2D Goal Pose`, or the equivalent goal-setting tool in the active RViz2 configuration.
5. On a Lanelet centerline near the end of the target route, drag in the direction of travel and
   release.
6. Confirm that a planned route appears on the Lanelets and that no route-planning error is reported.
7. Confirm that the Autoware operation panel permits a transition to autonomous driving, then click
   `Autonomous`, or the autonomous-driving start button in the version you are using. If that
   version requires a separate `Engage` operation, perform it as well.
8. Confirm that the simulated vehicle begins moving, follows the planned route, and stops at the
   goal.

The target route saved in this tool's GUI is not sent to Planning Simulator automatically. Set the
start and goal poses again in Autoware's RViz2.

To finish, press `Ctrl+C` in the terminal that launched Autoware and wait for the container to exit.
Then run `xhost -local:docker` to revoke the X-server permission. For detailed operation, consult
the [Autoware Planning Simulation instructions](https://docs.autoware.org/main/demos/planning-sim/)
for the version you are using.

#### Recording the results

Do not change control parameters ad hoc during map evaluation. Record the following items separately.

- Input data, LiDAR Mobility Map Generator version, and map-file hashes
- Autoware version and Docker image digest
- Vehicle model, sensor model, and fixed Planning/Control settings
- Lanelet2 and point-cloud-map loading results
- Route-planning result from the start to the goal
- Results of transitioning to autonomous mode, starting to drive, arriving, and stopping
- When speed limits or stop lines are configured, their static output and execution results

Evaluate the Planning Simulator result separately from the static-validation result in
`autoware_candidate_acceptance.json`. Do not ignore a static-validation error merely because the
vehicle could drive in the simulator, and do not change the road geometry without evidence to make
the control result fit.

## 6. Creating a navigation map

### 6.1 Generating the map

In the configuration file, set the input, coordinate frames, LiDAR mounting position and
orientation, and `robot.*` parameters for the target robot.

A map used for the Nav2 load test requires a traversable area supported by ground observations in
the point cloud. `traversability.free_space_evidence_mode` has the following meanings.

| Value | Evidence for the traversable area | Nav2 load test |
|---|---|---|
| `trajectory` | Trajectory area at each recorded pose | Out of scope |
| `ground_observations` | Cells observed directly as ground in the point cloud | Candidate |
| `combined` | Both trajectory areas and ground observations | Candidate |

Use `ground_observations` or `combined` only when the input point cloud contains ground
observations and you can review the traversable area in the GUI and RViz2 after generation. Do not
treat an area as traversable merely because no point was returned from it.

The following command overrides `output.target_mode` and `output.directory` in the configuration
file with values for the navigation map.

```bash
export LMMG_NAV_CONFIG="$LMMG_PROJECT/config/site_glim.yaml"
export LMMG_NAV_OUTPUT="$LMMG_PROJECT/output/navigation_map"

ros2 run lidar_mobility_map_generator lidar_mobility_map_generator \
  --ros-args \
  --params-file "$LMMG_NAV_CONFIG" \
  -p output.target_mode:=navigation_map \
  -p "output.directory:=$LMMG_NAV_OUTPUT"
```

When using rosbag2, set `LMMG_NAV_CONFIG` to the rosbag2 configuration file.

After generation, check the status of the load-test artifacts.

```bash
python3 - "$LMMG_NAV_OUTPUT/nav2_closed_course_experimental_readiness.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    report = yaml.safe_load(stream)

if (
    not isinstance(report, dict)
    or report.get("schema_version") != 2
    or report.get("target") != "nav2_closed_course_experimental"
):
    raise SystemExit("The Nav2 readiness file has an unexpected format or target")

artifact = report.get("artifact")
keys = ("ready", "static_map_ready", "follow_waypoints_ready", "route_server_ready")
if not isinstance(artifact, dict) or any(artifact.get(key) is not True for key in keys):
    raise SystemExit("The Nav2 load-test artifacts are not ready")

print("Load-test artifacts: ready")
print("production_ready:", report.get("production_ready"))
print("deployment.ready:", report.get("deployment", {}).get("ready"))
PY
```

The four fields under `artifact` indicate whether the artifacts satisfy the prerequisites for the
load test. `production_ready: false` and `deployment.ready: false` are separate determinations;
even when every field under `artifact` is `true`, the file does not indicate that the map can be
used to drive a real robot.

If the artifact check fails, inspect `map_blockers`, `follow_waypoints_blockers`, and
`route_server_blockers` in the same file. Correct the input or configuration and regenerate. Do not
mark an unobserved area as traversable merely to pass the check.

### 6.2 Reviewing and editing in the GUI and RViz2

Open the Web GUI and RViz2 with the following command.

```bash
ros2 launch lidar_mobility_map_generator edit_navigation_map.launch.py \
  "output_directory:=$LMMG_NAV_OUTPUT" \
  frame_id:=map
```

If the browser does not open automatically, open the localhost URL shown in the terminal.

![Navigation map editor](images/navigation_map_editor_en.png)

To configure a target route, perform the following steps.

1. Click `New target route` and enter a `Target-route name`.
2. Click `1. Select start`, then select the start on the road centerline.
3. If the route is not unique or you need to specify the order of travel, use
   `2. Select via segment (optional)` to select the via segments in order.
4. Click `3. Select end`, then select the end.
5. Click `Check connectivity` and confirm that the route is connected from start to end.
6. Click `Use this target route`.
7. To add a stopping position to the target route, select the position under `Stop lines`, then
   click `Save stop line`.
8. If you added a speed limit, no-entry restriction, position annotation, or similar setting, click
   `Save semantic features`.

For a navigation map, a stop line saved on the target route is output as a waypoint stopping
position. This is separate from the point annotation created with the `Stop position` editing tool.
You can save `Stop position`, `Wait position`, `Dock`, `Charger`, `Door`, speed-limit, no-entry, and
similar annotations in the GUI, but their effect on Nav2 behavior has not been verified in this
release.

After editing, press `Ctrl+C` in the terminal that launched the GUI. Run the generation command in
Section 6.1 again to apply the edits to the output. After regeneration, check both the readiness
file and the GUI again.

In RViz2, check the following display colors.

- White: area determined to be traversable
- Black: obstacles
- Gray: unverified area or area outside the map
- Yellow line: trajectory used to generate the map

![Navigation map for Nav2 in RViz2](images/navigation_map_rviz2.png)

Do not use output whose target route crosses an obstacle or an unverified area. Check the input
point cloud, coordinate frames, LiDAR mounting position and orientation, robot dimensions, and
`traversability.*` settings, then recreate the map.

### 6.3 Running the Nav2 load test

The load test in this release checks only the following scope.

- Map Server can load and publish the occupancy-grid map.
- Route Server can load the GeoJSON route and start.
- The `FollowWaypoints` client can validate the waypoint YAML and generate a summary with
  `--dry-run`.

The test does not send `ComputeRoute` or `FollowWaypoints` actions and does not perform
localization, path planning, control, simulator motion, or real-robot motion.

The example Docker image lock file intentionally contains an invalid placeholder. Before the load
test, build or obtain, using your organization's prescribed process, a Nav2 Jazzy image that
contains at least the following.

- `/opt/ros/jazzy/setup.bash`
- `nav2_map_server`, `nav2_route`, `rclpy`, `lifecycle_msgs`, and `nav_msgs`
- `python3`, the Python package `PyYAML`, `bash`, and `findmnt`

Verify the image contents and target platform, then set an immutable image digest in
`@sha256:...` form—not a tag—in the lock file. Specify the SHA-256 as 64 lowercase hexadecimal
digits, and do not use an all-zero value. Confirm that the specified reference exists locally with
the following command.

```bash
export LMMG_NAV2_ACCEPTANCE_IMAGE="registry.example/lmmg-nav2-jazzy@sha256:<64 hexadecimal digits>"
docker image inspect "$LMMG_NAV2_ACCEPTANCE_IMAGE"
```

The load test runs with `--pull never`, so it does not retrieve an image while the test is running.

Before the test, reconfirm that all four fields under `artifact` from Section 6.1 are `true`.

```bash
export LMMG_REPOSITORY="$LMMG_WORKSPACE/src/lidar_mobility_map_generator"
export LMMG_NAV_REPORT="$LMMG_PROJECT/output/nav2_load_report_01"
export LMMG_NAV2_IMAGE_LOCK="$LMMG_PROJECT/config/nav2-load-only-image.lock.env"

(
  set -euo pipefail
  test ! -e "$LMMG_NAV2_IMAGE_LOCK" || {
    echo "Specify a path for a new lock file: $LMMG_NAV2_IMAGE_LOCK" >&2
    exit 1
  }
  printf 'LMMG_NAV2_ACCEPTANCE_IMAGE=%s\n' \
    "$LMMG_NAV2_ACCEPTANCE_IMAGE" > "$LMMG_NAV2_IMAGE_LOCK"

  python3 "$LMMG_REPOSITORY/docker/acceptance/scripts/validate_lock.py" \
    "$LMMG_NAV2_IMAGE_LOCK" \
    --mode nav2 \
    --get LMMG_NAV2_ACCEPTANCE_IMAGE

  "$LMMG_REPOSITORY/scripts/run_nav2_load_only_acceptance.sh" \
    "$LMMG_NAV_OUTPUT" \
    "$LMMG_NAV_REPORT" \
    "$LMMG_NAV2_IMAGE_LOCK"
)
```

Set both `LMMG_NAV2_IMAGE_LOCK` and `LMMG_NAV_REPORT` to paths that do not yet exist. To rerun the
test, change both the lock-file path and the report-directory path to new names.

After the test finishes, check the report.

```bash
python3 - "$LMMG_NAV_REPORT/acceptance.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    report = json.load(stream)

if (
    not isinstance(report, dict)
    or report.get("schema_version") != 1
    or report.get("kind") != "lmmg_nav2_alpha_load_only_acceptance"
):
    raise SystemExit("The Nav2 load-test report has an unexpected format or kind")

scope = report.get("scope", {})
if report.get("accepted") is not True or report.get("errors") != []:
    raise SystemExit("The Nav2 load test did not pass")
if not all(scope.get(key) is True for key in (
    "map_server_load", "route_server_load", "waypoint_yaml_dry_run"
)):
    raise SystemExit("Cannot confirm the scope executed by the load test")
if not all(scope.get(key) is False for key in (
    "planning", "action_execution", "robot_motion"
)):
    raise SystemExit("The report records an unexpected test scope")

print("Nav2 load test: passed")
print("Path planning, action execution, and robot motion: not performed")
PY
```

It is normal for `planning`, `action_execution`, and `robot_motion` to be `false` in this load test.
Store the image lock file together with the test report.

## 7. Troubleshooting

### 7.1 The point cloud and trajectory are misaligned

For LiDAR SLAM input, check `input.glim.trajectory_frame`; for rosbag2 input, check
`input.rosbag2.pose_reference_frame`. Do not apply `T_base_sensor` twice to a trajectory already
expressed in the vehicle base frame, and do not treat a trajectory at the LiDAR origin as one in
the vehicle base frame. For rosbag2 input, also check whether `extrinsics.source` is set to
`tf_static` or `parameters`.

After correcting the settings, regenerate the map and confirm in the GUI and RViz2 that the
transformed point cloud and trajectory overlap.

### 7.2 The default speed remains between speed sections

Do not create the second section independently. Select the first speed section and use
`Continue next speed span from this end`. After saving, confirm the message indicating that there
is no gap, then regenerate.

### 7.3 Map generation stops before completion

In the terminal, identify the error that caused processing to stop. In particular, check the
following.

- Input-file paths, formats, and read permissions
- Topic names and coordinate frames in the configuration file
- Vehicle or robot dimensions and the LiDAR mounting position and orientation
- Road-centerline connectivity, minimum turning radius, and overlap with obstacles or unverified areas
- Confirm that none of the following are symbolic links: the configuration file,
  `vehicle_info.param.yaml`, or any component of their paths

Do not change the road geometry, speed, Z coordinate, or vehicle dimensions without evidence merely
to remove an error.

### 7.4 An input-change error appears during regeneration

If the input data, configuration file, or `vehicle_info.param.yaml` changes after `generate`, its
comparison against the generation-time contract fails. Close the browser and restart with
`generate` from Section 3.1. Do not operate the old browser page.

### 7.5 The Nav2 load test does not start

Check the following.

- The image reference in the lock file includes `@sha256:`.
- The specified image exists locally.
- `LMMG_NAV2_IMAGE_LOCK` names a file that does not yet exist.
- `LMMG_NAV_REPORT` names a directory that does not yet exist.
- All four fields under `artifact` in the readiness file are `true`.

When retrying after a failure, specify both a new lock-file path and a new report-directory path.

### 7.6 Autoware exceeds a speed limit

Check the speed limit stored in Lanelet2 separately from the speed Autoware actually followed.
Record speeding as a failed driving test and do not use the output until you have identified the
cause.

Revising the speed-section configuration based on stopping distance and control response is a
separate task from validating Autoware's control settings. Do not change the road geometry or Z
coordinate without evidence to make the control result fit.

## 8. Checks before use

### General

- Confirm that the input data covers the target area of the map being created.
- Confirm that the point cloud and trajectory overlap after applying the configured coordinate
  transforms.
- Accurately record whether the vehicle or robot dimensions and the LiDAR mounting position and
  orientation have been measured and verified. Do not set an unmeasured value to `verified: true`.
- Confirm that a manually edited road centerline and the corresponding vehicle envelope lie within
  the area that can be statically validated from the input point cloud.
- Review the point cloud, trajectory, boundaries, obstacles, and unverified areas in the GUI and RViz2.
- Regenerate the map after editing and use the latest output and reports.

### Vector map and Planning Simulator

- Record that `autoware_candidate_acceptance.json` has `accepted: true` and `errors: []`, and that
  every warning was reviewed.
- Before the driving test, verify the generation-time contract and confirm that the data collection
  vehicle and target vehicle are the same vehicle.
- Confirm that the `vehicle_info.param.yaml` used for map generation matches the Autoware vehicle
  model.
- Record the map's static-validation result separately from the Planning Simulator result.

### Navigation map and Nav2

- Distinguish the readiness file's `artifact` fields from its `production_ready` and `deployment`
  fields when reviewing them.
- After the load test, confirm that `acceptance.json` has `accepted: true` and `errors: []`.
- Record `planning`, `action_execution`, and `robot_motion` as not performed.

## Trademarks

Autoware is a trademark of the Autoware Foundation. ROS is a trademark of Open Source Robotics
Foundation. See [../TRADEMARKS.md](../TRADEMARKS.md) for the complete trademark and affiliation
notice.

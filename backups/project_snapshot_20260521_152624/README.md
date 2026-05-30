# TM5-700 Color Cube Camera Scene

This project is a minimal ROS 2 Humble + Gazebo demo for a TM5-700 robot in a
color cube scene.

It currently provides:

- A Gazebo world with fixed red, yellow, and blue cube initial positions.
- A 3x3 named joint-pose grid configuration.
- A TM5-700 robot model with a tool-mounted camera.
- A Gazebo-to-ROS camera bridge for `/tool_camera/image_raw`.
- A launch file that starts the scene, spawns the robot, and optionally opens
  the camera view.
- A combined Gazebo + MoveIt launch file for the color cube scene.
- A `move_to_ready` command that asks MoveIt to move to the configured `ready`
  pose.
- The robot base/home spawn position is kept at the normal Gazebo origin:
  `x=0.0`, `y=0.0`, `z=0.0`.

## Kept layout

- Grid definition: `src/tm5_color_pick_demo/config/grid_layout.yaml`
- End-effector work poses: `src/tm5_color_pick_demo/config/work_poses.yaml`
- Cube initial poses: `src/tm5_color_pick_demo/worlds/color_cubes.sdf`
- Camera model/sensor: `src/tm5_color_pick_demo/xacro/tm5-700_color_pick.urdf.xacro`
- Camera bridge/view launch: `src/tm5_color_pick_demo/launch/color_cubes_gazebo.launch.py`
- Gazebo + MoveIt launch:
  `src/tm5_color_pick_demo/launch/color_cubes_moveit_gazebo.launch.py`
- Ready motion command:
  `src/tm5_color_pick_demo/tm5_color_pick_demo/move_to_ready.py`
- Suction grasp manager:
  `src/tm5_color_pick_demo/tm5_color_pick_demo/suction_grasp_manager.py`
- Logical cube state:
  `src/tm5_color_pick_demo/config/cube_state.yaml`
- Cube model/link config:
  `src/tm5_color_pick_demo/config/cube_config.yaml`
- Grouped pick/place joint poses:
  `src/tm5_color_pick_demo/config/grid_pose_groups.yaml`

## Build and run

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
cd ~/桌面/tm5_color_pick_demo
colcon build --packages-select tm5_color_pick_demo
source install/setup.bash
ros2 launch tm5_color_pick_demo color_cubes_gazebo.launch.py gui:=true camera_view:=true
```

To run the color cube scene with MoveIt and Gazebo:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
cd ~/桌面/tm5_color_pick_demo
colcon build --packages-select tm5_color_pick_demo
source install/setup.bash
ros2 launch tm5_color_pick_demo color_cubes_moveit_gazebo.launch.py use_rviz:=true camera_view:=true
```

To command the arm to the configured `ready` end-effector position, open a
second terminal after the launch above is running:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo move_to_ready
```

To debug a recorded grid joint pose without suction attach/detach:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo move_to_grid_cell --ros-args -p cell:=right_bottom
```

To start the first suction service prototype, open another terminal after Gazebo
is running:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo suction_grasp_manager
```

Service calls for the first red-cube prototype:

```bash
ros2 service call /suction/move_red_to_center std_srvs/srv/Trigger "{}"
ros2 service call /suction/attach std_srvs/srv/Trigger "{}"
ros2 service call /suction/detach std_srvs/srv/Trigger "{}"
```

To run the fixed red-cube pick-place prototype, keep Gazebo + MoveIt and
`suction_grasp_manager` running, then open another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo red_pick_place_demo
```

To run the first generic single-cube pick-place prototype, keep Gazebo + MoveIt
and `suction_grasp_manager` running, then provide the cube and target grid cell:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo cube_pick_place_demo --ros-args -p cube:=blue -p place_cell:=right_bottom
```

The generic demo reads `cube_state.yaml` to find the cube's current logical
grid cell. The default logical state matches the current Gazebo initial cube
layout:

```yaml
red: left_top
yellow: left_middle
blue: left_bottom
```

For example, if the state contains `blue: left_bottom`, the command
above executes:

```text
ready -> left_bottom -> /suction/attach_blue -> ready -> right_bottom -> /suction/detach_blue -> ready
```

After successful motion and detach, it updates the runtime `cube_state.yaml` so
`blue: right_bottom`. If motion, attach, or detach fails, it does not update
the state file.

To run the rule-based language command entry point:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo language_pick_place_demo --ros-args -p command:="把蓝色方块移动到右下"
```

## Demo 1-6 Overview

All pick-place demos use the same nine logical grid cells:

```text
left_top      center_top      right_top
left_middle   center_middle   right_middle
left_bottom   center_bottom   right_bottom
```

Real execution uses the recorded joint-pose groups in `grid_pose_groups.yaml`.
No demo creates new grid poses at runtime. The shared motion sequence is:

```text
ready -> attach_place[source cell] -> suction attach -> ready -> detach_place[target cell] -> suction detach -> ready
```

For real motion, keep these terminals running:

Terminal A, Gazebo + MoveIt:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 launch tm5_color_pick_demo color_cubes_moveit_gazebo.launch.py use_rviz:=true camera_view:=false
```

Terminal B, optional camera viewer:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run rqt_image_view rqt_image_view /tool_camera/image_raw
```

Terminal D, suction attach/detach services:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo suction_grasp_manager
```

Check suction services before real execution:

```bash
ros2 service list | grep suction
```

Expected color-specific services include:

```text
/suction/attach_red
/suction/detach_red
/suction/attach_yellow
/suction/detach_yellow
/suction/attach_blue
/suction/detach_blue
```

The recorded logical cube state is stored in `cube_state.yaml`. Reset it after
restarting Gazebo or after an aborted motion if the file no longer matches the
scene:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo reset_cube_state
```

The default logical state is:

```yaml
red: left_top
yellow: left_middle
blue: left_bottom
```

### Demo 1: Fixed Red Move

`demo_1` is the simplest full-motion test. It resets `cube_state.yaml` to the
default layout, then moves `red` from `left_top` to `right_bottom`.

Run after Terminal A and Terminal D are ready:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo demo_1
```

Expected sequence:

```text
ready -> attach_place/left_top -> /suction/attach_red -> ready -> detach_place/right_bottom -> /suction/detach_red -> ready
```

### Demo 2: Rule-Based Single-Cube Command

`demo_2` accepts one rule-parsed Chinese or English command. It does not reset
`cube_state.yaml`; every successful move updates the recorded state, and the
next command starts from that updated state.

One-shot examples:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo demo_2 --ros-args -p command:="把红色方块移动到右上"
ros2 run tm5_color_pick_demo demo_2 --ros-args -p command:="把蓝色方块移动到右下"
```

Interactive mode:

```bash
ros2 run tm5_color_pick_demo demo_2 --ros-args -p interactive:=true
```

Example input:

```text
把黄色方块移动到中间
```

### Demo 3: LLM Ordered Moves

`demo_3` uses the DeepSeek LLM parser for ordered single-cube moves. It runs as
an interactive prompt and executes each parsed move in order.

Parse-only validation, no robot motion:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo demo_3 --ros-args -p parse_only:=true
```

At the `demo_3>` prompt, try:

```text
先把黄色方块移动到右下，然后把它移动到右上，最后把蓝色放到中间
```

Real execution:

```bash
ros2 run tm5_color_pick_demo demo_3
```

### Demo 4: LLM + Planner

`demo_4` adds the planner above `demo_3`. It supports occupied targets, region
targets, buffer moves, swaps, and sequential stages. It still uses
`cube_state.yaml` as the current state source.

Planning validation, no robot motion:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo demo_4 --ros-args -p plan_only:=true
```

At the `demo_4>` prompt, try:

```text
yellow and blue cube go to the top side
黄色和蓝色交换位置，然后红色和蓝色交换位置
把黄色放到中间
把黄色和蓝色放到中间一列
```

Real execution with manual confirmation:

```bash
ros2 run tm5_color_pick_demo demo_4 --ros-args -p confirm_before_execute:=true
```

For natural-language parsing, plain `中间`, `中心`, and `中间区域` mean the exact
center cell `center_middle`. Only explicit phrases such as `中间列`, `中间一列`,
or `center column` mean the center-column region.

### Demo 5: Vision Closed Loop

`demo_5` adds visual feedback above `demo_4`. Before planning, it moves to
`ready`, reads `/tool_camera/image_raw`, detects the red/yellow/blue cube cells,
maps camera cells to logic cells, writes that state to `cube_state.yaml`, then
uses the same LLM parser, planner, and executor as `demo_4`. After execution it
returns to `ready`, reads the camera again, and verifies the requested target
cubes.

The camera grid is rotated relative to the logic grid:

```text
camera left_top      -> logic right_top
camera center_top    -> logic right_middle
camera right_top     -> logic right_bottom
camera left_middle   -> logic center_top
camera center_middle -> logic center_middle
camera right_middle  -> logic center_bottom
camera left_bottom   -> logic left_top
camera center_bottom -> logic left_middle
camera right_bottom  -> logic left_bottom
```

Vision-only validation, no pick-place execution:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo demo_5 --ros-args -p vision_only:=true
```

At the `demo_5>` prompt, press Enter. The arm moves to `ready`, then prints
`camera_state` and mapped `current_state`.

Planner validation with vision, no pick-place execution:

```bash
ros2 run tm5_color_pick_demo demo_5 --ros-args -p plan_only:=true
```

At the `demo_5>` prompt, try:

```text
把黄色放到右上
```

Expected plan from the default scene after visual mapping:

```text
yellow from left_middle to right_top
```

Real visual closed-loop execution:

```bash
ros2 run tm5_color_pick_demo demo_5
```

`demo_5` uses `confirm_before_execute:=true` by default. After it prints the
plan, it asks:

```text
Execute this Demo 5 plan? [y/N]
```

Enter `y` only after checking the printed `camera_state`, `current_state`, and
`planned_stages`. Current vision limits: the HSV detector assumes the fixed
`ready` viewpoint clearly shows all three red/yellow/blue cubes.

### Demo 6: LLM Color Sorting

`demo_6` is built on the same visual closed-loop idea as `demo_5`, but it uses
a separate Demo 6 Gazebo world with a visual 3x3 sorting table. The original
world and `demo_1` through `demo_5` are not changed.

Demo 6 fixed color sorting targets:

```yaml
yellow: right_top
blue: right_middle
red: right_bottom
```

The Demo 6 table visual uses the same logical grid names:

```text
left_top        center_top        right_top
left_middle     center_middle     right_middle
left_bottom     center_bottom     right_bottom
```

Visual table layout:

```text
left column:   pickup area / initial cube slots
center column: neutral buffer area
right column:  sorting targets
               right_top    yellow
               right_middle blue
               right_bottom red
```

Launch the Demo 6 Gazebo + MoveIt world:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 launch tm5_color_pick_demo color_cubes_demo_6_moveit_gazebo.launch.py use_rviz:=true camera_view:=false
```

Optional camera viewer:

```bash
ros2 run rqt_image_view rqt_image_view /tool_camera/image_raw
```

Start suction services before real execution:

```bash
ros2 run tm5_color_pick_demo suction_grasp_manager
```

Vision-only validation:

```bash
ros2 run tm5_color_pick_demo demo_6 --ros-args -p vision_only:=true
```

Planner validation with LLM classification, no pick-place execution:

```bash
ros2 run tm5_color_pick_demo demo_6 --ros-args -p plan_only:=true
```

At the `demo_6>` prompt, try:

```text
把方块按照颜色分类
```

The LLM is used only to classify the user command as a supported color sorting
task. The target positions are fixed by `color_sort_target_generator.py`, so the
LLM does not choose the yellow/blue/red target cells.

Real Demo 6 execution:

```bash
ros2 run tm5_color_pick_demo demo_6
```

After printing `current_state`, `target_state`, and `action_plan`, Demo 6 asks:

```text
Execute this Demo 6 plan? [y/N]
```

Enter `y` only after confirming the plan. Demo 6 rechecks the camera after
execution and verifies that yellow, blue, and red reached the fixed sorting
targets.

## Current state

- The earlier incorrect `work_x/work_y/work_z` launch-argument approach was
  removed.
- The robot is spawned at the original home/base position.
- `work1` is configured as an end-effector target pose in `base_link`, with
  `tool0` at position `[0.4, 0.0, 0.8]`.
- `work1` orientation is still unresolved and must be chosen or derived before
  using it for IK/planned motion.
- `ready` is configured as a resolved end-effector pose for `link_6` in
  `base_link`:
  - The saved Cartesian xyz/xyzw fields are retained for reference, but current
    demo motion uses the joint-space `ready` pose.
  - Current joint positions:
    - `joint_1`: `2.5120967586876537`
    - `joint_2`: `-0.02575691717062636`
    - `joint_3`: `-1.3356944378650484`
    - `joint_4`: `1.3587778920080285`
    - `joint_5`: `-1.0189702481383465`
    - `joint_6`: `0.09387227471591081`
- The color cube world and tool camera are now merged with the TM5-700 MoveIt
  Gazebo launch path.
- MoveIt controls the simulated TM5-700 in Gazebo through
  `tmr_arm_controller`, but Gazebo cube models are not yet mirrored into the
  MoveIt planning scene, and there is not yet a physical gripper/vacuum model.
- In `color_cubes.sdf`, `red_cube`, `yellow_cube`, and `blue_cube` are dynamic
  collision objects with mass and box collisions.
- The local `tmr_ros2` checkout provides the TM arm model, `flange/tool0`, and
  end-effector IO/tool-pose APIs, but no official gripper/vacuum/finger model
  was found in the installed/source package files.
- A visual suction cup has been added as `suction_cup_link`, fixed to `tool0`
  at offset `[0.03, 0.0, 0.0]`. Its collision geometry is intentionally omitted
  so that, after DetachableJoint detach, the cup cannot keep pushing or carrying
  the cube through contact.
- The MoveIt semantic config used by `color_cubes_moveit_gazebo.launch.py` is
  now `src/tm5_color_pick_demo/config/tm5-700_color_pick_gz.srdf`, which adds
  allowed-collision entries for the suction cup and nearby tool links.
- `color_cubes_moveit_gazebo.launch.py` now starts `planning_scene_objects`,
  which adds only `pick_table` as a MoveIt planning-scene collision object.
  Cube collision objects are still intentionally omitted.
- The suction manager now uses Gazebo's `DetachableJoint` system instead of a
  timer-based `set_pose` follow loop. On first attach, it dynamically loads a
  detachable fixed-joint system onto the `tm5-700` model, using `link_6` as the
  parent link and the selected cube link as the child link, then controls it
  through Gazebo Empty-message topics.
- The cube models are dynamic Gazebo models. While attached, Gazebo physics
  maintains the fixed joint; while detached, the cube is free again.
- `/suction/move_red_to_center` is kept only as a quick Gazebo pose reset helper
  for red-cube testing.
- `suction_grasp_manager.py` keeps the old red compatibility services
  `/suction/attach` and `/suction/detach`, and also provides:
  - `/suction/attach_red`, `/suction/detach_red`
  - `/suction/attach_yellow`, `/suction/detach_yellow`
  - `/suction/attach_blue`, `/suction/detach_blue`
- A fixed red-cube pick-place prototype is available as
  `ros2 run tm5_color_pick_demo red_pick_place_demo`. It treats the 3x3 grid as
  nine named joint-space poses, not as XY target coordinates. The red demo
  sequence is:
  `initial/current -> ready -> left_top -> attach -> ready -> right_top -> detach -> ready`.
  Defaults are `pick_cell:=left_top` and `place_cell:=right_top`.
- The pick table and cubes have been raised by `0.18 m` for reachability
  testing, then shifted left by `0.15 m` in Gazebo and in the MoveIt table
  collision object:
  - `pick_table` size is now `[0.5, 0.5, 0.02]`, center pose is
    `[0.50, 0.0, 0.19]`, and its top surface is at `z=0.20`.
  - Cube center z is now `0.235`; cube top/grid contact height is `z=0.26`.
  - Current Gazebo cube centers are `x=0.367`, with y values `0.133`, `0.000`,
    and `-0.133` for red, yellow, and blue.
  - The old suction-follow offset is no longer used by attach/detach; grasp and
    place behavior now depends on the recorded joint poses and Gazebo's fixed
    joint state.

## Deferred issues

- The camera module currently shows no image content; a blocked physical model
  is suspected but not yet investigated.

## Suction Demo

First target behavior: attach `red_cube` after the arm reaches the configured
left-top joint pose, return to `ready` to lift the attached cube away from other
cubes, move the arm to the configured right-top joint pose, then detach.

- Current `red_cube` center in Gazebo/world SDF: `[0.367, 0.133, 0.235]`.
- Optional reset target from the grid center: `[0.60, 0.00]`.
- For Gazebo cube pose, keep cube center height at `z=0.235` so the 0.05 m cube
  remains at the same physical height as the other cubes.
- The corresponding legacy Gazebo reset target is `[0.60, 0.00, 0.235]`.
- The grid task height is now `z=0.26` for cube-top contact or placement-surface
  references, not for the cube model center pose.

Current first implementation:

- `suction_grasp_manager.py` provides `/suction/attach`, `/suction/detach`, and
  `/suction/move_red_to_center`.
- `/suction/attach` dynamically adds Gazebo's `DetachableJoint` system to the
  `tm5-700` model if needed. In the current Gazebo 6 implementation, loading
  this system creates the attached fixed joint immediately; the manager treats
  that successful load as attach success.
- `/suction/detach` publishes `ignition.msgs.Empty` on the configured detach
  topic. Detach publishes three Empty messages by default to improve Gazebo
  DetachableJoint release reliability.
- `/suction/move_red_to_center` directly sets `red_cube` to
  `[0.60, 0.00, 0.235]` for quick Gazebo pose-path validation.
- MoveIt planning-scene synchronization is intentionally deferred until Gazebo
  attach/detach behavior is working.
- Manual testing confirmed the DetachableJoint attach path works and the cube
  can follow the arm. The suction cup collision geometry has been removed to
  avoid contact carrying the cube after detach.

## Current demo validation

The current red-cube demo uses fixed joint-space poses:

- The 3x3 grid cells are named joint poses:
  `left_top`, `left_middle`, `left_bottom`, `center_top`, `center_middle`,
  `center_bottom`, `right_top`, `right_middle`, and `right_bottom`.
- Default red-cube sequence:
  - `ready`
  - `pick_cell` (default `left_top`)
  - `/suction/attach`
  - `ready`
  - `place_cell` (default `right_top`)
  - `/suction/detach`
  - `ready`
- Store these poses as explicit `joint_1` through `joint_6` values instead of
  using Cartesian IK targets. This should reduce IK ambiguity and make the path
  between known safe poses more repeatable.
- Start with one pose per cell. If table/cube clearance still needs more
  control, extend each cell to `approach` and `work` joint poses.
- The nine grid cell joint poses have been recorded in
  `src/tm5_color_pick_demo/config/grid_joint_poses.yaml`, and
  `red_pick_place_demo` now reads `pick_cell` / `place_cell` parameters instead
  of using Cartesian `suction_cup_link` targets.
- The current generic demos use `src/tm5_color_pick_demo/config/grid_pose_groups.yaml`
  instead of the older single-pose grid. It contains 18 validated poses:
  - `attach_place`: 9 pick poses for the cube source cell.
  - `detach_place`: 9 place poses for the target cell.
  - `joint_1` values were normalized onto the current positive-angle branch to
    reduce unnecessary full-turn motions from `ready`.
- Joint-space moves use a default MoveIt goal tolerance of `0.06 rad`.
  After MoveIt reports execution success, the demo waits for `/joint_states` to
  settle within `joint_settle_tolerance_rad` before continuing to attach or
  detach. The default settle tolerance is `0.06 rad` with a
  `joint_settle_timeout_sec` of `8.0`.
- The joint-settle check is shared by `demo_1`, `demo_2`, `demo_3`,
  `cube_pick_place_demo`, and `red_pick_place_demo`. It now requires the target
  joints to remain inside tolerance for multiple consecutive samples and checks
  that joint velocity has dropped before allowing suction attach or detach.
  Defaults are `joint_stable_sample_count:=5`,
  `joint_stable_sample_period_sec:=0.1`, and
  `joint_velocity_tolerance_rad_s:=0.03`.
- Generic cube place motions use a wider `place_settle_tolerance_rad` default
  of `0.12 rad` before detach, so a cube that is already close to the target
  and contacting the table can be released instead of failing the whole demo.
- Generic cube demos wait `detach_settle_sec:=2.0` seconds after a successful
  detach service call before moving back to `ready`, giving Gazebo time to
  remove the DetachableJoint.
- Suction detach is confirmed through the DetachableJoint state topic. If a
  detach publish does not produce `detached`, the manager retries in small
  rounds before failing. Defaults are `detach_publish_count:=3`,
  `detach_retry_count:=3`, `detach_retry_wait_sec:=1.0`, and
  `state_wait_timeout_sec:=3.0`.
- Suction attach is also confirmed through the DetachableJoint state topic
  after the system is already loaded. Defaults are `attach_retry_count:=3` and
  `attach_retry_wait_sec:=1.0`. On the first load, Gazebo creates the attached
  fixed joint immediately, so the manager treats a successful system load as
  attach success even if no fresh state message is emitted.
- A generic single-cube prototype is available as
  `ros2 run tm5_color_pick_demo cube_pick_place_demo --ros-args -p cube:=blue -p place_cell:=right_bottom`.
  It reads `cube_state.yaml`, moves from the selected cube's recorded current
  cell to the requested target cell, and updates the state only after success.
- `demo_1` is a fixed demonstration that resets the logical state and moves
  `red` from `left_top` to `right_bottom`. It has been manually validated with
  the current `ready`, `attach_place`, and `detach_place` poses.
- `demo_2` is a continuous single-cube command demonstration. It does not reset
  logical cube state; run `reset_cube_state` only when starting from a freshly
  reset Gazebo scene or recovering from a failed/inconsistent experiment. It has
  been manually validated for continuous one-command-at-a-time operation.
- The generic single-cube prototype now uses grouped joint poses from
  `grid_pose_groups.yaml`: pick motions use `attach_place` and place motions
  use `detach_place` by default. The pose groups can be overridden with
  `pick_pose_group` and `place_pose_group` parameters for debugging.
- A rule-based language wrapper is available as
  `ros2 run tm5_color_pick_demo language_pick_place_demo --ros-args -p command:="把蓝色方块移动到右下"`.
  It currently parses simple Chinese/English color and grid-cell phrases; it
  does not use an LLM.
- `reset_cube_state` resets the logical cube state to the Gazebo initial layout.
  Use it after restarting Gazebo or before repeating the same initial-layout
  experiment.
- `move_to_grid_cell` moves the arm to one recorded grid joint pose without
  suction attach/detach. Use it to check whether a cell's joint pose settles
  accurately before testing cube transport.
- If a command asks a cube to move to the same cell already recorded in
  `cube_state.yaml`, the generic demo exits without moving or calling suction
  services.
- If a generic cube motion fails after attach, the demo tries to call the
  matching detach service before aborting, and still does not update
  `cube_state.yaml`.
- The generic demo still intentionally ignores target-cell occupancy. It does
  not do overlap detection, rearrangement planning, visual recognition, or
  Gazebo-to-MoveIt cube collision synchronization.

Minimal validation commands after a fresh launch:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo suction_grasp_manager
```

In another terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo red_pick_place_demo
```

If MoveIt reports that `tmr_arm_controller` is not running, activate it with:

```bash
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: [joint_state_broadcaster, tmr_arm_controller], deactivate_controllers: [], strictness: 2, activate_asap: true, timeout: {sec: 5, nanosec: 0}}"
```

Expected visual behavior: red cube attaches at `left_top`, is lifted through
`ready`, moves to the requested `detach_place` target, detaches, and no longer
follows the suction cup as the arm returns to `ready`.

Joint-state capture command for each manually positioned grid pose:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/桌面/tm5_color_pick_demo/install/setup.bash
ros2 topic echo /joint_states --once
```

For each captured pose, keep the `name` and `position` arrays together because
the order may not be `joint_1` through `joint_6`.

Current manual verification fallback for the suction prototype:

- Launch Gazebo + MoveIt.
- Run `ros2 run tm5_color_pick_demo suction_grasp_manager`.
- Call `/suction/move_red_to_center` and confirm `red_cube` moves to
  `[0.60, 0.00, 0.235]`.
- Then test `/suction/attach` after moving the arm to a pick joint pose and
  `/suction/detach` after moving to a place joint pose.
- If Gazebo pose setting fails, inspect whether
  `/world/color_pick_world/set_pose` exists in `ign service -l`.

## Continue next time

Use this prompt to continue:

```bash
cd ~/桌面/tm5_color_pick_demo && sed -n '1,220p' README.md
```

Then tell Codex: continue from the README current demo validation section and
debug or extend the DetachableJoint demo_1/demo_2 single-cube pick-place demos;
do not change the robot base spawn/home position, do not replace the
joint-space `attach_place` / `detach_place` approach with Cartesian IK, and keep
advanced multi-object rearrangement deferred.

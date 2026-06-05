# TM5-700 Natural-Language Visual Color Sorting

English | [日本語](README.ja.md)

This repository contains a ROS 2 Humble and Gazebo simulation project for a
TM5-700 robot arm that interprets natural-language commands, recognizes colored
cubes through a tool-mounted camera, plans pick-and-place actions, and verifies
the final layout visually.

The current focus is not only moving one cube with a fixed script. The project
connects four layers into one workflow:

1. A simulated TM5-700 robot, work table, suction tool, and colored cube scene.
2. Camera-based red/yellow/blue cube detection on a 3x3 logical grid.
3. Natural-language parsing and planning for user commands.
4. Gazebo/MoveIt execution with suction attach/detach and post-action checking.

## Demo Materials

| Material | Description |
| --- | --- |
| [Midterm report PDF](docs/tm5-700_midterm_report.pdf) | Presentation document describing the system design, progress, and experiment direction. |
| [demo_5.mp4](media/demo_5.mp4) | Vision closed-loop experiment: camera state detection, natural-language planning, execution, and verification. |
| [demo_6.mp4](media/demo_6.mp4) | Color sorting experiment: LLM command classification, fixed color targets, execution, and final visual verification. |

## What This Project Does

The scene contains red, yellow, and blue cubes placed on a 3x3 grid. A user can
ask the robot to move cubes by color and position. The system converts that
request into grid-level actions, executes them with the TM5-700 in simulation,
and keeps a logical cube-state file synchronized with the observed scene.

The repository includes:

- Gazebo worlds for the standard color-cube scene and the Demo 6 sorting table.
- A TM5-700 robot model with a tool-mounted camera and visual suction cup.
- MoveIt launch files for simulated arm control.
- Camera bridge and image-viewer support for `/tool_camera/image_raw`.
- Grid, cube, and pick/place joint-pose configuration files.
- Suction attach/detach services using Gazebo's detachable joint mechanism.
- Demo programs from fixed single-cube motion through LLM-assisted visual sorting.

## System Flow

```text
Natural-language command
        |
        v
LLM / rule-based parser
        |
        v
Grid-level task planner
        |
        v
MoveIt joint-pose execution
        |
        v
Gazebo suction attach/detach
        |
        v
Camera-based verification
```

## Demo Progression

| Demo | Purpose | Main capability |
| --- | --- | --- |
| `demo_1` | Fixed red-cube motion | Validates the basic ready -> pick -> attach -> place -> detach sequence. |
| `demo_2` | Rule-based language command | Parses simple Chinese/English color and grid-cell commands. |
| `demo_3` | LLM ordered moves | Uses an LLM parser for ordered single-cube commands. |
| `demo_4` | LLM + planner | Adds planning for occupied targets, swaps, regions, and multi-step commands. |
| `demo_5` | Vision closed loop | Detects cube positions from the camera before planning and verifies after execution. |
| `demo_6` | Visual color sorting | Sorts cubes to fixed color targets in a dedicated table world. |

## Demo 5: Vision Closed Loop

`demo_5` builds on `demo_4` by adding visual feedback. Before planning, the arm
moves to the `ready` pose, reads `/tool_camera/image_raw`, detects the cube
positions, maps camera cells to the logical grid, and writes the observed state
to `cube_state.yaml`. It then uses the same language parser, planner, and
executor as `demo_4`. After execution, it returns to `ready`, reads the camera
again, and verifies the requested target cubes.

Run vision-only validation:

```bash
ros2 run tm5_color_pick_demo demo_5 --ros-args -p vision_only:=true
```

Run planner validation with vision but without pick-place execution:

```bash
ros2 run tm5_color_pick_demo demo_5 --ros-args -p plan_only:=true
```

Run the full visual closed-loop experiment:

```bash
ros2 run tm5_color_pick_demo demo_5
```

By default, Demo 5 asks for confirmation before executing the planned action.
Check the printed `camera_state`, `current_state`, and `planned_stages` before
entering `y`.

## Demo 6: LLM Color Sorting

`demo_6` uses a separate Gazebo world with a visual 3x3 sorting table. The user
asks for color sorting in natural language. The LLM classifies whether the
command is a supported color-sorting task, but the target cells are fixed by the
program so the result is deterministic:

```yaml
yellow: right_top
blue: right_middle
red: right_bottom
```

Launch the Demo 6 Gazebo + MoveIt world:

```bash
ros2 launch tm5_color_pick_demo color_cubes_demo_6_moveit_gazebo.launch.py use_rviz:=true camera_view:=false
```

Run vision-only validation:

```bash
ros2 run tm5_color_pick_demo demo_6 --ros-args -p vision_only:=true
```

Run planner validation:

```bash
ros2 run tm5_color_pick_demo demo_6 --ros-args -p plan_only:=true
```

Run the full sorting experiment:

```bash
ros2 run tm5_color_pick_demo demo_6
```

After printing `current_state`, `target_state`, and `action_plan`, Demo 6 asks
for confirmation before executing. It checks the camera again after execution
and verifies that the three cubes reached the fixed target cells.

## Repository Layout

```text
docs/
  tm5-700_midterm_report.pdf
media/
  demo_5.mp4
  demo_6.mp4
src/tm5_color_pick_demo/
  config/
  launch/
  tm5_color_pick_demo/
  worlds/
  xacro/
```

Important project files:

| Path | Role |
| --- | --- |
| `src/tm5_color_pick_demo/config/grid_layout.yaml` | 3x3 logical grid definition. |
| `src/tm5_color_pick_demo/config/cube_state.yaml` | Runtime logical cube state. |
| `src/tm5_color_pick_demo/config/grid_pose_groups.yaml` | Recorded pick/place joint poses. |
| `src/tm5_color_pick_demo/worlds/color_cubes.sdf` | Standard red/yellow/blue cube world. |
| `src/tm5_color_pick_demo/launch/color_cubes_moveit_gazebo.launch.py` | Main Gazebo + MoveIt launch file. |
| `src/tm5_color_pick_demo/launch/color_cubes_demo_6_moveit_gazebo.launch.py` | Demo 6 sorting-world launch file. |
| `src/tm5_color_pick_demo/tm5_color_pick_demo/suction_grasp_manager.py` | Gazebo detachable-joint suction services. |

## Build

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
cd ~/Desktop/tm5_color_pick_demo
colcon build --packages-select tm5_color_pick_demo
source install/setup.bash
```

If your desktop directory is localized, replace `~/Desktop/tm5_color_pick_demo`
with the actual checkout path.

## Standard Run Setup

Terminal A, Gazebo + MoveIt:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 launch tm5_color_pick_demo color_cubes_moveit_gazebo.launch.py use_rviz:=true camera_view:=false
```

Terminal B, optional camera viewer:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 run rqt_image_view rqt_image_view /tool_camera/image_raw
```

Terminal C, suction services:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo suction_grasp_manager
```

Check that color-specific suction services are available:

```bash
ros2 service list | grep suction
```

Expected services include:

```text
/suction/attach_red
/suction/detach_red
/suction/attach_yellow
/suction/detach_yellow
/suction/attach_blue
/suction/detach_blue
```

## Reset Logical State

The demos use `cube_state.yaml` as the current logical state. Reset it after
restarting Gazebo or after an interrupted run if the file no longer matches the
scene:

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/install/setup.bash
source ~/Desktop/tm5_color_pick_demo/install/setup.bash
ros2 run tm5_color_pick_demo reset_cube_state
```

Default state:

```yaml
red: left_top
yellow: left_middle
blue: left_bottom
```

## Current Limitations

- The visual detector currently assumes the fixed `ready` viewpoint clearly sees
  all three red/yellow/blue cubes.
- Cube objects are controlled in Gazebo and tracked logically, but they are not
  yet fully synchronized as MoveIt planning-scene collision objects.
- The suction tool is a simulated detachable-joint mechanism rather than a
  physically modeled vacuum gripper.
- Demo 6 uses fixed color target cells. The LLM classifies the task, but it does
  not choose target positions.

## Development Notes

The project intentionally uses recorded joint-space pick/place poses for the
3x3 grid instead of generating new Cartesian IK targets at runtime. This keeps
the path repeatable for the current simulated workcell and reduces IK ambiguity
around the TM5-700 arm.

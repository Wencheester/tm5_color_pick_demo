import json
import os
import sys

import rclpy
from rclpy.parameter import Parameter

from tm5_color_pick_demo.color_sort_goal_parser import ColorSortGoalParser
from tm5_color_pick_demo.color_sort_target_generator import (
    generate_color_sort_target_state,
    target_state_to_goals,
)
from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo
from tm5_color_pick_demo.grid_mapping import (
    camera_state_to_logic_state,
    format_state_chinese,
)
from tm5_color_pick_demo.task_planner import PlanningError, plan_actions
from tm5_color_pick_demo.verification import format_failures, verify_targets
from tm5_color_pick_demo.vision_detector import VisionDetector


class Demo6(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("demo_6")
        self.declare_parameter("vision_only", False)
        self.declare_parameter("parse_only", False)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("confirm_before_execute", True)
        self.declare_parameter("move_ready_before_vision", True)
        self.declare_parameter("camera_topic", "/tool_camera/image_raw")
        self.declare_parameter("vision_timeout_sec", 5.0)
        self.declare_parameter("vision_min_area", 80.0)
        self.declare_parameter("deepseek_api_key", "")
        self.declare_parameter("deepseek_base_url", "https://api.deepseek.com")
        self.declare_parameter("deepseek_model", "deepseek-v4-flash")
        self.declare_parameter("deepseek_timeout_sec", 30.0)
        self._vision_detector = VisionDetector(
            self,
            image_topic=str(self.get_parameter("camera_topic").value),
        )

    def run(self):
        return self._run_interactive()

    def _run_interactive(self):
        self.get_logger().info("Demo 6 color sorting mode. Enter q to quit.")
        while rclpy.ok():
            try:
                command = input("demo_6> ").strip()
            except EOFError:
                return 0

            if command.lower() in ("q", "quit", "exit"):
                return 0

            try:
                if bool(self.get_parameter("vision_only").value):
                    self._observe_logic_state()
                    continue
                if not command:
                    continue
                exit_code = self._plan_run_and_verify(command)
            except Exception as exc:
                self.get_logger().error(str(exc))
                continue

            if exit_code != 0:
                return exit_code

        return 0

    def _plan_run_and_verify(self, command):
        cube_state_path = self._cube_state_path()
        current_state = self._observe_logic_state()
        self._write_cube_state(cube_state_path, current_state)

        task = self._parse_color_sort_command(command)
        self.get_logger().info(
            "Demo 6 parsed task: %s" % json.dumps(task, ensure_ascii=False)
        )

        target_state = generate_color_sort_target_state()
        goals = target_state_to_goals(target_state)
        self.get_logger().info(
            "Demo 6 current_state: %s"
            % json.dumps(current_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 6 target_state: %s"
            % json.dumps(target_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 6 goal_constraints: %s"
            % json.dumps(goals, ensure_ascii=False)
        )

        if bool(self.get_parameter("parse_only").value):
            self.get_logger().info("parse_only is true; no planning or motion will be executed.")
            return 0

        target_state, action_plan = self._plan_color_sort(current_state, goals)

        if bool(self.get_parameter("plan_only").value):
            self.get_logger().info(
                "Demo 6 action_plan: %s"
                % json.dumps(action_plan, ensure_ascii=False, sort_keys=True)
            )
            self.get_logger().info("plan_only is true; no robot motion will be executed.")
            return 0

        self.get_logger().info(
            "Demo 6 executing action_plan: %s"
            % json.dumps(action_plan, ensure_ascii=False, sort_keys=True)
        )
        if bool(self.get_parameter("confirm_before_execute").value):
            answer = input("Execute this Demo 6 plan? [y/N] ").strip().lower()
            if answer not in ("y", "yes"):
                self.get_logger().info("Demo 6 execution cancelled by user.")
                return 0

        exit_code = self._execute_action_plan(action_plan)
        if exit_code != 0:
            return exit_code

        after_state = self._observe_logic_state()
        ok, failures = verify_targets(after_state, target_state)
        if ok:
            self.get_logger().info("Demo 6 color sorting succeeded.")
            return 0

        self.get_logger().error(
            "Demo 6 verification failed: %s" % format_failures(failures)
        )
        return 1

    def _parse_color_sort_command(self, command):
        parser = ColorSortGoalParser(
            api_key=self._deepseek_api_key(),
            base_url=str(self.get_parameter("deepseek_base_url").value).strip(),
            model=str(self.get_parameter("deepseek_model").value).strip(),
            timeout_sec=float(self.get_parameter("deepseek_timeout_sec").value),
        )
        return parser.parse(command)

    def _plan_color_sort(self, current_state, goals):
        try:
            target_state, action_plan = plan_actions(current_state, goals)
        except PlanningError as exc:
            raise RuntimeError("Demo 6 planning failed: %s" % exc) from exc

        self.get_logger().info(
            "Demo 6 planned target_state: %s"
            % json.dumps(target_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 6 action_plan: %s"
            % json.dumps(action_plan, ensure_ascii=False)
        )
        return target_state, action_plan

    def _execute_action_plan(self, action_plan):
        if not action_plan:
            self.get_logger().info("Demo 6 color sorting target state is already satisfied.")
            return 0

        for action_index, action in enumerate(action_plan, start=1):
            self.get_logger().info(
                "Demo 6 executing action %d/%d: cube=%s, from=%s, to=%s, reason=%s"
                % (
                    action_index,
                    len(action_plan),
                    action["cube"],
                    action["from"],
                    action["to"],
                    action["reason"],
                )
            )
            self._set_task_parameters(action["cube"], action["to"])
            exit_code = CubePickPlaceDemo.run(self)
            if exit_code != 0:
                self.get_logger().error(
                    "Demo 6 stopped after failed action %d." % action_index
                )
                return exit_code
        return 0

    def _observe_logic_state(self):
        if bool(self.get_parameter("move_ready_before_vision").value):
            self._move_to_ready_for_vision()

        camera_state = self._vision_detector.detect_camera_state(
            timeout_sec=float(self.get_parameter("vision_timeout_sec").value),
            min_area=float(self.get_parameter("vision_min_area").value),
        )
        logic_state = camera_state_to_logic_state(camera_state)
        self.get_logger().info(
            "Demo 6 camera_state: %s"
            % json.dumps(camera_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 6 camera_state_zh: %s"
            % json.dumps(format_state_chinese(camera_state), ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 6 current_state: %s"
            % json.dumps(logic_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 6 current_state_zh: %s"
            % json.dumps(format_state_chinese(logic_state), ensure_ascii=False, sort_keys=True)
        )
        return logic_state

    def _move_to_ready_for_vision(self):
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt action server /move_action is not available.")
        ready = self._load_ready_pose()
        if self._move_to_target("vision_ready", ready) != 0:
            raise RuntimeError("Failed to move to ready before vision capture.")

    def _set_task_parameters(self, cube, place_cell):
        self.set_parameters(
            [
                Parameter("cube", Parameter.Type.STRING, cube),
                Parameter("place_cell", Parameter.Type.STRING, place_cell),
                Parameter("pick_pose_group", Parameter.Type.STRING, "attach_place"),
                Parameter("place_pose_group", Parameter.Type.STRING, "detach_place"),
            ]
        )

    def _deepseek_api_key(self):
        configured = str(self.get_parameter("deepseek_api_key").value).strip()
        if configured:
            return configured
        return os.environ.get("DEEPSEEK_API_KEY", "").strip()


def main(args=None):
    rclpy.init(args=args)
    node = Demo6()
    try:
        exit_code = node.run()
    except Exception as exc:
        node.get_logger().error(str(exc))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

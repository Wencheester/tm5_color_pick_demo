import json
import os
import sys

import rclpy
from rclpy.parameter import Parameter

from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo
from tm5_color_pick_demo.grid_mapping import (
    camera_state_to_logic_state,
    format_state_chinese,
)
from tm5_color_pick_demo.llm_goal_parser import LlmGoalParser
from tm5_color_pick_demo.task_planner import PlanningError, plan_actions
from tm5_color_pick_demo.verification import format_failures, verify_targets
from tm5_color_pick_demo.vision_detector import VisionDetector


class Demo5(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("demo_5")
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
        self.get_logger().info("Demo 5 visual closed-loop mode. Enter q to quit.")
        while rclpy.ok():
            try:
                command = input("demo_5> ").strip()
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

        stages = self._parse_command(command)
        self.get_logger().info(
            "Demo 5 stages: %s" % json.dumps(stages, ensure_ascii=False)
        )

        if bool(self.get_parameter("parse_only").value):
            self.get_logger().info("parse_only is true; no planning or motion will be executed.")
            return 0

        planned_stages, final_targets = self._plan_stages(current_state, stages)

        if bool(self.get_parameter("plan_only").value):
            self.get_logger().info(
                "Demo 5 planned_stages: %s"
                % json.dumps(planned_stages, ensure_ascii=False, sort_keys=True)
            )
            self.get_logger().info("plan_only is true; no robot motion will be executed.")
            return 0

        self.get_logger().info(
            "Demo 5 executing planned_stages: %s"
            % json.dumps(planned_stages, ensure_ascii=False, sort_keys=True)
        )
        if bool(self.get_parameter("confirm_before_execute").value):
            answer = input("Execute this Demo 5 plan? [y/N] ").strip().lower()
            if answer not in ("y", "yes"):
                self.get_logger().info("Demo 5 execution cancelled by user.")
                return 0

        for stage_index, stage in enumerate(stages, start=1):
            current_stage_state = self._load_cube_state(cube_state_path)
            _target_state, action_plan = self._plan_stage(
                stage_index,
                current_stage_state,
                stage["goals"],
            )
            if not action_plan:
                self.get_logger().info(
                    "Demo 5 stage %d: requested state is already satisfied."
                    % stage_index
                )
                continue

            for action_index, action in enumerate(action_plan, start=1):
                self.get_logger().info(
                    "Demo 5 executing stage %d action %d/%d: cube=%s, from=%s, to=%s, reason=%s"
                    % (
                        stage_index,
                        action_index,
                        len(action_plan),
                        action["cube"],
                        action["from"],
                        action["to"],
                        action["reason"],
                    )
                )
                self._set_task_parameters(action["cube"], action["to"])
                exit_code = super().run()
                if exit_code != 0:
                    self.get_logger().error(
                        "Demo 5 stopped after failed stage %d action %d."
                        % (stage_index, action_index)
                    )
                    return exit_code

        after_state = self._observe_logic_state()
        ok, failures = verify_targets(after_state, final_targets)
        if ok:
            self.get_logger().info("Demo 5 task succeeded.")
            return 0

        self.get_logger().error(
            "Demo 5 verification failed: %s" % format_failures(failures)
        )
        return 1

    def _observe_logic_state(self):
        if bool(self.get_parameter("move_ready_before_vision").value):
            self._move_to_ready_for_vision()

        camera_state = self._vision_detector.detect_camera_state(
            timeout_sec=float(self.get_parameter("vision_timeout_sec").value),
            min_area=float(self.get_parameter("vision_min_area").value),
        )
        logic_state = camera_state_to_logic_state(camera_state)
        self.get_logger().info(
            "Demo 5 camera_state: %s"
            % json.dumps(camera_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 5 camera_state_zh: %s"
            % json.dumps(format_state_chinese(camera_state), ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 5 current_state: %s"
            % json.dumps(logic_state, ensure_ascii=False, sort_keys=True)
        )
        self.get_logger().info(
            "Demo 5 current_state_zh: %s"
            % json.dumps(format_state_chinese(logic_state), ensure_ascii=False, sort_keys=True)
        )
        return logic_state

    def _move_to_ready_for_vision(self):
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("MoveIt action server /move_action is not available.")
        ready = self._load_ready_pose()
        if self._move_to_target("vision_ready", ready) != 0:
            raise RuntimeError("Failed to move to ready before vision capture.")

    def _plan_stages(self, current_state, stages):
        planned_stages = []
        planning_state = dict(current_state)
        final_targets = {}
        for stage_index, stage in enumerate(stages, start=1):
            target_state, action_plan = self._plan_stage(
                stage_index,
                planning_state,
                stage["goals"],
            )
            final_targets.update(target_state)
            planned_stages.append(
                {
                    "stage": stage_index,
                    "current_state": dict(planning_state),
                    "target_state": target_state,
                    "action_plan": action_plan,
                }
            )
            for action in action_plan:
                planning_state[action["cube"]] = action["to"]
        return planned_stages, final_targets

    def _plan_stage(self, stage_index, current_state, goal_constraints):
        self.get_logger().info(
            "Demo 5 stage %d current_state: %s"
            % (stage_index, json.dumps(current_state, ensure_ascii=False, sort_keys=True))
        )
        self.get_logger().info(
            "Demo 5 stage %d goal_constraints: %s"
            % (stage_index, json.dumps(goal_constraints, ensure_ascii=False))
        )
        try:
            target_state, action_plan = plan_actions(current_state, goal_constraints)
        except PlanningError as exc:
            raise RuntimeError("Stage %d planning failed: %s" % (stage_index, exc)) from exc

        self.get_logger().info(
            "Demo 5 stage %d target_state: %s"
            % (stage_index, json.dumps(target_state, ensure_ascii=False, sort_keys=True))
        )
        self.get_logger().info(
            "Demo 5 stage %d action_plan: %s"
            % (stage_index, json.dumps(action_plan, ensure_ascii=False))
        )
        return target_state, action_plan

    def _parse_command(self, command):
        parser = LlmGoalParser(
            api_key=self._deepseek_api_key(),
            base_url=str(self.get_parameter("deepseek_base_url").value).strip(),
            model=str(self.get_parameter("deepseek_model").value).strip(),
            timeout_sec=float(self.get_parameter("deepseek_timeout_sec").value),
        )
        stages = parser.parse(command)
        self.get_logger().info(
            "Demo 5 parsed stages: %s" % json.dumps(stages, ensure_ascii=False)
        )
        return stages

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
    node = Demo5()
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

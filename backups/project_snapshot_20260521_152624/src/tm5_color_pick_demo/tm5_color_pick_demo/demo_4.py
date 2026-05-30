import json
import os
import sys

import rclpy
from rclpy.parameter import Parameter

from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo
from tm5_color_pick_demo.llm_goal_parser import LlmGoalParser
from tm5_color_pick_demo.task_planner import PlanningError, plan_actions


class Demo4(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("demo_4")
        self.declare_parameter("parse_only", False)
        self.declare_parameter("plan_only", False)
        self.declare_parameter("confirm_before_execute", False)
        self.declare_parameter("deepseek_api_key", "")
        self.declare_parameter("deepseek_base_url", "https://api.deepseek.com")
        self.declare_parameter("deepseek_model", "deepseek-v4-flash")
        self.declare_parameter("deepseek_timeout_sec", 30.0)

    def run(self):
        return self._run_interactive()

    def _run_interactive(self):
        self.get_logger().info("Demo 4 interactive planner mode. Enter q to quit.")
        while rclpy.ok():
            try:
                command = input("demo_4> ").strip()
            except EOFError:
                return 0

            if command.lower() in ("q", "quit", "exit"):
                return 0
            if not command:
                continue

            try:
                exit_code = self._plan_and_run(command)
            except Exception as exc:
                self.get_logger().error(str(exc))
                continue

            if exit_code != 0:
                return exit_code

        return 0

    def _plan_and_run(self, command):
        cube_state_path = self._cube_state_path()
        stages = self._parse_command(command)

        self.get_logger().info(
            "Demo 4 stages: %s" % json.dumps(stages, ensure_ascii=False)
        )

        if bool(self.get_parameter("parse_only").value):
            self.get_logger().info("parse_only is true; no planning or motion will be executed.")
            return 0

        planned_stages = []
        planning_state = self._load_cube_state(cube_state_path)
        for stage_index, stage in enumerate(stages, start=1):
            target_state, action_plan = self._plan_stage(
                stage_index,
                planning_state,
                stage["goals"],
            )
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

        if bool(self.get_parameter("plan_only").value):
            self.get_logger().info(
                "Demo 4 planned_stages: %s"
                % json.dumps(planned_stages, ensure_ascii=False, sort_keys=True)
            )
            self.get_logger().info("plan_only is true; no robot motion will be executed.")
            return 0

        self.get_logger().info(
            "Demo 4 executing planned_stages: %s"
            % json.dumps(planned_stages, ensure_ascii=False, sort_keys=True)
        )
        if bool(self.get_parameter("confirm_before_execute").value):
            answer = input("Execute this Demo 4 plan? [y/N] ").strip().lower()
            if answer not in ("y", "yes"):
                self.get_logger().info("Demo 4 execution cancelled by user.")
                return 0

        for stage_index, stage in enumerate(stages, start=1):
            current_state = self._load_cube_state(cube_state_path)
            _target_state, action_plan = self._plan_stage(
                stage_index,
                current_state,
                stage["goals"],
            )
            if not action_plan:
                self.get_logger().info(
                    "Demo 4 stage %d: requested state is already satisfied."
                    % stage_index
                )
                continue

            for action_index, action in enumerate(action_plan, start=1):
                self.get_logger().info(
                    "Demo 4 executing stage %d action %d/%d: cube=%s, from=%s, to=%s, reason=%s"
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
                        "Demo 4 stopped after failed stage %d action %d."
                        % (stage_index, action_index)
                    )
                    return exit_code

        self.get_logger().info("Demo 4 action plan finished.")
        return 0

    def _plan_stage(self, stage_index, current_state, goal_constraints):
        self.get_logger().info(
            "Demo 4 stage %d current_state: %s"
            % (stage_index, json.dumps(current_state, ensure_ascii=False, sort_keys=True))
        )
        self.get_logger().info(
            "Demo 4 stage %d goal_constraints: %s"
            % (stage_index, json.dumps(goal_constraints, ensure_ascii=False))
        )
        try:
            target_state, action_plan = plan_actions(current_state, goal_constraints)
        except PlanningError as exc:
            raise RuntimeError("Stage %d planning failed: %s" % (stage_index, exc)) from exc

        self.get_logger().info(
            "Demo 4 stage %d target_state: %s"
            % (stage_index, json.dumps(target_state, ensure_ascii=False, sort_keys=True))
        )
        self.get_logger().info(
            "Demo 4 stage %d action_plan: %s"
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
            "Demo 4 parsed stages: %s" % json.dumps(stages, ensure_ascii=False)
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
    node = Demo4()
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

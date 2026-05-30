import json
import os
import sys

import rclpy
from rclpy.parameter import Parameter

from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo
from tm5_color_pick_demo.llm_command_parser import LlmCommandParser


class Demo3(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("demo_3")
        self.declare_parameter("parse_only", False)
        self.declare_parameter("deepseek_api_key", "")
        self.declare_parameter("deepseek_base_url", "https://api.deepseek.com")
        self.declare_parameter("deepseek_model", "deepseek-v4-flash")
        self.declare_parameter("deepseek_timeout_sec", 30.0)

    def run(self):
        return self._run_interactive()

    def _run_interactive(self):
        self.get_logger().info("Demo 3 interactive mode. Enter q to quit.")
        while rclpy.ok():
            try:
                command = input("demo_3> ").strip()
            except EOFError:
                return 0

            if command.lower() in ("q", "quit", "exit"):
                return 0
            if not command:
                continue

            try:
                tasks = self._parse_command(command)
                exit_code = self._run_tasks(tasks)
            except Exception as exc:
                self.get_logger().error(str(exc))
                continue

            if exit_code != 0:
                return exit_code

        return 0

    def _parse_command(self, command):
        parser = LlmCommandParser(
            api_key=self._deepseek_api_key(),
            base_url=str(self.get_parameter("deepseek_base_url").value).strip(),
            model=str(self.get_parameter("deepseek_model").value).strip(),
            timeout_sec=float(self.get_parameter("deepseek_timeout_sec").value),
        )
        tasks = parser.parse(command)
        self.get_logger().info(
            "Demo 3 parsed tasks: %s" % json.dumps(tasks, ensure_ascii=False)
        )
        return tasks

    def _run_tasks(self, tasks):
        if bool(self.get_parameter("parse_only").value):
            self.get_logger().info("parse_only is true; no robot motion will be executed.")
            return 0

        for index, task in enumerate(tasks, start=1):
            self.get_logger().info(
                "Demo 3 executing task %d/%d: cube=%s, place_cell=%s"
                % (index, len(tasks), task["cube"], task["place_cell"])
            )
            self._set_task_parameters(task["cube"], task["place_cell"])
            exit_code = super().run()
            if exit_code != 0:
                self.get_logger().error("Demo 3 stopped after failed task %d." % index)
                return exit_code
        return 0

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
    node = Demo3()
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

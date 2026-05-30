import sys

import rclpy
from rclpy.parameter import Parameter

from tm5_color_pick_demo.command_parser import parse_command
from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo


class Demo2(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("demo_2")
        self.declare_parameter("command", "")
        self.declare_parameter("interactive", False)

    def run(self):
        if bool(self.get_parameter("interactive").value):
            return self._run_interactive()

        command = str(self.get_parameter("command").value).strip()
        if command:
            cube, place_cell = parse_command(command)
            self._set_task_parameters(cube, place_cell)
            self.get_logger().info(
                "Demo 2 parsed command='%s' -> cube=%s, place_cell=%s"
                % (command, cube, place_cell)
            )
        else:
            cube = str(self.get_parameter("cube").value).strip().lower()
            place_cell = str(self.get_parameter("place_cell").value).strip()
            self.get_logger().info(
                "Demo 2 using parameters -> cube=%s, place_cell=%s"
                % (cube, place_cell)
            )

        return super().run()

    def _run_interactive(self):
        self.get_logger().info("Demo 2 interactive mode. Enter q to quit.")
        while rclpy.ok():
            try:
                command = input("demo_2> ").strip()
            except EOFError:
                return 0

            if command.lower() in ("q", "quit", "exit"):
                return 0
            if not command:
                continue

            try:
                cube, place_cell = parse_command(command)
            except ValueError as exc:
                self.get_logger().error(str(exc))
                continue

            self._set_task_parameters(cube, place_cell)
            self.get_logger().info(
                "Demo 2 parsed command='%s' -> cube=%s, place_cell=%s"
                % (command, cube, place_cell)
            )
            exit_code = super().run()
            if exit_code != 0:
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


def main(args=None):
    rclpy.init(args=args)
    node = Demo2()
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

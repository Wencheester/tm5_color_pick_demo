import sys

import rclpy
from rclpy.parameter import Parameter

from tm5_color_pick_demo.command_parser import parse_command
from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo


class LanguagePickPlaceDemo(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("language_pick_place_demo")
        self.declare_parameter("command", "")

    def run(self):
        command = str(self.get_parameter("command").value).strip()
        if not command:
            raise RuntimeError("Parameter 'command' must not be empty.")

        cube, place_cell = parse_command(command)
        self.set_parameters(
            [
                Parameter("cube", Parameter.Type.STRING, cube),
                Parameter("place_cell", Parameter.Type.STRING, place_cell),
            ]
        )
        self.get_logger().info(
            "Parsed command='%s' -> cube=%s, place_cell=%s"
            % (command, cube, place_cell)
        )
        return super().run()


def main(args=None):
    rclpy.init(args=args)
    node = LanguagePickPlaceDemo()
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

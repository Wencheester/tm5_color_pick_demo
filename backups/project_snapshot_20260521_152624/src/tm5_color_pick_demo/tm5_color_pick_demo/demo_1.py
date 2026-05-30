import sys

import rclpy
from rclpy.parameter import Parameter
import yaml

from tm5_color_pick_demo.cube_pick_place_demo import CubePickPlaceDemo
from tm5_color_pick_demo.reset_cube_state import INITIAL_CUBE_STATE


class Demo1(CubePickPlaceDemo):
    def __init__(self):
        super().__init__("demo_1")

    def run(self):
        cube_state_path = self._cube_state_path()
        with open(cube_state_path, "w") as file:
            yaml.safe_dump(INITIAL_CUBE_STATE, file, sort_keys=False)

        self.set_parameters(
            [
                Parameter("cube", Parameter.Type.STRING, "red"),
                Parameter("place_cell", Parameter.Type.STRING, "right_bottom"),
                Parameter("pick_pose_group", Parameter.Type.STRING, "attach_place"),
                Parameter("place_pose_group", Parameter.Type.STRING, "detach_place"),
            ]
        )
        self.get_logger().info("Demo 1: reset cube state to initial layout.")
        self.get_logger().info("Demo 1: moving red cube from left_top to right_bottom.")
        return super().run()


def main(args=None):
    rclpy.init(args=args)
    node = Demo1()
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

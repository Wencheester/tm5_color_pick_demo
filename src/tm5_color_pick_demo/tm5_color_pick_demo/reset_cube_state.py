import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import yaml


INITIAL_CUBE_STATE = {
    "red": "left_top",
    "yellow": "left_middle",
    "blue": "left_bottom",
}


class ResetCubeState(Node):
    def __init__(self):
        super().__init__("reset_cube_state")
        self.declare_parameter("cube_state_path", "")

    def run(self):
        path = self._cube_state_path()
        with open(path, "w") as file:
            yaml.safe_dump(INITIAL_CUBE_STATE, file, sort_keys=False)
        self.get_logger().info("Reset cube state file: %s" % path)
        self.get_logger().info("Cube state: %s" % INITIAL_CUBE_STATE)
        return 0

    def _cube_state_path(self):
        configured = str(self.get_parameter("cube_state_path").value).strip()
        if configured:
            return os.path.expanduser(configured)
        package_dir = get_package_share_directory("tm5_color_pick_demo")
        return os.path.join(package_dir, "config", "cube_state.yaml")


def main(args=None):
    rclpy.init(args=args)
    node = ResetCubeState()
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

import sys

import rclpy

from tm5_color_pick_demo.red_pick_place_demo import RedPickPlaceDemo


class MoveToGridCell(RedPickPlaceDemo):
    def __init__(self, node_name="move_to_grid_cell"):
        super().__init__(node_name)
        self.declare_parameter("cell", "right_bottom")
        self.declare_parameter("pose_group", "")

    def run(self):
        cell = str(self.get_parameter("cell").value).strip()
        pose_group = str(self.get_parameter("pose_group").value).strip()
        if pose_group:
            target = self._load_grouped_grid_joint_pose(pose_group, cell)
            step_name = "grid_%s_%s" % (pose_group, cell)
            self.get_logger().info("Moving to grid cell: %s/%s" % (pose_group, cell))
        else:
            target = self._load_grid_joint_pose(cell)
            step_name = "grid_%s" % cell
            self.get_logger().info("Moving to grid cell: %s" % cell)

        self.get_logger().info("Waiting for MoveIt action server /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server /move_action is not available.")
            return 1

        return self._move_to_target(step_name, target)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGridCell()
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

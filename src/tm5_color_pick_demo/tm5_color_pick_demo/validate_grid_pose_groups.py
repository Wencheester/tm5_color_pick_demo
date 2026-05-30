import sys

import rclpy

from tm5_color_pick_demo.move_to_grid_cell import MoveToGridCell


DEFAULT_CELLS = (
    "left_top",
    "left_middle",
    "left_bottom",
    "center_top",
    "center_middle",
    "center_bottom",
    "right_top",
    "right_middle",
    "right_bottom",
)


class ValidateGridPoseGroups(MoveToGridCell):
    def __init__(self):
        super().__init__("validate_grid_pose_groups")
        self.declare_parameter("pose_groups", "attach_place,detach_place")
        self.declare_parameter("cells", ",".join(DEFAULT_CELLS))
        self.declare_parameter("return_ready_between_poses", True)

    def run(self):
        pose_groups = self._split_csv(self.get_parameter("pose_groups").value)
        cells = self._split_csv(self.get_parameter("cells").value)
        return_ready = bool(self.get_parameter("return_ready_between_poses").value)
        ready = self._load_ready_pose()

        self.get_logger().info("Waiting for MoveIt action server /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server /move_action is not available.")
            return 1

        failures = []
        for pose_group in pose_groups:
            for cell in cells:
                step_name = "%s_%s" % (pose_group, cell)
                self.get_logger().info("Validating %s ..." % step_name)
                target = self._load_grouped_grid_joint_pose(pose_group, cell)
                if self._move_to_target(step_name, target) != 0:
                    failures.append(step_name)
                    continue
                if return_ready and self._move_to_target("ready_after_%s" % step_name, ready) != 0:
                    failures.append("ready_after_%s" % step_name)

        if failures:
            self.get_logger().error("Grid pose validation failed: %s" % ", ".join(failures))
            return 1

        self.get_logger().info("All requested grid poses reached successfully.")
        return 0

    def _split_csv(self, value):
        return [item.strip() for item in str(value).split(",") if item.strip()]


def main(args=None):
    rclpy.init(args=args)
    node = ValidateGridPoseGroups()
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

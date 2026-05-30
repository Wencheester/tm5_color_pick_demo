import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from std_srvs.srv import Trigger
import yaml

from tm5_color_pick_demo.red_pick_place_demo import RedPickPlaceDemo


VALID_CUBES = ("red", "yellow", "blue")


class CubePickPlaceDemo(RedPickPlaceDemo):
    def __init__(self, node_name="cube_pick_place_demo"):
        super().__init__(node_name)
        self.declare_parameter("cube", "red")
        self.declare_parameter("cube_state_path", "")
        self.declare_parameter("pick_pose_group", "attach_place")
        self.declare_parameter("place_pose_group", "detach_place")

    def run(self):
        cube = str(self.get_parameter("cube").value).strip().lower()
        place_cell = str(self.get_parameter("place_cell").value).strip()
        cube_state_path = self._cube_state_path()
        cube_state = self._load_cube_state(cube_state_path)

        self._validate_cube(cube, cube_state)
        pick_cell = str(cube_state[cube]).strip()

        ready = self._load_ready_pose()
        pick_pose_group = str(self.get_parameter("pick_pose_group").value).strip()
        place_pose_group = str(self.get_parameter("place_pose_group").value).strip()
        pick = self._load_grouped_grid_joint_pose(pick_pose_group, pick_cell)
        place = self._load_grouped_grid_joint_pose(place_pose_group, place_cell)
        place["joint_settle_tolerance_rad"] = float(
            self.get_parameter("place_settle_tolerance_rad").value
        )

        self.get_logger().info(
            "Selected cube=%s, pick_cell=%s/%s, place_cell=%s/%s, cube_state=%s"
            % (
                cube,
                pick_pose_group,
                pick_cell,
                place_pose_group,
                place_cell,
                cube_state_path,
            )
        )
        if pick_cell == place_cell:
            self.get_logger().info(
                "Cube %s is already recorded at %s. Nothing to do."
                % (cube, place_cell)
            )
            return 0

        self._attach_client = self.create_client(Trigger, f"/suction/attach_{cube}")
        self._detach_client = self.create_client(Trigger, f"/suction/detach_{cube}")
        attach_service = f"/suction/attach_{cube}"
        detach_service = f"/suction/detach_{cube}"

        sequence = [
            ("ready", ready, None),
            ("pick_%s_%s" % (cube, pick["cell"]), pick, "attach"),
            ("ready_with_%s" % cube, ready, None),
            ("place_%s_%s" % (cube, place["cell"]), place, "detach"),
            ("ready_done", ready, None),
        ]

        self.get_logger().info("Waiting for MoveIt action server /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server /move_action is not available.")
            return 1

        if not self._wait_for_named_suction_services(
            [(attach_service, self._attach_client), (detach_service, self._detach_client)]
        ):
            return 1

        attached = False
        for step_name, target, service_after_motion in sequence:
            if self._move_to_target(step_name, target) != 0:
                if attached:
                    self._cleanup_attached_cube(cube, detach_service)
                return 1
            if service_after_motion == "attach" and not self._call_trigger(
                self._attach_client, attach_service
            ):
                return 1
            if service_after_motion == "attach":
                attached = True
            if service_after_motion == "detach" and not self._call_trigger(
                self._detach_client, detach_service
            ):
                return 1
            if service_after_motion == "detach":
                self._wait_after_detach(cube)
                attached = False

        cube_state[cube] = place_cell
        self._write_cube_state(cube_state_path, cube_state)
        self.get_logger().info("Updated cube state: %s" % cube_state)
        self.get_logger().info("Cube pick-place demo finished.")
        return 0

    def _cube_state_path(self):
        configured = str(self.get_parameter("cube_state_path").value).strip()
        if configured:
            return os.path.expanduser(configured)
        package_dir = get_package_share_directory("tm5_color_pick_demo")
        return os.path.join(package_dir, "config", "cube_state.yaml")

    def _load_cube_state(self, path):
        with open(path, "r") as file:
            data = yaml.safe_load(file) or {}
        if not isinstance(data, dict):
            raise RuntimeError("cube_state.yaml must contain a mapping of cube to grid cell.")
        return {str(key).strip().lower(): str(value).strip() for key, value in data.items()}

    def _cleanup_attached_cube(self, cube, detach_service):
        self.get_logger().error(
            "Motion failed while cube %s is attached. Calling %s before aborting."
            % (cube, detach_service)
        )
        if not self._call_trigger(self._detach_client, detach_service):
            self.get_logger().error(
                "Failed to detach cube %s during abort cleanup. Check Gazebo state manually."
                % cube
            )
            return
        self.get_logger().info("Detached cube %s during abort cleanup." % cube)

    def _wait_after_detach(self, cube):
        settle_sec = float(self.get_parameter("detach_settle_sec").value)
        if settle_sec <= 0.0:
            return
        self.get_logger().info(
            "Waiting %.2f sec after detaching cube %s before next motion."
            % (settle_sec, cube)
        )
        time.sleep(settle_sec)

    def _write_cube_state(self, path, cube_state):
        ordered = {cube: cube_state[cube] for cube in VALID_CUBES if cube in cube_state}
        for cube in sorted(cube_state):
            if cube not in ordered:
                ordered[cube] = cube_state[cube]
        with open(path, "w") as file:
            yaml.safe_dump(ordered, file, sort_keys=False, allow_unicode=True)

    def _validate_cube(self, cube, cube_state):
        if cube not in VALID_CUBES:
            raise RuntimeError(
                "Unknown cube '%s'. Available cubes: %s" % (cube, ", ".join(VALID_CUBES))
            )
        if cube not in cube_state:
            raise RuntimeError("Cube '%s' is missing from cube_state.yaml." % cube)

    def _wait_for_named_suction_services(self, services):
        for name, client in services:
            self.get_logger().info(f"Waiting for service {name} ...")
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service {name} is not available.")
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CubePickPlaceDemo()
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

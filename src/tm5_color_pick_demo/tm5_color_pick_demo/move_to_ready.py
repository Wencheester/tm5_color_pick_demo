import math
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class MoveToReady(Node):
    def __init__(self):
        super().__init__("move_to_ready")
        self._move_group = ActionClient(self, MoveGroup, "move_action")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.declare_parameter("pose_name", "ready")
        self.declare_parameter("group_name", "tmr_arm")
        self.declare_parameter("tolerance_m", 0.01)
        self.declare_parameter("orientation_tolerance_rad", 0.05)
        self.declare_parameter("allowed_planning_time", 10.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("velocity_scaling", 0.2)
        self.declare_parameter("acceleration_scaling", 0.2)
        self.declare_parameter("verify_timeout_sec", 8.0)

    def run(self):
        pose_name = self.get_parameter("pose_name").value
        target = self._load_pose(pose_name)

        self.get_logger().info("Waiting for MoveIt action server /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server /move_action is not available.")
            return 1

        goal = MoveGroup.Goal()
        goal.request = self._build_motion_request(target)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 1.0

        self.get_logger().info(
            "Planning and executing %s: %s -> %s at [%.3f, %.3f, %.3f]"
            % (
                pose_name,
                target["frame_id"],
                target["end_effector_link"],
                *target["xyz"],
            )
        )
        self.get_logger().info(
            "Ready orientation xyzw: [%.3f, %.3f, %.3f, %.3f]"
            % tuple(target["xyzw"])
        )

        send_future = self._move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveIt rejected the ready motion goal.")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveIt execution failed with error code {error_code}.")
            return 1

        self.get_logger().info("MoveIt reported successful execution.")
        return self._verify_link_position(target)

    def _load_pose(self, pose_name):
        package_dir = get_package_share_directory("tm5_color_pick_demo")
        path = f"{package_dir}/config/work_poses.yaml"
        with open(path, "r") as file:
            data = yaml.safe_load(file)

        pose = data["work_poses"][pose_name]
        xyz = pose["pose"]["position_m"]["xyz"]
        xyzw = pose["pose"]["orientation"]["xyzw"]
        if xyzw is None:
            raise RuntimeError(f"Pose '{pose_name}' has no resolved orientation.xyzw")
        return {
            "frame_id": pose["frame_id"],
            "end_effector_link": pose["end_effector_link"],
            "verify_link": pose.get("coincident_link", pose["end_effector_link"]),
            "xyz": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
            "xyzw": [float(xyzw[0]), float(xyzw[1]), float(xyzw[2]), float(xyzw[3])],
        }

    def _build_motion_request(self, target):
        tolerance = float(self.get_parameter("tolerance_m").value)

        target_pose = Pose()
        target_pose.position.x = target["xyz"][0]
        target_pose.position.y = target["xyz"][1]
        target_pose.position.z = target["xyz"][2]
        target_pose.orientation.x = target["xyzw"][0]
        target_pose.orientation.y = target["xyzw"][1]
        target_pose.orientation.z = target["xyzw"][2]
        target_pose.orientation.w = target["xyzw"][3]

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [tolerance]

        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [target_pose]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target["frame_id"]
        position_constraint.link_name = target["end_effector_link"]
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_tolerance = float(self.get_parameter("orientation_tolerance_rad").value)
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target["frame_id"]
        orientation_constraint.link_name = target["end_effector_link"]
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = "ready_pose"
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]

        request = MotionPlanRequest()
        request.group_name = self.get_parameter("group_name").value
        request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
        request.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        request.max_velocity_scaling_factor = float(self.get_parameter("velocity_scaling").value)
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value
        )
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        request.workspace_parameters.header.frame_id = target["frame_id"]
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -0.1
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.5
        return request

    def _verify_link_position(self, target):
        timeout = float(self.get_parameter("verify_timeout_sec").value)
        deadline = time.monotonic() + timeout
        last_error = None

        while time.monotonic() < deadline:
            try:
                transform = self._tf_buffer.lookup_transform(
                    target["frame_id"],
                    target["verify_link"],
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                actual = [translation.x, translation.y, translation.z]
                error = math.sqrt(
                    sum((actual[i] - target["xyz"][i]) ** 2 for i in range(3))
                )
                actual_q = [rotation.x, rotation.y, rotation.z, rotation.w]
                orientation_error = quaternion_angular_distance(actual_q, target["xyzw"])
                self.get_logger().info(
                    "%s position in %s: [%.4f, %.4f, %.4f], target: [%.4f, %.4f, %.4f], error: %.4f m"
                    % (
                        target["verify_link"],
                        target["frame_id"],
                        actual[0],
                        actual[1],
                        actual[2],
                        target["xyz"][0],
                        target["xyz"][1],
                        target["xyz"][2],
                        error,
                    )
                )
                self.get_logger().info(
                    "%s orientation xyzw: [%.4f, %.4f, %.4f, %.4f], target: [%.4f, %.4f, %.4f, %.4f], angular error: %.4f rad"
                    % (
                        target["verify_link"],
                        actual_q[0],
                        actual_q[1],
                        actual_q[2],
                        actual_q[3],
                        target["xyzw"][0],
                        target["xyzw"][1],
                        target["xyzw"][2],
                        target["xyzw"][3],
                        orientation_error,
                    )
                )
                if (
                    error <= float(self.get_parameter("tolerance_m").value)
                    and orientation_error
                    <= float(self.get_parameter("orientation_tolerance_rad").value)
                ):
                    return 0
                self.get_logger().error("Ready pose verification failed.")
                return 1
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(f"Could not verify ready transform: {last_error}")
        return 1


def quaternion_angular_distance(q1, q2):
    dot = abs(sum(q1[i] * q2[i] for i in range(4)))
    dot = max(-1.0, min(1.0, dot))
    return 2.0 * math.acos(dot)


def main(args=None):
    rclpy.init(args=args)
    node = MoveToReady()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

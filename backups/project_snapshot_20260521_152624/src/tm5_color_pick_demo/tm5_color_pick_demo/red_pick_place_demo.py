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
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


class RedPickPlaceDemo(Node):
    def __init__(self, node_name="red_pick_place_demo"):
        super().__init__(node_name)
        self._move_group = ActionClient(self, MoveGroup, "move_action")
        self._attach_client = self.create_client(Trigger, "/suction/attach")
        self._detach_client = self.create_client(Trigger, "/suction/detach")
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._joint_positions = {}
        self._joint_velocities = {}
        self.create_subscription(JointState, "/joint_states", self._joint_state_callback, 10)

        self.declare_parameter("group_name", "tmr_arm")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("ready_pose_name", "ready")
        self.declare_parameter("ready_link", "link_6")
        self.declare_parameter("ready_verify_link", "link_6")
        self.declare_parameter("task_link", "suction_cup_link")
        self.declare_parameter("pick_cell", "left_top")
        self.declare_parameter("place_cell", "right_top")
        self.declare_parameter("tolerance_m", 0.025)
        self.declare_parameter("joint_tolerance_rad", 0.06)
        self.declare_parameter("joint_settle_tolerance_rad", 0.06)
        self.declare_parameter("place_settle_tolerance_rad", 0.12)
        self.declare_parameter("joint_settle_timeout_sec", 8.0)
        self.declare_parameter("joint_stable_sample_count", 5)
        self.declare_parameter("joint_stable_sample_period_sec", 0.1)
        self.declare_parameter("joint_velocity_tolerance_rad_s", 0.03)
        self.declare_parameter("detach_settle_sec", 2.0)
        self.declare_parameter("orientation_tolerance_rad", 0.08)
        self.declare_parameter("allowed_planning_time", 10.0)
        self.declare_parameter("planning_attempts", 10)
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("acceleration_scaling", 0.15)
        self.declare_parameter("verify_timeout_sec", 8.0)

    def run(self):
        ready = self._load_ready_pose()
        pick = self._load_grid_joint_pose(self.get_parameter("pick_cell").value)
        place = self._load_grid_joint_pose(self.get_parameter("place_cell").value)

        sequence = [
            ("ready", ready, None),
            ("pick_%s" % pick["cell"], pick, "attach"),
            ("ready_with_red", ready, None),
            ("place_%s" % place["cell"], place, "detach"),
            ("ready_done", ready, None),
        ]

        self.get_logger().info("Waiting for MoveIt action server /move_action ...")
        if not self._move_group.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("MoveIt action server /move_action is not available.")
            return 1

        if not self._wait_for_suction_services():
            return 1

        for step_name, target, service_after_motion in sequence:
            if self._move_to_target(step_name, target) != 0:
                return 1
            if service_after_motion == "attach" and not self._call_trigger(
                self._attach_client, "/suction/attach"
            ):
                return 1
            if service_after_motion == "detach" and not self._call_trigger(
                self._detach_client, "/suction/detach"
            ):
                return 1

        self.get_logger().info("Red pick-place demo finished.")
        return 0

    def _load_ready_pose(self):
        package_dir = get_package_share_directory("tm5_color_pick_demo")
        path = f"{package_dir}/config/work_poses.yaml"
        with open(path, "r") as file:
            data = yaml.safe_load(file)

        pose = data["work_poses"][self.get_parameter("ready_pose_name").value]
        joints = pose.get("joint_positions", {})
        xyz = pose["pose"]["position_m"]["xyz"]
        xyzw = pose["pose"]["orientation"]["xyzw"]
        if xyzw is None:
            raise RuntimeError("Ready pose has no resolved orientation.xyzw")
        return {
            "name": "ready",
            "type": "joint" if joints else "pose",
            "frame_id": pose["frame_id"],
            "link": self.get_parameter("ready_link").value,
            "verify_link": self.get_parameter("ready_verify_link").value,
            "joints": {name: float(value) for name, value in joints.items()},
            "xyz": [float(xyz[0]), float(xyz[1]), float(xyz[2])],
            "xyzw": [float(xyzw[0]), float(xyzw[1]), float(xyzw[2]), float(xyzw[3])],
        }

    def _make_task_target(self, name, xy, suction_z, ready):
        return {
            "name": name,
            "type": "pose",
            "frame_id": self.get_parameter("frame_id").value,
            "link": self.get_parameter("task_link").value,
            "verify_link": self.get_parameter("task_link").value,
            "xyz": [float(xy[0]), float(xy[1]), float(suction_z)],
            "xyzw": list(ready["xyzw"]),
        }

    def _load_grid_joint_pose(self, cell_name):
        package_dir = get_package_share_directory("tm5_color_pick_demo")
        path = f"{package_dir}/config/grid_joint_poses.yaml"
        with open(path, "r") as file:
            data = yaml.safe_load(file)

        poses = data["grid_joint_poses"]
        if cell_name not in poses:
            available = ", ".join(sorted(poses))
            raise RuntimeError(
                "Unknown grid cell '%s'. Available cells: %s" % (cell_name, available)
            )

        return {
            "name": cell_name,
            "cell": cell_name,
            "type": "joint",
            "frame_id": self.get_parameter("frame_id").value,
            "link": self.get_parameter("ready_link").value,
            "verify_link": self.get_parameter("ready_verify_link").value,
            "joints": {name: float(value) for name, value in poses[cell_name].items()},
            "xyz": [0.0, 0.0, 0.0],
            "xyzw": [0.0, 0.0, 0.0, 1.0],
        }

    def _load_grouped_grid_joint_pose(self, pose_group, cell_name):
        package_dir = get_package_share_directory("tm5_color_pick_demo")
        path = f"{package_dir}/config/grid_pose_groups.yaml"
        with open(path, "r") as file:
            data = yaml.safe_load(file)

        if pose_group not in data:
            available = ", ".join(sorted(data))
            raise RuntimeError(
                "Unknown pose group '%s'. Available groups: %s" % (pose_group, available)
            )

        poses = data[pose_group]
        if cell_name not in poses:
            available = ", ".join(sorted(poses))
            raise RuntimeError(
                "Unknown grid cell '%s' in group '%s'. Available cells: %s"
                % (cell_name, pose_group, available)
            )

        return {
            "name": "%s_%s" % (pose_group, cell_name),
            "cell": cell_name,
            "type": "joint",
            "frame_id": self.get_parameter("frame_id").value,
            "link": self.get_parameter("ready_link").value,
            "verify_link": self.get_parameter("ready_verify_link").value,
            "joints": {name: float(value) for name, value in poses[cell_name].items()},
            "xyz": [0.0, 0.0, 0.0],
            "xyzw": [0.0, 0.0, 0.0, 1.0],
        }

    def _move_to_target(self, step_name, target):
        goal = MoveGroup.Goal()
        goal.request = self._build_motion_request(target)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 1.0

        self.get_logger().info(
            self._describe_target(step_name, target)
        )

        send_future = self._move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"MoveIt rejected step {step_name}.")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        error_code = result.error_code.val
        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveIt execution failed at step {step_name} with error code {error_code}."
            )
            return 1

        if target.get("type") == "joint":
            return self._verify_joint_position(step_name, target)
        return self._verify_link_position(step_name, target)

    def _describe_target(self, step_name, target):
        if target.get("type") == "joint":
            joint_text = ", ".join(
                "%s=%.4f" % (name, target["joints"][name])
                for name in sorted(target["joints"])
            )
            return f"Step {step_name}: moving to joint target [{joint_text}]"
        return "Step %s: moving %s to [%.3f, %.3f, %.3f]" % (
            step_name,
            target["link"],
            target["xyz"][0],
            target["xyz"][1],
            target["xyz"][2],
        )

    def _build_motion_request(self, target):
        if target.get("type") == "joint":
            return self._build_joint_motion_request(target)
        return self._build_pose_motion_request(target)

    def _build_joint_motion_request(self, target):
        tolerance = float(self.get_parameter("joint_tolerance_rad").value)
        joint_constraints = []
        for name in sorted(target["joints"]):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = target["joints"][name]
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance
            constraint.weight = 1.0
            joint_constraints.append(constraint)

        constraints = Constraints()
        constraints.name = target["name"]
        constraints.joint_constraints = joint_constraints

        request = self._make_base_motion_request()
        request.goal_constraints = [constraints]
        request.workspace_parameters.header.frame_id = target["frame_id"]
        return request

    def _build_pose_motion_request(self, target):
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
        sphere.dimensions = [float(self.get_parameter("tolerance_m").value)]

        region = BoundingVolume()
        region.primitives = [sphere]
        region.primitive_poses = [target_pose]

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target["frame_id"]
        position_constraint.link_name = target["link"]
        position_constraint.constraint_region = region
        position_constraint.weight = 1.0

        orientation_tolerance = float(self.get_parameter("orientation_tolerance_rad").value)
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target["frame_id"]
        orientation_constraint.link_name = target["link"]
        orientation_constraint.orientation = target_pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.name = target["name"]
        constraints.position_constraints = [position_constraint]
        constraints.orientation_constraints = [orientation_constraint]

        request = self._make_base_motion_request()
        request.goal_constraints = [constraints]
        request.workspace_parameters.header.frame_id = target["frame_id"]
        return request

    def _make_base_motion_request(self):
        request = MotionPlanRequest()
        request.group_name = self.get_parameter("group_name").value
        request.num_planning_attempts = int(self.get_parameter("planning_attempts").value)
        request.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        request.max_velocity_scaling_factor = float(self.get_parameter("velocity_scaling").value)
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value
        )
        request.start_state.is_diff = True
        request.workspace_parameters.min_corner.x = -1.0
        request.workspace_parameters.min_corner.y = -1.0
        request.workspace_parameters.min_corner.z = -0.1
        request.workspace_parameters.max_corner.x = 1.0
        request.workspace_parameters.max_corner.y = 1.0
        request.workspace_parameters.max_corner.z = 1.5
        return request

    def _verify_link_position(self, step_name, target):
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
                actual = [translation.x, translation.y, translation.z]
                error = math.sqrt(
                    sum((actual[i] - target["xyz"][i]) ** 2 for i in range(3))
                )
                self.get_logger().info(
                    "Step %s: %s actual [%.4f, %.4f, %.4f], target [%.4f, %.4f, %.4f], error %.4f m"
                    % (
                        step_name,
                        target["verify_link"],
                        actual[0],
                        actual[1],
                        actual[2],
                        target["xyz"][0],
                        target["xyz"][1],
                        target["xyz"][2],
                        error,
                    )
                )
                if error <= float(self.get_parameter("tolerance_m").value):
                    return 0
                self.get_logger().error(f"Step {step_name} position verification failed.")
                return 1
            except TransformException as exc:
                last_error = exc
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(f"Could not verify step {step_name}: {last_error}")
        return 1

    def _joint_state_callback(self, msg):
        self._joint_positions.update(
            {name: position for name, position in zip(msg.name, msg.position)}
        )
        if msg.velocity:
            self._joint_velocities.update(
                {name: velocity for name, velocity in zip(msg.name, msg.velocity)}
            )

    def _verify_joint_position(self, step_name, target):
        timeout = float(self.get_parameter("joint_settle_timeout_sec").value)
        tolerance = float(
            target.get(
                "joint_settle_tolerance_rad",
                self.get_parameter("joint_settle_tolerance_rad").value,
            )
        )
        deadline = time.monotonic() + timeout
        missing = []
        errors = {}
        max_error = None
        stable_samples = 0
        required_stable_samples = max(
            1, int(self.get_parameter("joint_stable_sample_count").value)
        )
        sample_period = max(
            0.0, float(self.get_parameter("joint_stable_sample_period_sec").value)
        )
        velocity_tolerance = float(
            self.get_parameter("joint_velocity_tolerance_rad_s").value
        )

        while time.monotonic() < deadline:
            missing = [name for name in target["joints"] if name not in self._joint_positions]
            if not missing:
                max_error = 0.0
                errors = {}
                max_velocity = 0.0
                for name in sorted(target["joints"]):
                    actual = self._joint_positions[name]
                    desired = target["joints"][name]
                    error = abs(actual - desired)
                    errors[name] = (actual, desired, error)
                    max_error = max(max_error, error)
                    max_velocity = max(
                        max_velocity,
                        abs(self._joint_velocities.get(name, 0.0)),
                    )
                if max_error <= tolerance and max_velocity <= velocity_tolerance:
                    stable_samples += 1
                else:
                    stable_samples = 0

                if stable_samples >= required_stable_samples:
                    for name in sorted(errors):
                        actual, desired, error = errors[name]
                        self.get_logger().info(
                            "Step %s: %s actual %.4f, target %.4f, error %.4f rad"
                            % (step_name, name, actual, desired, error)
                        )
                    self.get_logger().info(
                        "Step %s: joint target stable; max observed joint error %.4f rad <= %.4f rad, max velocity %.4f rad/s <= %.4f rad/s for %d samples"
                        % (
                            step_name,
                            max_error,
                            tolerance,
                            max_velocity,
                            velocity_tolerance,
                            stable_samples,
                        )
                    )
                    return 0
            rclpy.spin_once(self, timeout_sec=sample_period)

        if missing:
            self.get_logger().error(
                "Step %s: could not verify joint target; missing joints: %s"
                % (step_name, ", ".join(missing))
            )
            return 1

        for name in sorted(errors):
            actual, desired, error = errors[name]
            self.get_logger().error(
                "Step %s: %s actual %.4f, target %.4f, error %.4f rad"
                % (step_name, name, actual, desired, error)
            )
        self.get_logger().error(
            "Step %s: joint target did not settle within %.4f rad before %.1f sec timeout; max observed joint error %.4f rad"
            % (step_name, tolerance, timeout, max_error if max_error is not None else -1.0)
        )
        return 1

    def _wait_for_suction_services(self):
        for name, client in [
            ("/suction/attach", self._attach_client),
            ("/suction/detach", self._detach_client),
        ]:
            self.get_logger().info(f"Waiting for service {name} ...")
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().error(f"Service {name} is not available.")
                return False
        return True

    def _call_trigger(self, client, service_name):
        self.get_logger().info(f"Calling {service_name} ...")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error(f"{service_name} call failed with no response.")
            return False
        if not response.success:
            self.get_logger().error(f"{service_name} failed: {response.message}")
            return False
        self.get_logger().info(f"{service_name} succeeded: {response.message}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RedPickPlaceDemo()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

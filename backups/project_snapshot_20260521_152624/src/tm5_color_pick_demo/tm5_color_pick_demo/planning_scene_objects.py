import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PlanningSceneObjects(Node):
    def __init__(self):
        super().__init__("planning_scene_objects")
        self._client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("table_id", "pick_table")
        self.declare_parameter("table_center_xyz", [0.50, 0.0, 0.19])
        self.declare_parameter("table_size_xyz", [0.5, 0.5, 0.02])
        self.declare_parameter("service_timeout_sec", 15.0)

    def run(self):
        timeout = float(self.get_parameter("service_timeout_sec").value)
        self.get_logger().info("Waiting for /apply_planning_scene ...")
        if not self._client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error("/apply_planning_scene is not available.")
            return 1

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [self._make_table_object()]

        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("ApplyPlanningScene returned no response.")
            return 1
        if not response.success:
            self.get_logger().error("MoveIt rejected the pick_table planning scene object.")
            return 1

        self.get_logger().info("Added pick_table collision object to MoveIt planning scene.")
        return 0

    def _make_table_object(self):
        size = [float(v) for v in self.get_parameter("table_size_xyz").value]
        center = [float(v) for v in self.get_parameter("table_center_xyz").value]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.get_parameter("frame_id").value
        collision_object.id = self.get_parameter("table_id").value
        collision_object.primitives = [primitive]
        collision_object.primitive_poses = [pose]
        collision_object.operation = CollisionObject.ADD
        return collision_object


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneObjects()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

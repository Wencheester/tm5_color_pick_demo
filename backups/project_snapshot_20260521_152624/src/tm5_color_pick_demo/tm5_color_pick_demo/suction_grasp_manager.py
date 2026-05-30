import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class SuctionGraspManager(Node):
    def __init__(self):
        super().__init__("suction_grasp_manager")
        self.declare_parameter("world_name", "color_pick_world")
        self.declare_parameter("robot_model_name", "tm5-700")
        self.declare_parameter("model_name", "red_cube")
        self.declare_parameter("parent_link", "link_6")
        self.declare_parameter("child_link", "link")
        self.declare_parameter("attach_topic", "/suction/red_cube/attach")
        self.declare_parameter("detach_topic", "/suction/red_cube/detach")
        self.declare_parameter("state_topic", "/suction/red_cube/state")
        self.declare_parameter("plugin_name", "gz::sim::systems::DetachableJoint")
        self.declare_parameter("plugin_filename", "ignition-gazebo-detachable-joint-system")
        self.declare_parameter("red_center_target_xyz", [0.60, 0.0, 0.235])
        self.declare_parameter("gz_timeout_ms", 2000)
        self.declare_parameter("attach_retry_count", 3)
        self.declare_parameter("attach_retry_wait_sec", 1.0)
        self.declare_parameter("detach_publish_count", 3)
        self.declare_parameter("detach_retry_count", 3)
        self.declare_parameter("detach_retry_wait_sec", 1.0)
        self.declare_parameter("state_wait_timeout_sec", 3.0)

        self._cubes = {
            "red": {"model_name": "red_cube", "child_link": "link"},
            "yellow": {"model_name": "yellow_cube", "child_link": "link"},
            "blue": {"model_name": "blue_cube", "child_link": "link"},
        }
        self._cube_states = {
            cube: {"loaded": False, "attached": False, "gazebo_state": None}
            for cube in self._cubes
        }
        self._state_lock = threading.Lock()
        self._state_listeners = []

        self.create_service(Trigger, "/suction/attach", self._make_attach_callback("red"))
        self.create_service(Trigger, "/suction/detach", self._make_detach_callback("red"))
        for cube in sorted(self._cubes):
            self.create_service(
                Trigger,
                f"/suction/attach_{cube}",
                self._make_attach_callback(cube),
            )
            self.create_service(
                Trigger,
                f"/suction/detach_{cube}",
                self._make_detach_callback(cube),
            )
        self.create_service(Trigger, "/suction/move_red_to_center", self._move_red_to_center)
        self._start_state_listeners()

    def _make_attach_callback(self, cube):
        def callback(_request, response):
            return self._attach_cube(cube, response)

        return callback

    def _make_detach_callback(self, cube):
        def callback(_request, response):
            return self._detach_cube(cube, response)

        return callback

    def _attach_cube(self, cube, response):
        was_loaded = self._get_cube_state(cube, "loaded")
        ok, load_message = self._ensure_detachable_joint_system(cube)
        if not ok:
            response.success = False
            response.message = load_message
            self.get_logger().error(response.message)
            return response

        if self._get_cube_state(cube, "attached"):
            ok = True
            message = f"{cube} DetachableJoint is already attached."
        else:
            if not was_loaded:
                # Loading the DetachableJoint system creates the initial fixed
                # joint. Gazebo may not emit a fresh state message for this
                # implicit attach, so treat the successful load as attached.
                ok = True
                self._set_gazebo_state(cube, "attached")
                message = "DetachableJoint system load created the attached fixed joint."
            else:
                ok, message = self._publish_attach_until_state(cube)

        response.success = ok
        response.message = f"{load_message} {message}"
        if ok:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response

    def _detach_cube(self, cube, response):
        was_loaded = self._get_cube_state(cube, "loaded")
        if not was_loaded:
            ok, load_message = self._ensure_detachable_joint_system(cube)
            if not ok:
                response.success = False
                response.message = load_message
                self.get_logger().error(response.message)
                return response
        else:
            load_message = "DetachableJoint system is already loaded."

        if not was_loaded and not self._get_cube_state(cube, "attached"):
            if not self._wait_for_gazebo_state(cube, "attached"):
                response.success = False
                response.message = (
                    f"{load_message} Timed out waiting for Gazebo state 'attached'."
                )
                self.get_logger().error(response.message)
                return response

        if not self._get_cube_state(cube, "attached"):
            ok = True
            message = f"{cube} DetachableJoint is already detached."
        else:
            ok, message = self._publish_detach_until_state(cube)

        response.success = ok
        response.message = f"{load_message} {message}"
        if ok:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)
        return response

    def _move_red_to_center(self, _request, response):
        model_name = self.get_parameter("model_name").value
        xyz = [float(v) for v in self.get_parameter("red_center_target_xyz").value]
        ok, message = self._set_model_pose(model_name, xyz)
        response.success = ok
        response.message = message
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return response

    def _attach_topic(self, cube):
        return f"/suction/{self._cubes[cube]['model_name']}/attach"

    def _detach_topic(self, cube):
        return f"/suction/{self._cubes[cube]['model_name']}/detach"

    def _state_topic(self, cube):
        return f"/suction/{self._cubes[cube]['model_name']}/state"

    def _start_state_listeners(self):
        for cube in sorted(self._cubes):
            cmd = [
                "ign",
                "topic",
                "-e",
                "-t",
                self._state_topic(cube),
            ]
            try:
                process = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    text=True,
                    bufsize=1,
                )
            except OSError as exc:
                self.get_logger().warn(
                    "Could not start state listener for %s: %s" % (cube, exc)
                )
                continue

            thread = threading.Thread(
                target=self._read_state_listener,
                args=(cube, process),
                daemon=True,
            )
            thread.start()
            self._state_listeners.append((process, thread))

    def _read_state_listener(self, cube, process):
        if process.stdout is None:
            return
        for line in process.stdout:
            match = re.search(r'data:\s*"([^"]+)"', line)
            if not match:
                continue
            state = match.group(1).strip().lower()
            if state in ("attached", "detached"):
                self._set_gazebo_state(cube, state)
                self.get_logger().info("Gazebo state for %s: %s" % (cube, state))

    def _set_gazebo_state(self, cube, state):
        attached = state == "attached"
        self._set_cube_state(cube, attached=attached, gazebo_state=state)

    def _get_cube_state(self, cube, key):
        with self._state_lock:
            return self._cube_states[cube][key]

    def _set_cube_state(self, cube, **updates):
        with self._state_lock:
            self._cube_states[cube].update(updates)

    def _wait_for_gazebo_state(self, cube, expected_state):
        timeout = float(self.get_parameter("state_wait_timeout_sec").value)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._state_lock:
                state = self._cube_states[cube].get("gazebo_state")
            if state == expected_state:
                return True
            time.sleep(0.05)
        return False

    def _publish_attach_until_state(self, cube):
        retry_count = max(1, int(self.get_parameter("attach_retry_count").value))
        wait_sec = max(0.05, float(self.get_parameter("attach_retry_wait_sec").value))

        for attempt in range(1, retry_count + 1):
            ok, message = self._publish_empty_topic(self._attach_topic(cube))
            if not ok:
                return False, message

            if self._wait_for_gazebo_state_for(cube, "attached", wait_sec):
                self._set_cube_state(cube, attached=True)
                return (
                    True,
                    "Published attach for %s; Gazebo reported attached on attempt %d/%d."
                    % (cube, attempt, retry_count),
                )

            self.get_logger().warn(
                "Attach attempt %d/%d for %s did not report attached."
                % (attempt, retry_count, cube)
            )

        return False, "Timed out waiting for Gazebo state 'attached'."

    def _publish_detach_until_state(self, cube):
        retry_count = max(1, int(self.get_parameter("detach_retry_count").value))
        publish_count = max(1, int(self.get_parameter("detach_publish_count").value))
        wait_sec = max(0.05, float(self.get_parameter("detach_retry_wait_sec").value))

        for attempt in range(1, retry_count + 1):
            ok, message = self._publish_empty_topic(
                self._detach_topic(cube),
                publish_count,
            )
            if not ok:
                return False, message

            if self._wait_for_gazebo_state_for(cube, "detached", wait_sec):
                self._set_cube_state(cube, attached=False)
                return (
                    True,
                    "Published detach for %s; Gazebo reported detached on attempt %d/%d."
                    % (cube, attempt, retry_count),
                )

            self.get_logger().warn(
                "Detach attempt %d/%d for %s did not report detached."
                % (attempt, retry_count, cube)
            )

        return False, "Timed out waiting for Gazebo state 'detached'."

    def _wait_for_gazebo_state_for(self, cube, expected_state, timeout):
        deadline = time.monotonic() + float(timeout)
        while time.monotonic() < deadline:
            with self._state_lock:
                state = self._cube_states[cube].get("gazebo_state")
            if state == expected_state:
                return True
            time.sleep(0.05)
        return False

    def _ensure_detachable_joint_system(self, cube):
        if self._get_cube_state(cube, "loaded"):
            return True, "DetachableJoint system is already loaded."

        robot_model = self.get_parameter("robot_model_name").value
        entity_id, message = self._find_model_entity_id(robot_model)
        if entity_id is None:
            return False, message

        world_name = self.get_parameter("world_name").value
        timeout_ms = int(self.get_parameter("gz_timeout_ms").value)
        innerxml = (
            "<parent_link>{parent_link}</parent_link>"
            "<child_model>{child_model}</child_model>"
            "<child_link>{child_link}</child_link>"
            "<child_model_link>{child_link}</child_model_link>"
            "<attach_topic>{attach_topic}</attach_topic>"
            "<detach_topic>{detach_topic}</detach_topic>"
            "<output_topic>{state_topic}</output_topic>"
        ).format(
            parent_link=self.get_parameter("parent_link").value,
            child_model=self._cubes[cube]["model_name"],
            child_link=self._cubes[cube]["child_link"],
            attach_topic=self._attach_topic(cube),
            detach_topic=self._detach_topic(cube),
            state_topic=self._state_topic(cube),
        )
        request = (
            f"entity {{ id: {entity_id} type: MODEL }} "
            "plugins { "
            f'name: "{self.get_parameter("plugin_name").value}" '
            f'filename: "{self.get_parameter("plugin_filename").value}" '
            f'innerxml: "{innerxml}" '
            "}"
        )
        cmd = [
            "ign",
            "service",
            "-s",
            f"/world/{world_name}/entity/system/add",
            "--reqtype",
            "ignition.msgs.EntityPlugin_V",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            request,
        ]
        ok, output = self._run_command(cmd, timeout_ms)
        if not ok:
            return False, f"Failed to add DetachableJoint system: {output}"
        if "data: true" not in output and "true" not in output.lower():
            return False, f"Gazebo did not accept DetachableJoint system: {output}"

        self._set_cube_state(cube, loaded=True)
        return (
            True,
            "Loaded DetachableJoint for %s::%s -> %s::%s."
            % (
                robot_model,
                self.get_parameter("parent_link").value,
                self._cubes[cube]["model_name"],
                self._cubes[cube]["child_link"],
            ),
        )

    def _find_model_entity_id(self, model_name):
        world_name = self.get_parameter("world_name").value
        timeout_ms = int(self.get_parameter("gz_timeout_ms").value)
        cmd = [
            "ign",
            "service",
            "-s",
            f"/world/{world_name}/scene/graph",
            "--reqtype",
            "ignition.msgs.Empty",
            "--reptype",
            "ignition.msgs.StringMsg",
            "--timeout",
            str(timeout_ms),
            "--req",
            "",
        ]
        ok, output = self._run_command(cmd, timeout_ms)
        if not ok:
            return None, f"Failed to read Gazebo scene graph: {output}"

        pattern = r'\[label=\\?"%s \((\d+)\)\\?"\]' % re.escape(model_name)
        match = re.search(pattern, output)
        if not match:
            return None, f"Could not find model '{model_name}' in Gazebo scene graph."
        return int(match.group(1)), "Found model entity."

    def _publish_empty_topic(self, topic, count=1):
        timeout_ms = int(self.get_parameter("gz_timeout_ms").value)
        count = max(1, int(count))
        cmd = [
            "ign",
            "topic",
            "-t",
            topic,
            "-m",
            "ignition.msgs.Empty",
            "-p",
            "",
            "-d",
            str(count),
        ]
        ok, output = self._run_command(cmd, timeout_ms + 1500)
        if not ok:
            return False, f"Failed to publish Empty message to {topic}: {output}"
        return True, f"Published {count} Empty message(s) to {topic}."

    def _set_model_pose(self, model_name, xyz):
        world_name = self.get_parameter("world_name").value
        timeout_ms = int(self.get_parameter("gz_timeout_ms").value)
        request = (
            f'name: "{model_name}" '
            f"position {{ x: {xyz[0]:.6f} y: {xyz[1]:.6f} z: {xyz[2]:.6f} }} "
            "orientation { x: 0.0 y: 0.0 z: 0.0 w: 1.0 }"
        )
        cmd = [
            "ign",
            "service",
            "-s",
            f"/world/{world_name}/set_pose",
            "--reqtype",
            "ignition.msgs.Pose",
            "--reptype",
            "ignition.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            request,
        ]
        ok, output = self._run_command(cmd, timeout_ms)
        if not ok:
            return False, f"Gazebo set_pose failed for {model_name}: {output}"
        if "data: true" not in output and "true" not in output.lower():
            return False, f"Gazebo set_pose did not report success for {model_name}: {output}"
        return True, "Set %s pose to [%.4f, %.4f, %.4f]." % (
            model_name,
            xyz[0],
            xyz[1],
            xyz[2],
        )

    def _run_command(self, cmd, timeout_ms):
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=(timeout_ms / 1000.0) + 1.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, str(exc)

        output = (result.stdout + result.stderr).strip()
        if result.returncode != 0:
            return False, output
        return True, output

    def stop_state_listeners(self):
        for process, thread in self._state_listeners:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    process.kill()
            thread.join(timeout=0.2)
        self._state_listeners = []


def main(args=None):
    rclpy.init(args=args)
    node = SuctionGraspManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_state_listeners()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

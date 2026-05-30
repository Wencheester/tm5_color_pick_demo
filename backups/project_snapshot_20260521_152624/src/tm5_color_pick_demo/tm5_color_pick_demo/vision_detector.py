import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from sensor_msgs.msg import Image


VALID_COLORS = ("red", "yellow", "blue")

GRID_CELLS = (
    ("left_top", "center_top", "right_top"),
    ("left_middle", "center_middle", "right_middle"),
    ("left_bottom", "center_bottom", "right_bottom"),
)


class VisionDetectionError(RuntimeError):
    pass


class VisionDetector:
    def __init__(self, node, image_topic="/tool_camera/image_raw"):
        self._node = node
        self._bridge = CvBridge()
        self._latest_image = None
        self._subscription = node.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            10,
        )

    def detect_camera_state(self, timeout_sec=5.0, min_area=80.0):
        self._latest_image = None
        image = self._wait_for_image(timeout_sec)
        return detect_camera_state_from_bgr(image, min_area=min_area)

    def _image_callback(self, msg):
        try:
            self._latest_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self._node.get_logger().warn("Failed to convert camera image: %s" % exc)

    def _wait_for_image(self, timeout_sec):
        deadline = time.monotonic() + float(timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            if self._latest_image is not None:
                return self._latest_image
            rclpy.spin_once(self._node, timeout_sec=0.1)
        raise VisionDetectionError("No image received from camera topic.")


def detect_camera_state_from_bgr(image, min_area=80.0):
    if image is None or image.size == 0:
        raise VisionDetectionError("Camera image is empty.")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masks = _make_color_masks(hsv)
    camera_state = {}

    for color in VALID_COLORS:
        cell = _detect_color_cell(masks[color], min_area)
        if cell is None:
            raise VisionDetectionError("No %s cube detected in camera image." % color)
        camera_state[color] = cell

    return camera_state


def _make_color_masks(hsv):
    red_low_1 = cv2.inRange(hsv, np.array([0, 80, 50]), np.array([10, 255, 255]))
    red_low_2 = cv2.inRange(hsv, np.array([170, 80, 50]), np.array([180, 255, 255]))
    red = cv2.bitwise_or(red_low_1, red_low_2)
    yellow = cv2.inRange(hsv, np.array([18, 70, 50]), np.array([38, 255, 255]))
    blue = cv2.inRange(hsv, np.array([95, 70, 40]), np.array([130, 255, 255]))

    return {
        "red": _clean_mask(red),
        "yellow": _clean_mask(yellow),
        "blue": _clean_mask(blue),
    }


def _clean_mask(mask):
    kernel = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel)


def _detect_color_cell(mask, min_area):
    contours, _hierarchy = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < float(min_area):
        return None

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    height, width = mask.shape[:2]
    col = min(2, max(0, int(cx * 3 / max(1, width))))
    row = min(2, max(0, int(cy * 3 / max(1, height))))
    return GRID_CELLS[row][col]

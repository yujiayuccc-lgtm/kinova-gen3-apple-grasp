#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self):
        super().__init__('apple_vision_probe')
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.create_subscription(Image, '/camera/color/image_raw', self.on_color, qos)
        self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos)

    def on_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def on_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')


def main():
    rclpy.init()
    node = Probe()
    deadline = time.monotonic() + 12.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.color is not None and node.depth is not None:
            break
    if node.color is None or node.depth is None:
        print('RGBD_TIMEOUT')
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(2)

    rgb = node.color
    depth = node.depth
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(hsv, (0, 80, 45), (12, 255, 255))
    mask |= cv2.inRange(hsv, (168, 80, 45), (179, 255, 255))
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print(f'RGB_SHAPE={rgb.shape[1]}x{rgb.shape[0]} DEPTH_SHAPE={depth.shape[1]}x{depth.shape[0]}')
        print('APPLE_NOT_FOUND')
        raise SystemExit(3)

    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    u = int(round(moments['m10'] / moments['m00']))
    v = int(round(moments['m01'] / moments['m00']))
    x, y, w, h = cv2.boundingRect(contour)

    # Initial cross-resolution depth candidate.  A calibrated 3-D projection is
    # performed by the final locator; this probe only verifies data and target.
    du = int(round(u * depth.shape[1] / rgb.shape[1]))
    dv = int(round(v * depth.shape[0] / rgb.shape[0]))
    r = 4
    patch = depth[max(0, dv-r):dv+r+1, max(0, du-r):du+r+1].astype(np.float32)
    valid = patch[(patch > 50) & (patch < 5000)]
    depth_mm = float(np.median(valid)) if valid.size else float('nan')

    annotated = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.drawContours(annotated, [contour], -1, (0, 255, 0), 3)
    cv2.circle(annotated, (u, v), 8, (255, 0, 0), -1)
    cv2.putText(annotated, f'({u},{v}) depth~{depth_mm:.0f}mm',
                (max(0, x), max(25, y-10)), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255), 2)
    cv2.imwrite('/tmp/apple_color_annotated.png', annotated)
    cv2.imwrite('/tmp/apple_red_mask.png', mask)

    valid_all = depth[(depth > 50) & (depth < 5000)]
    print(f'RGB_SHAPE={rgb.shape[1]}x{rgb.shape[0]} DEPTH_SHAPE={depth.shape[1]}x{depth.shape[0]}')
    print(f'APPLE_PIXEL=({u},{v}) BBOX=({x},{y},{w},{h}) AREA={area:.1f}')
    print(f'DEPTH_CANDIDATE_MM={depth_mm:.1f} VALID_DEPTH_PIXELS={valid_all.size}')
    print('ANNOTATED=/tmp/apple_color_annotated.png MASK=/tmp/apple_red_mask.png')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

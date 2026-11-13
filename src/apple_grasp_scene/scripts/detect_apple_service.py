#!/usr/bin/python3
"""ROS 2 service wrapper around the stable RGB-D apple detector."""

import sys

# Ubuntu's OpenCV package is built against the distro NumPy 1.x package. This
# workstation also has NumPy 2.x under /usr/local, so prefer the matching
# distro packages just as stable_apple_locator.sh does.
sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.node import Node

from apple_grasp_interfaces.srv import DetectApple

import stable_apple_locator


class DetectAppleService(Node):
    def __init__(self):
        super().__init__('detect_apple_service')
        self._service = self.create_service(
            DetectApple, 'detect_apple', self._detect_callback)
        self.get_logger().info('Ready to provide stable apple detections.')

    def _detect_callback(self, _request, response):
        self.get_logger().info('Capturing a stable three-frame RGB-D observation.')
        try:
            result = stable_apple_locator.detect_stable()
            center = result['apple_base_m']
            grasp = result['grasp_wrist_m']

            response.success = True
            response.message = 'Stable apple detection completed.'
            response.detection.header.stamp = self.get_clock().now().to_msg()
            response.detection.header.frame_id = 'base_link'
            response.detection.detected = True
            response.detection.center.x = center['x']
            response.detection.center.y = center['y']
            response.detection.center.z = center['z']
            response.detection.grasp_point.x = grasp['x']
            response.detection.grasp_point.y = grasp['y']
            response.detection.grasp_point.z = grasp['z']
            response.detection.radius = result['apple_radius_m']
            # The current geometric detector has no calibrated probability
            # model. Keep confidence explicitly unset instead of presenting a
            # heuristic quality score as a probability.
            response.detection.confidence = 0.0
            response.detection.point_count = result['fit_points']
            response.detection.fit_residual = result['fit_median_residual_m']
            response.detection.failure_reason = ''
            self.get_logger().info(
                f"Apple detected at ({center['x']:.3f}, "
                f"{center['y']:.3f}, {center['z']:.3f}) m.")
        except Exception as exc:  # Hardware and vision errors are task results.
            message = str(exc)
            response.success = False
            response.message = message
            response.detection.header.stamp = self.get_clock().now().to_msg()
            response.detection.header.frame_id = 'base_link'
            response.detection.detected = False
            response.detection.failure_reason = message
            self.get_logger().warning(f'Apple detection failed: {message}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DetectAppleService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

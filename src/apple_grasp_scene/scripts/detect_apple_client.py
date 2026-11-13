#!/usr/bin/python3
"""Call the typed DetectApple service and print a JSON-compatible result."""

import json
import sys

sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.node import Node

from apple_grasp_interfaces.srv import DetectApple


class DetectAppleClient(Node):
    def __init__(self):
        super().__init__('detect_apple_client')
        self._client = self.create_client(DetectApple, 'detect_apple')

    def detect(self):
        if not self._client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/detect_apple service is not available')

        future = self._client.call_async(DetectApple.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=75.0)
        if not future.done():
            raise RuntimeError('/detect_apple service timed out')
        if future.exception() is not None:
            raise RuntimeError(str(future.exception()))
        return future.result()


def detection_to_dict(response):
    detection = response.detection
    return {
        'apple_base_m': {
            'x': detection.center.x,
            'y': detection.center.y,
            'z': detection.center.z,
        },
        'apple_radius_m': detection.radius,
        'grasp_wrist_m': {
            'x': detection.grasp_point.x,
            'y': detection.grasp_point.y,
            'z': detection.grasp_point.z,
        },
        'fit_points': detection.point_count,
        'fit_median_residual_m': detection.fit_residual,
        'frame_id': detection.header.frame_id,
    }


def main(args=None):
    rclpy.init(args=args)
    node = DetectAppleClient()
    exit_code = 0
    try:
        response = node.detect()
        if not response.success or not response.detection.detected:
            reason = response.detection.failure_reason or response.message
            print(f'DETECTION_SERVICE_REJECTED: {reason}', file=sys.stderr)
            exit_code = 3
        else:
            print(json.dumps(detection_to_dict(response), indent=2))
    except Exception as exc:
        print(f'DETECTION_SERVICE_ERROR: {exc}', file=sys.stderr)
        exit_code = 4
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()


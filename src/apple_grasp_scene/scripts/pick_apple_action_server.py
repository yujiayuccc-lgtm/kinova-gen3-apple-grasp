#!/usr/bin/python3
"""Safe, no-motion ROS 2 action skeleton for the apple-picking task."""

import sys
import time

sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from apple_grasp_interfaces.action import PickApple
from apple_grasp_interfaces.srv import DetectApple


class PickAppleActionServer(Node):
    """Publish task-state feedback without invoking hardware commands."""

    PREVIEW_STATES = (
        'PLAN_APPROACH',
        'APPROACH_PREVIEW',
        'GRASP_PREVIEW',
        'PLACE_PREVIEW',
        'RETURN_HOME_PREVIEW',
    )

    def __init__(self):
        super().__init__('pick_apple_action_server')
        self._server = ActionServer(
            self,
            PickApple,
            'pick_apple',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            'PickApple action server is ready (live perception, no motion).')

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal: execute={goal_request.execute}, '
            f'max_search_attempts={goal_request.max_search_attempts}, '
            f'max_grasp_attempts={goal_request.max_grasp_attempts}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info('Cancellation requested.')
        return CancelResponse.ACCEPT

    def _publish_feedback(
            self, goal_handle, state, progress,
            search_attempt=0, grasp_attempt=0):
        feedback = PickApple.Feedback()
        feedback.current_state = state
        feedback.search_attempt = search_attempt
        feedback.grasp_attempt = grasp_attempt
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(
            f'Dry-run state: {state} ({feedback.progress:.0%})')

    def _request_detection(self):
        """Use an isolated client node so the action callback can wait safely."""
        client_node = rclpy.create_node(
            'pick_apple_detection_client', enable_rosout=False)
        client = client_node.create_client(DetectApple, 'detect_apple')
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('/detect_apple service is not available')
            future = client.call_async(DetectApple.Request())
            rclpy.spin_until_future_complete(
                client_node, future, timeout_sec=75.0)
            if not future.done():
                raise RuntimeError('/detect_apple service timed out')
            if future.exception() is not None:
                raise RuntimeError(str(future.exception()))
            return future.result()
        finally:
            client_node.destroy_node()

    def _execute_callback(self, goal_handle):
        started_at = time.monotonic()
        result = PickApple.Result()

        # This first implementation is intentionally incapable of motion.
        # Real execution is connected only after the action lifecycle and
        # cancellation behavior have been tested independently.
        if goal_handle.request.execute:
            result.success = False
            result.final_state = 'SAFETY_REJECTED'
            result.message = (
                'Real execution is disabled in the action skeleton. '
                'Send execute=false for a no-motion workflow test.')
            result.total_time = float(time.monotonic() - started_at)
            goal_handle.abort()
            self.get_logger().warning(result.message)
            return result

        self._publish_feedback(goal_handle, 'CHECK_SYSTEM', 0.10)
        time.sleep(0.25)
        self._publish_feedback(goal_handle, 'SEARCH', 0.20, search_attempt=1)
        time.sleep(0.25)
        self._publish_feedback(goal_handle, 'DETECT', 0.30, search_attempt=1)

        try:
            detection_response = self._request_detection()
        except Exception as exc:
            goal_handle.abort()
            result.success = False
            result.final_state = 'DETECTION_SERVICE_ERROR'
            result.message = str(exc)
            result.total_time = float(time.monotonic() - started_at)
            return result

        if (not detection_response.success or
                not detection_response.detection.detected):
            reason = (detection_response.detection.failure_reason or
                      detection_response.message)
            goal_handle.abort()
            result.success = False
            result.final_state = 'DETECTION_FAILED'
            result.message = reason
            result.total_time = float(time.monotonic() - started_at)
            self.get_logger().warning(f'Detection rejected: {reason}')
            return result

        detection = detection_response.detection
        self._publish_feedback(
            goal_handle, 'VALIDATE', 0.45, search_attempt=1)
        self.get_logger().info(
            f'Apple detected at ({detection.center.x:.3f}, '
            f'{detection.center.y:.3f}, {detection.center.z:.3f}) m.')

        state_count = len(self.PREVIEW_STATES)
        for index, state in enumerate(self.PREVIEW_STATES, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.final_state = 'CANCELED'
                result.message = f'Dry-run task canceled during {state}.'
                result.total_time = float(time.monotonic() - started_at)
                return result

            progress = 0.45 + 0.55 * (index / state_count)
            self._publish_feedback(
                goal_handle, state, progress,
                search_attempt=1, grasp_attempt=0)
            time.sleep(0.25)

        goal_handle.succeed()
        result.success = True
        result.final_state = 'DRY_RUN_COMPLETE'
        result.message = (
            'Live detection and action preview completed without robot motion. '
            f'Apple=({detection.center.x:.3f}, '
            f'{detection.center.y:.3f}, {detection.center.z:.3f}) m.')
        result.total_time = float(time.monotonic() - started_at)
        return result

    def destroy_node(self):
        self._server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickAppleActionServer()
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


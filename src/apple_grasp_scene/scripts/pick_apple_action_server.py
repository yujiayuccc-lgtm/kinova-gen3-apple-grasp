#!/usr/bin/python3
"""Safe, no-motion ROS 2 action skeleton for the apple-picking task."""

import sys
import time

sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from apple_grasp_interfaces.action import PickApple


class PickAppleActionServer(Node):
    """Publish task-state feedback without invoking hardware commands."""

    DRY_RUN_STATES = (
        'CHECK_SYSTEM',
        'SEARCH',
        'DETECT',
        'VALIDATE',
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
            'PickApple action skeleton is ready (no-motion mode only).')

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal: execute={goal_request.execute}, '
            f'max_search_attempts={goal_request.max_search_attempts}, '
            f'max_grasp_attempts={goal_request.max_grasp_attempts}')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self.get_logger().info('Cancellation requested.')
        return CancelResponse.ACCEPT

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

        state_count = len(self.DRY_RUN_STATES)
        for index, state in enumerate(self.DRY_RUN_STATES, start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.final_state = 'CANCELED'
                result.message = f'Dry-run task canceled during {state}.'
                result.total_time = float(time.monotonic() - started_at)
                return result

            feedback = PickApple.Feedback()
            feedback.current_state = state
            feedback.search_attempt = 0
            feedback.grasp_attempt = 0
            feedback.progress = float(index / state_count)
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(
                f'Dry-run state: {state} ({feedback.progress:.0%})')
            time.sleep(0.25)

        goal_handle.succeed()
        result.success = True
        result.final_state = 'DRY_RUN_COMPLETE'
        result.message = 'Action lifecycle test completed without robot motion.'
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


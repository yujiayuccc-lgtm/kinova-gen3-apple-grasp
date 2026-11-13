#!/usr/bin/python3
"""ROS 2 action server for safely integrating the apple-picking task."""

import os
import subprocess
import sys
import time

sys.path.insert(0, '/usr/lib/python3/dist-packages')

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from apple_grasp_interfaces.action import PickApple
from apple_grasp_interfaces.srv import DetectApple
from std_msgs.msg import Bool


class PickAppleActionServer(Node):
    """Preview the task or execute only the observation-search phase."""

    PREVIEW_STATES = (
        'PLAN_APPROACH',
        'APPROACH_PREVIEW',
        'GRASP_PREVIEW',
        'PLACE_PREVIEW',
        'RETURN_HOME_PREVIEW',
    )

    def __init__(self):
        super().__init__('pick_apple_action_server')
        self._declare_parameters()
        self._viewpoints = self._load_viewpoints()
        self._server = ActionServer(
            self,
            PickApple,
            'pick_apple',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(
            'PickApple action server is ready (observation motion gated). '
            f'Loaded {len(self._viewpoints)} viewpoints: '
            f'{", ".join(point[0] for point in self._viewpoints)}.')

    def _declare_parameters(self):
        self.declare_parameter('viewpoint_names', ['center'])
        self.declare_parameter('viewpoint_x', [0.680])
        self.declare_parameter('viewpoint_y', [-0.260])
        self.declare_parameter('viewpoint_z', [0.400])
        self.declare_parameter('workspace_min_x', 0.55)
        self.declare_parameter('workspace_max_x', 0.80)
        self.declare_parameter('workspace_min_y', -0.45)
        self.declare_parameter('workspace_max_y', -0.10)
        self.declare_parameter('apple_min_z', 0.025)
        self.declare_parameter('apple_max_z', 0.060)
        self.declare_parameter('apple_min_radius', 0.025)
        self.declare_parameter('apple_max_radius', 0.055)
        self.declare_parameter('min_fit_points', 1000)
        self.declare_parameter('max_fit_residual', 0.003)
        self.declare_parameter('dogleg_threshold', 0.075)
        self.declare_parameter('pregrasp_height', 0.200)
        self.declare_parameter('grasp_height', 0.150)
        self.declare_parameter('lift_height', 0.200)

    def _load_viewpoints(self):
        names = list(self.get_parameter('viewpoint_names').value)
        xs = list(self.get_parameter('viewpoint_x').value)
        ys = list(self.get_parameter('viewpoint_y').value)
        zs = list(self.get_parameter('viewpoint_z').value)
        if not names or not (len(names) == len(xs) == len(ys) == len(zs)):
            raise RuntimeError(
                'viewpoint_names/x/y/z must be non-empty arrays of equal length')
        if len(set(names)) != len(names):
            raise RuntimeError('viewpoint_names must be unique')
        return tuple(zip(names, xs, ys, zs))

    def _validate_detection(self, detection):
        checks = (
            (self.get_parameter('workspace_min_x').value <= detection.center.x <=
             self.get_parameter('workspace_max_x').value,
             'apple x outside configured workspace'),
            (self.get_parameter('workspace_min_y').value <= detection.center.y <=
             self.get_parameter('workspace_max_y').value,
             'apple y outside configured workspace'),
            (self.get_parameter('apple_min_z').value <= detection.center.z <=
             self.get_parameter('apple_max_z').value,
             'apple center height outside configured range'),
            (self.get_parameter('apple_min_radius').value <= detection.radius <=
             self.get_parameter('apple_max_radius').value,
             'apple radius outside configured range'),
            (detection.point_count >=
             self.get_parameter('min_fit_points').value,
             'insufficient fitted depth points'),
            (detection.fit_residual <=
             self.get_parameter('max_fit_residual').value,
             'sphere-fit residual exceeds configured limit'),
        )
        return [message for passed, message in checks if not passed]

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
            f'Task state: {state} ({feedback.progress:.0%})')

    def _read_hardware_fault(self):
        """Return the current fault flag, failing closed if none is received."""
        fault_node = rclpy.create_node(
            'pick_apple_fault_reader', enable_rosout=False)
        state = {'received': False, 'fault': True}

        def callback(message):
            state['received'] = True
            state['fault'] = bool(message.data)

        subscription = fault_node.create_subscription(
            Bool, '/fault_controller/internal_fault', callback, 1)
        deadline = time.monotonic() + 5.0
        try:
            while not state['received'] and time.monotonic() < deadline:
                rclpy.spin_once(fault_node, timeout_sec=0.2)
            if not state['received']:
                raise RuntimeError(
                    'hardware fault state was not received within 5 seconds')
            return state['fault']
        finally:
            fault_node.destroy_subscription(subscription)
            fault_node.destroy_node()

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

    def _run_viewpoint(self, viewpoint, execute_authorized):
        """Plan to a viewpoint, executing only after all external gates pass."""
        name, x, y, z = viewpoint
        command = [
            'ros2', 'run', 'apple_grasp_scene', 'scene_plan_only',
            '--ros-args',
            '-p', f'target_x:={x}',
            '-p', f'target_y:={y}',
            '-p', f'target_z:={z}',
            '-p', f'execute_authorized:={str(execute_authorized).lower()}',
        ]
        completed = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=45,
            check=False,
        )
        if completed.returncode != 0:
            lines = [line for line in completed.stdout.splitlines() if line]
            detail = lines[-1] if lines else 'no planner output'
            raise RuntimeError(
                f'viewpoint {name} planning failed: {detail}')
        operation = 'Motion completed' if execute_authorized else 'Planning preview succeeded'
        self.get_logger().info(
            f'{operation} for {name}: ({x:.3f}, {y:.3f}, {z:.3f}) m.')

    def _execute_callback(self, goal_handle):
        started_at = time.monotonic()
        result = PickApple.Result()

        real_execution = bool(goal_handle.request.execute)
        if real_execution:
            if os.environ.get('APPLE_ACTION_EXECUTE') != 'YES':
                result.success = False
                result.final_state = 'SAFETY_REJECTED'
                result.message = (
                    'Real observation motion requires '
                    'APPLE_ACTION_EXECUTE=YES and execute=true.')
                result.total_time = float(time.monotonic() - started_at)
                goal_handle.abort()
                self.get_logger().warning(result.message)
                return result
            try:
                hardware_fault = self._read_hardware_fault()
            except RuntimeError as exc:
                result.success = False
                result.final_state = 'HARDWARE_STATE_UNAVAILABLE'
                result.message = str(exc)
                result.total_time = float(time.monotonic() - started_at)
                goal_handle.abort()
                return result
            if hardware_fault:
                result.success = False
                result.final_state = 'HARDWARE_FAULT'
                result.message = 'Robot reports an active internal fault.'
                result.total_time = float(time.monotonic() - started_at)
                goal_handle.abort()
                return result

        self._publish_feedback(goal_handle, 'CHECK_SYSTEM', 0.05)
        time.sleep(0.25)

        requested_attempts = int(goal_handle.request.max_search_attempts)
        attempt_limit = (
            len(self._viewpoints) if requested_attempts == 0 else
            min(requested_attempts, len(self._viewpoints)))
        if attempt_limit <= 0:
            goal_handle.abort()
            result.success = False
            result.final_state = 'INVALID_GOAL'
            result.message = 'max_search_attempts must be zero or greater.'
            result.total_time = float(time.monotonic() - started_at)
            return result

        planning_errors = []
        planned_count = 0
        for attempt, viewpoint in enumerate(
                self._viewpoints[:attempt_limit], start=1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.final_state = 'CANCELED'
                result.message = 'Canceled during viewpoint planning preview.'
                result.total_time = float(time.monotonic() - started_at)
                return result

            name = viewpoint[0]
            progress = 0.10 + 0.40 * (attempt / attempt_limit)
            operation = 'MOVE' if real_execution else 'PLAN'
            self._publish_feedback(
                goal_handle, f'{operation}_VIEWPOINT_{name.upper()}', progress,
                search_attempt=attempt)
            viewpoint_ready = False
            try:
                self._run_viewpoint(viewpoint, real_execution)
                planned_count += 1
                viewpoint_ready = True
            except (RuntimeError, subprocess.TimeoutExpired) as exc:
                message = str(exc)
                planning_errors.append(message)
                self.get_logger().warning(message)

            if real_execution and viewpoint_ready:
                self._publish_feedback(
                    goal_handle, 'DETECT', progress + 0.02,
                    search_attempt=attempt)
                try:
                    viewpoint_detection = self._request_detection()
                except Exception as exc:
                    planning_errors.append(str(exc))
                    continue
                if (not viewpoint_detection.success or
                        not viewpoint_detection.detection.detected):
                    reason = (viewpoint_detection.detection.failure_reason or
                              viewpoint_detection.message)
                    self.get_logger().warning(
                        f'Detection failed at {name}: {reason}')
                    continue

                detection = viewpoint_detection.detection
                validation_errors = self._validate_detection(detection)
                if validation_errors:
                    goal_handle.abort()
                    result.success = False
                    result.final_state = 'SAFETY_REJECTED'
                    result.message = '; '.join(validation_errors)
                    result.total_time = float(time.monotonic() - started_at)
                    return result

                goal_handle.succeed()
                result.success = True
                result.final_state = 'TARGET_FOUND_GRASP_DISABLED'
                result.message = (
                    f'Apple found from viewpoint {name}; grasp motion remains '
                    f'disabled. Apple=({detection.center.x:.3f}, '
                    f'{detection.center.y:.3f}, {detection.center.z:.3f}) m.')
                result.total_time = float(time.monotonic() - started_at)
                return result

        if planned_count == 0:
            goal_handle.abort()
            result.success = False
            result.final_state = 'VIEWPOINT_PLANNING_FAILED'
            result.message = '; '.join(planning_errors)
            result.total_time = float(time.monotonic() - started_at)
            return result

        if real_execution:
            goal_handle.abort()
            result.success = False
            result.final_state = 'SEARCH_FAILED'
            result.message = (
                f'Apple was not found from {planned_count} executed viewpoints.')
            result.total_time = float(time.monotonic() - started_at)
            return result

        self._publish_feedback(
            goal_handle, 'DETECT_CURRENT_VIEW', 0.60,
            search_attempt=planned_count)

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
        validation_errors = self._validate_detection(detection)
        if validation_errors:
            goal_handle.abort()
            result.success = False
            result.final_state = 'SAFETY_REJECTED'
            result.message = '; '.join(validation_errors)
            result.total_time = float(time.monotonic() - started_at)
            self.get_logger().warning(
                f'Configured safety validation rejected detection: '
                f'{result.message}')
            return result

        self._publish_feedback(
            goal_handle, 'VALIDATE', 0.72,
            search_attempt=planned_count)
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

            progress = 0.72 + 0.28 * (index / state_count)
            self._publish_feedback(
                goal_handle, state, progress,
                search_attempt=planned_count, grasp_attempt=0)
            time.sleep(0.25)

        goal_handle.succeed()
        result.success = True
        result.final_state = 'DRY_RUN_COMPLETE'
        result.message = (
            'Live detection and action preview completed without robot motion. '
            f'Planned {planned_count}/{attempt_limit} viewpoints. '
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


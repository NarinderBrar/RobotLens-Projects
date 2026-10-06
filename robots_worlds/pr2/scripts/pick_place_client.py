#!/usr/bin/env python3
"""PickPlace action client for the P4 pick-and-place gate.

Picks the graspable object from its nominal pose and places it back at the
same pose (a deterministic loop), then exits reporting each run's outcome.
Run with the PR2 Gazebo fixture and move_group both up:

  source /opt/ros/jazzy/setup.bash
  source install/setup.bash
  python3 robots_worlds/pr2/scripts/pick_place_client.py [num_runs]
"""
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from rclpy.node import Node

from pr2_pick_place_interfaces.action import PickPlace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import grasp_object as go


class PickPlaceClient(Node):
    def __init__(self):
        super().__init__('pick_place_client')
        self._client = ActionClient(self, PickPlace, '/pick_place')
        self.get_logger().info('waiting for /pick_place action server...')
        if not self._client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError('no /pick_place action server')

    def run_once(self, index):
        goal = PickPlace.Goal()
        goal.object_id = go.OBJECT_MODEL_NAME
        goal.grasp_candidates = []
        goal.placement_pose.position.x = go.OBJECT_INITIAL_POSE_XYZ[0]
        goal.placement_pose.position.y = go.OBJECT_INITIAL_POSE_XYZ[1]
        goal.placement_pose.position.z = go.OBJECT_INITIAL_POSE_XYZ[2]
        goal.placement_pose.orientation.w = 1.0
        goal.planning_timeout = 0.0
        goal.velocity_scaling = 0.0

        future = self._client.send_goal_async(goal, feedback_callback=lambda _f: None)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done():
            self.get_logger().error(f'run {index}: goal send timed out')
            return False, 'send timeout', 0.0
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'run {index}: goal rejected')
            return False, 'goal rejected', 0.0
        self.get_logger().info(f'run {index}: goal accepted')
        result_future = handle.get_result_async()
        started = time.monotonic()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - started > 120.0:
                self.get_logger().error(f'run {index}: result timed out (120s)')
                return False, 'result timeout', time.monotonic() - started
        result = result_future.result()
        status = result.status if hasattr(result, 'status') else None
        outcome = result.result if result is not None else None
        if outcome is None:
            return False, f'no outcome (status {status})', time.monotonic() - started
        elapsed = time.monotonic() - started
        return outcome.success, f'{outcome.message} (failed_state={outcome.failed_state})', elapsed


def main():
    num_runs = int(sys.argv[1]) if len(sys.argv) > 1 else 20
    rclpy.init()
    node = PickPlaceClient()
    results = []
    for i in range(1, num_runs + 1):
        ok, message, elapsed = node.run_once(i)
        results.append(ok)
        node.get_logger().info(
            f'run {i}: {"SUCCESS" if ok else "FAIL"} ({elapsed:.1f}s) {message}')
        if not ok:
            print(f'FAILED at run {i}: {message}', flush=True)
            break
        # brief settle so Gazebo physics/state settles before the next grasp
        time.sleep(2.0)
    succeeded = sum(1 for r in results if r)
    print(f'RESULT: {succeeded}/{len(results)} runs succeeded', flush=True)
    rclpy.shutdown()
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())

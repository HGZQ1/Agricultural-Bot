"""Simulate work/retraction feedback without commanding any arm joint."""

import time

from agri_harvest_interfaces.action import HarvestWork
from agri_harvest_interfaces.msg import HarvestWorkerState
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future

from .harvest_coordinator import target_quality


class MockHarvestWorker(Node):
    """Provide a test-only action and a leased simulated STOWED state."""

    def __init__(self):
        super().__init__('mock_harvest_worker')
        self.declare_parameter('work_action', '/harvest_work')
        self.declare_parameter('worker_state_topic', '/harvest_worker/state')
        self.declare_parameter('work_duration_s', 1.0)
        self.declare_parameter('retract_duration_s', 0.5)
        self.declare_parameter('failure_mode', 'none')
        self.declare_parameter('require_segmentation', True)
        self._busy = False
        self._phase = 'IDLE'
        self._arm_stowed = True
        self._work_context = ('', '', '')
        self._waiters = []
        self._seen_work_ids = set()
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self._pub = self.create_publisher(
            HarvestWorkerState,
            str(self.get_parameter('worker_state_topic').value), 10)
        self._server = ActionServer(
            self, HarvestWork, str(self.get_parameter('work_action').value),
            execute_callback=self._execute, goal_callback=self._goal,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT)
        self.create_timer(0.05, self._tick, clock=self._steady_clock)
        self.get_logger().warning(
            'MOCK worker: no IK, real grasp, collision or arm-motion validation')

    def _goal(self, request):
        if (self._busy or not self._arm_stowed or
                self.get_parameter('failure_mode').value == 'reject' or
                not all((request.work_id, request.run_id, request.waypoint_id,
                         request.plant_id)) or request.work_id in self._seen_work_ids or
                request.header.frame_id != 'base_footprint' or
                request.work_side not in {'left', 'right'} or
                not target_quality(request.target, require_segmentation=bool(
                    self.get_parameter('require_segmentation').value))):
            return GoalResponse.REJECT
        self._busy = True
        self._seen_work_ids.add(request.work_id)
        self._work_context = (request.run_id, request.waypoint_id, request.work_id)
        self._phase = 'DEPLOYING'
        self._arm_stowed = False
        self._publish_state()
        return GoalResponse.ACCEPT

    def _publish_state(self):
        if (self.get_parameter('failure_mode').value == 'heartbeat_loss' and
                self._phase not in {'IDLE', 'STOWED'}):
            return
        message = HarvestWorkerState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.run_id, message.waypoint_id, message.work_id = self._work_context
        message.arm_stowed = self._arm_stowed
        message.phase = self._phase
        self._pub.publish(message)

    def _tick(self):
        now = time.monotonic()
        for deadline, future in list(self._waiters):
            if now >= deadline:
                if not future.done():
                    future.set_result(True)
                self._waiters.remove((deadline, future))
        self._publish_state()

    async def _delay(self, duration):
        future = Future()
        self._waiters.append((time.monotonic() + duration, future))
        await future

    def _feedback(self, handle):
        feedback = HarvestWork.Feedback()
        feedback.phase = self._phase
        feedback.arm_stowed = self._arm_stowed
        handle.publish_feedback(feedback)

    async def _execute(self, handle):
        mode = str(self.get_parameter('failure_mode').value)
        self._phase = 'SCANNING'
        self._feedback(handle)
        await self._delay(0.2)
        self._phase = 'WORKING'
        self._feedback(handle)
        deadline = time.monotonic() + float(self.get_parameter('work_duration_s').value)
        while (not handle.is_cancel_requested and
               (mode in {'stall', 'heartbeat_loss'} or time.monotonic() < deadline)):
            await self._delay(0.1)
        if mode == 'no_stow' and not handle.is_cancel_requested:
            result = HarvestWork.Result(success=False, arm_stowed=False,
                                        message='mock retraction deliberately unconfirmed')
            handle.abort()
            self._busy = False
            return result
        self._phase = 'RETRACTING'
        self._feedback(handle)
        await self._delay(float(self.get_parameter('retract_duration_s').value))
        self._phase = 'STOWED'
        self._arm_stowed = True
        self._publish_state()
        self._feedback(handle)
        result = HarvestWork.Result()
        result.arm_stowed = True
        result.success = not handle.is_cancel_requested and mode != 'fail'
        result.message = ('mock work complete; actual grasp unverified' if result.success
                          else 'mock work canceled or deliberate failure; simulated arm stowed')
        if handle.is_cancel_requested:
            handle.canceled()
        elif result.success:
            handle.succeed()
        else:
            handle.abort()
        self._busy = False
        return result


def main(args=None):
    """Run the explicit test-only worker."""
    rclpy.init(args=args)
    node = MockHarvestWorker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

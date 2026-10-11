"""Execute a version-bound field route through Nav2 NavigateToPose."""

from __future__ import annotations

import hashlib
import json
import math
import os
import time
from pathlib import Path
from typing import Callable

from action_msgs.msg import GoalStatus

from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import NavigateToPose

from nav_msgs.msg import OccupancyGrid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import String

from std_srvs.srv import SetBool, Trigger

from .route_validation import verify_mission_map
from .waypoint_schema import (
    HarvestWaypoint, MissionConfigError, Pose2D, load_mission, sha256_file)


TERMINAL_STATES = frozenset({'COMPLETED', 'FAILED', 'CANCELED'})
ACTIVE_GOAL_STATES = frozenset({
    'SENDING_GOAL', 'NAVIGATING', 'RETURNING', 'CANCELING',
})


class FieldRouteExecutor(Node):
    """Run enabled plant waypoints, coalescing targets at a shared stop."""

    def __init__(self) -> None:
        """Load the bound mission and expose its ROS control interface."""
        super().__init__('field_route_executor')
        share = Path(get_package_share_directory('agri_task_manager'))
        default_mission = (
            share / 'config' / 'missions'
            / 'tomato_field_default_single_corridor.yaml')
        self.declare_parameter('mission_file', str(default_mission))
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('keepout_yaml', '')
        self.declare_parameter('field_metadata', '')
        self.declare_parameter('field_sdf', '')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'keepout_topic', '/keepout_filter_mask')
        self.declare_parameter('require_live_map', True)
        self.declare_parameter('require_live_keepout', True)
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('motion_lock_service', '/base_motion/set_lock')
        self.declare_parameter('require_motion_lock', True)
        self.declare_parameter('require_station_ack', False)
        self.declare_parameter('autostart', False)
        self.declare_parameter('server_wait_timeout_s', 30.0)
        self.declare_parameter('goal_response_timeout_s', 10.0)
        self.declare_parameter('cancel_timeout_s', 5.0)
        self.declare_parameter('motion_lock_timeout_s', 3.0)
        self.declare_parameter('log_directory', '')

        mission_file = Path(self.get_parameter('mission_file').value)
        self._mission = load_mission(mission_file)
        map_override = str(self.get_parameter('map_yaml').value).strip()
        if map_override:
            map_yaml = Path(map_override)
        else:
            map_share = Path(get_package_share_directory(
                self._mission.map_binding.package))
            map_yaml = map_share / self._mission.map_binding.yaml_relative_path
        keepout_override = str(
            self.get_parameter('keepout_yaml').value).strip()
        if keepout_override:
            keepout_yaml = Path(keepout_override)
        else:
            map_share = Path(get_package_share_directory(
                self._mission.map_binding.package))
            keepout_yaml = (
                map_share /
                self._mission.map_binding.keepout_yaml_relative_path)
        _, self._map_reports = verify_mission_map(
            self._mission, map_yaml, keepout_yaml)
        field_metadata = str(
            self.get_parameter('field_metadata').value).strip()
        field_sdf = str(self.get_parameter('field_sdf').value).strip()
        if bool(field_metadata) != bool(field_sdf):
            raise MissionConfigError(
                'field_metadata and field_sdf must be supplied together')
        if field_metadata:
            try:
                if (sha256_file(field_metadata) !=
                        self._mission.field.metadata_sha256):
                    raise MissionConfigError(
                        'field metadata SHA-256 does not match mission')
                if sha256_file(field_sdf) != self._mission.field.sdf_sha256:
                    raise MissionConfigError(
                        'field SDF SHA-256 does not match mission')
            except OSError as exc:
                raise MissionConfigError(
                    f'unable to read bound field asset: {exc}') from exc
        self._waypoints = sorted(
            self._mission.enabled_waypoints, key=lambda point: point.sequence)
        if not self._waypoints:
            raise MissionConfigError('mission has no enabled waypoints')

        self._require_lock = bool(
            self.get_parameter('require_motion_lock').value)
        self._require_station_ack = bool(
            self.get_parameter('require_station_ack').value)
        self._require_live_map = bool(
            self.get_parameter('require_live_map').value)
        self._require_live_keepout = bool(
            self.get_parameter('require_live_keepout').value)
        self._server_wait_timeout = float(
            self.get_parameter('server_wait_timeout_s').value)
        self._goal_response_timeout = float(
            self.get_parameter('goal_response_timeout_s').value)
        self._cancel_timeout = float(
            self.get_parameter('cancel_timeout_s').value)
        self._lock_timeout = float(
            self.get_parameter('motion_lock_timeout_s').value)
        if any(timeout <= 0.0 for timeout in (
                self._server_wait_timeout, self._goal_response_timeout,
                self._cancel_timeout, self._lock_timeout)):
            raise ValueError('server, goal, cancel, and lock timeouts '
                             'must be positive')

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(String, '~/status', qos)
        self._point_pub = self.create_publisher(
            String, '~/current_harvest_point', qos)
        self._live_map_valid = not self._require_live_map
        self._live_map_error: str | None = None
        self._live_keepout_valid = not self._require_live_keepout
        self._live_keepout_error: str | None = None
        self._map_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('map_topic').value),
            self._map_callback,
            qos,
        )
        self._keepout_subscription = self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter('keepout_topic').value),
            self._keepout_callback,
            qos,
        )
        self._navigate = ActionClient(
            self, NavigateToPose,
            str(self.get_parameter('navigate_action').value))
        self._lock_client = self.create_client(
            SetBool, str(self.get_parameter('motion_lock_service').value))
        self.create_service(Trigger, '~/start', self._start_service)
        self.create_service(Trigger, '~/pause', self._pause_service)
        self.create_service(Trigger, '~/resume', self._resume_service)
        self.create_service(Trigger, '~/skip', self._skip_service)
        self.create_service(Trigger, '~/cancel', self._cancel_service)
        self.create_service(
            Trigger, '~/return_home', self._return_home_service)
        self.create_service(
            Trigger, '~/complete_current', self._complete_current_service)

        self._state = 'IDLE'
        self._detail = 'mission validated; waiting for start'
        self._index = 0
        self._returning = False
        self._last_completed_stop_id: str | None = None
        self._goal_handle = None
        self._goal_generation = 0
        self._goal_send_started_at: float | None = None
        self._goal_started_at: float | None = None
        self._goal_timeout_s: float | None = None
        self._wait_started_at: float | None = None
        self._cancel_started_at: float | None = None
        self._after_cancel: str | None = None
        self._cancel_goal_terminal = False
        self._cancel_lock_confirmed = False
        self._cancel_request_generation: int | None = None
        self._dwell_until: float | None = None
        self._paused_dwell_remaining: float | None = None
        self._lock_pending = False
        self._lock_target: bool | None = None
        self._lock_started_at: float | None = None
        self._lock_generation = 0
        self._base_lock_confirmed: bool | None = (
            True if not self._require_lock else None)
        self._pending_terminal: tuple[str, str] | None = None
        self._unsafe_active_goal = False
        self._run_id = ''
        self._start_pending = bool(self.get_parameter('autostart').value)
        self._log_path = self._make_log_path()
        self._timer = self.create_timer(0.1, self._tick)
        self._record('validated', {
            'mission_file': str(mission_file),
            'map_yaml': str(map_yaml),
            'keepout_yaml': str(keepout_yaml),
            'waypoints': len(self._waypoints),
            'unique_stops': len({point.stop_id for point in self._waypoints}),
        })
        self._publish_status()

    def _make_log_path(self) -> Path:
        configured = str(self.get_parameter('log_directory').value).strip()
        if configured:
            directory = Path(configured).expanduser()
        else:
            ros_log_dir = os.environ.get('ROS_LOG_DIR')
            base = (
                Path(ros_log_dir) if ros_log_dir else
                Path.home() / '.ros' / 'log')
            directory = base / 'agri_task_manager'
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')
        safe_id = ''.join(
            character if character.isalnum() or character in '-_' else '_'
            for character in self._mission.mission_id)
        return directory / f'{safe_id}_{stamp}.jsonl'

    def _record(self, event: str, extra: dict | None = None) -> None:
        payload = {
            'wall_time_s': time.time(),
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'mission_id': self._mission.mission_id,
            'run_id': self._run_id,
            'event': event,
            'state': self._state,
            'index': self._index,
        }
        if self._index < len(self._waypoints):
            payload['waypoint_id'] = self._waypoints[self._index].waypoint_id
        if extra:
            payload.update(extra)
        try:
            with self._log_path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(payload, ensure_ascii=False) + '\n')
        except OSError as exc:
            self.get_logger().error(f'unable to append mission log: {exc}')

    def _current_point(self) -> HarvestWaypoint | None:
        if 0 <= self._index < len(self._waypoints):
            return self._waypoints[self._index]
        return None

    def _grid_identity_errors(
        self,
        message: OccupancyGrid,
        expected_digest: str,
    ) -> list[str]:
        """Return identity differences for one live occupancy grid."""
        binding = self._mission.map_binding
        origin = message.info.origin
        quaternion = origin.orientation
        yaw = math.atan2(
            2.0 * (quaternion.w * quaternion.z +
                   quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y +
                         quaternion.z * quaternion.z),
        )
        actual_origin = (origin.position.x, origin.position.y, yaw)
        digest = hashlib.sha256(
            bytes(int(value) & 0xff for value in message.data)).hexdigest()
        errors = []
        if message.header.frame_id != binding.frame_id:
            errors.append('frame_id')
        if (message.info.width != binding.width_px or
                message.info.height != binding.height_px):
            errors.append('dimensions')
        if not math.isclose(
                message.info.resolution, binding.resolution_m,
                rel_tol=0.0, abs_tol=1e-9):
            errors.append('resolution')
        if any(not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1e-6)
               for actual, expected in zip(actual_origin, binding.origin)):
            errors.append('origin')
        if digest != expected_digest:
            errors.append('occupancy data')
        return errors

    def _runtime_binding_failed(self, detail: str) -> None:
        """Stop an active mission after a map or mask identity change."""
        if not hasattr(self, '_state') or self._state in {
                'IDLE', 'COMPLETED', 'FAILED', 'CANCELED'}:
            return
        if self._state in ACTIVE_GOAL_STATES:
            if self._after_cancel is None:
                self._begin_cancel('binding_mismatch')
            else:
                self._after_cancel = 'binding_mismatch'
                self._set_state('CANCELING', detail)
        else:
            self._fail(detail)

    def _map_callback(self, message: OccupancyGrid) -> None:
        """Fail closed unless the live map is the mission's bound map."""
        if not self._require_live_map:
            return
        was_valid = self._live_map_valid
        errors = self._grid_identity_errors(
            message, self._mission.map_binding.occupancy_sha256)
        if errors:
            self._live_map_valid = False
            self._live_map_error = (
                'live /map does not match mission binding: ' +
                ', '.join(errors))
            self._runtime_binding_failed(self._live_map_error)
            if was_valid and hasattr(self, '_state'):
                self._publish_status()
            return
        self._live_map_valid = True
        self._live_map_error = None
        if not was_valid and hasattr(self, '_state'):
            self._publish_status()

    def _keepout_callback(self, message: OccupancyGrid) -> None:
        """Fail closed unless the live keepout mask matches the mission."""
        if not self._require_live_keepout:
            return
        was_valid = self._live_keepout_valid
        errors = self._grid_identity_errors(
            message,
            self._mission.map_binding.keepout_occupancy_sha256,
        )
        if errors:
            self._live_keepout_valid = False
            self._live_keepout_error = (
                'live keepout mask does not match mission binding: ' +
                ', '.join(errors))
            self._runtime_binding_failed(self._live_keepout_error)
            if was_valid and hasattr(self, '_state'):
                self._publish_status()
            return
        self._live_keepout_valid = True
        self._live_keepout_error = None
        if not was_valid and hasattr(self, '_state'):
            self._publish_status()

    def _binding_error(self) -> str | None:
        """Return the current map or keepout failure, if any."""
        return self._live_map_error or self._live_keepout_error

    def _set_state(self, state: str, detail: str) -> None:
        changed = state != self._state or detail != self._detail
        self._state = state
        self._detail = detail
        if changed:
            self.get_logger().info(f'{state}: {detail}')
            self._record('state', {'detail': detail})
            self._publish_status()

    def _publish_status(self) -> None:
        point = self._current_point()
        payload = {
            'mission_id': self._mission.mission_id,
            'field_id': self._mission.field.field_id,
            'state': self._state,
            'detail': self._detail,
            'waypoint_index': self._index,
            'waypoint_count': len(self._waypoints),
            'returning': self._returning,
            'run_id': self._run_id,
            'live_map_valid': self._live_map_valid,
            'live_keepout_valid': self._live_keepout_valid,
            'log_path': str(self._log_path),
        }
        if point is not None and self._state not in TERMINAL_STATES:
            payload['current'] = point.as_event()
        self._status_pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    @staticmethod
    def _service_reply(response, success: bool, message: str):
        response.success = success
        response.message = message
        return response

    def _start_service(self, _request, response):
        if self._lock_pending:
            return self._service_reply(
                response, False, 'motion-lock transition is still active')
        if self._unsafe_active_goal:
            return self._service_reply(
                response, False,
                'a Nav2 goal or motion-lock transition is unconfirmed; '
                'restart the navigation stack before restarting this mission')
        if self._state not in {'IDLE', 'COMPLETED', 'FAILED', 'CANCELED'}:
            return self._service_reply(
                response, False, f'mission is already {self._state}')
        self._start_pending = True
        return self._service_reply(response, True, 'mission start queued')

    def _pause_service(self, _request, response):
        if self._lock_pending:
            return self._service_reply(
                response, False, 'motion-lock transition is still active')
        if self._state == 'AT_STATION':
            if self._dwell_until is not None:
                self._paused_dwell_remaining = max(
                    0.0, self._dwell_until - time.monotonic())
            self._dwell_until = None
            self._set_state('PAUSED', 'paused at harvest point')
            return self._service_reply(response, True, 'mission paused')
        if self._state in {'SENDING_GOAL', 'NAVIGATING', 'RETURNING'}:
            self._begin_cancel('pause')
            return self._service_reply(response, True, 'pause queued')
        return self._service_reply(
            response, False, f'cannot pause while {self._state}')

    def _resume_service(self, _request, response):
        if self._lock_pending:
            return self._service_reply(
                response, False, 'motion-lock transition is still active')
        if self._state != 'PAUSED':
            return self._service_reply(
                response, False, f'cannot resume while {self._state}')
        if self._paused_dwell_remaining is not None:
            self._dwell_until = (
                time.monotonic() + self._paused_dwell_remaining)
            self._paused_dwell_remaining = None
            self._set_state('AT_STATION', 'resumed at harvest point')
        elif self._returning:
            self._start_return_from_locked()
        else:
            self._advance_from_locked()
        return self._service_reply(response, True, 'mission resumed')

    def _skip_service(self, _request, response):
        if self._lock_pending:
            return self._service_reply(
                response, False, 'motion-lock transition is still active')
        if self._returning:
            return self._service_reply(
                response, False, 'cannot skip the return-home goal')
        if self._state in {'SENDING_GOAL', 'NAVIGATING'}:
            self._begin_cancel('skip')
            return self._service_reply(response, True, 'skip queued')
        if self._state in {'AT_STATION', 'PAUSED'}:
            self._paused_dwell_remaining = None
            self._finish_station(skipped=True)
            return self._service_reply(response, True, 'current point skipped')
        return self._service_reply(
            response, False, f'cannot skip while {self._state}')

    def _cancel_service(self, _request, response):
        if self._state in TERMINAL_STATES or self._state == 'IDLE':
            return self._service_reply(
                response, False, f'no active mission while {self._state}')
        if self._state == 'CANCELING':
            self._upgrade_cancel_command('cancel')
            return self._service_reply(response, True, 'cancel updated')
        if self._lock_pending:
            self._pending_terminal = (
                'CANCELED', 'mission canceled and base locked')
            self._set_state(
                'CANCELING', 'waiting for motion-lock transition to stop')
        elif self._state in {'SENDING_GOAL', 'NAVIGATING', 'RETURNING'}:
            self._begin_cancel('cancel')
        else:
            self._lock_then(
                lambda: self._terminal_after_lock(
                    'CANCELED', 'mission canceled and base locked'))
        return self._service_reply(response, True, 'cancel queued')

    def _return_home_service(self, _request, response):
        if self._state == 'CANCELING':
            return self._service_reply(
                response, False, 'goal cancellation is still in progress')
        if self._lock_pending:
            return self._service_reply(
                response, False, 'motion-lock transition is still active')
        if self._state in TERMINAL_STATES or self._state == 'IDLE':
            return self._service_reply(
                response, False, f'no active mission while {self._state}')
        if self._returning:
            return self._service_reply(
                response, False, 'already returning home')
        if self._state in {'SENDING_GOAL', 'NAVIGATING'}:
            self._begin_cancel('return_home')
        elif self._state in {'AT_STATION', 'PAUSED'}:
            self._start_return_from_locked()
        else:
            return self._service_reply(
                response, False, f'cannot return home while {self._state}')
        return self._service_reply(response, True, 'return-home queued')

    def _complete_current_service(self, _request, response):
        if self._lock_pending:
            return self._service_reply(
                response, False, 'motion-lock transition is still active')
        if self._state != 'AT_STATION':
            return self._service_reply(
                response, False, f'no current work while {self._state}')
        self._finish_station()
        return self._service_reply(response, True, 'current work completed')

    def _start_mission(self) -> None:
        self._start_pending = False
        self._index = 0
        self._returning = False
        self._last_completed_stop_id = None
        self._paused_dwell_remaining = None
        self._pending_terminal = None
        self._after_cancel = None
        self._cancel_goal_terminal = False
        self._cancel_lock_confirmed = False
        self._cancel_request_generation = None
        self._unsafe_active_goal = False
        self._run_id = str(self.get_clock().now().nanoseconds)
        self._publish_point_tombstone('mission_started')
        self._record('mission_started')
        self._wait_started_at = time.monotonic()
        self._set_state(
            'WAITING_FOR_SERVERS', 'waiting for Nav2 and motion lock')

    def _servers_ready(self) -> bool:
        if ((self._require_live_map and not self._live_map_valid) or
                (self._require_live_keepout and
                 not self._live_keepout_valid)):
            return False
        if not self._navigate.wait_for_server(timeout_sec=0.0):
            return False
        if self._require_lock and not self._lock_client.wait_for_service(
                timeout_sec=0.0):
            return False
        return True

    def _finish_pending_terminal(self, lock_is_set: bool) -> bool:
        """Honor a cancel/failure queued during a lock transition."""
        if self._pending_terminal is None:
            return False
        state, detail = self._pending_terminal
        if lock_is_set or not self._require_lock:
            self._pending_terminal = None
            self._terminal_after_lock(state, detail)
        else:
            self._pending_terminal = None
            self._lock_then(lambda: self._terminal_after_lock(state, detail))
        return True

    def _terminal_after_lock(self, state: str, detail: str) -> None:
        self._returning = False
        self._after_cancel = None
        self._cancel_started_at = None
        self._publish_point_tombstone(detail)
        self._set_state(state, detail)

    def _lock_transition_failed(self, detail: str) -> None:
        """Enter a non-restartable failure when lock state is unknown."""
        handle = self._goal_handle
        self._goal_generation += 1
        self._after_cancel = None
        self._cancel_started_at = None
        if handle is not None:
            handle.cancel_goal_async()
        self._lock_generation += 1
        self._lock_pending = False
        self._lock_target = None
        self._lock_started_at = None
        self._base_lock_confirmed = None
        self._pending_terminal = None
        self._unsafe_active_goal = True
        self._publish_point_tombstone(detail)
        self._set_state('FAILED', detail)

    def _lock_then(self, callback: Callable[[], None]) -> None:
        if self._lock_pending:
            self._fail('overlapping motion-lock request')
            return
        if not self._require_lock:
            self._base_lock_confirmed = True
            callback()
            return
        if self._base_lock_confirmed is True:
            callback()
            return
        if not self._lock_client.service_is_ready():
            self._lock_transition_failed(
                'motion-lock service is unavailable; base lock is unconfirmed')
            return
        self._lock_pending = True
        self._lock_target = True
        self._lock_started_at = time.monotonic()
        self._lock_generation += 1
        generation = self._lock_generation
        future = self._lock_client.call_async(SetBool.Request(data=True))

        def done(completed) -> None:
            if generation != self._lock_generation:
                return
            self._lock_pending = False
            self._lock_target = None
            self._lock_started_at = None
            try:
                result = completed.result()
            # rclpy future transports arbitrary errors.
            except Exception as exc:
                self._lock_transition_failed(
                    f'motion lock failed; base lock is unconfirmed: {exc}')
                return
            if not result.success:
                self._lock_transition_failed(
                    'motion lock rejected; base lock is unconfirmed: '
                    f'{result.message}')
                return
            self._base_lock_confirmed = True
            self._record('motion_locked')
            if self._finish_pending_terminal(lock_is_set=True):
                return
            callback()

        future.add_done_callback(done)

    def _unlock_then(self, callback: Callable[[], None]) -> None:
        if self._lock_pending:
            self._fail('overlapping motion-lock request')
            return
        if not self._require_lock:
            self._base_lock_confirmed = False
            callback()
            return
        if self._base_lock_confirmed is False:
            callback()
            return
        if not self._lock_client.service_is_ready():
            self._lock_transition_failed(
                'motion-lock service is unavailable during unlock')
            return
        self._lock_pending = True
        self._lock_target = False
        self._lock_started_at = time.monotonic()
        self._lock_generation += 1
        generation = self._lock_generation
        future = self._lock_client.call_async(SetBool.Request(data=False))

        def done(completed) -> None:
            if generation != self._lock_generation:
                return
            self._lock_pending = False
            self._lock_target = None
            self._lock_started_at = None
            try:
                result = completed.result()
            except Exception as exc:
                self._lock_transition_failed(
                    f'motion unlock failed; gate state is unknown: {exc}')
                return
            if not result.success:
                self._lock_transition_failed(
                    f'motion unlock rejected: {result.message}')
                return
            self._base_lock_confirmed = False
            self._record('motion_unlocked')
            if self._finish_pending_terminal(lock_is_set=False):
                return
            callback()

        future.add_done_callback(done)

    def _advance_from_locked(self) -> None:
        if self._binding_error() is not None:
            self._fail(self._binding_error() or 'map binding error')
            return
        if self._index >= len(self._waypoints):
            if self._mission.return_home:
                self._start_return_from_locked()
            else:
                self._set_state('COMPLETED', 'all harvest points completed')
            return
        point = self._current_point()
        if point.stop_id == self._last_completed_stop_id:
            self._enter_station(point, reused_pose=True)
            return
        self._set_state(
            'ADVANCING', f'unlocking for {point.waypoint_id}')
        self._unlock_then(lambda: self._send_goal(point.pose, point))

    def _start_return_from_locked(self) -> None:
        if self._binding_error() is not None:
            self._fail(self._binding_error() or 'map binding error')
            return
        self._publish_point_tombstone('return_home')
        self._returning = True
        self._dwell_until = None
        self._paused_dwell_remaining = None
        self._set_state('ADVANCING', 'unlocking for return-home goal')
        self._unlock_then(
            lambda: self._send_goal(self._mission.home, None))

    def _send_goal(
        self,
        pose: Pose2D,
        point: HarvestWaypoint | None,
    ) -> None:
        if self._binding_error() is not None:
            self._fail(self._binding_error() or 'map binding error')
            return
        if not self._navigate.server_is_ready():
            self._fail('NavigateToPose action server became unavailable')
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._mission.map_binding.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        goal.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        self._goal_generation += 1
        generation = self._goal_generation
        self._goal_send_started_at = time.monotonic()
        self._goal_timeout_s = (
            point.goal_timeout_s if point is not None else
            max(item.goal_timeout_s for item in self._waypoints))
        label = point.waypoint_id if point is not None else 'home'
        self._set_state('SENDING_GOAL', f'sending NavigateToPose goal {label}')
        self._record('goal_sent', {
            'label': label,
            'pose': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw},
        })
        future = self._navigate.send_goal_async(
            goal, feedback_callback=self._feedback_callback)
        future.add_done_callback(
            lambda completed: self._goal_response(
                completed, generation, label))

    def _goal_response(self, future, generation: int, label: str) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            if generation != self._goal_generation:
                return
            self._goal_send_started_at = None
            if self._after_cancel is not None:
                self._cancel_goal_terminal = True
                self._maybe_finish_cancel()
            else:
                self._fail(f'NavigateToPose send failed: {exc}')
            return
        if generation != self._goal_generation:
            if handle.accepted:
                handle.cancel_goal_async()
                self._record('stale_goal_canceled', {'label': label})
            return
        self._goal_send_started_at = None
        if not handle.accepted:
            if self._after_cancel is not None:
                self._cancel_goal_terminal = True
                self._maybe_finish_cancel()
            else:
                self._fail(f'NavigateToPose rejected {label}')
            return
        self._goal_handle = handle
        self._goal_started_at = time.monotonic()
        result_future = handle.get_result_async()
        result_future.add_done_callback(
            lambda completed: self._goal_result(completed, generation, label))
        if self._after_cancel is not None:
            self._cancel_accepted_goal()
            return
        state = 'RETURNING' if self._returning else 'NAVIGATING'
        self._set_state(state, f'goal {label} accepted')

    def _feedback_callback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        if hasattr(feedback, 'distance_remaining'):
            self._detail = (
                'distance remaining '
                f'{float(feedback.distance_remaining):.2f} m')

    def _goal_result(self, future, generation: int, label: str) -> None:
        if generation != self._goal_generation:
            return
        try:
            wrapped = future.result()
            status = wrapped.status
        except Exception as exc:
            self._unsafe_goal_failure(
                f'NavigateToPose result is unconfirmed: {exc}')
            return
        self._goal_handle = None
        self._goal_send_started_at = None
        self._goal_started_at = None
        if self._after_cancel is not None:
            self._cancel_goal_terminal = True
            self._maybe_finish_cancel()
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'NavigateToPose {label} ended with status {status}')
            return
        self._record('goal_succeeded', {'label': label})
        if self._returning:
            self._set_state('LOCKING_HOME', 'locking base at home')
            self._lock_then(lambda: self._terminal_after_lock(
                'COMPLETED', 'route complete and robot returned home'))
            return
        point = self._current_point()
        if point is None:
            self._fail('goal succeeded without a current waypoint')
            return
        self._last_completed_stop_id = point.stop_id
        self._set_state(
            'LOCKING_STATION', f'locking base at {point.stop_id}')
        self._lock_then(lambda: self._enter_station(point, reused_pose=False))

    def _enter_station(
        self,
        point: HarvestWaypoint,
        reused_pose: bool,
    ) -> None:
        event = point.as_event()
        event['mission_id'] = self._mission.mission_id
        event['run_id'] = self._run_id
        event['active'] = True
        event['reused_base_pose'] = reused_pose
        self._point_pub.publish(String(
            data=json.dumps(event, ensure_ascii=False, sort_keys=True)))
        self._record('at_harvest_point', event)
        self._dwell_until = None
        if not self._require_station_ack:
            self._dwell_until = time.monotonic() + point.station_dwell_s
        self._set_state(
            'AT_STATION',
            f'{point.plant.plant_id} ready on {point.work_side} side')

    def _publish_point_tombstone(self, reason: str) -> None:
        """Clear the transient station topic for late-joining consumers."""
        payload = {
            'mission_id': self._mission.mission_id,
            'run_id': self._run_id,
            'active': False,
            'reason': reason,
        }
        self._point_pub.publish(String(
            data=json.dumps(payload, ensure_ascii=False, sort_keys=True)))

    def _finish_station(self, skipped: bool = False) -> None:
        point = self._current_point()
        if point is not None:
            self._record(
                ('harvest_point_skipped' if skipped else
                 'harvest_point_completed'),
                point.as_event())
        self._publish_point_tombstone(
            'harvest_point_skipped' if skipped else
            'harvest_point_completed')
        self._dwell_until = None
        self._index += 1
        self._advance_from_locked()

    def _begin_cancel(self, command: str) -> None:
        if self._after_cancel is not None:
            self._upgrade_cancel_command(command)
            return
        self._after_cancel = command
        self._cancel_started_at = time.monotonic()
        self._cancel_goal_terminal = False
        self._cancel_lock_confirmed = not self._require_lock
        self._cancel_request_generation = None
        self._set_state('CANCELING', f'canceling goal for {command}')
        self._lock_then(self._cancel_lock_ready)
        if self._goal_handle is not None:
            self._cancel_accepted_goal()

    def _upgrade_cancel_command(self, command: str) -> None:
        """Promote an in-flight cancel to the safer requested outcome."""
        priorities = {
            'pause': 0,
            'skip': 1,
            'return_home': 2,
            'cancel': 3,
            'timeout': 4,
            'binding_mismatch': 5,
        }
        current = self._after_cancel
        if current is None or priorities[command] > priorities[current]:
            self._after_cancel = command
            self._set_state('CANCELING', f'canceling goal for {command}')

    def _cancel_lock_ready(self) -> None:
        """Record that the velocity gate is locked during cancellation."""
        self._cancel_lock_confirmed = True
        self._maybe_finish_cancel()

    def _maybe_finish_cancel(self) -> None:
        """Finish a cancel only after both Nav2 and the gate are safe."""
        if not self._cancel_goal_terminal or not self._cancel_lock_confirmed:
            return
        command = self._after_cancel
        if command is None:
            self._unsafe_goal_failure('cancel completed without an outcome')
            return
        self._after_cancel = None
        self._cancel_started_at = None
        self._cancel_request_generation = None
        self._goal_generation += 1
        self._after_locked_cancel(command)

    def _cancel_accepted_goal(self) -> None:
        if self._goal_handle is None:
            return
        generation = self._goal_generation
        if self._cancel_request_generation == generation:
            return
        handle = self._goal_handle
        self._cancel_request_generation = generation
        future = handle.cancel_goal_async()

        def done(completed) -> None:
            if (generation != self._goal_generation or
                    self._goal_handle is not handle or
                    self._after_cancel is None):
                return
            try:
                response = completed.result()
                accepted = bool(response.goals_canceling)
            except Exception as exc:
                self.get_logger().warning(
                    f'goal cancel request failed; waiting for result: {exc}')
                self._record('goal_cancel_request_failed', {
                    'error': str(exc),
                })
                return
            if not accepted:
                self.get_logger().warning(
                    'NavigateToPose did not acknowledge cancellation; '
                    'waiting for the goal result')
                self._record('goal_cancel_not_acknowledged')

        future.add_done_callback(done)

    def _after_locked_cancel(self, command: str) -> None:
        if command == 'pause':
            self._set_state('PAUSED', 'navigation paused at current waypoint')
        elif command == 'skip':
            point = self._current_point()
            if point is not None:
                self._record('harvest_point_skipped', point.as_event())
            self._publish_point_tombstone('harvest_point_skipped')
            self._index += 1
            self._advance_from_locked()
        elif command == 'return_home':
            self._start_return_from_locked()
        elif command == 'cancel':
            self._terminal_after_lock(
                'CANCELED', 'mission canceled and base locked')
        elif command == 'timeout':
            self._terminal_after_lock(
                'FAILED', 'navigation timed out and base locked')
        elif command == 'binding_mismatch':
            self._terminal_after_lock(
                'FAILED', self._binding_error() or
                'live map binding no longer matches the mission')
        else:
            self._fail(f'unknown post-cancel command {command}')

    def _unsafe_goal_failure(self, message: str) -> None:
        """Lock permanently when Nav2 goal cancellation is unconfirmed."""
        self._unsafe_active_goal = True
        self._fail(message, preserve_goal=True)

    def _fail(self, message: str, preserve_goal: bool = False) -> None:
        self.get_logger().error(message)
        self._after_cancel = None
        self._cancel_started_at = None
        self._cancel_request_generation = None
        self._goal_generation += 1
        if not preserve_goal:
            self._goal_handle = None
        self._goal_send_started_at = None
        self._goal_started_at = None
        self._publish_point_tombstone(message)
        if self._lock_pending:
            self._pending_terminal = ('FAILED', message)
            self._set_state('FAILED', message)
            return
        if self._state == 'FAILED':
            self._set_state('FAILED', message)
            return
        if self._require_lock and not self._lock_client.service_is_ready():
            self._set_state('FAILED', message)
            return
        self._lock_then(lambda: self._set_state('FAILED', message))

    def _tick(self) -> None:
        now = time.monotonic()
        if (self._lock_pending and self._lock_started_at is not None and
                now - self._lock_started_at > self._lock_timeout):
            action = 'lock' if self._lock_target else 'unlock'
            self._lock_transition_failed(
                f'motion {action} request timed out; gate state is unknown')
            return
        if (self._state == 'CANCELING' and
                self._cancel_started_at is not None and
                now - self._cancel_started_at > self._cancel_timeout):
            command = self._after_cancel or 'cancel'
            # The old Nav2 goal may still be active when cancellation times
            # out. Never unlock the base or send a replacement goal in that
            # situation; retain a non-restartable failure for inspection.
            self._unsafe_goal_failure(
                f'NavigateToPose cancellation timed out during {command}; '
                'base remains locked if the lock request succeeded')
            return
        if self._lock_pending:
            return
        if self._start_pending and self._state in {
                'IDLE', 'COMPLETED', 'FAILED', 'CANCELED'}:
            self._start_mission()
            return
        if self._state == 'WAITING_FOR_SERVERS':
            if self._binding_error() is not None:
                self._fail(self._binding_error() or 'map binding error')
            elif self._servers_ready():
                self._set_state(
                    'PREPARING', 'locking base before mission start')
                self._lock_then(self._advance_from_locked)
            elif (self._wait_started_at is not None and
                  now - self._wait_started_at > self._server_wait_timeout):
                self._fail('timed out waiting for Nav2 or motion-lock service')
            return
        if (self._state == 'SENDING_GOAL' and
                self._goal_send_started_at is not None and
                now - self._goal_send_started_at >
                self._goal_response_timeout):
            self._begin_cancel('timeout')
            return
        if (self._state in {'NAVIGATING', 'RETURNING'} and
                self._goal_started_at is not None and
                self._goal_timeout_s is not None and
                now - self._goal_started_at > self._goal_timeout_s):
            self._begin_cancel('timeout')
            return
        if (self._state == 'AT_STATION' and
                self._dwell_until is not None and now >= self._dwell_until):
            self._finish_station()

    def stop(self) -> None:
        """Cancel and lock with short bounded waits during orderly shutdown."""
        if not rclpy.ok():
            return
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if self._require_lock and self._lock_client.service_is_ready():
            future = self._lock_client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)


def main(args=None) -> None:
    """Run the field-route executor until shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = FieldRouteExecutor()
        rclpy.spin(node)
    except (MissionConfigError, ValueError) as exc:
        if node is not None:
            node.get_logger().fatal(str(exc))
        else:
            print(f'field_route_executor: {exc}')
        raise SystemExit(2) from exc
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

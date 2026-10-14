"""Trajectory deadlines follow ROS time while independent wall bounds remain."""

import math


class TrajectoryDeadline:
    """Wait for actual controller terminal state, bounded in both time domains."""

    def __init__(self, trajectory_duration, ros_now_ns, wall_now, maximum_sim_time,
                 maximum_wall_time, clock_stall_time, margin_factor=1.25, margin_s=2.0):
        values = (trajectory_duration, maximum_sim_time, maximum_wall_time,
                  clock_stall_time, margin_factor, margin_s)
        if not all(math.isfinite(value) and value > 0 for value in values):
            raise ValueError('trajectory duration and deadline bounds must be finite and positive')
        if trajectory_duration+.5 >= maximum_sim_time:
            raise ValueError('trajectory duration exceeds configured simulation-time bound')
        self.duration = trajectory_duration
        self.sim_limit = min(maximum_sim_time, trajectory_duration*margin_factor+margin_s)
        self.wall_limit, self.stall_limit = maximum_wall_time, clock_stall_time
        self.start_ros, self.start_wall = ros_now_ns, wall_now
        self.last_ros, self.last_progress_wall = ros_now_ns, wall_now
        self.report = {}
        self.update(ros_now_ns, wall_now)

    def update(self, ros_now_ns, wall_now):
        wall_elapsed = wall_now-self.start_wall
        sim_elapsed = (ros_now_ns-self.start_ros)/1e9
        if ros_now_ns > self.last_ros:
            self.last_progress_wall = wall_now
        error = None
        if ros_now_ns < self.last_ros:
            error = 'ROS time moved backwards during controller execution'
        elif wall_elapsed >= self.wall_limit:
            error = 'hard wall execution deadline reached'
        elif sim_elapsed >= self.sim_limit:
            error = 'trajectory simulation-time deadline reached'
        elif wall_now-self.last_progress_wall >= self.stall_limit:
            error = 'ROS clock stopped advancing during controller execution'
        self.last_ros = ros_now_ns
        self.report = {'trajectory_duration_sim_s': self.duration,
                       'execution_sim_budget_s': self.sim_limit,
                       'execution_wall_budget_s': self.wall_limit,
                       'elapsed_sim_s': sim_elapsed, 'elapsed_wall_s': wall_elapsed,
                       'observed_realtime_factor': sim_elapsed/wall_elapsed if wall_elapsed > 0 else None,
                       'clock_stall_wall_s': wall_now-self.last_progress_wall}
        return error

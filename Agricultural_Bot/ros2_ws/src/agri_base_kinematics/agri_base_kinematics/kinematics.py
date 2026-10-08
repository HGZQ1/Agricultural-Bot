"""Dependency-light four-wheel-steering kinematics and odometry helpers."""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin
from typing import Dict, List, Mapping, Sequence, Tuple


def normalize_angle(angle: float) -> float:
    """Return an angle in [-pi, pi)."""
    return (angle + pi) % (2.0 * pi) - pi


def apply_joint_signs(values: Sequence[float], signs: Sequence[float]) -> List[float]:
    """
    Convert joint values to physical values, or vice versa.

    Sign-only coordinate transforms are self-inverse, so the same helper is
    deliberately used on controller commands and joint-state feedback.
    """
    if len(values) != len(signs):
        raise ValueError('values and signs must have the same length')
    return [float(value) * float(sign) for value, sign in zip(values, signs)]


def steering_feedback_to_physical(
    values: Sequence[float],
    signs: Sequence[float],
    offsets: Sequence[float],
) -> List[float]:
    """
    Convert steering-joint feedback to physical wheel headings.

    The offsets describe the physical wheel heading at joint position zero.
    This accounts for the small yaw offsets present in the imported CAD wheel
    modules while keeping the kinematic model in the base frame.
    """
    if not (len(values) == len(signs) == len(offsets)):
        raise ValueError('values, signs and offsets must have the same length')
    return [
        normalize_angle(float(value) * float(sign) + float(offset))
        for value, sign, offset in zip(values, signs, offsets)
    ]


def physical_steering_to_joint(
    values: Sequence[float],
    signs: Sequence[float],
    offsets: Sequence[float],
) -> List[float]:
    """Convert requested physical wheel headings to joint positions."""
    if not (len(values) == len(signs) == len(offsets)):
        raise ValueError('values, signs and offsets must have the same length')
    return [
        normalize_angle(float(value) - float(offset)) * float(sign)
        for value, sign, offset in zip(values, signs, offsets)
    ]


def steering_alignment_scale(
    errors: Sequence[float],
    full_speed_error: float,
    stop_error: float,
) -> float:
    """
    Return a global wheel-speed scale for the measured steering errors.

    Wheel drive is held at zero while any steering module is far from its
    requested angle.  Between the two thresholds the scale rises linearly.
    Using one global scale prevents individual wheels from dragging the base
    while the other modules are still changing direction.
    """
    if (not isfinite(full_speed_error) or not isfinite(stop_error) or
            full_speed_error < 0.0 or stop_error <= full_speed_error):
        raise ValueError(
            'steering thresholds must satisfy 0 <= full_speed_error < stop_error')
    normalized_errors = [normalize_angle(float(error)) for error in errors]
    if not all(isfinite(error) for error in normalized_errors):
        raise ValueError('steering errors must be finite')
    maximum_error = max((abs(error) for error in normalized_errors), default=0.0)
    if maximum_error <= full_speed_error:
        return 1.0
    if maximum_error >= stop_error:
        return 0.0
    return (stop_error - maximum_error) / (stop_error - full_speed_error)


def integrate_planar_pose(
    x: float,
    y: float,
    yaw: float,
    vx: float,
    vy: float,
    wz: float,
    dt: float,
) -> Tuple[float, float, float]:
    """Integrate a body-frame twist with midpoint heading integration."""
    if dt <= 0.0:
        return x, y, yaw
    middle_yaw = yaw + 0.5 * wz * dt
    return (
        x + (vx * cos(middle_yaw) - vy * sin(middle_yaw)) * dt,
        y + (vx * sin(middle_yaw) + vy * cos(middle_yaw)) * dt,
        normalize_angle(yaw + wz * dt),
    )


@dataclass(frozen=True)
class WheelCommand:
    """Steering angle in radians and wheel angular velocity in rad/s."""

    steering: float
    velocity: float


class FourWheelSteeringKinematics:
    """
    Inverse kinematics and odometry for a planar four-wheel-steering base.

    Wheel positions are expressed in the base frame as (x, y), with +x forward
    and +y to the left.  Each steering joint rotates about +z in this model;
    a wheel command is therefore represented by one planar angle and one
    angular wheel velocity.
    """

    def __init__(
        self,
        wheel_positions: Mapping[str, Sequence[float]],
        wheel_radius: float,
        max_wheel_speed: float = 2.0,
        steering_limits: Tuple[float, float] = (-pi, pi),
    ) -> None:
        if wheel_radius <= 0.0:
            raise ValueError('wheel_radius must be positive')
        if len(wheel_positions) != 4:
            raise ValueError('exactly four wheel positions are required')
        self.wheel_positions = {
            name: (float(position[0]), float(position[1]))
            for name, position in wheel_positions.items()
        }
        self.wheel_radius = float(wheel_radius)
        self.max_wheel_speed = abs(float(max_wheel_speed))
        self.steering_limits = steering_limits

    def inverse(
        self,
        vx: float,
        vy: float,
        wz: float,
        current_steering: Mapping[str, float] | None = None,
    ) -> Dict[str, WheelCommand]:
        """
        Compute wheel commands for a body twist.

        For a wheel whose desired direction differs by more than 90 degrees
        from its current direction, the steering angle is flipped by pi and
        the wheel velocity sign is inverted.  This avoids unnecessary full
        steering rotations while preserving the commanded velocity vector.
        """
        current_steering = current_steering or {}
        result: Dict[str, WheelCommand] = {}
        is_zero = hypot(vx, vy) < 1e-12 and abs(wz) < 1e-12
        for name, (x, y) in self.wheel_positions.items():
            local_x = vx - wz * y
            local_y = vy + wz * x
            speed = hypot(local_x, local_y) / self.wheel_radius
            angle = atan2(local_y, local_x) if speed > 1e-12 else current_steering.get(name, 0.0)
            if not is_zero and name in current_steering:
                delta = normalize_angle(angle - current_steering[name])
                if abs(delta) > pi / 2.0:
                    angle = normalize_angle(angle + pi)
                    speed = -speed
            angle = max(self.steering_limits[0], min(self.steering_limits[1], angle))
            if is_zero:
                speed = 0.0
            result[name] = WheelCommand(angle, speed)

        peak_speed = max(abs(command.velocity) for command in result.values())
        if self.max_wheel_speed > 0.0 and peak_speed > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak_speed
            result = {
                name: WheelCommand(command.steering, command.velocity * scale)
                for name, command in result.items()
            }
        return result

    def body_velocity(
        self,
        steering: Mapping[str, float],
        wheel_velocity: Mapping[str, float],
    ) -> Tuple[float, float, float]:
        """Estimate (vx, vy, wz) from measured wheel states by least squares."""
        rows: List[Tuple[Tuple[float, float, float], float]] = []
        for name, (x, y) in self.wheel_positions.items():
            if name not in steering or name not in wheel_velocity:
                continue
            linear_speed = wheel_velocity[name] * self.wheel_radius
            theta = steering[name]
            # vx - y*wz = wheel_v*cos(theta)
            rows.append(((1.0, 0.0, -y), linear_speed * cos(theta)))
            # vy + x*wz = wheel_v*sin(theta)
            rows.append(((0.0, 1.0, x), linear_speed * sin(theta)))
        if len(rows) < 3:
            return 0.0, 0.0, 0.0
        ata = [[0.0] * 3 for _ in range(3)]
        atb = [0.0] * 3
        for row, value in rows:
            for i in range(3):
                atb[i] += row[i] * value
                for j in range(3):
                    ata[i][j] += row[i] * row[j]
        return tuple(_solve_3x3(ata, atb))  # type: ignore[return-value]


def _solve_3x3(matrix: List[List[float]], vector: List[float]) -> List[float]:
    """Solve a 3x3 system with pivoting, without a numpy runtime dependency."""
    augmented = [matrix[i][:] + [vector[i]] for i in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            return [0.0, 0.0, 0.0]
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(3):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                augmented[row][index] - factor * augmented[column][index]
                for index in range(4)
            ]
    return [augmented[i][3] for i in range(3)]

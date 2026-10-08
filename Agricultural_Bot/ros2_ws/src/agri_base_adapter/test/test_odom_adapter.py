import math

from agri_base_adapter.odom_adapter_node import odometry_is_finite
from nav_msgs.msg import Odometry


def valid_odometry():
    message = Odometry()
    message.pose.pose.orientation.w = 1.0
    message.pose.covariance[0] = 0.01
    message.twist.covariance[0] = 0.02
    return message


def test_finite_normalized_odometry_is_accepted():
    assert odometry_is_finite(valid_odometry())


def test_nonfinite_covariance_is_rejected():
    message = valid_odometry()
    message.pose.covariance[5] = math.nan
    assert not odometry_is_finite(message)


def test_invalid_quaternion_is_rejected():
    message = valid_odometry()
    message.pose.pose.orientation.w = 0.0
    assert not odometry_is_finite(message)

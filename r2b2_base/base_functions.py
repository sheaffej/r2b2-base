from math import pi
from typing import Tuple

from nav_msgs.msg import Odometry

from roboclaw_interfaces.msg import SpeedCommand
from r2b2_base.odometry_helpers import (
    create_odometry_message, calc_world_frame_pose, calc_world_frame_velocity
)


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  Below are used by the BaseNode directly
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def calc_create_speed_cmd(
    x_linear_cmd: float,
    z_angular_cmd: float,
    wheel_dist: float,
    wheel_radius: float,
    wheel_slip_factor: float,
    ticks_per_rotation: int,
    max_drive_secs: int,
    max_qpps: int,
    max_accel: int
) -> SpeedCommand:
    """Calculate and send motor commands

    Args:
        x_linear_cmd (float): Twist message's linear.x value
        z_angular_cmd(float): Twist message's angular.z value
        wheel_dist (float): Distance between wheels (m)
        wheel_radius (float): Wheel radius (m)
        wheel_slip_factor (float): Decimal % of angular motion lost to slip
        ticks_per_rotation (int): Number of encoder ticks per wheel rotation
        max_drive_secs (int): Maximum seconds drive should run before stopping
        max_accel (int): Max QPPS of acceleration

    Returns: roboclaw_interfaces.msg.SpeedCommand
    """
    # Wheels only slip when turning (i.e. z_angular_cmd > 0), and then increase z_angular_cmd
    if (wheel_slip_factor > 0.0):
        z_angular_cmd = z_angular_cmd / wheel_slip_factor

    right_angular_v = (
        (x_linear_cmd + z_angular_cmd * (wheel_dist / 2.0)) / wheel_radius
    )
    left_angular_v = (
        (x_linear_cmd - z_angular_cmd * (wheel_dist / 2.0)) / wheel_radius
    )

    ticks_per_radian = ticks_per_rotation / (pi * 2)

    right_qpps_target = right_angular_v * ticks_per_radian
    left_qpps_target = left_angular_v * ticks_per_radian

    # Clamp and scale the target QPPS within the max_qpps
    if abs(right_qpps_target) > max_qpps or abs(left_qpps_target) > max_qpps:

        # Find the largest and smallest values
        if abs(right_qpps_target) >= abs(left_qpps_target):
            largest_qpps = right_qpps_target
            smallest_qpps = left_qpps_target
        else:
            largest_qpps = left_qpps_target
            smallest_qpps = right_qpps_target

        # Find the ratio of the smallest to the largest
        smallest_ratio = float(smallest_qpps) / largest_qpps

        # Scale the largest by clamping it
        largest_qpps_scaled = max(-max_qpps, min(largest_qpps, max_qpps))

        # Scale the smallest
        smallest_qpps_scaled = largest_qpps_scaled * smallest_ratio

        # Reassign to the targets
        if largest_qpps == right_qpps_target:
            right_qpps_target = largest_qpps_scaled
            left_qpps_target = smallest_qpps_scaled
        else:
            right_qpps_target = smallest_qpps_scaled
            left_qpps_target = largest_qpps_scaled

    cmd = SpeedCommand()
    cmd.m1_qpps = int(right_qpps_target)
    cmd.m2_qpps = int(left_qpps_target)
    cmd.max_secs = max_drive_secs
    cmd.accel = max_accel
    return cmd


def calc_base_frame_velocity_from_encoder_diffs(
    m1_enc_diff, m2_enc_diff,
    ticks_per_rotation, wheel_radius, wheel_dist, wheel_slip_factor,
    begin_odom_time, end_odom_time
) -> Tuple[float, float, float]:

    # print("begin_odom_time:", begin_odom_time.to_sec())
    # print("end_odom_time:", end_odom_time.to_sec())
    time_delta_secs = (end_odom_time - begin_odom_time).nanoseconds / 1e9
    # print("time_delta_secs: {}".format(time_delta_secs))

    m1_qpps_actual, m2_qpps_actual = _calc_qpps(m1_enc_diff, m2_enc_diff, time_delta_secs)
    # print("m1_qpps_actual: {} / {} = {}".format(m1_enc_diff, time_delta_secs, m1_qpps_actual))
    # print("m2_qpps_actual: {} / {} = {}".format(m2_enc_diff, time_delta_secs, m2_qpps_actual))

    # Calculate how fast the wheel is rotating around the axel
    left_angular_v, right_angular_v = _calc_wheel_angular_velocity(
        m1_qpps_actual, m2_qpps_actual, ticks_per_rotation
    )
    # print("right_angular_v: {}".format(right_angular_v))
    # print("left_angular_v: {}".format(left_angular_v))

    # Then calculate how fast the wheel is traveling in a line
    left_linear_v, right_linear_v = _calc_wheel_linear_velocity(
        left_angular_v, right_angular_v, wheel_radius
    )
    # print("right_linear_v: {}".format(right_linear_v))
    # print("left_linear_v: {}".format(left_linear_v))

    x_linear_v, y_linear_v, z_angular_v = _calc_base_frame_velocity(
        left_linear_v, right_linear_v, wheel_dist, wheel_slip_factor
    )

    return x_linear_v, y_linear_v, z_angular_v


def calc_odometry_from_base_velocity(
    x_linear_v, y_linear_v, z_angular_v,
    world_x, world_y, world_theta,
    time_delta_secs, odom_time,
    base_frame_id, world_frame_id
) -> Odometry:
    # Helpful rotation calculator
    # https://www.andre-gaschler.com/rotationconverter/
    world_x_velocity, world_y_velocity, world_angular_velocity = calc_world_frame_velocity(
        x_linear_v, y_linear_v, z_angular_v, world_theta)

# ------------
    # rclpy.logging.get_logger("base_node").info(f"Velocities x_lin {x_linear_v}, y_lin {y_linear_v}, z_ang {z_angular_v}")
# ------------

    world_x, world_y, world_theta = calc_world_frame_pose(
        world_x_velocity, world_y_velocity, world_angular_velocity,
        world_x, world_y, world_theta, time_delta_secs)

# ------------
    # rclpy.logging.get_logger("base_node").info("world coordinates: (x:{}, y:{}, th:{})".format(world_x, world_y, world_theta))
# ------------

    odom = create_odometry_message(
        world_x, world_y, world_theta,
        world_x_velocity, world_y_velocity, world_angular_velocity,
        odom_time, base_frame_id, world_frame_id
    )
    return odom


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  Below are used by the functions above
#  Separated out so they can be unit tested
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


def _calc_qpps(m1_enc_diff, m2_enc_diff, time_delta_secs) -> Tuple[int, int]:
    if time_delta_secs > 0:
        m1_qpps_actual = m1_enc_diff / float(time_delta_secs)
        m2_qpps_actual = m2_enc_diff / float(time_delta_secs)
    else:
        m1_qpps_actual = m2_qpps_actual = 0
    return (m1_qpps_actual, m2_qpps_actual)


def _calc_wheel_angular_velocity(m1_qpps, m2_qpps, ticks_per_rotation) -> Tuple[float, float]:
    ticks_per_radian = ticks_per_rotation / (2 * pi)
    if ticks_per_radian <= 0.0:
        return (0.0, 0.0)
    right_angular_v = m1_qpps / float(ticks_per_radian)
    left_angular_v = m2_qpps / float(ticks_per_radian)
    return (left_angular_v, right_angular_v)


def _calc_wheel_linear_velocity(left_angular_v, right_angular_v, wheel_radius) -> Tuple[float, float]:
    if wheel_radius <= 0.0:
        wheel_radius = 0.0  # Set it to zero so the linear_v calculates to 0.0
    right_linear_v = right_angular_v * float(wheel_radius)
    left_linear_v = left_angular_v * float(wheel_radius)
    return (left_linear_v, right_linear_v)


def _calc_base_frame_velocity(left_linear_v, right_linear_v, wheel_dist, wheel_slip_factor) -> Tuple[float, float, float]:
    x_linear_v = (right_linear_v + left_linear_v) / 2.0
    y_linear_v = 0  # Because the robot is nonholonomic
    z_angular_v = (right_linear_v - left_linear_v) / float(wheel_dist)

    # Reduce z_angular_v to compensate for wheel slip in turns
    z_angular_v = z_angular_v * wheel_slip_factor

    return (x_linear_v, y_linear_v, z_angular_v)

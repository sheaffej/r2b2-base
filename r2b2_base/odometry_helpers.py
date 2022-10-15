from math import pi, sin, cos
from numpy import sign

from transforms3d._gohlketransforms import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from rclpy.time import Time


def yaw_from_odom_message(odom):
    """Converts an Odometry message into an Euler yaw value
    Parameters:
        :param Odometry odom:

    :rtype: float
    """

    return euler_from_quaternion(
        [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ])[2]


def heading_from_odometry(odom):
    return yaw_from_odom_message(odom)


def normalize_theta(theta):
    """Convert the result of adding or subtracting angular movements to a theta
    that falls within the range of 0.0 >= theta => 2pi, where theta is always positive.
    """
    norm_theta = theta % (pi * 2)
    if norm_theta < 0.0:
        norm_theta = (pi * 2) + norm_theta
    return norm_theta


def add_radians(a, b):
    """Adds two radian angles, and adjusts sign +/- to be closest to 0
    Parameters:
        a (float): Angle a
        b (float): Angle b
    Returns (float): The resulting angle in radians, with sign adjusted

    """
    return (a + b + pi) % (2 * pi) - pi


def radians_between(a: float, b: float) -> float:
    """Calculates the absolute minimum radians between two angles.

    Args:
        a (float): First angle in radians
        b (float): Second angle in radians
    """

    # Example if the angles can go from: 0.0 -- 6.0

    # a=1.0, b=5.0:  diff is 4.0 or [2.0] <-- this is the answer
    # 1 - 5 = -4 
    #   Since the answer is negative, subtract from the max --> 6 + -4 = 2
    # 5 - 1 = 4
    # Take the smallest answer min(2, 4) --> 2

    # a=0.5, b=5.5: 5.0, [1.0]
    # 0.5 - 5.5 = -5 -> 6 + -5 = 1.0
    # 5.5 - 0.5 = 5.0

    d1 = a - b
    if d1 < 0.0:
        d1 = (2 * pi) + d1
    d2 = b - a
    if d2 < 0.0:
        d2 = (2 * pi) + d2
    return min(d1, d2)


def calc_steering_angle(current_heading, target_heading):
    diff_angle = normalize_theta(target_heading) - normalize_theta(current_heading)
    _sign = sign(diff_angle)
    if abs(diff_angle) > pi:
        # Subtract from a full circle
        diff_angle = (2 * pi) - abs(diff_angle)
        # Reverse the sign
        diff_angle = diff_angle * (_sign * -1)
    return diff_angle


def calc_world_frame_pose(world_x_velocity, world_y_velocity, world_angular_velocity,
                          begin_world_x, begin_world_y, begin_world_theta, time_delta_secs):
    """Given world velocity vectors, movement duration, and beginning world coordinates
    calculate the new world coordinates.
    """
    new_world_x = begin_world_x + (world_x_velocity * time_delta_secs)
    new_world_y = begin_world_y + (world_y_velocity * time_delta_secs)
    new_world_theta = begin_world_theta + (world_angular_velocity * time_delta_secs)
    new_world_theta = normalize_theta(new_world_theta)
    return (new_world_x, new_world_y, new_world_theta)


def calc_world_frame_velocity(x_linear_v, y_linear_v, z_angular_v, world_theta):
    # 2D rotation matrix math https://en.wikipedia.org/wiki/Rotation_matrix
    # But since y_linear_v = 0, we don't actually need the second part of each equation
    world_x_velocity = x_linear_v * cos(world_theta) - y_linear_v * sin(world_theta)
    world_y_velocity = x_linear_v * sin(world_theta) + y_linear_v * cos(world_theta)
    world_angular_velocity = z_angular_v
    return (world_x_velocity, world_y_velocity, world_angular_velocity)


def create_odometry_message(world_x, world_y, world_theta,
                            world_x_linear_v, world_y_linear_v, world_z_angular_v,
                            odom_time: Time, base_frame_id, world_frame_id):
    # Convert world orientation (theta) to a Quaternion for use with tf and Odometry
    quat_vals = quaternion_from_euler(0, 0, world_theta)
    quat = Quaternion()
    quat.x = quat_vals[0]
    quat.y = quat_vals[1]
    quat.z = quat_vals[2]
    quat.w = quat_vals[3]

    odom = Odometry()
    odom.header.stamp = odom_time.to_msg()
    odom.header.frame_id = world_frame_id
    odom.pose.pose.position.x = world_x
    odom.pose.pose.position.y = world_y
    odom.pose.pose.position.z = 0.0   # Because this robot can't fly to a vertical position
    odom.pose.pose.orientation = quat
    odom.child_frame_id = base_frame_id
    odom.twist.twist.linear.x = world_x_linear_v
    odom.twist.twist.linear.y = world_y_linear_v
    odom.twist.twist.angular.z = world_z_angular_v
    return odom

import math

import pytest
from rclpy.time import Time

from nav_msgs.msg import Odometry

from r2b2_base.odometry_helpers import (
    normalize_theta, calc_world_frame_pose, calc_steering_angle,
    create_odometry_message, yaw_from_odom_message
)


pi = math.pi
twopi = pi * 2


def test_normalize_theta():
    # (input_theta, exp_normal_theta)
    tests = [
        # Positive & Negatives
        (pi, pi),               # 180 = 180
        (-pi, pi),              # -180 = 180
        (twopi, 0.0),           # 360 = 0
        (-twopi, 0.0),          # -360 = 0
        (0, 0),                 # 0 = 0
        (pi / 2 - pi, pi * 1.5),  # -90 = 270

        # More that 2 Pi cases
        (twopi + pi, pi),       # 3 pi = pi
        (-twopi - pi, pi),      # -3 pi = pi
        (pi + pi * 1.5, pi / 2)   # 2.5 pi = 0.5 pi
    ]

    for input_theta, exp_normal_theta in tests:
        normal_theta = normalize_theta(input_theta)
        print("Input theta: {}, actual: {}, expected: {}".format(
            input_theta, normal_theta, exp_normal_theta))
        pytest.approx(normal_theta, exp_normal_theta)


def test_calc_steering_angle():
    # (current_heading, target_heading, exp_steering_angle)
    tests = [
        (pi / 2, pi, pi / 2),       # 90 deg -> 180 deg = +90 deg
        (pi / 4, -pi / 4, -pi / 2),   # 45 deg -> -45 deg = -90 deg
        (1.1, 0.0, -1.1),
        (twopi - 1.0, 0.0, 1.0),
        (1.0, twopi - 1.0, -2.0),
    ]

    for (current_heading, target_heading, exp_steering_angle) in tests:
        steering_angle = calc_steering_angle(current_heading, target_heading)
        print(
            "Current heading: {}, Target heading: {} ==> "
            "Steering angle actual: {}, expected: {}".format(
                current_heading, target_heading, steering_angle, exp_steering_angle))
        pytest.approx(steering_angle, exp_steering_angle)


def test_calc_world_frame_pose():
    """Tests the odometry_helpers.calc_world_frame_pose() function.
    Inputs: world x-y-theta velocities, and world starting coordinates, and duration
    Outputs: new world x-y-theta pose
    """
    tests = [
        # (world_x_velocity, world_y_velocity, world_angular_velocity,
        #  begin_world_x, begin_world_y, begin_world_theta, time_delta_secs),
        # ==> (new_world_x, new_world_y, new_world_theta)

        # Drive straight forward at 0.5 m/s for 1 sec from origin
        ((0.5, 0.0, 0.0,
            0.0, 0.0, 0.0, 1),
            (0.5, 0.0, 0.0)),

        # Rotate left at 1 r/s for 1 sec from origin
        ((0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 1),
            (0.0, 0.0, 1.0)),

        # Rotate left at 3 r/s for 3 sec from origin
        ((0.0, 0.0, 3.0,
            0.0, 0.0, 0.0, 3),
            (0.0, 0.0, (3 * 3) % math.pi)),

        # Rotate right at 1 r/s for 1 sec from origin
        ((0.0, 0.0, -1.0,
            0.0, 0.0, 0.0, 1),
            (0.0, 0.0, 5.28)),

        # Drive x = 1.0 m/s, y = 1.0ms/s and rotation 1.0 r/s for 1 sec
        # from the origin and 0.0 heading
        ((1.0, 1.0, 1.0,
            0.0, 0.0, 0.0, 1),
            (1.0, 1.0, 1.0)),

        # Drive x = 1.0 m/s, y = 1.0ms/s and rotation 1.0 r/s for 1 sec
        # from the location (-123, 345) heading = 4 radians
        ((1.0, 1.0, 1.0,
            -123.0, 345.0, 4.0, 1),
            (-122.0, 346.0, 5.0)),
    ]

    for inputs, expects in tests:
        (world_x_velocity, world_y_velocity, world_angular_velocity,
            begin_world_x, begin_world_y, begin_world_theta, time_delta_secs) = inputs

        exp_world_x, exp_world_y, exp_world_theta = expects

        new_world_x, new_world_y, new_world_theta = calc_world_frame_pose(
            world_x_velocity, world_y_velocity, world_angular_velocity,
            begin_world_x, begin_world_y, begin_world_theta,
            time_delta_secs
        )

        pytest.approx(new_world_x, exp_world_x)
        pytest.approx(new_world_y, exp_world_y)
        pytest.approx(new_world_theta, exp_world_theta)


def test_yaw_from_odom_message():
    # This test was created because ROS1 tf.euler_from_quaternion expects (x, y, z, w)
    # but transforms3d._gohlketransforms.euler_from_quaternion expects (w, x, y, z)

    tests = [
        ((-0.0, 0.0, 0.11773918055157037, -0.99304455356396), 6.047159470221761)     # quat(x, y, z, w), theta
    ]

    for quat, exp_theta in tests:
        odom = Odometry()
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        assert normalize_theta(yaw_from_odom_message(odom)) == pytest.approx(exp_theta)


def test_create_odom_message():
    # This test was created because ROS1 tf.quaternion_from_euler returns (x, y, z, w)
    # but transforms3d._gohlketransforms.quaternion_from_euler returns (w, x, y, z)

    tests = [
        (1.00020, -0.97575, 6.047159,   # world_x, world_y, world_theta
        0.0, 0.0, 0.0,                  # world_x_linear_v, world_y_linear_v, world_z_angular_v
        Time(), 'base_link', 'odom',    # odom_time, base_frame_id, world_frame_id
        (0.0, 0.0, 0.11773918055157037, -0.99304455356396)  # quat=(x, y, z, w)
        )
    ]

    for (
        world_x, world_y, world_theta,
        world_x_linear_v, world_y_linear_v, world_z_angular_v,
        odom_time, base_frame_id, world_frame_id,
        exp_quat
    ) in tests:
        odom = create_odometry_message(
            world_x, world_y, world_theta,
            world_x_linear_v, world_y_linear_v, world_z_angular_v,
            odom_time, base_frame_id, world_frame_id
        )
        assert odom.pose.pose.position.x == pytest.approx(world_x, abs=1e-6)
        assert odom.pose.pose.position.y == pytest.approx(world_y, abs=1e-6)
        assert odom.pose.pose.position.z == pytest.approx(0.0, abs=1e-6)
        assert odom.pose.pose.orientation.x == pytest.approx(exp_quat[0], abs=1e-6)
        assert odom.pose.pose.orientation.y == pytest.approx(exp_quat[1], abs=1e-6)
        assert odom.pose.pose.orientation.z == pytest.approx(exp_quat[2], abs=1e-6)
        assert odom.pose.pose.orientation.w == pytest.approx(exp_quat[3], abs=1e-6)
        assert odom.twist.twist.linear.x == pytest.approx(world_x_linear_v, abs=1e-6)
        assert odom.twist.twist.linear.y == pytest.approx(world_y_linear_v, abs=1e-6)
        assert odom.twist.twist.linear.z == pytest.approx(0.0, abs=1e-6)
        assert odom.twist.twist.angular.x == pytest.approx(0.0, abs=1e-6)
        assert odom.twist.twist.angular.y == pytest.approx(0.0, abs=1e-6)
        assert odom.twist.twist.angular.z == pytest.approx(world_z_angular_v, abs=1e-6)

#!/usr/bin/env python
import math
from typing import List
import time

import pytest
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from r2b2_base.base_node import BaseNode
from r2b2_base.odometry_helpers import yaw_from_odom_message, normalize_theta, add_radians, radians_between
from roboclaw_interfaces.msg import SpeedCommand, Stats


TWIST_TOPIC = "cmd_vel"
ODOM_TOPIC = "odom"
SPEEDCOMMAND_TOPIC = 'roboclaw/speed_command'
ROBOCLAW_FRONT_STATS_TOPIC = "roboclaw_front/stats"
# ROBOCLAW_REAR_STATS_TOPIC = "roboclaw_rear/stats"
TEST_NODE_LOOP_HZ = 20


class NodeTestingNode(Node):
    def __init__(self, *args):
        super().__init__('node_tester')

        self.odom = Odometry()

        self.create_subscription(
            msg_type=Odometry,
            topic=ODOM_TOPIC,
            callback=self._odom_callback,
            qos_profile=1
        )

        self.pub_cmd_vel = self.create_publisher(
            msg_type=Twist,
            topic=TWIST_TOPIC,
            qos_profile=1
        )

    def _odom_callback(self, msg):
        self.odom = msg


class MockRoboclawNode(Node):
    def __init__(self, *args):
        super().__init__('mock_roboclaw')

        self.m1_enc_qpps = 0
        self.m2_enc_qpps = 0
        self.m1_enc_val = 0
        self.m2_enc_val = 0

        stats_hz = 20

        self.last_stats_time = time.perf_counter()

        self.create_subscription(
            msg_type=SpeedCommand,
            topic=SPEEDCOMMAND_TOPIC,
            callback=self._speed_cmd_callback,
            qos_profile=1
        )

        self.pub_stats = self.create_publisher(
            msg_type=Stats,
            topic=ROBOCLAW_FRONT_STATS_TOPIC,
            qos_profile=1
        )

        self.create_timer(1 / stats_hz, self._stats_timer_callback)

    def _speed_cmd_callback(self, msg: SpeedCommand):
        self.m1_enc_qpps = msg.m1_qpps
        self.m2_enc_qpps = msg.m2_qpps
        self.get_logger().debug(f"Received cmd: {msg}")

    def _stats_timer_callback(self):
        now = time.perf_counter()
        delta_secs = now - self.last_stats_time
        self.last_stats_time = now

        self.m1_enc_val += self.m1_enc_qpps * delta_secs
        self.m2_enc_val += self.m2_enc_qpps * delta_secs

        stats = Stats()
        stats.header.stamp = self.get_clock().now().to_msg()
        stats.m1_enc_val = int(self.m1_enc_val)
        stats.m2_enc_val = int(self.m2_enc_val)
        stats.m1_enc_qpps = int(self.m1_enc_qpps)
        stats.m2_enc_qpps = int(self.m2_enc_qpps)
        self.pub_stats.publish(stats)


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node(ros_context):
    node = NodeTestingNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    return node


@pytest.fixture
def base_node(ros_context):
    node = BaseNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    return node


@pytest.fixture
def roboclaw_node(ros_context):
    node = MockRoboclawNode()
    node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    return node


def test_drive_forward(test_node: NodeTestingNode, base_node: BaseNode, roboclaw_node: MockRoboclawNode):
    nodes = [test_node, base_node, roboclaw_node]
    _spin_for_secs(nodes, 1)

    linear_x = 0.400
    angular_z = 0.0
    secs = 1
    loop_hz = TEST_NODE_LOOP_HZ

    world_x_exp = 0.400
    world_y_exp = 0.0
    world_theta_exp = 0.0

    _drive(
        test_node, nodes,
        linear_x, angular_z, secs, loop_hz,
        world_x_exp, world_y_exp, world_theta_exp
    )


def test_drive_reverse(test_node: NodeTestingNode, base_node: BaseNode, roboclaw_node: MockRoboclawNode):
    nodes = [test_node, base_node, roboclaw_node]
    _spin_for_secs(nodes, 1)

    linear_x = -0.400
    angular_z = 0.0
    secs = 1
    loop_hz = TEST_NODE_LOOP_HZ

    world_x_exp = -0.400
    world_y_exp = 0.0
    world_theta_exp = 0.0

    _drive(
        test_node, nodes,
        linear_x, angular_z, secs, loop_hz,
        world_x_exp, world_y_exp, world_theta_exp
    )


def test_turn_left(test_node: NodeTestingNode, base_node: BaseNode, roboclaw_node: MockRoboclawNode):
    nodes = [test_node, base_node, roboclaw_node]
    _spin_for_secs(nodes, 1)

    linear_x = 0.0
    angular_z = math.pi / 4
    secs = 2
    loop_hz = TEST_NODE_LOOP_HZ

    world_x_exp = 0.0
    world_y_exp = 0.0
    world_theta_exp = math.pi / 2

    _drive(
        test_node, nodes,
        linear_x, angular_z, secs, loop_hz,
        world_x_exp, world_y_exp, world_theta_exp
    )


def test_turn_right(test_node: NodeTestingNode, base_node: BaseNode, roboclaw_node: MockRoboclawNode):
    nodes = [test_node, base_node, roboclaw_node]
    _spin_for_secs(nodes, 1)

    linear_x = 0.0
    angular_z = -math.pi / 4
    secs = 2
    loop_hz = TEST_NODE_LOOP_HZ

    world_x_exp = 0.0
    world_y_exp = 0.0
    world_theta_exp = -math.pi / 2

    _drive(
        test_node, nodes,
        linear_x, angular_z, secs, loop_hz,
        world_x_exp, world_y_exp, world_theta_exp
    )


def test_multi_movement(test_node: NodeTestingNode, base_node: BaseNode, roboclaw_node: MockRoboclawNode):
    test_drive_forward(test_node, base_node, roboclaw_node)
    test_turn_left(test_node, base_node, roboclaw_node)
    test_drive_forward(test_node, base_node, roboclaw_node)
    test_turn_right(test_node, base_node, roboclaw_node)
    test_drive_reverse(test_node, base_node, roboclaw_node)
    test_turn_right(test_node, base_node, roboclaw_node)
    test_drive_forward(test_node, base_node, roboclaw_node)
    test_turn_left(test_node, base_node, roboclaw_node)


def test_forward_left(test_node: NodeTestingNode, base_node: BaseNode, roboclaw_node: MockRoboclawNode):
    test_drive_forward(test_node, base_node, roboclaw_node)
    test_turn_left(test_node, base_node, roboclaw_node)


def _spin_for_secs(nodes: List[Node], secs: float, timeout_sec: float = 0.001):
    start_time = time.perf_counter()
    while time.perf_counter() < start_time + secs:
        for node in nodes:
            rclpy.spin_once(node, timeout_sec=timeout_sec)


def _drive(
    test_node: NodeTestingNode, nodes: List[Node],
    linear_x: float, angular_z: float, secs: int, loop_hz: int,
    world_x_exp: float, world_y_exp: float, world_theta_exp: float,
):
    """Simulate driving the node `secs` seconds.

    Args:
        linear_x (float): Linear X velocity in m/sec
        angular_z (float): Angular Z velocity in rad/sec
        secs (int): Duration of the drive
        loop_hz (int): Number of loops per second
        world_x_exp (float): Expected world x position
        world_y_exp (float): Expected world y position
        world_theta_exp (float): Expected world orientation
    """
    print()
    print("Command: x:{}, z:{} for {} secs".format(linear_x, angular_z, secs))

    cmd = Twist()
    cmd.linear.x = linear_x
    cmd.angular.z = angular_z

    # Record where the robot is before we drive it
    start_world_x = test_node.odom.pose.pose.position.x
    start_world_y = test_node.odom.pose.pose.position.y
    start_world_theta = yaw_from_odom_message(test_node.odom)

    print(f"Expected Relative World: ({world_x_exp}, {world_y_exp}, {world_theta_exp})")
    print(f"Starting World: ({start_world_x}, {start_world_y}, {start_world_theta})")

    # Run for "secs" seconds
    for i in range(secs * loop_hz):
        test_node.get_logger().debug(f"Pub cmd: {cmd}")
        test_node.pub_cmd_vel.publish(cmd)
        _spin_for_secs(nodes, 1.0 / loop_hz)

    # Issue stop command
    test_node.get_logger().debug(f"Issuing stop command")
    cmd.linear.x = 0.0
    cmd.angular.z = 0.0
    test_node.get_logger().debug(f"Pub cmd: {cmd}")
    test_node.pub_cmd_vel.publish(cmd)

    _spin_for_secs(nodes, 1)

    _compare_odometry(
        test_node.odom, world_x_exp, world_y_exp, world_theta_exp,
        start_world_x, start_world_y, start_world_theta
    )


def _compare_odometry(odom: Odometry, world_x_exp: float, world_y_exp: float, world_theta_exp: float,
                      start_world_x: float, start_world_y: float, start_world_theta: float,
                      xy_diff_dist: float = 0.05, th_diff_rads: float = 0.314  # pi / 10
):
    """Compares the expected world coordinates with an Odometry message
    Parameters:
        odom (Odometry): The Odometry message
        world_x_exp (float): The expected world X coordinate
        world_y_exp (float): The expected world Y coordinate
        world_theta_exp (float): The expected world Theta orientation
        start_world_x (float): The staring world X coordinate
        start_world_y (float): The staring world Y coordinate
        start_world_theta (float): The staring world Theta coordinate
        xy_diff_dist (float): Distance (meters) allowed diff in xy actual vs expected values
        th_diff_rads (float): Angle (radians) allowed diff in theta actual vs expected values
    """
    x_actual = odom.pose.pose.position.x
    y_actual = odom.pose.pose.position.y
    th_actual = euler_from_quaternion(
        [
            odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w,
        ])[2]
    print("Actual World/Odom: ({}, {}, {})".format(x_actual, y_actual, th_actual))

    # Adjust expected pose to accomodate the starting pose using a 2D transformation
    x_exp_adj, y_exp_adj, theta_exp_adj = transform_pose_2d(
        world_x_exp, world_y_exp, world_theta_exp,
        start_world_x, start_world_y, start_world_theta)

    print("Actual: ({}, {}, {}), Expected: ({}, {}, {})".format(
        x_actual, y_actual, th_actual, x_exp_adj, y_exp_adj, theta_exp_adj))
    assert (x_exp_adj - xy_diff_dist) <= x_actual <= (x_exp_adj + xy_diff_dist), f"X exp: {x_exp_adj}, actual {x_actual}, diff allowed {xy_diff_dist}"
    assert (y_exp_adj - xy_diff_dist) <= y_actual <= (y_exp_adj + xy_diff_dist), f"Y exp: {y_exp_adj}, actual {y_actual}, diff allowed {xy_diff_dist}"

    assert radians_between(theta_exp_adj, th_actual) < th_diff_rads, f"Theta exp: {theta_exp_adj}, actual {th_actual}, diff between {radians_between(theta_exp_adj, th_actual)}, diff allowed {th_diff_rads}"
    print(f"Theta exp: {theta_exp_adj}, actual {th_actual}, diff between {radians_between(theta_exp_adj, th_actual)}, diff allowed {th_diff_rads}")


def transform_pose_2d(x, y, theta, x_offset, y_offset, theta_offset):
    """Performs a 2D tranformation of a pose (x, y, theta) by transforming
    it by an offset (x_offset, y_offset, theta_offset).

    Parameters:
        :param float x: The x to transform
        :param float y: The y to transform
        :param float theta: The theta to transform
        :param float x_offset: The x offset to transform by
        :param float y_offset: The y offset to transform by
        :param float theta_offset: The theta offset to transform by

    Returns: Tuple representing the transformed pose (x, y, theta)
        :rtype: (float, float, float)
    """
    # First rotate
    x_rotated = (x * math.cos(theta_offset) - y * math.sin(theta_offset))
    y_rotated = (x * math.sin(theta_offset) + y * math.cos(theta_offset))
    print("Rotated XY: ({}, {})".format(x_rotated, y_rotated))

    # Then translate
    x_translated = x_rotated + x_offset
    y_translated = y_rotated + y_offset
    print("Translated XY: ({}, {})".format(x_translated, y_translated))

    # Finally adjust orientation
    theta_new = add_radians(theta, theta_offset)
    print("Adjusted Expected Theta: {}".format(theta_new))

    return (x_translated, y_translated, theta_new)

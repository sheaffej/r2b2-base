#!/usr/bin/env python

from functools import partial
from math import pi as pi
import sys
import threading
import traceback
from typing import List

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.time import Time

from rcl_interfaces.msg import ParameterDescriptor
# import tf
import tf2_ros

from roboclaw_interfaces.msg import SpeedCommand, Stats

from r2b2_base.base_functions import (
    calc_create_speed_cmd, calc_base_frame_velocity_from_encoder_diffs,
    calc_odometry_from_base_velocity
)
from r2b2_base.odometry_helpers import (
    yaw_from_odom_message
)


class DEFAULTS:

    NODE_NAME = "base_node"

    # Subscribes
    CMD_VEL_TOPIC = "cmd_vel"
    ROBOCLAW_FRONT_STATS_TOPIC = "roboclaw_front/stats"
    ROBOCLAW_REAR_STATS_TOPIC = "roboclaw_rear/stats"

    # Publishes
    SPEED_CMD_TOPIC = "roboclaw/speed_command"
    ODOM_TOPIC = "odom"

    # Default Parameters
    LOOP_HZ = 20  # hertz
    WHEEL_DIST = 0.180  # meters
    WHEEL_RADIUS = 0.0325  # meters
    WHEEL_SLIP_FACTOR = 0.5  # Decimal % of angular motion lost to slip
    TICKS_PER_ROTATION = 48 * 34
    MAX_QPPS = 3700
    MAX_ACCEL = 20000
    MAX_X_LINEAR_VEL = 0.5      # meters/sec
    MAX_Z_ANGULAR_VEL = pi / 2  # radians/sec
    MAX_DRIVE_SECS = 1
    ODOM_FRAME_ID = "odom"
    PUBLISH_ODOM_TF = True
    BASE_FRAME_ID = "base_link"
    DEADMAN_SECS = 1
    LOG_LEVEL = "info"


class PARAMS:
    LOG_LEVEL = 'log_level'
    WHEEL_DIST = 'wheel_dist'
    WHEEL_RADIUS = 'wheel_radius'
    WHEEL_SLIP_FACTOR = 'wheel_slip_factor'
    TICKS_PER_ROTATION = 'ticks_per_rotation'
    MAX_DRIVE_SECS = 'max_drive_secs'
    DEADMAN_SECS = 'deadman_secs'
    MAX_QPPS = 'max_qpps'
    MAX_X_LIN_VEL = 'max_x_lin_vel'
    MAX_Z_ANG_VEL = 'max_z_ang_vel'
    MAX_ACCEL = 'max_accel'
    BASE_FRAME_ID = 'base_frame_id'
    WORLD_FRAME_ID = 'odom_frame_id'
    LOOP_HZ = 'loop_hz'
    PUBLISH_ODOM_TF = 'publish_odom_tf'
    SPEED_CMD_TOPIC = 'speed_command_topic'
    ODOM_TOPIC = 'odom_topic'
    CMD_VEL_TOPIC = 'cmd_vel_topic'
    ROBOCLAW_FRONT_STATS_TOPIC = 'roboclaw_front_stats_topic'
    ROBOCLAW_REAR_STATS_TOPIC = 'roboclaw_rear_stats_topic'


class BaseNode(rclpy.node.Node):

    def __init__(self):
        super().__init__(DEFAULTS.NODE_NAME)

        self.declare_parameters(
            namespace='',
            parameters=[
                (PARAMS.LOG_LEVEL, DEFAULTS.LOG_LEVEL, ParameterDescriptor(description='Log verbosity, defaults to `info`')),
                (PARAMS.WHEEL_DIST, DEFAULTS.WHEEL_DIST, ParameterDescriptor(description='Width between the wheels in meters')),
                (PARAMS.WHEEL_RADIUS, DEFAULTS.WHEEL_RADIUS, ParameterDescriptor(description='Radius of the wheels in meters')),
                (PARAMS.WHEEL_SLIP_FACTOR, DEFAULTS.WHEEL_SLIP_FACTOR, ParameterDescriptor(description='Factor to compensate for wheel slip as a float >=0.0 <1.0')),
                (PARAMS.TICKS_PER_ROTATION, DEFAULTS.TICKS_PER_ROTATION, ParameterDescriptor(description='Wheel encoder ticks per rotation')),
                (PARAMS.MAX_DRIVE_SECS, DEFAULTS.MAX_DRIVE_SECS, ParameterDescriptor(description='Maximum seconds drive should run before stopping')),
                (PARAMS.DEADMAN_SECS, DEFAULTS.DEADMAN_SECS, ParameterDescriptor(description='Max time tolerated between cmd_vel messages before stopping')),
                (PARAMS.MAX_QPPS, DEFAULTS.MAX_QPPS, ParameterDescriptor(description='Max wheel rotation speed in QPPS')),
                (PARAMS.MAX_X_LIN_VEL, DEFAULTS.MAX_X_LINEAR_VEL, ParameterDescriptor(description='Max linear velocity in m/s')),
                (PARAMS.MAX_Z_ANG_VEL, DEFAULTS.MAX_Z_ANGULAR_VEL, ParameterDescriptor(description='Max angular velocity in rad/s')),
                (PARAMS.MAX_ACCEL, DEFAULTS.MAX_ACCEL, ParameterDescriptor(description='Max QPPS/sec of acceleration')),
                (PARAMS.BASE_FRAME_ID, DEFAULTS.BASE_FRAME_ID, ParameterDescriptor(description='Frame ID of the base')),
                (PARAMS.WORLD_FRAME_ID, DEFAULTS.ODOM_FRAME_ID, ParameterDescriptor(description='Frame ID of the world (e.g. `odom`')),
                (PARAMS.LOOP_HZ, DEFAULTS.LOOP_HZ, ParameterDescriptor(description='Frequency of the main logic loop')),
                (PARAMS.PUBLISH_ODOM_TF, DEFAULTS.PUBLISH_ODOM_TF, ParameterDescriptor(description='If true, publish the odom TF transform')),
                (PARAMS.SPEED_CMD_TOPIC, DEFAULTS.SPEED_CMD_TOPIC, ParameterDescriptor(description='Topic to publish for SpeedCommand messages to the Roboclaws')),
                (PARAMS.CMD_VEL_TOPIC, DEFAULTS.CMD_VEL_TOPIC, ParameterDescriptor(description='Topic to listen for Twist messages')),
                (PARAMS.ODOM_TOPIC, DEFAULTS.ODOM_TOPIC, ParameterDescriptor(description='Topic to publish for Odometry messages')),
                (PARAMS.ROBOCLAW_FRONT_STATS_TOPIC, DEFAULTS.ROBOCLAW_FRONT_STATS_TOPIC, ParameterDescriptor(description='Stats topic for front Roboclaw')),
                (PARAMS.ROBOCLAW_REAR_STATS_TOPIC, DEFAULTS.ROBOCLAW_REAR_STATS_TOPIC, ParameterDescriptor(description='Stats topic for rear Roboclaw'))
            ]
        )

        self._wheel_dist = self.get_parameter(PARAMS.WHEEL_DIST).value
        self._wheel_radius = self.get_parameter(PARAMS.WHEEL_RADIUS).value
        self._wheel_slip_factor = self.get_parameter(PARAMS.WHEEL_SLIP_FACTOR).value
        self._ticks_per_rotation = self.get_parameter(PARAMS.TICKS_PER_ROTATION).value
        self._max_drive_secs = self.get_parameter(PARAMS.MAX_DRIVE_SECS).value
        self._deadman_secs = self.get_parameter(PARAMS.DEADMAN_SECS).value
        self._max_qpps = self.get_parameter(PARAMS.MAX_QPPS).value
        self._max_x_lin_vel = self.get_parameter(PARAMS.MAX_X_LIN_VEL).value
        self._max_z_ang_vel = self.get_parameter(PARAMS.MAX_Z_ANG_VEL).value
        self._max_accel = self.get_parameter(PARAMS.MAX_ACCEL).value
        self._base_frame_id = self.get_parameter(PARAMS.BASE_FRAME_ID).value
        self._world_frame_id = self.get_parameter(PARAMS.WORLD_FRAME_ID).value
        self._publish_odom_tf = self.get_parameter(PARAMS.PUBLISH_ODOM_TF).value

        # Publishes
        self._speed_cmd_pub = self.create_publisher(
            msg_type=SpeedCommand,
            topic=self.get_parameter(PARAMS.SPEED_CMD_TOPIC).value,
            qos_profile=1
        )
        self._odom_pub = self.create_publisher(
            msg_type=Odometry,
            topic=self.get_parameter(PARAMS.ODOM_TOPIC).value,
            qos_profile=1
        )

        self._tf_broadcaster: tf2_ros.TransformBroadcaster = None
        if self._publish_odom_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self, qos=1)

        # Twist message Subscriber
        self.create_subscription(
            msg_type=Twist,
            topic=self.get_parameter(PARAMS.CMD_VEL_TOPIC).value,
            callback=self._cmd_vel_callback,
            qos_profile=1
        )

        # Roboclaw Stats message Subscriber
        self.create_subscription(
            msg_type=Stats,
            topic=self.get_parameter(PARAMS.ROBOCLAW_FRONT_STATS_TOPIC).value,
            callback=partial(self._roboclaw_stats_callback, 'front'),
            qos_profile=1
        )

        self.create_subscription(
            msg_type=Stats,
            topic=self.get_parameter(PARAMS.ROBOCLAW_REAR_STATS_TOPIC).value,
            callback=partial(self._roboclaw_stats_callback, 'rear'),
            qos_profile=1
        )

        # Main loop timer
        loop_secs = 1.0 / self.get_parameter(PARAMS.LOOP_HZ).value
        self.create_timer(loop_secs, self._base_loop_callback)

        # Init Twist command state
        self._x_linear_cmd = 0.0
        self._z_angular_cmd = 0.0

        # Last time we received a Twist message
        # If we don't get a message after deadman_secs, we stop the base
        self._last_cmd_vel_time: Time = self.get_clock().now()

        # Init Odometry state
        self._world_x = 0.0
        self._world_y = 0.0
        self._world_theta = 0.0
        self._last_odom_time: Time = None

        # Init Roboclaw stats state
        self._roboclaw_front_stats = None  # type: Stats
        # self._roboclaw_rear_stats = None  # type: Stats

        # Roboclaw encoder state
        self._m1_front_enc_prev = 0
        self._m2_front_enc_prev = 0
        # self._m1_rear_enc_prev = 0
        # self._m2_rear_enc_prev = 0

        self._stats_lock = threading.RLock()  # To serialize access to the qpps stats
        self._cmd_vel_lock = threading.RLock()  # To serialize access to x/z command variables

        # Set initial states
        if self._roboclaw_front_stats is not None:
            self._m1_front_enc_prev = self._roboclaw_front_stats.m1_enc_val
            self._m2_front_enc_prev = self._roboclaw_front_stats.m2_enc_val
        # if self._roboclaw_rear_stats is not None:
        #     self._m1_rear_enc_prev = self._roboclaw_rear_stats.m1_enc_val
        #     self._m2_rear_enc_prev = self._roboclaw_rear_stats.m2_enc_val
        self._last_odom_time: Time = self.get_clock().now()
        self._last_cmd_vel_time: Time = self.get_clock().now()

    # def run(self, loop_hz):
    #     """Runs the main loop of the node.
    #     Sends motor commands, and publishes odometry.
    #     """
    #     rospy.logdebug("Running node")
    #     looprate = rospy.Rate(loop_hz)

    #     # Set initial states
    #     if self._roboclaw_front_stats is not None:
    #         self._m1_front_enc_prev = self._roboclaw_front_stats.m1_enc_val
    #         self._m2_front_enc_prev = self._roboclaw_front_stats.m2_enc_val
    #     # if self._roboclaw_rear_stats is not None:
    #     #     self._m1_rear_enc_prev = self._roboclaw_rear_stats.m1_enc_val
    #     #     self._m2_rear_enc_prev = self._roboclaw_rear_stats.m2_enc_val
    #     self._last_odom_time = rospy.get_rostime()
    #     self._last_cmd_vel_time = rospy.get_rostime()

    #     try:
    #         while not rospy.is_shutdown():
    #             self.process_base_loop()
    #             looprate.sleep()

    #     except rospy.ROSInterruptException:
    #         rospy.logwarn("ROSInterruptException received in main loop")

    def _cmd_vel_callback(self, msg: Twist):
        """Called by the Twist cmd_vel message subscriber.

        Parameters:
            msg (Twist): Twist command velocity message
        """
        with self._cmd_vel_lock:

            # if ((self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9 > 0
            #         and msg.linear.x == self._x_linear_cmd
            #         and msg.angular.z == self._z_angular_cmd):
            #     self.get_logger().debug("Cmd Vel received, but no change in values")

            # else:
            self._x_linear_cmd = msg.linear.x
            self._z_angular_cmd = msg.angular.z
            self._last_cmd_vel_time = self.get_clock().now()
            self.get_logger().debug(f"CMD Vel - X: {msg.linear.x} | Z: {msg.angular.z}")

    def _roboclaw_stats_callback(self, position: str, stats: Stats):
        """Called by the Roboclaw Stats message subscriber

        Parameters:
            stats (Stats): Roboclaw Stats message
            callback_args (List): Arguments to this function (i.e. "front" or "rear)
        """
        with self._stats_lock:
            if "front" in position:
                self._roboclaw_front_stats = stats
            elif "rear" in position:
                self._roboclaw_rear_stats = stats
            else:
                self.get_logger().warn("roboclaw_stats_callback: Unsure which stats to read")
                self.get_logger().warn(f"callback_args: {position}")
        self.get_logger().debug(
            f"Stats received: t=({stats.header.stamp.sec}.{stats.header.stamp.nanosec}), frame_id=({stats.header.frame_id}), "
            f"m1_enc_val=({stats.m1_enc_val}), m2_enc_val=({stats.m2_enc_val}), "
            f"m1_enc_qpps=({stats.m1_enc_qpps}), m2_enc_qpps=({stats.m2_enc_qpps})"
        )

    def _base_loop_callback(self):
        _process_base_loop(self)


# Separated out of node object so it can be more easily unit tested
def _process_base_loop(self: BaseNode):
    # ------------------------------------------------------------
    # If the last command was over deadman_secs ago, stop the base
    # ------------------------------------------------------------
    if (
        self._last_cmd_vel_time is None or
        (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9 > self._deadman_secs
    ):
        self._x_linear_cmd = 0.0
        self._z_angular_cmd = 0.0

    # ---------------------------------
    # Calculate and send motor commands
    # ---------------------------------
    with self._cmd_vel_lock:
        x_linear_cmd = self._x_linear_cmd
        z_angular_cmd = self._z_angular_cmd

    # Clamp the velocities to the max configured for the base
    x_linear_cmd = max(-self._max_x_lin_vel, min(x_linear_cmd, self._max_x_lin_vel))
    z_angular_cmd = max(-self._max_z_ang_vel, min(z_angular_cmd, self._max_z_ang_vel))

    cmd = calc_create_speed_cmd(
        x_linear_cmd, z_angular_cmd,
        self._wheel_dist, self._wheel_radius, self._wheel_slip_factor,
        self._ticks_per_rotation, self._max_drive_secs, self._max_qpps, self._max_accel
    )
    self.get_logger().debug(f"Publishing: {cmd}")
    self._speed_cmd_pub.publish(cmd)

    # -------------------------------
    # Calculate and publish Odometry
    # -------------------------------

    # if self._roboclaw_front_stats is None or self._roboclaw_rear_stats is None:
    if self._roboclaw_front_stats is None:
        self.get_logger().info("Insufficient roboclaw stats received, skipping odometry calculation")
        return

    with self._stats_lock:
        # Calculate change in encoder readings
        m1_front_enc_diff = self._roboclaw_front_stats.m1_enc_val - self._m1_front_enc_prev
        m2_front_enc_diff = self._roboclaw_front_stats.m2_enc_val - self._m2_front_enc_prev
        # m1_rear_enc_diff = self._roboclaw_rear_stats.m1_enc_val - self._m1_rear_enc_prev
        # m2_rear_enc_diff = self._roboclaw_rear_stats.m2_enc_val - self._m2_rear_enc_prev

        self._m1_front_enc_prev = self._roboclaw_front_stats.m1_enc_val
        self._m2_front_enc_prev = self._roboclaw_front_stats.m2_enc_val
        # self._m1_rear_enc_prev = self._roboclaw_rear_stats.m1_enc_val
        # self._m2_rear_enc_prev = self._roboclaw_rear_stats.m2_enc_val

        # Since we have a two Roboclaw robot, take the average of the encoder diffs
        # from each Roboclaw for each side.
        # m1_enc_diff = (m1_front_enc_diff + m1_rear_enc_diff) / 2
        # m2_enc_diff = (m2_front_enc_diff + m2_rear_enc_diff) / 2
        m1_enc_diff = m1_front_enc_diff
        m2_enc_diff = m2_front_enc_diff

        # We take the nowtime from the Stats message so it matches the encoder values.
        # Otherwise we would get timing variances based on when the loop runs compared to
        # when the stats were measured.
        # Since we have a two Roboclaw robot, take the latest stats timestamp from either
        # Roboclaw.
        front_stamp = self._roboclaw_front_stats.header.stamp
        # print(type(front_stamp))
        # import builtin_interfaces
        # print(isinstance(front_stamp, builtin_interfaces.msg.Time))
        # rear_stamp = self._roboclaw_rear_stats.header.stamp
        # nowtime = max(front_stamp, rear_stamp)
        nowtime = rclpy.time.Time.from_msg(front_stamp)

    x_linear_v, y_linear_v, z_angular_v = calc_base_frame_velocity_from_encoder_diffs(
        m1_enc_diff, m2_enc_diff, self._ticks_per_rotation,
        self._wheel_radius, self._wheel_dist, self._wheel_slip_factor,
        self._last_odom_time, nowtime
    )

    time_delta_secs = (nowtime - self._last_odom_time).nanoseconds / 1e9
    self._last_odom_time = nowtime

    odom = calc_odometry_from_base_velocity(
        x_linear_v, y_linear_v, z_angular_v,
        self._world_x, self._world_y, self._world_theta,
        time_delta_secs, nowtime,
        self._base_frame_id, self._world_frame_id
    )
    # self.get_logger().debug(f"Publishing: {odom}")
    self._odom_pub.publish(odom)

    # Update world pose
    self._world_x = odom.pose.pose.position.x
    self._world_y = odom.pose.pose.position.y
    self._world_theta = yaw_from_odom_message(odom)

    # -----------------------------------------
    # Calculate and broacast tf transformation
    # -----------------------------------------
    if self._publish_odom_tf:
        quat = odom.pose.pose.orientation
    #     self._tf_broadcaster.sendTransform(
    #         (self._world_x, self._world_y, 0),
    #         (quat.x, quat.y, quat.z, quat.w),
    #         nowtime,
    #         self._base_frame_id,
    #         self._world_frame_id
    #     )
        t = TransformStamped()
        t.header.stamp = nowtime.to_msg()
        t.header.frame_id = self._world_frame_id
        t.child_frame_id = self._base_frame_id
        t.transform.translation.x = self._world_x
        t.transform.translation.y = self._world_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat.x
        t.transform.rotation.y = quat.y
        t.transform.rotation.z = quat.z
        t.transform.rotation.w = quat.w
        # self.get_logger().debug(f"Publishing: {t}")
        self._tf_broadcaster.sendTransform(t)

    self._last_odom_time = nowtime

    self.get_logger().debug(
        "World position: [{}, {}] heading: {}, forward speed: {}, turn speed: {}".format(
            self._world_x, self._world_y, self._world_theta,
            self._x_linear_cmd, self._z_angular_cmd
        )
    )


def main(args=None):

    # Setup the ROS node
    rclpy.init(args=args)
    node = BaseNode()

    try:
        # # Initialize the Roboclaw controllers
        # for dev in dev_names.split(','):
        #     node.connect(dev, baud_rate, address, test_mode)
        rclpy.spin(node)

    except Exception:
        node.get_logger().fatal("Unhandled exeption...printing stack trace then shutting down node")
        node.get_logger().fatal(traceback.format_exc())

    # Shutdown and cleanup
    rclpy.shutdown()


if __name__ == "__main__":
    main(args=sys.argv)

    # log_level = parse_log_level(rospy.get_param("~log_level", LOG_LEVEL))
    # rospy.init_node(DEFAULT_NODE_NAME, log_level=log_level)
    # node_name = rospy.get_name()

    # wheel_dist = rospy.get_param("~wheel_dist", DEFAULT_WHEEL_DIST)
    # wheel_radius = rospy.get_param("~wheel_radius", DEFAULT_WHEEL_RADIUS)
    # wheel_slip_factor = rospy.get_param("~wheel_slip_factor", DEFAULT_WHEEL_SLIP_FACTOR)
    # ticks_per_rotation = rospy.get_param("~ticks_per_rotation", DEFAULT_TICKS_PER_ROTATION)
    # max_drive_secs = rospy.get_param("~max_drive_secs", DEFAULT_MAX_DRIVE_SECS)
    # deadman_secs = rospy.get_param("~deadman_secs", DEFAULT_DEADMAN_SECS)
    # max_qpps = rospy.get_param("~max_qpps", DEFAULT_MAX_QPPS)
    # max_x_lin_vel = rospy.get_param("~max_x_lin_vel", DEFAULT_MAX_X_LINEAR_VEL)
    # max_z_ang_vel = rospy.get_param("~max_z_ang_vel", DEFAULT_MAX_Z_ANGULAR_VEL)
    # max_accel = rospy.get_param("~max_accel", DEFAULT_MAX_ACCEL)
    # base_frame_id = rospy.get_param("~base_frame_id", DEFAULT_BASE_FRAME_ID)
    # world_frame_id = rospy.get_param("~odom_frame_id", DEFAULT_ODOM_FRAME_ID)
    # loop_hz = rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)
    # publish_odom_tf = rospy.get_param("~publish_odom_tf", DEFAULT_PUBLISH_ODOM_TF)

    # # Publishes
    # speed_cmd_pub = rospy.Publisher(
    #     rospy.get_param('~speed_cmd_topic', DEFAULT_SPEED_CMD_TOPIC),
    #     SpeedCommand,
    #     queue_size=1
    # )
    # odom_pub = rospy.Publisher(
    #     rospy.get_param('~odom_topic', DEFAULT_ODOM_TOPIC),
    #     Odometry,
    #     queue_size=1
    # )

    # tf_broadcaster = None
    # if publish_odom_tf:
    #     tf_broadcaster = tf.broadcaster.TransformBroadcaster()

    # node = BaseNode(wheel_dist, wheel_radius, wheel_slip_factor, ticks_per_rotation,
    #                 max_drive_secs, deadman_secs,
    #                 max_qpps, max_x_lin_vel, max_z_ang_vel, max_accel,
    #                 base_frame_id, world_frame_id,
    #                 speed_cmd_pub, odom_pub, publish_odom_tf, tf_broadcaster)

    # # Twist message Subscriber
    # rospy.Subscriber(
    #     rospy.get_param("~cmd_vel_topic", DEFAULT_CMD_VEL_TOPIC),
    #     Twist,
    #     node.cmd_vel_callback
    # )

    # # Roboclaw Stats message Subscriber
    # rospy.Subscriber(
    #     rospy.get_param("~roboclaw_front_stats_topic", DEFAULT_ROBOCLAW_FRONT_STATS_TOPIC),
    #     Stats,
    #     node.roboclaw_stats_callback,
    #     "front"
    # )

    # rospy.Subscriber(
    #     rospy.get_param("~roboclaw_rear_stats_topic", DEFAULT_ROBOCLAW_REAR_STATS_TOPIC),
    #     Stats,
    #     node.roboclaw_stats_callback,
    #     "rear"
    # )

    # node.run(loop_hz)

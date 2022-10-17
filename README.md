# r2b2-base: ROS2 base node for my Recovery Robot B2 (R2B2)

This is the ROS2 base node for my R2B2 robot project.

***This project is still under development, and is in a state of change as I continue to build out the other nodes of the robot.***

This base node is a ROS2 conversion of the prior ROS1 robot's (aka B2) base node: https://github.com/sheaffej/b2-base

The node exposes the following parameters:

| Parameter | Default | Notes |
| --- | --- | --- |
| node_name | "base_node" | |
| log_level | "info" | |
| wheel_dist | 0.180  | meters | 
| wheel_radius | 0.0325  | meters | 
| wheel_slip_factor | 0.5  | Decimal % of angular motion lost to slip | 
| ticks_per_rotation | 48 * 34  | Encoder ticks per rotation | 
| max_x_lin_vel | 0.5 | meters/sec | 
| max_z_ang_vel | pi / 2 | radians/sec | 
| max_drive_secs | 1 | |
| deadman_secs | 1 | |
| max_qpps | 3700 | |
| max_accel | 20000 | |
| base_frame_id | "base_link" | |
| odom_frame_id | "odom" | |
| loop_hz | 20  | hertz | 
| publish_odom_tf | True | |
| speed_command_topic | "roboclaw/speed_command" | |
| odom_topic | "odom" | |
| cmd_vel_topic | "cmd_vel" | |
| roboclaw_front_stats_topic | "roboclaw_front/stats" | |
| roboclaw_rear_stats_topic | "roboclaw_rear/stats" | |

The `r2b2-base` node publishes `sheaffej/roboclaw_interfaces/SpeedCommand` messages to a single ROS2 roboclaw node ([sheaffej/roboclaw2](https://github.com/sheaffej/roboclaw_driver2)), and subscribes to `sheaffej/roboclaw_interfaces/Stats` messages to monitor the movement of the Robloclaw devices. 

For simplicity in this robot, the `r2b2-base` node sends the SpeedCommand message to a single Robloclaw node which drives two identical Roboclaw devices at the same speed. Therefore, the 4-wheel (i.e. 2 Roboclaws) motion on R2B2 is similar to how a tank tread works (i.e. there is slippage when turning). The roboclaw node subsequently publishes stats for both Roboclaw devices (front and rear), but the `r2b2-base` only reads the stats related to the front Roboclaw node. This assumes the stats from both Roblclaws are similar (which they should be unless one of the Robloclaws is malfunctioning), simplifies the base node logic considerably, and in practice works very well.

The `r2b2-base` node also subscribes to a `geometry_messages/Twist` topic as its velocity commands input, which it converts to `SpeedCommand` commands for the Roboclaw, and calculates and publishes the base's `nav_msgs/Odometry` messages. The base node also publishes the TF2 transform between the base frame and the odom frame.
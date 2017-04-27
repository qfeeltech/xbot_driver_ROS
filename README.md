# XBOT driver
## Quick Start
Clone this repository to your catkin_ws/src folder, then excute
```
cd catkin_ws
catkin_make
```
When you first run this package, excute follow script to create an udev rules for xbot.
```
rosrun qfeel_xbot create_udev_rules
```
To run the driver, excute:
```
roslaunch qfeel_xbot qfeel_xbot.launch
```
You can control the xbot with keyboard by excute：
```
rosrun qfeel_keyop qfeel_keyop
```
## Topics
The driver publish all sensor messages to ros topics.
### Published topics:
 * /qfeel/encoder [std_msgs/UInt16MultiArray]
 <br> The encoder data of front left, front right, back left and back right wheels.
 * /qfeel/ultrasound [std_msgs/UInt16MultiArray]
 <br> The ultrasound data of left, central and right in centimeter.
 * /tf [tf2_msgs/TFMessage]
 <br> Publish the tf from odom to base_link, the frame name can be set as parameter. See qfeel_xbot.launch.
 * /qfeel/charger [std_msgs/Byte]
 <br> The charger status, 1 means the robot is charging.
 * /qfeel/infrared [std_msgs/UInt16MultiArray]
 <br> The infrared data of back left, back central and back right.
 * /qfeel/imu [sensor_msgs/Imu]
 <br> The imu data of xbot.
 * /qfeel/power [std_msgs/Byte]
 <br> The power percentage of xbot.
 * /qfeel/odom [nav_msgs/Odometry]
 <br> The odom data of xbot.
### Subscribed topics:
 * /qfeel/cmd_vel [geometry_msgs/Twist]
 <br> The velocity command sent to xbot.

> Copyright (C) 2017  QFeeltech.

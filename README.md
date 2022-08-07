# scuttle_driver

Translates the ROS [Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html)
commands to motor input commands for the SCUTTLE robot.

**NOTE:** This package is only used on the physical SCUTTLE robot as it sends motor commands
via the GPIO pins of the compute board.

## Dependencies

The configurations in this repository assume you have the following prerequisites installed on the
physical SCUTTLE that you want to drive.

1. [ROS Noetic](http://wiki.ros.org/noetic) with the `ros-noetic-navigation`, `ros-noetic-robot` and
   `ros-noetic-tf2` packages.
1. A working [ROS workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
1. If launched via a [systemd daemon](https://en.wikipedia.org/wiki/Systemd) then the
   [teleop_twist_joy](https://github.com/scuttlerobot/teleop_twist_joy) package should be installed.

## Usage

`scuttle_driver` can be launched using `rosrun` as follows

    rosrun scuttle_driver scuttle_driver.py

Or included in a launch file as follows

    <node name="scuttle_driver" pkg="scuttle_driver" type="scuttle_driver.py" output="screen"/>

When the `scuttle_driver` node is operating it subscribes to the following topics:

* `/cmd_vel` - Provides Twist messages that are translated into motor commands for SCUTTLE
* `/initial_pose` - Provides the current position of the robot. Overwrites the estimated
  position as obtained using odometry.

Additionally [Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages
are published on the `odom` topic

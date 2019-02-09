# ASV Joy

This package contains Gazebo joystick, teleop and simple link / joint controller plugins.

The joystick plugin provides a mechanism to publish joystick messages to the Ignition Transport
communication middleware. It is an alternative to the `JoyPlugin` distributed with Gazebo, which will
only run on Linux.

The teleop plugin may be used to subscribe to joystick messages and convert them
to command velocity messages on Ignition Transport. These are a Gazebo equivalent of
[`geometry_msgs/Twist`](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
messages in ROS and provide a convenient way to control simple robots.

The link / joint controller plugin allows a single link and joint in a Gazebo model to be
controlled by subscribing to a command velocity message.

Combined, these plugins provide a quick method to test the movement of simple robots in Gazebo
without having to run separate ROS nodes.

## Installation

### Create and configure a workspace

Source your ROS and Gazebo installations:

```bash
source /opt/ros/melodic/setup.bash
source /usr/local/share/gazebo-9/setup.bash
```

Create a catkin workspace:

```bash
mkdir -p asv_ws/src
cd asv_ws
catkin init
```

Configure catkin:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Clone and build the package

Clone the `asv_joy` repository:

```bash
cd src
git clone https://github.com/srmainwaring/asv_joy.git
```

Compile the packages:

```bash
catkin build
```

## Usage

The wiki has details about how to configure and use the plugins:

- [JoySDLPlugin](https://github.com/srmainwaring/asv_joy/wiki/JoySDLPlugin)
- [TeleopTwistJoyPlugin](https://github.com/srmainwaring/asv_joy/wiki/TeleopTwistJoyPlugin)
- [LinkJointControllerPlugin](https://github.com/srmainwaring/asv_joy/wiki/LinkJointControllerPlugin)

## Example

![image](https://github.com/srmainwaring/asv_joy/wiki/images/asv_joy_demo_world.gif)

Launch the example world in a Gazebo session:

```bash
roslaunch asv_joy_gazebo asv_joy_demo_world.launch verbose:=true
```

A joystick can be used to control two blocks connected by a revolute joint.

Use `ignition-tools` to list the topics:

```bash
$ ign topic --list
/cmd_vel
...
/joy
```

Use `ignition-tools` to view the joystick messages:

```bash
$ ign topic --echo --topic /joy

header {
  stamp {
    sec: 1549557188
    nsec: 742552000
  }
}
axes: -0.793224931
axes: -0.907307744
axes: 0
axes: -0.386805028
buttons: 0
buttons: 0
buttons: 0
...
```

Use `ignition-tools` to view the command messages:

```bash
$ ign topic --echo --topic /cmd_vel

header {
  stamp {
    sec: 1549557188
    nsec: 742552000
  }
}
velocity: -0.386805028
theta: -0.793224931

```

For more detail see the [Example](https://github.com/srmainwaring/asv_joy/wiki/Example) page in the wiki.

## License

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.

## Acknowledgments

- Kevin J. Walchko's [osx_joystick](https://github.com/BrainSpawnInfosphere/osx_joystick) package which demonstrates how to use [SDL](http://www.libsdl.org/) to poll and publish joystick events.
- Mike Purvis's [teleop_twist_joy](http://wiki.ros.org/teleop_twist_joy) package which provided a template for the TeleopTwistJoyPlugin.
- The OSRF Gazebo [tutorials](http://gazebosim.org/tutorials?tut=set_velocity) 
and [source code](https://bitbucket.org/osrf/gazebo/src) for guidance on the parameter
interface for the JoySDLPlugin and configuring Gazebo PID controllers for the LinkJointControllerPlugin.  

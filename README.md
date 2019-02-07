# ASV Joy

Joystick and teleop plugins for Gazebo for OSX.

## Installation

This package was built and tested with:

- Gazebo version 9.4.1
- ROS Melodic Morenia
- OSX 10.11.6

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

On OSX force catkin to use the [homebrew](https://brew.sh/) version of python
to avoid any conflict with the system version.
See Mike Purvis's [ros-install-osx](https://github.com/mikepurvis/ros-install-osx)
for details.

```bash
catkin config \
  --cmake-args \
    -DCATKIN_ENABLE_TESTING=1 \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_FIND_FRAMEWORK=LAST \
    -DPYTHON_EXECUTABLE=$(which python2) \
    -DPYTHON_LIBRARY=$(python2 -c "import sys; print sys.prefix")/lib/libpython2.7.dylib \
    -DPYTHON_INCLUDE_DIR=$(python2 -c "import sys; print sys.prefix")/include/python2.7
```

Clone the `asv_joy` repository:

```bash
cd src
git clone https://github.com/srmainwaring/asv_joy.git
```

Compile the packages:

```bash
catkin build
```

## JoySDLPlugin

This is a version of the Gazebo `JoyPlugin` for OSX (it should work on other platforms too).
It registers button presses and joystick movement and has a user configurable
deadzone region, but does not support the 'sticky-button' feature of the Linux
version.

### Usage

Add the SDF for the plugin to the `<world>` element of your world file:

```xml
<sdf version="1.6">
  <world name="world">

    <!-- World plugins -->
    <plugin name="joy" filename="libJoySDLPlugin.so">
      <joy_id>0</joy_id>
      <topic>/joy</topic>
      <rate>50</rate>
      <dead_zone>0.1</dead_zone>
    </plugin>

    <!-- Other world elements... -->

  </world>
</sdf>  
```

### Published Topics

1. `/joy` ([`ignition::msgs::Joy`](https://bitbucket.org/ignitionrobotics/ign-msgs))

### Parameters

1. `<joy_id>` (`int`, default: `0`) \
The joystick identifier.

2. `<output_topic>` (`string`, default: `/joy`) \
The name of the topic to which `ignition::msgs::Joy` are published.

3. `<rate>` (`double`, default: `50.0`): \
The rate in Hz at which joystick messages are published.

4. `<dead_zone>` (`double` between `0` and `0.9`, default: `0.1`) \
The amount by which the joystick has to move to register a value.

## TeleopTwistJoyPlugin

This is a Gazebo world plugin that maps joystick messages into
command messages and publishes them to an Ignition transport topic.
It's behaviour and usage are similar to the ROS [`teleop_twist_joy`](http://wiki.ros.org/teleop_twist_joy)
package upon which it is based.

### Usage

Add the SDF for the plugin to the `<world>` element of your world file:

```xml
<sdf version="1.6">
  <world name="world">

    <!-- World plugins -->
    <plugin name="teleop_twist_joy" filename="libTeleopTwistJoyPlugin.so">
      <input_topic>/joy</input_topic>
      <output_topic>/cmd_vel</output_topic>
      <enable_button>10</enable_button>
      <axis_linear>3</axis_linear>
      <scale_linear>1.0</scale_linear>
      <axis_angular>0</axis_angular>
      <scale_angular>1.0</scale_angular>
    </plugin>

    <!-- Other world elements... -->

  </world>
</sdf>  
```

### Subscribed Topics

1. `/joy` ([`ignition::msgs::Joy`](https://bitbucket.org/ignitionrobotics/ign-msgs))

### Published Topics

1. `/cmd_vel` ([`ignition::msgs::CmdVel2D`](https://bitbucket.org/ignitionrobotics/ign-msgs))

### Parameters

1. `<input_topic>` (`string`, default: `/joy`) \
The name of the topic to subscribe to `ignition::msgs::Joy` messages.

2. `<output_topic>` (`string`, default: `/cmd_vel`) \
The name of the topic to which `ignition::msgs::CmdVel2` messages are published.

3. `<enable_button>` (`int`, default: `0`): \
The index of the button that must be held for command messages to be published (deadman switch).

4. `<axis_linear>` (`int`, default: `3`) \
The index of the joystick axis that is used to set the value of `ignition::msgs::CmdVel2D.velocity`.

5. `<scale_linear>` (`double`, default: `1.0`) \
A scale factor to apply to the value of the linear joystick axis (which is in the range -1 to 1).

6. `<axis_angular>` (`int`, default: `3`) \
The index of the joystick axis that is used to set the value of `ignition::msgs::CmdVel2D.theta`.

7. `<scale_angular>` (`double`, default: `1.0`) \
A scale factor to apply to the value of the angular joystick axis (which is in the range -1 to 1).

## LinkJointControllerPlugin

This is a Gazebo model plugin that controls the movement of
a Link and Joint based upon an `ignition::msgs::CmdVel2D` message.
A pair of PID controllers are used to set either the `velocity`
or `force` for a single link and either the `position`, `velocity` or `force`
for a single joint.

### Usage

Add the SDF for the plugin to the `<model>` element of your Gazebo model:

```xml
<sdf version="1.6">
  <model name="model">

    <!-- Model plugins -->
    <plugin name="link_joint_controller" filename="libLinkJointControllerPlugin.so">
      <input_topic>/cmd_vel</input_topic>
      <link_name>thruster_link</link_name>
      <link_scale>2</link_scale>
      <link_type>velocity</link_type>
      <link_pid>1000 0 10</link_pid>
      <joint_name>thruster_joint</joint_name>
      <joint_scale>0.8</joint_scale>
      <joint_type>position</joint_type>
      <joint_pid>1000 0 10</joint_pid>
    </plugin>

    <!-- Other Model elements... -->

  </model>
</sdf>  
```

### Subscribed Topics

1. `/cmd_vel` ([`ignition::msgs::CmdVel2D`](https://bitbucket.org/ignitionrobotics/ign-msgs))

### Parameters

1. `<input_topic>` (`string`, default: `/cmd_vel`) \
The name of the topic to subscribe to `ignition::msgs::CmdVel2D` messages.

2. `<link_name>` (`string`, default: `link`) \
The name of the link to control.

3. `<link_scale>` (`double`, default: `1`): \
A scale factor to apply to the value of `ignition::mgs::CmdVel2D.velocity` that sets the target motion of the link.

4. `<link_type>` (`string`, default: `velocity`) \
The type of motion or action applied to the link. Options are `velocity` or `force`.

5. `<link_pid>` (`Vector3D`, default: `(1000 0 10)`) \
PID gains (kp, ki, kd) for the link PID controller. Only applied when the motion type is `velocity`.

2. `<joint_name>` (`string`, default: `joint`) \
The name of the joint to control.

3. `<joint_scale>` (`double`, default: `1`): \
A scale factor to apply to the value of `ignition::mgs::CmdVel2D.theta` that sets the target motion of the joint.

4. `<joint_type>` (`string`, default: `position`) \
The type of motion or action applied to the joint. Options are `position`, `velocity` or `force`.

5. `<joint_pid>` (`Vector3D`, default: `(1000 0 10)`) \
PID gains (kp, ki, kd) for the joint PID controller. Only applied when the motion type is `position` or `velocity`.


## Example

Launch a Gazebo session:

```bash
roslaunch asv_joy_gazebo demo.world verbose:=true
```

Use `ignition-tools` to list the topics:

```bash
$ ign topic --list
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

## License

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.


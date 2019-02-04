# ASV Joy

An alternative joystick plugin for Gazebo with support for OSX.

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

```bash
catkin config --cmake-args \
  -DCATKIN_ENABLE_TESTING=1 \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
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

## Usage

Add the `SDLJoyPlugin` plugin elements to your world file:

```xml
<sdf version="1.6">
  <world name="ocean_world">

    <!-- World plugins -->
    <plugin name="joy" filename="libSDLJoyPlugin.so">
      <joy_id>0</joy_id>
      <topic>/joy</topic>
      <rate>50</rate>
      <accumulation_rate>1000</accumulation_rate>
      <dead_zone>0</dead_zone>
      <sticky_buttons>false</sticky_buttons>
    </plugin>

    <!-- Remainder of world declarations... -->

  </world>
</sdf>  
```

Use `roslaunch` to start a Gazebo session:

```bash
roslaunch asv_joy demo.world verbose:=true
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
    sec: 1549285972
    nsec: 788044000
  }
}
axes: 0.003921628
axes: 0.003921628
axes: -0.019607842
axes: 0.003921628
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0
buttons: 0

```

## License

ASV Joy is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

ASV Joy is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.


# MobiSyst - TBot

This project is a ROS Catkin package that includes usefull elements for turtlebot2 robots.

Last system version: **Ubuntu 20.04 lts** / **ROS neotic**.

Thanks:

- [gaunthan for Turtlebot2 on Melodic](https://github.com/gaunthan/Turtlebot2-On-Melodic)
- [TheConstruct](https://www.theconstructsim.com/) the virtualisation and the simulation environment they provide.

## Installation

### Dependencies

**TBot Requires an Ubuntu/ROS machine.**

- [Installation procedure](https://wiki.ros.org/noetic/Installation/Ubuntu)

Then it depends also on several packages not installed with the desktop version of ROS:

```bash
./install_apt_deps.sh
# or
sudo apt update
sudo apt install -y ros-noetic-ecl-exceptions \
    ros-noetic-ecl-threads \
    ros-noetic-ecl-geometry \
    ros-noetic-ecl-streams \
    ros-noetic-kobuki-* \
    ros-noetic-depthimage-to-laserscan \
    ros-noetic-joy \
    ros-noetic-urg-node \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-plugins \
    ros-noetic-depth-image-proc
```

To finalize kobuki installation (http://wiki.ros.org/kobuki/Tutorials/Installation)

```bash
sudo usermod -a -G dialout $USER
roscore &
rosrun kobuki_ftdi create_udev_rules
```

This script normally allows udev to recgnize a connected kobuki and to install it on `/dev/kobuki` when the usb connector is plugged to the computer.

Restart the machine to have everything configured properly.

**TBot perception is based on RealSense Camera**

Install RealSense drivers:

* [Linux instruction](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md))
* [ROS installation](https://github.com/IntelRealSense/realsense-ros#installation-instructions)
* [wiki ros](http://wiki.ros.org/RealSense) - The interesting pkg is the **realsense2_camera**

First you want to configure the software manager (`apt`) to follow Intel Realsense repository.

1. Register the server's public key:

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
#or, if you are on a restricted network with closed ports...
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```

2. Then add the server to the list of repositories: 

```bash
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
```

Now, you can easily install the libraries and tools:

```bash
sudo apt install \
    librealsense2-dkms \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg
```

Connect the Intel RealSense depth camera and run: `realsense-viewer`.

Then you can install appropriate ROS packages

Ressources:


Installation:

```bash
sudo apt-get install \
    ros-noetic-librealsense2 \
    ros-noetic-realsense2-camera \
    ros-noetic-realsense2-description
```

Then you can launch the camera to publish the data into ROS topics:

```bash
roslaunch realsense2-camera rs_camera.launch
```


### TBot

Clone this repo in your catkin directory:

```bash
cd $HOME/catkin_ws
git clone https://bitbucket.org/imt-mobisyst/mb6-tbot src/mb6-tbot
```

Build-it:

```bash
catkin_make
source devel/setup.bash
```

## How to use it

### real robot

Switch-on a turtlebot and connect it to the machine.

Start ROS with the robot:

```bash
roslaunch turtlebot_bringup minimal.launch
```

Stop the programs (`ctrl-c`).


For a complete configuration: (turtlebot + laser + realsens...):

```bash
roslaunch tbot_bringup minimal.launch
```


### Gazebo simulation of tbot

```bash
roslaunch tbot_gazebo start_world.launch
roslaunch tbot_gazebo spawn_tbot.launch
```

### larm

```bash
roslaunch larm challenge-3.launch
```


# FAQ

## view_frames

There is a known bug in noetic: https://github.com/ros/geometry/pull/193

```
$ rosrun tf view_frames
Listening to /tf for 5.0 seconds
Done Listening
b'dot - graphviz version 2.43.0 (0)\n'
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/tf/view_frames", line 119, in <module>
    generate(dot_graph)
  File "/opt/ros/noetic/lib/tf/view_frames", line 89, in generate
    m = r.search(vstr)
TypeError: cannot use a string pattern on a bytes-like object
```

Line 89 of `/opt/ros/noetic/lib/tf/view_frames`:

```python
m = r.search(vstr.decode('utf-8'))
```

```
sudo sed -i 's/m = r.search(vstr)/m = r.search(vstr.decode("utf-8"))/' /opt/ros/noetic/lib/tf/view_frames
```

## python error

Certains scripts utilisent :

```python
#!/usr/bin/env python
```

or `python` n'existe plus en ubuntu 20.04 et il faut privil�gier l'utilisation explicite de :

```python
#!/usr/bin/env python3
```

# TODO

- tbot rgbd visual model broken in rviz

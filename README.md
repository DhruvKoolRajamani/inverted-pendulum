# Linear Inverted Pendulum ROS Package

This package serves as a tutorial for the open source community for ROS, Gazebo and various control methodologies

## Check Ubuntu Version
### Install ROS-Kinetic for Ubuntu 16
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Install ROS-Indigo for Ubuntu 14
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-indigo-desktop
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```

## Install Gazebo

```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo7 libgazebo7-dev
```

## Install other dependencies : Check Ubuntu Version
### ROS Kinetic - Ubuntu 16

```
sudo apt-get install ros-kinetic-gazebo-plugins ros-kinetic-kdl-parser ros-kinetic-kdl-parser-py ros-kinetic-kdl-conversions ros-kinetic-kdl-typekit ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-msgs ros-kinetic-controller-interface ros-kinetic-controller-manager ros-kinetic-control-toolbox ros-kinetic-control-msgs ros-kinetic-effort-controllers ros-kinetic-position-controllers ros-kinetic-joint-limits-interface ros-kinetic-joint-state-controller ros-kinetic-joint-state-publisher ros-kinetic-joint-states-settler ros-kinetic-joint-trajectory-action ros-kinetic-joint-trajectory-action-tools ros-kinetic-joint-trajectory-generator ros-kinetic-srdfdom ros-kinetic-urdf ros-kinetic-urdfdom-py ros-kinetic-urdf-parser-plugin ros-kinetic-gazebo-ros-pkgs
```
### ROS Indigo - Ubuntu 14

```
sudo apt-get install ros-indigo-gazebo-plugins ros-indigo-kdl-parser ros-indigo-kdl-parser-py ros-indigo-kdl-conversions ros-indigo-kdl-typekit ros-indigo-gazebo7-ros ros-indigo-gazebo7-ros-control ros-indigo-gazebo7-msgs ros-indigo-controller-interface ros-indigo-controller-manager ros-indigo-control-toolbox ros-indigo-control-msgs ros-indigo-effort-controllers ros-indigo-position-controllers ros-indigo-joint-limits-interface ros-indigo-joint-state-controller ros-indigo-joint-state-publisher ros-indigo-joint-states-settler ros-indigo-joint-trajectory-action ros-indigo-joint-trajectory-action-tools ros-indigo-joint-trajectory-generator ros-indigo-srdfdom ros-indigo-urdf ros-indigo-urdfdom-py ros-indigo-urdf-parser-plugin ros-indigo-gazebo7-ros-pkgs
```
## Instructions

In order to run this package, add it to your catkin workspace. You can use
the following commands:

If you dont have a catkin workspace initialized:
```
mkdir -p ~/catkin_ws/src
cd catkin_ws/src/
git clone https://gitlab.com/biorobotics-group-manipal/lip.git
cd ~/catkin_ws/
catkin_make
```
Now source your workspace from the catkin base directory and continue: 

`source devel/setup.bash`

or add to .bashrc

`echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc`

Start roscore: `roscore`

### Open a New Terminal Window

Gazebo Simulaton: `rosrun lip_description pysim.py`

rViz Simulation: `rosrun lip_description pyviz.py`

### Open a New Terminal Window

Load controllers: `rosrun lip_control controllers.py`

### Open a New Terminal Window

Run CPP Controller: `rosrun lip_control PID_controller`

## Acknowledgement

This project is a Work In Progress (WIP) under the Biorobotics Group.
# TurtleBot3-MPC

## Install 

### ROS Noetic install (Ubuntu 20.04)

```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init

rosdep update
```

### RTabMAP and Octomap install

```
sudo apt install ros-noetic-octomap-server
sudo apt install ros-noetic-octomap ros-noetic-octomap-rviz-plugins ros-noetic-rtabmap ros-noetic-rtabmap-ros
```

### Corbo systems
```
git clone -b master https://github.com/rst-tu-dortmund/control_box_rst.git

 
cd control_box_rst
mkdir build
cd build
cmake ..
make -j8 -l8
sudo make install
```

### Run 

```
# Export turtlebot model
export TURTLEBOT3_MODEL=waffle
```

#### Build Octomap

```
roslaunch turtlebot_octomap turtlebot3_build_map.launch
```

#### Control
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

#### Save map
```
rosrun octomap_server octomap_saver -f octomap_test.bt
```

#### Navigation
```
export TURTLEBOT_MAP_FILE=/home/lacie-life/catkin_ws/src/TurtleBot3-MPC/turtlebot_navigation/maps/willow-2010-02-18-0.10.yaml


```


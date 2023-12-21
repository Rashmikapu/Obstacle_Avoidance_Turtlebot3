
# TurtleBot3_Walker
Simple algorithm to implement obstacle avoiding algorithm and move TurtleBot3 in a pre built world using ROS2 Humble and Gazebo.
<br>
If you have ROS2 Humble already installed, please skip the installation steps.

## Installing ROS2 on Ubuntu 22.04
Follow this ![link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Install-Binary.html) to install ROS2 Humble on your new Ubuntu 22.04 Desktop (binary installation).

## Installing ROS2 on Ubuntu 20.04
Follow this ![link](http://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html) carefully and select Ubuntu 20.04 wherever it is necessary and install from source.

## 1.) Install docker 

For native Ubuntu :
```
sudo apt install docker.io 

sudo usermod  -aG docker $USER   # add yourself to the "docker" group
newgrp docker  # or logout and re-login completely (not just opening a new terminal)
```

# Verify installation (for both native and WSL2 Ubuntus):

```
sudo service docker start
sudo service docker status
```

# For Ubuntu > 18

```
sudo apt install python3-rocker
```

# Verify by :

```
rocker --version
rocker alpine echo "hello-world"
```

# 2.) Install and modify ROS2 Humble Docker image

Terminal 1:

## download ROS 2 image
```
docker pull osrf/ros:humble-desktop-full

# Run ROS 2 container
rocker osrf/ros:humble-desktop-full bash

# inside ROS 2 container, install packages
apt update;
apt -y install terminator
apt -y install ros-humble-gazebo-ros-pkgs 
apt -y install ros-humble-turtlebot3*
apt -y install python3-colcon-common-extensions
apt -y install ros-humble-turtlebot4-desktop

apt clean all
```

Terminal 2:
Open another terminal and run the command below to save a snapshot of the container.

```
# get the ID of the container running on terminal 1
# assume there is only on container running
CONTAINER_ID=$(docker ps --format {{.ID}})
echo $CONTAINER_ID

# Save the Docker image snapshot 
docker commit $CONTAINER_ID my-docker2-humble
```

The modified docker image is now called my-docker2-humble.

## Tutorials
- Follow these ![tutorial](http://docs.ros.org/en/humble/Tutorials.html) to understand the basics of ROS2. If you have installed the ROS2 from source, the ros2 setup path will be your 
```
. <installation directory>/install/setup.bash
```
- Source installation installs turtlesim and other packages. No need to use the commands listed in the above tutorial. 
- `colcon` can be installed by running the command:
```
sudo apt install python3-colcon-common-extensions
```

## Dependencies
- Make sure that you have TurtleBot3 packages installed from ![TB3 ROBOTIS Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup). In this tutorial, replace foxy with humble.
- To get the gazebo_ros_packages, follow the instructions from ![Gazebo](http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros#InstallGazebo) manual and replace foxy with humble.

- Required packages to be in the source directory if installed from source:
1. DynamixelSDK
2. gazebo_ros_packages
3. turtlebot3
4. turtlebot3_msgs
5. turtlebot3_simulations
6. vision_opencv


## Clone and Build the package
- Clone this repository in your ROS2 Workspace /src folder. It should create TurtleBot3_Walker folder in your /src.
```
git clone https://github.com/Rashmikapu/Obstacle_Avoidance_Turtlebot3.git
```
Make sure that your terminal is sourced

```
source /opt/ros/humble/setup.bash
```

- Run the below commands:
```
cd TurtleBot3_Walker
rosdep install -i --from-path src --rosdistro humble -y
cd ../.. # It should take your present working directory to <ROS2_workspace>
colcon build --packages-select obstacle_avoidance
. install/setup.bash
```

## Set the Gazebo model path and TurtleBot3 model
- Run the below commands in the terminal you wish to run the obstacle avoidance.
```
export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=`ros2 pkg prefix turtlebot3_gazebo`/share/turtlebot3_gazebo/models/

```

- When a new terminal is opened, please source your terminal using
```
. <ros2_installation_directory>/install/setup.bash
```
or, in the workspace, run :

```
. ./install/setup.bash
```

## Using ROS2 Bag files to store the published data

- Command to launch with recording
```
ros2 launch turtlebot_walker launch.py record_flag:=True
```

- Command to launch without recording
```
ros2 launch turtlebot_walker launch.py record_flag:=False
```

- Run this for about 15 seconds and press Ctrl+C to store the data published. 
- To play the bag file, run the below commands

In terminal 1 (first source ros2 humble):

In original world :
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
In empty world :

```
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
In terminal 2, move to the workspace and run (first source ros2 humble) :
```
ros2 bag play walker_bag
```

## Output Video :

![Alt Text](result_fast.mp4)

## Issues encountered

- The required dependencies should be installed prior to run this package. 
- Encountered issues with linking of libstdc++ while using docker image to run ros2 humble. Solved by running the following in docker humble : 
```
ln -sf /usr/lib/x86_64-linux-gnu/libstdc++.so.6 /home/rashmikapu/anaconda3/lib/libstdc++.so.6
```

## Static Code Analysis
### cpplint
Run the below command from inside the package folder `Obstacle_Avoidance_Turtlebot3`
```
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order src/*.cpp &> ./results/cpplint.txt
```
### cppcheck
Run the below command from the project root folder `Obstacle_Avoidance_Turtlebot3`
```
cppcheck --enable=all --std=c++17 src/*.cpp --suppress=missingIncludeSystem --suppress=missingInclude --suppress=unmatchedSuppression > ./results/cppcheck.txt
```
## Tutorials followed :
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html
http://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros

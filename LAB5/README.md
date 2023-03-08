# LAB5 - limo vehicle Simulation

-- Course: *Intelligent Robotics – Professor: Qi Hao*

-- Environment: Ubuntu 20.04, ros-noetic

编写 孙耀威 12232434

[TOC]

# 1.gazebo中limo小车的移动仿真

## 安装


 limo_description: The file is the function package of model file

 limo_gazebo_sim: The folder is gazebo simulation function package

### 下载环境

 Download and install ros-control function package, ros-control is the robot control middleware provided by ROS

```
# ubuntu18
sudo apt-get install ros-melodic-ros-control

# ubuntu20
sudo apt-get install ros-noetic-ros-control
```

 Download and install ros-controllers function package, ros-controllers are the kinematics plug-in of common models provided by ROS

```
# ubuntu18
sudo apt-get install ros-melodic-ros-controllers

# ubuntu20
sudo apt-get install ros-noetic-ros-controllers
```

 Download and install gazebo-ros function package, gazebo-ros is the communication interface between gazebo and ROS, and connect the ROS and Gazebo

```
# ubuntu18
sudo apt-get install ros-melodic-gazebo-ros

# ubuntu20
sudo apt-get install ros-noetic-gazebo-ros
```

 Download and install gazebo-ros-control function package, gazebo-ros-control is the communication standard controller between ROS and Gazebo

```
# ubuntu18
sudo apt-get install ros-melodic-gazebo-ros-control

# ubuntu20
sudo apt-get install ros-noetic-gazebo-ros-control
```

 Download and install joint-state-publisher-gui package.This package is used to visualize the joint control.

```
# ubuntu18

sudo apt-get install ros-melodic-joint-state-publisher-gui 
# ubuntu20
sudo apt-get install ros-noetic-joint-state-publisher-gui 
```

 Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```
# ubuntu18
sudo apt-get install ros-melodic-rqt-robot-steering 

# ubuntu20
sudo apt-get install ros-noetic-rqt-robot-steering 
```

 Download and install teleop-twist-keyboard function package, telop-twist-keyboard is keyboard control function package, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
# ubuntu18
sudo apt-get install ros-melodic-teleop-twist-keyboard 

# ubuntu20
sudo apt-get install ros-noetic-teleop-twist-keyboard 
```

### 安装limo 仿真库

#### 1.	Create workspace, download simulation model function package and compile

 Open a new terminal and create a workspace named limo_ws, enter in the terminal:

```
mkdir limo_ws
```

 Enter the limo_ws folder

```
cd limo_ws
```

 Create a folder to store function package named src
```
mkdir src
```

 Enter the src folder

```
cd src
```

 Initialize folder

```
catkin_init_workspace
```

 Download simulation model function package

```
git clone https://github.com/agilexrobotics/ugv_sim.git
```

 Enter the limo_ws folder

```
cd limo_ws
```

 Confirm whether the dependency of the function package is installed

```
rosdep install --from-paths src --ignore-src -r -y 
```

 Compile

```
catkin_make
```

## 2.	在rviz中查看limo小车的模型


 Enter the limo_ws folder

```
cd limo_ws
```

Declare the environment variable

```
source devel/setup.bash
```

Run the start file of limo and visualize the model in Rviz

```
roslaunch limo_description display_models.launch 
```

![img](image/rviz.png) 

## 3.	Start the gazebo simulation environment of limo and control limo movement in the gazebo

在gazebo中进行limo小车的运动仿真

Enter the limo_ws folder

```
cd limo_ws
```

Declare the environment variable

```
source devel/setup.bash
```

Start the simulation environment of limo, limo have two movement mode, the movement mode is Ackerman mode

开启limo的gazebo环境仿真
1. 打开空白的世界诶
```
#这条命令会打开一个空白的世界
roslaunch limo_gazebo_sim limo_ackerman.launch
```
2. 打开含有地图的世界

```
roscd limo_gazebo_sim/
cd launch
gedit limo_ackerman.launch
```
可以看到 limo_ackerman.launch 文件，其中第五行有个world_name的参数,默认参数是empty.world

可以选择的世界有两种
```
willowgarage.world
clearpath_playpen.world
```
选择其一替换掉empty.world，然后再运行
```
roslaunch limo_gazebo_sim limo_ackerman.launch
```
这里因为修改了默认模型，所以会打开新的世界而不是空白的世界


### 控制方法1
Start rqt_robot_steering movement control plug-in, the sliding bar can control the robot motion

打开一个简单的控制器，可以通过简单的速度控制和角度控制来控制车辆移动
```
rosrun rqt_robot_steering rqt_robot_steering
```

![img](image/limo_ackerman.png) 

### 控制方法2

Four-wheel differential steering movement mode

```
roslaunch limo_gazebo_sim limo_four_diff.launch 
```

### 控制方法3

Control by keyboard, the robot can be controlled to move forward, left, right and backward through "i", "j", "l",and "," on the keyboard

```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

![img](image/limo_diff.png) 

 

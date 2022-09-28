# Intelligent-Robots-Lab6

Lab6 materials for the intelligent robotics course


# 3D Modeling and Simulation

### Learning objectives
1. Make the smartcar appear in gazebo  
2. Let the smartcar go round in Gazebo  
3. Add various sensors to the samrtcar and view the data in RVIZ  
4. Control the smartcar with the keyboard 
5. Let the smartcar in the maze

#### 1. Make the smartcar appear in gazebo  
**Related file**: smartcar_base.gazebo.xacro  
Firstly, let the Smartcar model be shown in the Gazebo.  

##### 1.1 Add the inertial and collision properties to the link  
01.png
02.png

##### 1.2 Add the gazebo texture to the link  
03.png
04.png
05.png


#### 2. Let the smartcar go round in Gazebo
**Related file**: smartcar_base.gazebo.xacro   
Secondly, let the Smartcar model move in the Gazebo.

##### 2.1 Three kinds of models  
06.png
07.png
08.png

##### 2.2 How to control the robot
**From top to down:**  
· **Controller manager:**  
Providing a general interface to manage different controllers.   
· **Controller:**  
Reading Hardware Status, Sending control command, finishing the control of every joint.  
· **Hardware resources:**  
The interface that provides hardware resources for upper and lower layers.  
· **Robot hardware abstraction:**  
Immediately access hardware resources. Finish operations by invoking methods 'write' and 'read'.  
· **Real Robot:**  
Execute the received commands.
09.png

##### 2.3 Let the robot move with two ways 
**(In this lab, we choose the second way)**

· **The first option:**
  1. Add transmission elements  
    把joint关节转换成了各式各样的在gazebo下的电机
    10.png  
    **<joint name=""\>name必须于URDF中其它地方的joint对应**  
    **<type\> ：transmission类型，目前仅仅实现了"transmission_interface/SimpleTransmission"**  
    **<hardwareInterface\> ：放置在<actuator> 和<joint> tags之内,用来告知gazebo_ros_control插件哪些硬件接口要加载进来(position, velocity or effort interfaces)。**
  2. Add the gazebo_ros_control plugin  
    除了transmission tags, Gazebo插件也需要添加到URDF文件，该插件作用是解析transmission tags，加载 hardware interfaces和controller manager。  
    11.png

· **The second option:**  
  1. Add the gazebo controller plugin  
    12.png
    13.png

  **(In this lab, we choose the second way for the smartcar!!!)**

  **Load the robot model in gazebo:**   
  · **Run the command below:**  

  > $ cd ~/smartcar_ws/src  
  >
  > $ catkin_create_pkg smartcar_gazebo gazebo_plugins gazebo_ros gazebo_ros_control roscpp rospy

  · **Create the view_smartcar_gazebo_empty_world.launch file in the smartcar_gazebo/launch folder, and put the code in it:**  
  14.png

  · **Launch it with the following commands:**
  > $ cd ~/smartcar_ws/
  >
  > $ catkin_make
  >
  > $ roslaunch smartcar_gazebo view_smartcar_gazebo_empty_world.launch

  15.png


#### 3. Install various sensors

##### 3.1 Sensors for Gazebo(kinect)
  · **kinect simulation**  
    · <sensor\>   
    · <camera\>  
    · <plugin\>  
  16.png

##### 3.2 Sensors for Gazebo(lidar)
  · **lidar simulation**  
  17.png

##### 3.3 Sensors for Gazebo(camera)
  · **kinect simulation**  
    · <sensor\>   
    · <camera\>  
    · <plugin\>  
  18.png

##### 3.4 Install lidar and camera for the smartcar
  · **Adding lidar and camera in xacro file**  
  19.png  

  · **Create launch file**  
    · **Create the view_smartcar_with_laser_camera.launch file in the smartcar_gazebo/launch folder, and put the code in it:**  
  20.png

  · **Excute following commands:**
  > $ cd ~/smartcar_ws/
  >
  > $ catkin_make
  >
  > $ roslaunch smartcar_gazebo view_smartcar_with_laser_camera.launch

  21.png
  22.png

##### 3.5 Install kinect for the smartcar
  · **Adding kinect in xacro file:**  
  23.png  
  ` **Create launch file**  
  24.png  

  > $ roslaunch smartcar_gazebo view_smartcar_with_kinect_gazebo.launch

  25.png  


#### 4. Create a physical simulation environment
##### 4.1 Add the environment model using the insert TAB
  > $ roslaunch smartcar_gazebo smartcar_with_laser_nav.launch

  26.png  
  27.png

##### 4.2 Using Building Editor
  28.png  
  29.png

#### 5. Control the smartcar with the keyboard
· **Excute following commands:**
  > $ cd ~/smartcar_ws/src
  >
  > $ catkin_create_pkg smartcar_telep geometry_msgs roscpp rospy
  >
  > $ cd smartcar_telep
  >
  > $ mkdir scripts
  >
  > $ cd scripts
  >
  > $ touch smartcar_teleop_key.py

  30.png  
  31.png

  > $ cd ~/smartcar_ws/src
  >
  > $ catkin_create_pkg smartcar_telep geometry_msgs roscpp rospy

  32.png

#### Finally. Lab Task
  · **Complete the lab content of lab ppt ,and submit the zip file of 'smartcar_description' and 'smartcar_gazebo' packages to BB .**

#### Futher References:
  · http://wiki.ros.org/urdf  
  · http://wiki.ros.org/urdf/Tutorials  
  · http://wiki.ros.org/urdf/XML  
  · http://wiki.ros.org/xacro  
  · http://gazebosim.org/tutorials  
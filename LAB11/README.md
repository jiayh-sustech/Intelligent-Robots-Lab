# Intelligent-Robots-Lab11

## How to publish goal to smartcar ?

### move_base

<img width="1220" alt="image" src="https://user-images.githubusercontent.com/73091180/192712662-2e8f9573-9e99-421c-b923-f32126f8274c.png">

- Topics and services in `move_base` package

<img width="1260" alt="image" src="https://user-images.githubusercontent.com/73091180/192713039-43ce847a-b1aa-4b06-8739-886c39d4c719.png">

### Actionlib

<img width="930" alt="image" src="https://user-images.githubusercontent.com/73091180/192713098-92472995-28f6-4ef6-9406-d8e8fe39ce35.png">

<img width="651" alt="image" src="https://user-images.githubusercontent.com/73091180/192713132-8661d474-4f07-49a5-9b00-c387c9942963.png">

Ros Messages:

- goal: Used to send new goals to servers
- cancel: Used to send cancel requests to servers
- status: Used to notify clients on the current state of every goal in the system
- feedback: Used to send clients periodic auxiliary information for a goal
- result: Used to send clients one-time auxiliary information upon completion of a goal

### Programming Demo

Step 1.

```
$cd ~/smartcar_ws/src/smartcar_navigation
$mkdir scripts
$cd scripts
$touch move_goal.py
```

- move_goal.py

  ```python
  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import roslib
  import rospy
  import actionlib
  from actionlib_msgs.msg import *
  from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
  from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

  # Initialize the node
  rospy.init_node('move_test', anonymous=True)
  # Subscribe messages from move_base server
  move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  rospy.loginfo('Waiting for move_base action server...')

  # Wait for response from server for at most 5 seconds
  while move_base.wait_for_server(rospy.Duration(5.0)) == 0:
      rospy.loginfo("Connected to move base server")

  # Set target
  target = Pose(Point(-0.543, 0.779, 0.000), Quaternion(0.000, 0.000, 0.645, 0.764))
  goal = MoveBaseGoal()
  goal.target_pose.pose = target
  goal.target_pose.header.frame_id = 'map'
  goal.target_pose.header.stamp = rospy.Time.now()

  rospy.loginfo('Going to ' + str(target))

  # Set out to the goal
  move_base.send_goal(goal)

  # Five minutes limit
  finished_within_time = move_base.wait_for_result(rospy.Duration(300))

  # Check if succeeded
  if not finished_within_time:
      move_base.cancel_goal()
      rospy.loginfo('Timed out achieving goal')
  else:
      state = move_base.get_state()
      if state == GoalStatus.SUCCEEDED:
          rospy.loginfo('Goal Succeeded!')
      else:
          rospy.loginfo('Goal Failed!')
  ```

  

Step 3.

- Terminal 1:

  ```
  roslaunch limo_gazebo_sim limo_four_diff.launch
  ```

- Terminal 2:

  ```
  roslaunch smartcar_navigation smartcar_navigation_teb.launch
  ```

- Terminal 3:

  ```
  rosrun smartcar_navigation move_goal.py
  ```

### Programming Demo 2

- maze_patrol.py

  ```python
  #!/usr/bin/env python
  # -*- coding: utf-8 -*-

  import rospy
  import actionlib
  import tf
  from math import pi
  from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
  from actionlib_msgs.msg import *

  waypoints = [ # List of way points
      [(0, -1, 0.0), tf.transformations.quaternion_from_euler(0, 0, 270*pi/180)],
      [(-1, -1, 0.0), tf.transformations.quaternion_from_euler(0, 0, 180*pi/180)],
      [(-1, 0, 0.0), tf.transformations.quaternion_from_euler(0, 0, 90*pi/180)],
      [(0, 0, 0.0), tf.transformations.quaternion_from_euler(0, 0, 0*pi/180)],
  ]

  def goal_pose(pose): # Convert from waypoints to goal positions
      goal_pose = MoveBaseGoal()

      goal_pose.target_pose.header.frame_id = 'map'

      goal_pose.target_pose.pose.position.x = pose[0][0]
      goal_pose.target_pose.pose.position.y = pose[0][1]
      goal_pose.target_pose.pose.position.z = pose[0][2]

      goal_pose.target_pose.pose.orientation.x = pose[1][0]
      goal_pose.target_pose.pose.orientation.y = pose[1][1]
      goal_pose.target_pose.pose.orientation.z = pose[1][2]
      goal_pose.target_pose.pose.orientation.w = pose[1][3]

      return goal_pose

  if __name__ == '__main__':
      rospy.init_node('maze_patrol')

      move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction) # Creates the action client
      move_base.wait_for_server()

      for pose in waypoints: # Loop through each waypoints, sending them as goals
          goal = goal_pose(pose)
          move_base.send_goal(goal)
          print('Moving towards goal...')

          # Five minutes budget
          finished_within_time = move_base.wait_for_result(rospy.Duration(100))

          # Check if succeeded
          if not finished_within_time:
              move_base.cancel_goal()
              rospy.loginfo('Timed out achieving goal')
          else:
              state = move_base.get_state()
              if state == GoalStatus.SUCCEEDED:
                  rospy.loginfo('Goal Succeeded!')
              else:
                  rospy.loginfo('Goal Failed!')

      print('Completed.')
  ```

- Terminal 1

  ```
  roslaunch limo_gazebo_sim limo_four_diff.launch
  ```

- Terminal 2

  ```
  roslaunch smartcar_navigation smartcar_navigation_teb.launch
  ```

- Terminal 3

  ```
  rosrun smartcar_navigation maze_patrol.py
  ```

## The robot moves and maps autonomously

Step 1. Write  a launch file for move_base + slam

```
cd ~/smartcar_ws/src/smartcar_navigation
mkdir launch
cd launch
touch smartcar_slam_navigation.launch
```

- smartcar_slam_navigation.launch

  ```xml
  <launch>
      <!--Activate SLAM node-->
      <include file="$(find smartcar_slam)/launch/smartcar_gmapping.launch"/>
      <!--Activate move_base node-->
      <include file="$(find smartcar_navigation)/launch/move_base_teb.launch"/>
      <!--Run rviz-->
      <node pkg="rviz" type="rviz" name="rviz" args="-d $(find smartcar_navigation)/rviz/nav_teb.rviz"/>
  </launch>
  ```

Step 2.test

- bring up the smartcar

  ```
  roslaunch limo_gazebo_sim limo_four_diff.launch
  ```

- run the slam_navigation launch file

  ```
  roslaunch smartcar_navigation smartcar_slam_navigation.launch
  ```

## RRT exploration

简单来说， RRT算法是一种树型算法，它由一个起始点Xinit作为树的起始节点（或者叫根节点），然后从这 个起始点进行随机生长，通过随机采样增加叶子节点Xnew的方式，生成一个随机扩展树，当随机树中的叶子 节点包含了目标点或进入了目标区域，便从随机树中找到一条由从初始点到目标点的路径

<img width="1154" alt="image" src="https://user-images.githubusercontent.com/73091180/192713266-b9aa8337-717c-4db8-88dc-dd40ce422b34.png">

### Programming Demo

For more details, please refer to http://wiki.ros.org/rrt_exploration

Step 1. Download code

```
cd ~/smartcar_ws/src
git clone https://github.com/hasauino/rrt_exploration.git
git clone https://github.com/hasauino/rrt_exploration_tutorials
```

Step 2. Package Installation

```
sudo apt-get install python3-opencv python3-numpy python3-sklearn ros-noetic-gmapping ros-noetic-navigation
```

Step 3. Building

```
$cd ~/smartcar_ws/src
$catkin_make
```

Step 4. Write a move_base + slam launch file for rrt

```
cd ~/smartcar_ws/src/smartcar_navigation
mkdir launch
cd launch
touch smartcar_slam_navigation_rrt.launch
touch move_base_safe.launch
```

- smartcar_slam_navigation_rrt.launch

  ```xml
  <launch>
      <include file="$(find smartcar_slam)/launch/smartcar_gmapping.launch"/>
      <include file="$(find smartcar_navigation)/launch/move_base_safe.launch"/>
  </launch>
  ```

- move_base_safe.launch

  ```xml
  <launch>
      <arg name="cmd_vel_topic" default="/cmd_vel"/>
      <arg name="odom_topic" default="odom"/>
      <arg name="move_forward_only" default="false"/>
      
      <node pkg="move_base" type="move_base" respawn="false" name="move_base_node" output="screen">
          <param name="base_local_planner" value="teb_local_planner/TebLocalPlannerROS"/>
          
          <rosparam file="$(find smartcar_navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap"/>
          <rosparam file="$(find smartcar_navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap"/>
          <rosparam file="$(find smartcar_navigation)/config/local_costmap_params.yaml" command="load"/>
          <rosparam file="$(find smartcar_navigation)/config/global_costmap_params.yaml" command="load"/>
          <rosparam file="$(find smartcar_navigation)/config/move_base_params.yaml" command="load"/>
          
          <rosparam file="$(find smartcar_navigation)/config/teb_local_planner_params.yaml" command="load"/>
          <remap from="cmd_vel" to="$(arg cmd_vel_topic)"/>
          <remap from="odom" to="$(arg odom_topic)"/>
          <param name="DWAPlannerROS/min_vel_x" value="0.0" if="$(arg move_forward_only)"/>
      </node>
  </launch>
  ```

Step 5. Modify `src/rrt_exploration/launch/single.launch` 

- single.launch   Modified

  ```xml
  <launch>
      <arg name="eta" value="1.0"/>
      <arg name="Geta" value="15.0"/>
      <param name="name_space_init_count" value="1"/>
      
      <node pkg="rrt_exploration" type="global_rrt_detector" name="global_detector" output="screen">
          <param name="eta" value="$(arg eta)"/>
          <param name="map_topic" value="map"/>
      </node>
      
      <node pkg="rrt_exploration" type="local_rrt_detector" name="local_detector" output="screen">
          <param name="eta" value="$(arg eta)"/>
          <param name="map_topic" value="map"/>
          <param name="robot_frame" value="base_foorprint"/>
      </node>
      
      <node pkg="rrt_exploration" type="filter.py" name="filter" output="screen">
      	<param name="map_topic" value="map"/>
    		<param name="info_radius" value="1"/> 
    		<param name="costmap_clearing_threshold" value="70"/> 
    		<param name="goals_topic" value="/detected_points"/>
    		<param name="n_robots" value="1"/>
    		<param name="rate" value="100"/>
      </node>
      
      <node pkg="rrt_exploration" type="assigner.py" name="assigner" output="screen">
          <param name="map_topic" value="map"/>
          <param name="global_frame" value="map"/>
          <param name="info_radius" value="1"/> 
          <param name="info_multiplier" value="3.0"/> 
          <param name="hysteresis_radius" value="3.0"/> 
          <param name="hysteresis_gain" value="2.0"/> 
          <param name="frontiers_topic" value="/filtered_points"/> 
          <param name="n_robots" value="1"/>
          <param name="delay_after_assignement" value="0.5"/>
          <param name="rate" value="100"/>
       </node>
      
      <node pkg="rviz" type="rviz" name="rviz" args="-d $(find rrt_exploration)/rviz/single.rviz"/>
  </launch>
  ```

Step 6.

save the *single.rviz* file to *~/smartcar_ws/src/rrt_exploration/rviz/single.rviz*

Step 7. Test

- bring up the smartcar

  ```
  roslaunch limo_gazebo_sim limo_four_diff.launch
  ```

- run the slam_navigation launch file

  ```
  roslaunch smartcar_navigation smartcar_slam_navigation_rrt.launch
  ```

- run rrt

  ```
  roslaunch rrt_exploration single.launch
  ```
  
The result is similar as below:

![single](https://raw.githubusercontent.com/hasauino/storage/master/pictures/sequence_of_points_small.gif)

## Lab Task

Based on the Lab Project we provided, you are required to complete the following tasks:

- Use the rrt_exploration package to try out robot map exploration.
- Write a lab report for the task, the report contains code, steps, results, results analysis and others.

# Moveit2-Ros1-Control-AUBO
This is a repo for using Moveit2 and Ros1 Control to moving Aubo_i5, due to lack of Ros2 Control of Aubo.

## Pre-requests
 - Ubuntu 20.04
 - ROS Noetic
 - ROS Foxy and Moveit2
 - Docker Engine

## Install
### action_bridge and ros1_bridge
These 2 bridges are necessary for ROS2 <---> ROS1 inter-communication. Make sure you have installed [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) and [ROS Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). 

Install ros1_bridge for bridging topics and services:
``````
sudo apt get install ros-foxy-ros1-bridge
``````
Install action_bridge for bridging action, before that, install some dependence first:
``````
sudo apt install ros-noetic-actionlib ros-noetic-actionlib-tutorials ros-noetic-control-msgs ros-noetic-roscpp ros-foxy-control-msgs ros-foxy-rclcpp ros-foxy-rclcpp-action ros-foxy-action-tutorials-interfaces
``````
Then, cd to 'action_bridge_ws' to build it.
``````
cd ~/Moveit2-Ros1-Control-AUBO/action_bridge_ws/
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
colcon build
``````

### aubo_ros1_control
Make sure you have installed [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) already. You can use docker without 'sudo' the cmd. If you never done this before, it is suggested that typing in following cmd:
````
sudo groupadd docker
sudo gpasswd -a ${USER} docker
sudo systemctl restart docker
sudo chmod a+rw /var/run/docker.sock
````
Then, cd to 'aubo_ros1_control_ws' to build the docker image.
``````
cd ~/Moveit2-Ros1-Control-AUBO/aubo_ros1_control_ws/
docker build -t aubo_ros1 .
``````
Now, You just built a docker image named "aubo_ros1"

### aubo_ros2_moveit2
Install aubo ros2 moveit2 packages for motion planning, before that, make sure you have installed [moveit2](https://moveit.ros.org/install-moveit2/source/) already.

Start a new terminal, then cd to 'aubo_ros2_moveit2_ws' to build it.
``````
cd ~/Moveit2-Ros1-Control-AUBO/aubo_ros2_moveit2_ws/
source /opt/ros/foxy/setup.bash
source ~/moveit2_ws/install/setup.bash
colcon build
``````

## Usage
### Real robot
If you want to connect to real robot, follow the step 1 to 5 below.
1. ros1 master(Terminal 1)
````
source /opt/ros/noetic/setup.bash
roscore
````
2. action_bridge(Terminal 2)
````
source ~/Moveit2-Ros1-Control-AUBO/action_bridge_ws/install/setup.bash
ros2 run action_bridge action_bridge_follow_joint_trajectory_2_1 aubo_i5_controller
````
3. ros1_bridge(Terminal 3)
````
source ~/Moveit2-Ros1-Control-AUBO/action_bridge_ws/install/setup.bash
ros2 run ros1_bridge dynamic_bridge
````
4. aubo_ros1_control(Terminal 4)  

Note: you have to make sure the TCP connection to robot is available. here the ip address of robot is set as 192.168.1.115 and make sure your computer's ip address is set as 192.168.1.X. (The X could be 1~254). Then, creat and enter a docker container named "aubo_interface" of the docker image "aubo_ros1" you built before:
````
docker run --network=host -it --rm --name aubo_interface aubo_ros1 bash
````
Then, launch the aubo ros control:
````
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch
````
5. aubo_ros2_moveit2 (Terminal 5) 

launch the aubo ros2 move_group. Then, you can move aubo to pre-defined poses, such as 'home', 'zero'. Also, you can use MoveGroupInterface to control the robot too.
````
source ~/Moveit2-Ros1-Control-AUBO/aubo_ros2_moveit2_ws/install/setup.bash
ros2 launch run_move_group auboi5_moveit.launch.py
````
6. perception pipeline for dynamic collision avoidance (Optional)  

After you launch ros2 move_group, there will be a ros2 topic called '/test_point_cloud'. This is a topic to receive the sensor_msgs::msg::PointCloud2 and transform PointCloud2 to Octomap into moveit_planning_scene for collision detection. Thus, you can use ros2 node to send PointCloud2 to this topic to achieve dynamic collision avoidance.  

If you want to clear Octomap generated, you can use ros2 service call like below to do so:
````
ros2 service call /clear_octomap std_srvs/srv/Empty
````
### Simulation robot
If you want to do simple simulation with fake robot and test the feasibility of MoveGroupInterface, follow the step 1 to 2 below.
1. aubo_ros2_moveit2 (Terminal 1)   

Revise the last row in [auboi5_moveit.launch.py](aubo_ros2_moveit2_ws/src/run_move_group/launch/auboi5_moveit.launch.py) from "return LaunchDescription([static_tf, move_group_node, rviz_node, robot_state_publisher])" to "return LaunchDescription([static_tf, move_group_node, rviz_node, robot_state_publisher, fake_joint_driver_node])". Save this python script and type in following cmd.
````
cd ~/Moveit2-Ros1-Control-AUBO/aubo_ros2_moveit2_ws/
source ~/Moveit2-Ros1-Control-AUBO/aubo_ros2_moveit2_ws/install/setup.bash
colcon build
ros2 launch run_move_group auboi5_moveit.launch.py
```` 
2. move_group_interface_example (Terminal 2)

After doing this launch, you can press 'enter' to make aubo moving in Rviz2, and you will have a better understanding about moveGroupInterface.
````
source ~/Moveit2-Ros1-Control-AUBO/aubo_ros2_moveit2_ws/install/setup.bash
ros2 launch run_move_group run_move_group_interface.launch.py
````
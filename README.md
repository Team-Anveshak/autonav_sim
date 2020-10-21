# autonav_sim

prereqiuisites

1 - catkin_make catkin_ws and simulation_ws

2 - install gazebo_ros

3 - install rviz


getting simulation running

1 - source all terminals

2 - run launch files spawn.launch(gazebo) and rviz.launch(rviz)

3 - run "rosrun gazebo_ros gazebo"

4 - now you'll have your gazebo and rviz running


running your algo on bot

1 - catkin_ws has motion_plan package and your code goes here

2 - see existing codes for topics to publish to.

Additional installations for gazebo joint control:

sudo apt-get install ros-melodic-joint-state-controller : This will install joint_state_controller package(Needed)

sudo apt install ros-melodic-velocity-controllers: This will install velocity controllers(Needed)

sudo apt-get install ros-melodic-effort-controllers : This will install Effort controller

sudo apt-get install ros-melodic-position-controllers : This will install position controllers

Sample command in terminal for camera movement:

rostopic pub -1 /i214/camera_joint_position_controller/command std_msgs/Float64 "data: 2.0"



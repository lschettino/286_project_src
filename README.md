# Robotics - Roscore - CS286 Final Project (SLAM with AOA)

## Hardware Experimentation

### Hardware Setup

#### PC Setup (Follow Mini Hack 3 Instructions)[https://weiyingwang.notion.site/CS286-Mini-hack-3-Mapping-and-Sensor-Fusion-dc8bd80d4e6c42ba976539a5d1c00175]

1) Connect lab workstation to internal wifi network
2) Check config
3) Update .bashrc
3.1) (Editted from MH3)
	- source devel/setup.bash 
	- export TURTLEBOT3_MODEL=waffle
 
4) Run `source ~/.bashrc`
5) Run `roscore`





#### Robot Setup (Follow Mini Hack 3 Instructions)
1) Check connection with `ping`
2) Ssh into robot `ssh hostname@ipaddress_of_robot`
3) Update variables `ROS_MASTER_URI` and `ROS_HOSTNAME` in `.bashrc` file
	- Update line `export ROS_MASTER_URI=http://yourmasterup:11311` where `yourmasterup` is the ip address of where you are running `roscore`
	- Update line `export ROS_HOSTNAME=ipaddress_of_robot`
4) Run `source ~/.bashrc`


#### Robot Setup (Follow Mini Hack 3 Instructions)



### Software setup








## Gazebo simulation setup (Follow Weijings email)

### First Time (On new computer)
1) Create personal catkin workspace
2) Unzip Weijing's folder and put it in a catkin workspace

### Subsequent times 


### Simulation settings
- Edit starting position of robot
	-Edit `~/catkin_ws_teamALK/src/turtlebot3_simulations/turtlebot3_gazebo/launch/turtlebot3_world.launch`
- Launch gazebo: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`
	- Check available lauch worlds : `cd ~/catkin_ws_teamALK/src/turtlebot3_simulations/turtlebot3_gazebo/launch/`
- Launch Rtab: `roslaunch rtabmap_ros demo_turtlebot3_navigation.launch `
- Run python code: ` python robot_pointop_nav_clean_copy.py --goal_topic /move_base --position_topic /odom --bot 2`
- Post to channel: `rostopic pub /aoa_topic std_msgs/Float32 "data:180.0"`
- Check topic: `rostopi echo /odom`



CAVEAT: on all new terminals,


source devel/setup.bash











## Random tricks

- Backtrack search of commands you have used before `Ctrl + R` 





lschettino

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

simulation: 

open new terminal (every time you open new terminal) -> go to root of catking ws: 
1. source devel/setup.bash 
2. export TURTLEBOT3_MODEL=waffle 

then... 

move into src folder in ws: launch gazebo 
1. roslaunch turtlebot3_gazebo turtlebot3_world.launch

new terminal in src folder (remember to do step above setup.bash...): launch rtab
1. roslaunch rtabmap_ros demo_turtlebot3_navigation.launch 

then... run python code

 python robot_pointop_nav_clean_copy.py --goal_topic /move_base --position_topic /odom --bot 2









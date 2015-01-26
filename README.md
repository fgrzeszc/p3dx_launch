p3dx_launch
===========

ROS package for P3DX robots used on Wroclaw University of Technology
equipped with Hokuyo laser ranger
## Instalation
Clone the repository to the src subdirectory of your catkin workspace:
[http://wiki.ros.org/catkin/Tutorials/create_a_workspace]
and build it with catkin_make:
<pre>
lab1_5@P3-DX-4322:~$ catkin_make
</pre>
To ensure that new package can be found by ROS source the setup.bash file from the devel subdirectory of your catkin workspace.

To install all ROS dependencies for the p3dx_launch package proceed with:
<pre>
rosdep install p3dx_launch
</pre>

## Running 

### ROS network configuration
Assuming that the visualisation is run on other computer than the one on P3DX it is necessary to 
configure the ROS enviroment properly.
Commands on the robot can be executed directly or via SSH.

1. On the robot:
<pre>
lab1_5@P3-DX-4322:~$ export ROS_IP=adres_ip_robota
lab1_5@P3-DX-4322:~$ export ROS_MASTER_URI=http://adres_ip_robota:11311
</pre>
2. On the external computer:
<pre>
student@lab15:~$ export ROS_IP=adres_ip_komputera
student@lab15:~$ export ROS_MASTER_URI=http://adres_ip_robota:11311
</pre>

### Initialisation
First, the ROSARIA software and a hokuyo ranger driver have to be run:
<pre>
lab1_5@P3-DX-4322:~$ roslaunch p3dx_launch p3dx.launch
</pre>
If the network configuration is correct the same topics should be available on both machines:
<pre>
rostopic list
</pre>
<pre>
/diagnostics
/hokuyo_node/parameter_descriptions
/hokuyo_node/parameter_updates
/rosaria/battery_recharge_state
/rosaria/battery_state_of_charge
/rosaria/battery_voltage
/rosaria/bumper_state
/rosaria/cmd_vel
/rosaria/motors_state
/rosaria/parameter_descriptions
/rosaria/parameter_updates
/rosaria/pose
/rosaria/sonar
/rosout
/rosout_agg
/scan
/tf
</pre>


### Driving the robot with a pad controller
The command should be run on the computer with a controller connected:
<pre>
roslaunch p3dx.launch teleop_joy.launch
</pre>

### Mapping
To make a static map of an robot's enviroment run:
<pre>
lab1_5@P3-DX-4322:~$ roslaunch p3dx_launch mapping.launch
</pre>
To see the results of mapping open RViz:
<pre>
student@lab15:~$ rosrun rviz rviz
</pre>
and select map.rviz configuration file in File/Open Config dialog. 

Drive the robot around with a pad controller and when the results will be satisfactory
save a map with:
<pre>
lab1_5@P3-DX-4322:~$ rosrun map_server map_saver
</pre>

### Autonymous navigation
On the robot run configured ROS navigation stack with:
<pre>
lab1_5@P3-DX-4322:~$ roslaunch p3dx_launch navigation.launch
</pre>
Open RViz with nav.rviz config file.
Now you can set an initial pose of the robot and send navigation goals to the robot with RViz.
More detailed instruction on using RViz for navigation with ROS is available on:
[http://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack]

## Adjusting configuration
Configutation of the ROS navigation stack is placed in the following files:
base_local_planner_params.yaml	
costmap_common_params.yaml
global_costmap_params.yaml 
local_costmap_params.yaml
Most of parameters from these files can be dynamically adjusted with rqt_reconfigure tool:
<pre>
student@lab15:~$ rosrun rqt_reconfigure rqt_reconfigure
</pre>



In case of using other pad controller than Logitech Gamepad F710 it is likely that adjusting parameters teleop_joy.launch is necessary.



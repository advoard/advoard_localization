# ROS IEU AGV Localization
In this project we aimed to develop a system that works in ROS environment and can localize itself. These packages tested under turtlebot3, but it can also be used on any other system that has an Ultra-wideband ranging sensor and odometry sensors mounted. It is also possible to initialize the robot and send the initial pose estimation to navigation stack without the need of manually initializing it on the RViz window but keep in mind that in order to the use this feature LiDAR is required. AMCL also needs an initial pose so this feature is very crucial to have a fully autonomous system. We offer a simulation package with synthetic UWB data for now, but it can be adjusted so that it will be possible to the use it on real life robot with real UWB sensors.


![](https://raw.githubusercontent.com/advoard/ros_advoard_localization/master/docs/map_matcher.gif)


## Setup 
First you will need to utilize at least 4 UWB sensors one is mounted on the robot and rest should be placed in a way that it will cover the area of interest where you want to localize your robot. Simulation is possible with the shared scripts. However, this package doesn’t support direct use so slight adjustments on the pozyx scripts are required. So install the package ros_pozyx_simulation and run it before

- [pozyx uwb similation](https://github.com/advoard/ros_pozyx_simulation)</br>
- [pozyx uwb real (coming soon)]()


rosrun pozyx_simulation uwb_simulation.py
rostopic echo /uwb_data_topic 

After you confirm that uwb_data_topic is available in ROS you can use this project </br>


## How to Run Localization 
This project includes 2 different localization algorithms and they use different sensory information. </br>
- Kalman Filter = ultra wide band + odometry 
- Source Localization = ultra wide band

### Kalman Filter  

rosrun advoard_localization kalman_filter_localization.py


### Source Localization 

rosrun advoard_localization sqrrange_leastsqr_localization.py 


## How to Run Initial Pose Estimation 
There are 2 different initial pose estimation algorithms in this repository. First one uses LiDAR measurement, preconstructed map and UWB range measurements. Second one only uses UWB sensor and the vehicle must move 20cm forward to figure out the direction it is facing.

### (Lidar + Map + UWB Sensor) Initialization 
This algorithm uses uwb sensors and odometry so localization_data_topic and odom topics must to be up and running.</br>

roslaunch advoard_localization lidar_uwb_initial_pose.launch 


### (UWB Sensor) Initialization
This algorithm uses only uwb sensors so localization_data_topic topic must to be up and running.

roslaunch advoard_localization uwb_initial_pose.launch 




## How to Use Localization Algorithm
Kalman filter and source localization use same topic thus you must select the one that you want to run because only one of them can run. The topic that mentioned is Pose. In python you can use this topic with writing a subscriber like below. 

from geometry_msgs.msg import Pose

rospy.Subscriber("localization_data_topic", Pose, subscribe_data)

def subscribe_data(self,Pose):
  robot_realtime_pose= Pose

![](https://raw.githubusercontent.com/advoard/ros_advoard_localization/master/docs/localization_gui.png)

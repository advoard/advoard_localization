# ROS IEU AGV Localization
This project localization system can use for different commercial robots for example turtlebot 3 burger, waffle or turtlebot 2. This localization system different from the others is uwb sensor. Also localization algorithm use lidar, uwb  and odom sensors same time altogether. Furthermore <b>amcl</b> algorithm need the initial pose estimation. In this project this estimation done automatically. 

![](https://raw.githubusercontent.com/ieuagv/ros_ieuagv_localization/master/docs/map_matcher.gif)


## Setup 
Firstly your robot must be supported with uwb sensors to use this project. You can try with this similation with this project  After the install package ros_pozyx_simulation (in simulation) or ros_ieuagv_pozyx (in real robot).</br>

- [pozyx uwb similation](https://github.com/ieuagv/ros_pozyx_simulation)</br>
- [pozyx uwb real (coming soon)]()

```
rosrun pozyx_simulation uwb_simulation.py
rostopic echo /uwb_data_topic 
```
After the get uwb_data_topic you can use this project </br>


## How to Run Localization 
This project include 2 different localization algorithm. </br>
- Kalman Filter = ultra wide band + odom 
- Square Range Least Square = ultra wide band

### Kalman Filter  
```
rosrun ieuagv_localization kalman_filter_localization.py
```

### Square Range Least Square 
```
rosrun ieuagv_localization sqrrange_leastsqr_localization.py 
```

## How to Run Initial Pose Estimation 
There are 2 different initial pose estimation algorithm. First one use lidar, map and uwb sensors and second one is use only uwb sensor however for calculate to direction of robot, it must move 20 cm.

### (Lidar + Map + UWB Sensor) Initializ 
This algorithm use uwb sensors and odom so that **localization_data_topic** and **odom** topics must to be up and running.</br>
```
roslaunch ieuagv_localization lidar_uwb_initial_pose.launch 
```

### (UWB Sensor) Initializ 
This algorithm use only uwb sensors so that **localization_data_topic** topic must to be up and running.
```
roslaunch ieuagv_localization uwb_initial_pose.launch 
```

## How to Use Localization Algorithm
Kalman filter and square range least square localization nodes are use same topic thus you have to select one of them. This topic type is Pose. In python you can get with a subscriber. 
```
from geometry_msgs.msg import Pose

rospy.Subscriber("localization_data_topic", Pose, subscribe_data)

def subscribe_data(self,Pose):
  robot_realtime_pose= Pose
```
![](https://raw.githubusercontent.com/ieuagv/ros_ieuagv_localization/master/docs/localization_gui.png)


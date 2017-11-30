#Simple readme :
That is just a plotter ! The simulation is handle by SITL.
ROS subscribes to mavros and publish pose data to gazebo model for visualization.
NOT A SIMULATOR, JUST A VISUALIZATION

##Installation:
Install Gazebo, ROS and Mavros (and follow ROS tutorial to setup everything correctly)  

Install gazebo_ardu:  
````
mkdir -p catkin_ws/src
cd catkin_ws/src 
git clone https://github.com/khancyr/gazebo_ardu 
cd .. 
catkin build 
source ./devel/setup.bash
roslaunch gazebo_ardu gazebo_ardu.py
````

ENJOY

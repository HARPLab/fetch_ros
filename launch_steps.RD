############
Set it all up with standard catkin_make
sudo apt-get install ros-melodic-<missing package> for almost all dependencies

##################
## Installing current libs
catkin_make in the ~/catkin_ws directory

Before each command, devel/setup.bash so it knows about all the ROS objects and where they live

###########
roscore
# start the node that runs everything


roslaunch fetch_gazebo simulation.launch
rosrun rviz rviz
roslaunch fetch_gazebo playground.launch


####### TODO WRITE UP
mapserver running to give map
tf2 static frame for just looking at map




##########


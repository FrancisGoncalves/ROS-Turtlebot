Setup description: I am running Ubuntu 20.04 in a dedicated machine, ROS noetic,
and using python 3.8.10 for creating nodes, topics, and services.

First Part (Mapping): 1. Run 'roslaunch turtlebot3_gazebo turtlebot3_house.launch'
                      2. Run the custom launch file 'roslaunch turtlebot3_slam turtlebot3_slam_withnav'
                      3. Run 'python3 mapping.py'

		       Note: Sometimes (not many) the robot got stuck and needed to be moved to continue the mapping

Second Part (Navigation): 1. Run 'roslaunch turtlebot3_gazebo turtlebot3_house.launch'
                          2. Run 'roslaunch turtlebot3_navigation turtlebot3_navigation map_file:=<path_to_file>/map_francisco.yaml'
                          3. Run 'python3 conv_and_nav.py'
                          
                          Note: You also need to change 'map_francisco.yaml' to call for the .mpg file in the correct directory 

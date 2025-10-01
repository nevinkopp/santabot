# SantaBot (WIP)
SantaBot is an automated guided vehicle (AGV) that delivers presents during the holiday season! 

# Installation
To install the santabot package, simply clone it to your ROS 2 workspace (such as "/home/$USER/ros2_ws/src"). Then run "colcon build --packages-select santabot" from within the root of your workspace. Now open a new terminal, source your ROS 2 installation (such as "source /opt/ros/jazzy/setup.bash") and run "source install/local_setup.bash" from inside the ros2_ws directory. Now you should be able to run executables from within the santabot package.

# Test Drive 
I've included a test drive node you can use to see if everything is communicating properly. Run with "ros2 run santabot testdrive" and then publish a twist message such as "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"". 
The robot should move forward at 20 cm/s. 


# SantaBot (WIP)
SantaBot is an automated guided vehicle (AGV) that delivers presents during the holiday season! 

# Hardware
SantaBot is built on an iRobot Roomba 600 series base and takes advantage of the open serial interface released by iRobot to create custom robotics applications. In addition to the built in sensors, I plan on using a webcam for symbol recognition and indoor navigation. I'm also using a Raspberry Pi 5 for the brain of the robot.

# Software
SantaBot is completely built on ROS2. I plan on using Nav2 for navigation and OpenCV for object recognition. 

# Installation
To install the santabot package, simply clone it to your ROS 2 workspace (such as "/home/$USER/ros2_ws/src"). Then run "colcon build --packages-select santabot" from within the root of your workspace. Now open a new terminal, source your ROS 2 installation (such as "source /opt/ros/jazzy/setup.bash") and run "source install/local_setup.bash" from inside the ros2_ws directory. Now you should be able to run executables from within the santabot package.

# Test Drive 
I've included a test drive node you can use to see if everything is communicating properly. Run with "ros2 run santabot testdrive" and then publish a twist message such as "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"". 
If everything is configured properly, the robot should move forward at 20 cm/s. 

# TODO
Build driver on open serial interface (done)
Create camera node (in progress)

# SantaBot
SantaBot is an automated guided vehicle (AGV) that delivers presents during the holiday season! WIP

# Test Drive 
Use the testdrive node to test the setup. Run with "ros2 run santabot testdrive" and then publish a twist message such as: ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}". 
The robot should move forward at 20 cm/s. 



# Rover Driver

This repo contains the set of nodes needed to get a rover to start running. 

I am using a Nvidia TX2 as my ros-master. It is connected to a roboclaw brushed motor controller, to control the motor speeds. 

The encoder that is on the motors is only used for the lowest level PID (internal to the roboclaw) which regulates the motor speeds to the commanded speed. 

The roboclaw is driven by messages on `/cmd_vel`, which is of type `geometry_msgs/Twist`. It extracts the `msg.linear.x` and `msg.angular.z` to determine the linear and angular commanded velocites in the robot's body frame. 

A `joystick_safety` node can also be launched. This subscribes to two topics: `/des_vel` (of type `geometry_msgs/Twist`) and `joy` of `sensor_msgs/joy`. It will use the joysticks state to publish messages to the `cmd_vel` topic. If the kill switch is active, it always publises zero speeds. If the arm switch is armed, based on whether the autonomous mode, or joy-stick mode is selected, the msgs could be from either `/des_vel` or computed from the joystick and max range. 



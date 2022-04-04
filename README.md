
# Rover Driver

This repo contains the set of nodes needed to get a rover to start running. 

I am using a Nvidia TX2 as my ros-master. It is connected to a roboclaw brushed motor controller, to control the motor speeds. 

The encoder that is on the motors is only used for the lowest level PID (internal to the roboclaw) which regulates the motor speeds to the commanded speed. 

The roboclaw is driven by messages on `/cmd_vel`, which is of type `geometry_msgs/Twist`. It extracts the `msg.linear.x` and `msg.angular.z` to determine the linear and angular commanded velocites in the robot's body frame. 

A `joystick_safety` node can also be launched. This subscribes to two topics: `/des_vel` (of type `geometry_msgs/Twist`) and `joy` of `sensor_msgs/joy`. It will use the joysticks state to publish messages to the `cmd_vel` topic. If the kill switch is active, it always publises zero speeds. If the arm switch is armed, based on whether the autonomous mode, or joy-stick mode is selected, the msgs could be from either `/des_vel` or computed from the joystick and max range. 


## Jetson TX2 Ubuntu setup
Here's a good picture guide for the installation process. Steps repeated here for convenience. The official Linux support on Jetson TX2 is only for Ubuntu 18.04.

1. Download the latest version of Nvidia’s SDK Manager on a PC running Ubuntu 18.04. I tried running this SDK on Ubuntu 20.04 laptop but it could not detect linux to be installed on Jetson TX2. THE SDK version used at time of writing this guide is 1.7.3.9053
  https://developer.nvidia.com/nvidia-sdk-manager
3. Connect the TX2 to your PC using the provided microUSB cable.
4. Make sure the TX2 is powered off
5. Connect a monitor, mouse, and keyboard to the Jetson. (The mouse is optional, but recommended. If you do not have an all-in-one mouse+keyboard you will need to use a small USB hub, as the Jetson TX2 only has a single USB port.). 
6. Press and hold the REC button
7. Press the power button. OR in our case just connect LiPo battery to the roverclaw board which powers the Jetson too. The steps are same if powered with barrel jack.
8. Note: if SDK does not detect Jetson, then hit refresh. and connect to its micr USB without a hub and directly to your laptop's USB A(and not USB C).

Then follow the steps in SDK. make sure to Choose the appropriate board and Linux version in Step 1. In Step 3, I used the Force Recovery method andf followed the instructions. Instead of power button, the Jetson just starts up automatically when Lipo power is passed on through the barrel jack. Therefore, whenever it says press power button, then just connect LiPo instead.

## Jetson TX2 Internet Setup
After Ubuntu installation, simnply connect to Wifi and in its settings GUI, give permission to all users to connect to Wifi and permission to connect automatically.

## Jetson TX2 ROS installation
Follow the offical ROS installation guide http://wiki.ros.org/melodic/Installation/Ubuntu.

ROS GPG keys were changed 1.5 years ago and they released an offical notidication. However the offical installation guide never updated the instruction. So, when trying to install ROS, it shows GPG key error during `sudo apt update`, then follow the instructions at https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454.

## Required ROS packages
After ROS installation, clone and build this repo

In order to enable ROS to communicate between Laptop and Jetson, you need to set ROS IP and Hostname. Open ~/.bashrc and add following lines
```
export ROS_MASTER_URI=http://hardik-legion-s7.local:11311/ (laptop's name)
export ROS_HOSTNAME=rover3.local (jetson's name)
```

Similarly on the laptop(assuming this runs tghe master in ROS), you need to add
```
export ROS_MASTER_URI=http://hardik-legion-s7.local:11311 (laptop)
export ROS_HOSTNAME=hardik-legion-s7.local (laptop)
```

You also need to change hostname in following two places otherwise the following error shows up: "Hostname not found. possible reason might be that the Machine is unable to communicate with itself"
1. In file `etc/hostname`, change to rover3
2. In file `\etc/hosts`, change the entry 127.0.1.1 xxxx to 127.0.1.1 rover3

## Additional Info
For more info and joy stick guide, visit
https://dev10110.github.io/tech-notes/research/setting-up-rovers.html

## USB Communication Setup
For seamless communication with roverclaw, we need to give it permission to use the usb port. This can be done with
```
sudo chmod 666 /dev/ttyACM0
```

OR you can also do the following to avoid giving this permission everytime
1. Run following two commands (required once and u should log out and log back in)
 ```
sudo adduser $USER dialout
sudo adduser $USER tty
```

2. Remove modemmanager as it interferes with USB permission a lot
```
sudo apt-get remove modemmanager
```

# Custom gripper for youbot with ROS control 

Language: [English](https://github.com/Alex-T-RU-DE/Custom-gripper-for-youbot-with-ROS-control/blob/main/README.md), [Русский](https://github.com/Alex-T-RU-DE/Custom-gripper-for-youbot-with-ROS-control/blob/main/README.ru.md) 

This project shows and explains how to make and implement a new gripper with ROS connection for the arm of the Kuka Youbot and how to implement gripper's Gazebo simulation. 

## Required parts

-   Two servos 
-   Printed with 3D printer and assembled gripper from the following [link](https://www.thingiverse.com/thing:4764063)
-   Arduino mega/[pro](https://www.amazon.de/ARCELI-Arduino-Mega-ATmega2560-CH340G-Elektronik/dp/B07MQ1J9MR/ref=sr_1_13?dchild=1&keywords=arduino+pro&qid=1613692717&sr=8-13) 
-   Wires
-   [DC-DC transformer](https://www.amazon.de/LAOMAO-Wandler-einstellbar-Spannungswandler-Converter/dp/B00HV4EPG8/ref=asc_df_B00HV4EPG8/?tag=googshopde-21&linkCode=df0&hvadid=231941675984&hvpos=&hvnetw=g&hvrand=3852759402861473550&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9068552&hvtargid=pla-420005320986&psc=1&th=1&psc=1)


## Software:

Before starting, make sure that you installed Arduino IDE.

1. Copy folder "youbot_gripper" into your catkin_ws/src directory.

2. Navigate yourself into catking_ws download and compile folder "Youbot gripper" with catkin_make.

3. installing ROSserial:
 ```
sudo apt-get install ros-<your_ros_version>-rosserial-arduino
sudo apt-get install ros-<your_ros_version>-rosserial
 ```
4. Open <your Arduino directory>/Arduino/library in a terminal and run the following command:
```
rosrun rosserial_arduino make_libraries.py .
```
It will automatically prepare and copy all your custom messages, services, and packages (including youbot gripper) in the library of Arduino.

5. Download file "Gripper_service_server.ino" and put it into Arduino directory.

6. Firstly, try to compile it. If it works - upload the program to your Arduino board and go to step 7. If not and you have the following error:

```
#include <cstring>
    ^~~~~~~~~
compilation terminated. 
 ```
you should change the "**msg.h**" in `~/Arduino/libraries/ros_lib/ros` destination generated by ```rosrun rosserial_arduino make_libraries.py .``` with "**msg.h**" file from "*rosserial lib*" (0.7.8 version) which could be downloaded from the Library manager of Arduino IDE (directory of this file in downloaded library: `~/Rosserial_Arduino_Library/src/ros/`).  

   
7. To start your gipper, you should start following command on your device, which is connected to the gripper:
```
rosrun rosserial_python serial_node.py /dev/tty_YOUR_PORT_FOR_ARDUINO 
```
example:  
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
This command will start the server on your Arduino device, and it will share the service for gripping. You could play with this service by starting a simple client with the following command:
```
rosrun youbot_gripper youbot_gripper_client NUMBER_OF_INSTRUCTIONS 
```
Where ```NUMBER_OF_INSTRUCTIONS``` should be replaced according to numbers, which listed below
        
                                 
## Instructions for the service

To use different commands for this gripper, you could use the following numbers to achieve the desired result:

```0-100``` opening %

```1000-1340``` set the angle of the first servo to the position in range 0-380

```2000-2340``` set the angle of the second servo to the position in range 0-380

## Feedback

Program sends back a feedback (bool) after it got the command. 
The positive feedback will be when:
- the object was grasped by gripper (checking servo's positions and loads) 
- the command for left for the left finger has been successfully sent
- the command for left for the right finger has been successfully sent

### Simulation:
 coming soon
 
### Integration with the Youbot arm in Gazebo:

Working demo: https://youtu.be/vPzOUe2N2ss

The simulations uses grippers URDF files, which were generated by [SolidWorks to URDF Exporter](http://wiki.ros.org/sw_urdf_exporter) and adjusted with a few additional files (like .transmission, .gazebo, control.yaml and etc).

If you already have youbot package, for the implementation of this gripper with its functionality to the Kuka youbot Gazebo simulation, you have to do the following actions:

1. Copy File `gripper_controller.launch` and put it into your `~catkin_ws/scr/youbot_simulation/youbot_gazebo_control/launch` directory
2. Copy files `gripper_controller.yaml` and `gripper_controller_1.yaml` into your `~catkin_ws/scr/youbot_simulation/youbot_gazebo_control/config` directory
3. Replace your `~catkin_ws/scr/youbot_description/urdf/youbot_gripper` and `~catkin_ws/scr/youbot_description/meshes/youbot_gripper` folders with the same folders from the same directories from this repository.

Or you can download `youbot_descriptions` and `youbot_simulation` folders into your `catkin_ws`.

after you did these changes, you should start the simulation with the following command:

```
roslaunch youbot_gazebo_robot youbot.launch
``` 

Sometimes, simulation requires 2 or 3 restarts to spawn robot and controllers correctly.
If your simulation works well, you can check the fingers of the gripper with:

```
rostopic pub -1 /gripper/gripper_controller_1/command std_msgs/Float64 "data: -1.5" 
rostopic pub -1 /gripper/gripper_controller/command std_msgs/Float64 "data: 1.5"
``` 

### Electrical parts:

# How to connect to and control UR robots and Robotiq grippers using Python 2 or Python 3
## Brief summary

This repository allows for a quick setup with plug-and-play functionality when using Python to control UR robot arms and Robotiq grippers.

The necessary setup to begin controlling this equipment:
1. Install Python package **numpy**.
2. *OPTIONAL* Install Python package **pyserial** to use the Robotiq gripper from a Windows OS.
3. Connect the UR robot with an Ethernet cable and connect the Robotiq gripper with a USB cable.
4. Input the connection information IP/PORT for the UR robot and USB PORT for the Robotiq gripper in the respective **config_robot.py** and **config_gripper.py** files.
5. Import **UR** class from **src/ur/class_ur.py** and **Gripper** class from **src/gripper/class_gripper.py**.
6. *IMPORTANT* Remember to use the **shutdown** functions in each equipment's class to close the connections when the scripts have finished using the equipment.

## UR robots
The **UR** class contains all the functionality needed to control the robot arm. Create a UR object of the class and use this object to communicate with the robot.

With this object it is possible to read the data retrieved from the robot server and send commands to the robot. Multiple object functions have been implemented to ease the command communication; these include the following (but are not limited to):
1. **move**: moves the robot to a point defined by (x, y, z, rx, ry, rz).
2. **wait**: waits for the robot arm to finish moving.
3. **get_position**: gets the robot's current position in either robot base coordinates or in task frame coordinates.
4. **shutdown**: closes the connection to the robot.

### Setting up a task frame
Functionality has been implemented to work in a task frame instead of working in robot base coordinates. It is necessary to find the transformation for the desired task frame before this functionality can be utilized. To setup a task frame:
1. Find the transformation parameters by placing the robot tool point at the following three points in the task frame and read the robot base coordinates (x, y, z) for each point.
    1. **p0i**, task frame origin.
    2. **pxi**, at the point 10cm along the x-axis.
    3. **pyi**, at the point 10cm along the y-axis.
2. Either input these three points in the **TRANSFORM** dictionary in the **config_robot.py** configuration file or manually conduct the initialization of the transformation using the **UR** object function **transform_init** and inputting these three points as arguments.

### Reading robot data
In the UR configuration file, **config_ur.py**, the data to be read from the robot server is given by the dictionary **DATA_MAP**. This chosen data from this map is non-exhaustive, and depending on the robot server's software version more data can be managed. The source for reading and understanding the read data is from UR's own site: https://www.universal-robots.com/articles/ur/remote-control-via-tcpip/. A downloadable Excel sheet contains the necessary information about which bytes contain which robot information.

Additionally, this sheet can be found in the **resources** folder of this repository.


## Robotiq grippers
The **Gripper** class contains all the functionality needed to control the gripper. Create a gripper object of the class and use this object to communicate with the gripper.

With this object it is possible to read data from and send commands to the gripper. Multiple functions have been implemented to ease the command communication; these include the following (but are not limited to):
1. **set**: sets the gripper fingers at a specific position with a specified speed and force.
2. **open** and **close**: utilizes the **set** function to either completely open or completely close the gripper.
3. **wait**: waits for the gripper fingers to finish moving.
4. **get_position**: gets the gripper's current fingers' position.
5. **shutdown**: closes the connection to the gripper.

### Controlling the gripper fingers
The gripper takes three inputs: *position*, *speed* and *force* to determine how to move the gripper fingers. Each of these values are between 0 and 255.

### Connection on Linux OS
When using a Linux OS it is not required to install the Python package **pyserial**. However, there is no automated way of determining the gripper's USB port, and as such the file path to the USB port has to be manually set in the gripper configuration file **config_gripper.py**.

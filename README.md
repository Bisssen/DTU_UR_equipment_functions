# How to connect to and control UR robots and Robotiq grippers using Python 2 or Python 3
## Brief summary

This repository allows for a quick setup with plug-and-play functionality when using Python to control UR robot arms and Robotiq grippers.

The necessary setup to begin controlling this equipment:
1. Connect the UR robot with an Ethernet cable and connect the Robotiq gripper with a USB cable.
2. Input the connection information IP/PORT for the UR robot and USB PORT for the Robotiq gripper in the respective **config_robot.py** and **config_gripper.py** files.
3. *OPTIONAL* Install Python package **pyserial** to use the Robotiq gripper from a Windows OS.
4. Import **UR** class from **src/ur/class_ur.py** and **Gripper** class from **src/gripper/class_gripper**.

## UR robots
The **UR** class contains all the functionality needed to control the robot arm. Create a robot object of the class and use this object to communicate with the robot.

With this object it is possible to read the data retrieved from the robot server and send commands to the robot. Multiple object functions have been implemented to ease the command communication; these include the following (but are not limited to):
1. **move**: move the robot to a point defined by (x, y, z, rx, ry, rz).
2. **wait**: wait for the robot arm to finish moving.
3. **get_position**: get the robot's current position in either robot base coordinates or in task frame coordinates.

### Setting up a task frame
Functionality has been implemented to work in a task frame instead of working in robot base coordinates. It is necessary to find the transformation for the desired task frame before this functionality can be utilized. To setup a task frame:
1. ***Find the transformation***: place the robot tool point at the following three points in the task frame and read the robot base coordinates (x, y, z) for each point.
    1. Task frame origin, **p0i**.
    2. At the point 10cm along the x-axis, **pxi**.
    3. At the point 10cm along the y-axis, **pyi**.
2. Either input these three points in the **TRANSFORM** dictionary in the **config_robot.py** configuration file or manually conduct the initialization of the transformation using the **UR** object function **transform_init** and inputting these three points.

### Reading robot data
In the robot configuration file, **config_robot.py**, the data to be read from the robot server is given by the dictionary **DATA_MAP**. This chosen data from this map is non-exhaustive, and depending on the robot server's software version more data can be managed. The source for reading and understanding the read data is from UR's own site: https://www.universal-robots.com/articles/ur/remote-control-via-tcpip/. A downloadable Excel sheet contains the necessary information about which bytes contain which robot information.

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

*IMPORTANT* On earlier versions ( < 3.2 ) of the Client Interface software on the UR robots, the wait function uses the speed of the joints to determine when it has finished a command, and as such is prone to error at low speeds. The **VELOCITY_MEAN_THRESHOLD** threshold variable can be adjusted in the **src/ur/config_ur.py** file to combat this issue.

## ROS2 interface
This code cannot be used in ROS2 as it is. It is meant to be part of a ROS2 package, and to use the UR interface with ROS2, a package must be created that correctly points to this code using its **setup.py**. Addind the following lines, to **setup.py** will achieve this:

```python
packages=find_packages(where='.'),  # Look in current directory
package_dir={'': '.'},  # Root is current directory
entry_points={
    'console_scripts': [
        'ur_node = ur_code.ur_ros2_node:main',
        'action_send_test = ur_code.action_sender_test:main',
        'action_recv_test = ur_code.action_recv_test:main',
    ],
```

### Collision safety
An opt-in safety feature that stops the robot on collision detection, blocks all movement commands, and resumes normal operation on a recovery signal.

Enable it by setting the `collision_safety_enabled` parameter to `True`:
```bash
ros2 run ur_socket_connection ur_node --ros-args -p collision_safety_enabled:=True
```

**Parameters:**
| Parameter | Default | Description |
|---|---|---|
| `collision_safety_enabled` | `False` | Master enable/disable switch |
| `collision_topic` | `/collision_monitor/collision` | Subscribed topic (`std_msgs/Bool`) that triggers a collision stop |
| `recovery_topic` | `/collision_safety/recovery` | Subscribed topic (`std_msgs/Bool`) that clears the collision state |

**Published topics:**
| Topic | Type | Description |
|---|---|---|
| `/collision_safety/active` | `std_msgs/Bool` | Current collision state |
| `/collision_safety/joints` | `sensor_msgs/JointState` | Joint positions at the moment of collision |

An external collision monitor node must publish `True` on the collision topic to trigger a stop. To recover, publish `True` on the recovery topic.

### Buffered trajectory execution
When streaming joint configurations to `set_ur_joints` at high frequency (e.g. from a planner publishing one configuration per message), the robot executes each as an individual `movej` command. This causes a full accelerate/decelerate cycle per waypoint, resulting in jerky, slow motion.

Buffered trajectory execution solves this by collecting incoming joint configurations into a buffer and periodically sending them as a single batched `path()` call with blending radius between intermediate waypoints. Within each batch the robot moves smoothly without stopping at each waypoint.

**How it works:**
1. The `set_ur_joints` callback appends each joint configuration to a thread-safe buffer instead of sending it immediately.
2. A timer (default 10Hz) periodically checks if the previous batch has finished executing.
3. Once the robot reaches the last waypoint of the previous batch, the timer drains the buffer and sends all accumulated waypoints as a single URScript `def` block with `movej` commands and blending radius `r`.
4. Waypoints keep accumulating in the buffer while the robot executes, so the next batch is ready as soon as the current one finishes.

Enable it by setting `buffered_trajectory_enabled` to `True`:
```bash
ros2 run ur_socket_connection ur_node --ros-args -p buffered_trajectory_enabled:=True
```

**Parameters:**
| Parameter | Default | Description |
|---|---|---|
| `buffered_trajectory_enabled` | `False` | Enable/disable buffered execution |
| `buffer_flush_interval` | `0.1` | Seconds between flush attempts (0.1 = 10Hz) |
| `buffer_min_batch_size` | `2` | Minimum waypoints before flushing (blending needs at least 2) |
| `buffer_max_batch_size` | `50` | Maximum waypoints per batch |
| `buffer_max_age` | `0.2` | Force flush if oldest waypoint has waited this long (seconds) |
| `buffer_blend_radius` | `0.05` | Blending radius `r` (meters) for intermediate waypoints |

**Collision safety integration:** When a collision is detected, the buffer is immediately cleared and new waypoints are rejected until a recovery signal is received.

**Tuning tips:**
- Increase `buffer_flush_interval` to collect larger batches (smoother but higher latency).
- Increase `buffer_blend_radius` for more aggressive blending between waypoints.
- Adjust `buffer_max_batch_size` based on how many waypoints your planner produces per batch window.

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
    2. **pxi**, a point along the x-axis.
    3. **pyi**, a point along the y-axis.
2. Either input these three points in the **TRANSFORM** dictionary in the **config_ur.py** configuration file or manually conduct the initialization of the transformation using the **UR** object function **transform_init** and inputting these three points as arguments.

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
The gripper takes three inputs: *position*, *speed* and *force* to determine how to move the gripper fingers. Each of these values need to be integers between 0 and 255.

### Vacuum Gripper
The **Vacuum** class contains functionallity to activate and deactivate the sucking on a connected **Robotiq Vacuum Gripper**. Functions implemented is as following:
1. **activate**: Activates the automatic sucking feature of the vacuum gripper. The vacuum gripper will try to establish the maximum possible vacuum level over a period of 2 seconds. If the vacuum level cannot exceed 30% after the two seconds, then the gripper will deactivate.
2. **deactivate**: Deactivates the vacuum to drop the objects held.
3. **wait**: waits until the activate function have managed to create a sufficient vacuum level.  


### Connection on Linux OS
When using a Linux OS it is not required to install the Python package **pyserial**. However, there is no automated way of determining the gripper's USB port, and as such the file path to the USB port has to be manually set in the gripper configuration file **config_gripper.py**.


## Servers
Two servers exist, one for the UR class and one for the Gripper class. These servers pass information sent to them to their respective classes. The servers are completely optional, and exist to allow the framework to be used with other programming languages than python. The servers are called **ur_server.py** and **gripper_server.py**, and they will automatically start and connect to the UR and Gripper if started.

To start a server, do the following:
1. Create a **GripperServer** or **URServer** object.
2. *OPTIONAL* Give the server objects a port which they should create the server on by passing it as an input. The default port is 4444 for **URServer** and 4443 for **GripperServer**.
3. Connect to the server via socket.
4. Send commands to the server via socket. The commands that can be sent are the same as described above. A command must be sent as a string  in a specific pattern, with the command name first, its inputs separated  by ":" and the string must end with "\\n", such that it looks like *"command:input1:input2:input3\\n"*. E.g. a move command could look like: *"move:0.1:0.2:-0.1\\n"*. 
5. There is one exception to sending commands. That is if the user wants to send a send_line command to the **UR**. The user can instead directly send the line intended for the **UR** to the **URServer**, and the server will automatically pass it forward to the **UR**.



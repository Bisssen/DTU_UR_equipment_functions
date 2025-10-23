from src.ur.class_ur import UR
from math import pi
import time

# Open UR
ur = UR()

# Set TCP
ur.set_tcp(0,0,0,0,0,0)

# Go to home position and get fwdkin
ur.home(wait=True)
print(ur.get_forward_kinematics())

# Test move commands
ur.move(pose=[0.05, 0, 0, 0, 0, 0], transform=False, relative=True, wait=True)
ur.move(pose=[0.05, 0, 0, 0, 0, 0], relative=True, wait=True)
ur.move_tool(z=-0.05, ry=-pi/6, wait=True)
ur.move_tool(ry=pi/6, wait=True)
ur.move_tool(rz=pi/4, wait=True)

# Change default orientation and test move commands
ur.set_default_orientation([0, -pi, 0])
ur.move(x=0.15, y=0.12, z=0.35, wait=True)

# Test speed commands
ur.speed(x=0.2)
time.sleep(1)
ur.stop(wait=True)
ur.speed(x=-0.2, time=1, wait=True)
ur.speed_tool(x=0.2)
time.sleep(1)
ur.stop(wait=True)

# Shut down UR
ur.shutdown()

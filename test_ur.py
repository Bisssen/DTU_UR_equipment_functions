from src.ur.class_ur import UR
from math import pi
import time

# Open UR
ur = UR(None)

# Set TCP


# Go to home position and get fwdkin

# Test move commands
# ur.move(b=90, speed=0.05, mode='j', transform=False, relative=True, wait=False)
# time.sleep(0.5)
# ur.move(b=-90, speed=0.05, mode='j', transform=False, relative=True, wait=False)
# ur.move(b=0, speed=0.05, mode='j', transform=False, relative=True, wait=False)
poses = [[-1.57, 0, 0, 0, 0, 0, 'j', 0.05]]
ur.path(poses, transform=False, relative=True,
        acc=0.5, speed=0.5, wait=False, r=0.05)
time.sleep(0.5)
poses = [[1.57, 0, 0, 0, 0, 0, 'j', 0.05]]
ur.path(poses, transform=False, relative=True,
        acc=0.5, speed=0.5, wait=False, r=0.05)

# Shut down UR
ur.shutdown()

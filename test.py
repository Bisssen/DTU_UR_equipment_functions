from src.gripper.class_gripper import Gripper

import time

if __name__ == '__main__':
	gripper = Gripper()

	gripper.close(wait=True)

	gripper.open(wait=True)

	gripper.set(pos=150, speed=10, wait=True)
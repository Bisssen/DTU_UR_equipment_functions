import time
import sys

# Vacuum and gripper uses same config
from ..gripper import config_gripper
from .communication_vacuum import VacuumSerial


class Vacuum:
    def __init__(self, usb_port=None):
        # Checking python version
        self.python_2 = (sys.version_info.major == 2)

        # Getting gripper connection class
        self.vacuum_serial = VacuumSerial(usb_port)

        # The vacuum does not need to be activated

    def activate(self, wait=False):
        self.vacuum_serial.activate()
        if wait:
            self.wait()

    def deactivate(self, wait=False):
        self.vacuum_serial.deactivate()
        if wait:
            self.wait()

    def read(self):
        response = self.vacuum_serial.read()
        split = response.split(';')
        register_07D0 = split[0]
        register_07D1 = split[1]
        register_07D2 = split[2]
        return register_07D0, register_07D1, register_07D2

    '''
    # Saved for reference
    # Seems to give presure or something like that
    # No suck = 100
    # Suck ~= 50
    def get_position(self):
        _, _, register_07D2 = self.read()
        return int(register_07D2[0:2], 16)
    '''

    def get_status(self):
        register_07D0, _, _ = self.read()
        status = bin(int(register_07D0[0:2], 16))[2:].zfill(8)
        gOBJ = int(status[0:2], 2)
        gSTA = int(status[2:4], 2)
        gGTO = int(status[4], 2)
        gACT = int(status[7], 2)
        return gOBJ, gSTA, gGTO, gACT


    def wait(self):
        while True:
            gOBJ, _, _, _ = self.get_status()
            if gOBJ != 0:
                break

    def shutdown(self):
        self.gripper_serial.shutdown()
        time.sleep(0.5)

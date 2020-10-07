import time

from . import config_gripper as cfg_gripper
from .CommunicationGripper import gripperSerial

class Gripper:
    def __init__(self):
        self.gripper_serial = gripperSerial()
        
        # Activating gripper
        print('    Activating gripper...')
        success = self.activate()
        if success:
            print('        Gripper connection succesfully established.')
        else:
            print('        Activation timeout. No connection with gripper.')

    def activate(self):
        return self.gripper_serial.activate()

    def read(self):
        response = self.gripper_serial.read()
        split = response.split(';')
        reg07D0 = split[0]
        reg07D1 = split[1]
        reg07D2 = split[2]
        return reg07D0, reg07D1, reg07D2

    def getPosition(self):
        _, _, reg07D2 = self.read()
        return int(reg07D2[0:2], 16)

    def getStatus(self):
        reg07D0, _, _ = self.read()
        status = bin(int(reg07D0[0:2], 16))[2:].zfill(8)
        gOBJ = int(status[0:2], 2)
        gSTA = int(status[2:4], 2)
        gGTO = int(status[4], 2)
        gACT = int(status[7], 2)
        return gOBJ, gSTA, gGTO, gACT

    def set(self, pos, speed=255, force=255):
        if (0 <= pos <= 255) & (0 <= speed <= 255) & (0 <= force <= 255):
            self.gripper_serial.set(('{:d};{:d};{:d}'.format(int(pos), int(speed), int(force))))
            while True:
                gOBJ, _, _, _ = self.getStatus()
                if gOBJ != 0:
                    break
        else:
            if not 0 <= pos <= 255:
                print('Gripper position not allowed.')
            if not 0 <= speed <= 255:
                print('Gripper speed not allowed.')
            if not 0 <= force <= 255:
                print('Gripper force not allowed.')

    def close(self, speed=255, force=255):
        self.set(pos=255, speed=speed, force=force)

    def open(self, speed=255, force=255):
        self.set(pos=0, speed=speed, force=force)

    def shutdown(self):
        self.gripper_serial.shutdown()
        time.sleep(0.5)
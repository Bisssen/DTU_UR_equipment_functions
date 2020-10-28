import time
from math import pi
import numpy as np
import socket
import sys

from . import config_force_sensor
from .communication_force_sensor import communication_thread


class ForceSensor:
    def __init__(self, ip=None, port=None):
        # Whether the program is run in python 2 or not
        self.python_2 = (sys.version_info.major == 2)
        
        # If no ip is provided, then use default
        if ip is None:
            self.ip = config_force_sensor.IP
        else:
            self.ip = ip

        # If no port is provided, then use default
        if port is None:
            self.port = config_force_sensor.PORT
        else:
            self.port = port

        # Starting communication script
        self.communication_thread = communication_thread(self.ip, self.port)


    # Creates a reference force to use in force feedback movements
    # The return_rz is for legacy reasons
    def create_reference_force(self, amount_of_measurements,
                               list_of_desired_forces):
        # Make sure data have been recieved
        while True:
            if len(self.communication_thread.data) < 6:
                continue
            break

        # Init variables
        count = 0
        old_measurement = 0
        force_measurements = [[]] * len(list_of_desired_forces)

        # Keep going until the desired amount of measurements have been achieved
        while True:
            time.sleep(0.005)
            # Read the force on the arm
            if str(measurement) == old_measurement:
                continue
            else:
                old_measurement = str(measurement)

            for i, desired_force in enumerate(list_of_desired_forces):
                # Check that the inputs given to the function is valid
                if not desired_force in config_force_sensor.FORCE_LIST:
                    print(str(desired_force) + ' is not a valid force direction. ' +
                          'It must be a value from: ' +
                          str(config_force_sensor.FORCE_LIST))
                    continue

                # Read the force data from the communication thread
                desired_data = self.communication_thread.data[desired_force]

                # The first measurement needs to create the list
                if len(force_measurements[i]) == 0:
                    force_measurements[i] = [desired_data]
                # The subsequent must be appended
                else:
                    force_measurements[i].append(desired_data)

            # If enough measurements have been made then break
            if count > amount_of_measurements:
                break
            else:
                count += 1

        # Return the average results
        average_measurements = []
        for measurements in force_measurements:
            average_measurements.append(sum(measurements)/len(measurements))
        
        return average_measurements


    def shutdown(self):
        self.communication_thread.shutdown()

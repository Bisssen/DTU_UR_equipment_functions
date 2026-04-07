from src.force_serial.class_force import Force
import time

f = Force('/dev/ttyUSB0')

while True:

    f.read()
    time.sleep(1)



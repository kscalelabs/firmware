# This is an implementation of the Waveshare 1.28" LCD RP2040 Board
# https://www.waveshare.com/wiki/1.28inch_LCD_RP2040_Board
# It uses the CircuitPython libraries and the GC9A01 display driver
# https://circuitpython.readthedocs.io/projects/gc9a01/en/latest/
# I had to implement my own battery and IMU classes because the
# existing ones didn't work with CircuitPython.  
# @author: Jesse R. Castro
# @TODO: Add menu class
# @TODO: Record accelerometer data to a file for calibration

import random
import time
from math import floor


from circuitrunner import wsRP2040128



# Additional functionality for eyes. Subclass from wsRP2040128 
class EyesDisplay(wsRP2040128):
    def __init__(self):
        super().__init__()
    

hardware = EyesDisplay()
hardware.draw_circle('eye', 115, 120, 100, hardware.color('white'))
hardware.sprites['eye'].x = 80




# X go from 90 to 160
# y go from 80 to 170

sleep_time = 0.01
moving_pos = True

while True:
    if(hardware.sprites['eye'].x < 80):
        moving_pos = True
        hardware.sprites['eye'].x = 80
    if(hardware.sprites['eye'].x >170):
        moving_pos = False
        hardware.sprites['eye'].x = 170
    
    if moving_pos:
        hardware.sprites['eye'].x += 2
    else:
        hardware.sprites['eye'].x -= 2
    
    if(hardware.time + sleep_time) < time.monotonic():
        hardware.update()
        hardware.time = time.monotonic()
    
    time.sleep(sleep_time)

        
        

# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 01:16:39 2020

@author: vital
"""

import time
import Adafruit_ADS1x15

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2/3

print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
print('-' * 37)

while True:
    values = [0]*4
    values[0] = adc.read_adc(0, GAIN)
    values[1] = adc.read_adc(1, GAIN)
    values[2] = adc.read_adc(2, GAIN)
    values[3] = adc.read_adc(3, GAIN)
    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
    time.sleep(0.5)
# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 01:11:25 2020

@author: vital
"""

import RPi.GPIO as GPIO # Импортируем нужный модуль
import time
import Adafruit_ADS1x15 # from Adafruit_ADS1x15 import ADS1x15
#import numpy as np

adc = Adafruit_ADS1x15.ADS1115()
GAIN = 2/3
#sps = 128 # CHECK!!!! from a different library! # how long it takes for the ADC to carry out a single conversion (1/sps)
#adc.start_adc(0, gain=GAIN)

bits = 32767.5
base = 4.096 * 3 / 2

#mode = GPIO.getmode() # Проверяем метод нумерации
GPIO.setmode(GPIO.BOARD) # Устанавливаем метод BOARD, если не установлен ранее

chan_list_left = [11,13,15,16] # Управляющие пины для левой руки # S0 S1 S2 S3   
chan_list_right = [18,22,29,31] # Управляющие пины для правой руки # S0 S1 S2 S3                            

GPIO.setup(chan_list_left, GPIO.OUT, initial=GPIO.LOW) # на выход, на правую руку на всё подается LOW  
GPIO.setup(chan_list_right, GPIO.OUT, initial=GPIO.LOW) # на выход, на левую руку на всё подается LOW      
        
muxChannel=[[0,0,0,0], # channel 0 
            [1,0,0,0], # channel 1 
            [0,1,0,0], # channel 2 
            [1,1,0,0], # channel 3 
            [0,0,1,0], # channel 4
            [1,0,1,0], # channel 5
            [0,1,1,0], # channel 6
            [1,1,1,0], # channel 7
            [0,0,0,1], # channel 8
            [1,0,0,1], # channel 9
            [0,1,0,1], # channel 10
            [1,1,0,1], # channel 11
            [0,0,1,1], # channel 12
            [1,0,1,1], # channel 13
            [0,1,1,1], # channel 14
            [1,1,1,1]] # channel 15
 
number_of_sensors_left = 5
number_of_sensors_right = 5  

def get_analog_input(hand, sensor):
    if hand == 'L':
        GPIO.output(chan_list_left, muxChannel[i])
        val = adc.read_adc(0, gain=GAIN)
        vol = val * (base / bits)
        return vol
    if hand == 'R':
        GPIO.output(chan_list_right, muxChannel[i])
        val = adc.read_adc(1, gain=GAIN)
        vol = val * (base / bits)
        return vol

frequency = 5 # how often the Pi reads the ADS1115, Hz
deltat = 1.0 / frequency        # Calculate sampling period
 
startTime=time.time()                   # Time of first sample
t1=startTime                            # T1 is last sample time
t2=t1                                   # T2 is current time

while True:                         # Loop in which data is sampled
    while (t2-t1 < deltat):         # Check if t2-t1 is less then sample period, if it is then update t2
        t2=time.time()              # and check again       
    t1+=deltat                  # Update last sample time by the sampling period
    
    values_left = [0]*number_of_sensors_left
    print 'Left: ',
    for i in range(number_of_sensors_left):
        values_left[i] = get_analog_input('L', i)
        print i, ': ', "%.2f" % values_left[i], " V    ",
    print '    ',
    values_right = [0]*number_of_sensors_right
    print 'Right: ',
    for i in range(number_of_sensors_right):
        values_right[i] = get_analog_input('R', i)
        print i, ': ', "%.2f" % values_right[i], " V    ",
    print ("%.2f" % (t2 - startTime) , " s")

GPIO.cleanup() # Очистка

# /home/pi/.local/lib/python2.7/site-packages/Adafruit_PureIO/smbus.py REQUIRED FIXING
# AT LINE 216 - ADDED str()
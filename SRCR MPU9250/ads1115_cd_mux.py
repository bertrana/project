# -*- coding: utf-8 -*-
"""
Created on Wed Apr  1 01:11:25 2020

@author: vital
"""

import RPi.GPIO as GPIO # Импортируем нужный модуль
import time
from Adafruit_ADS1x15 import ADS1x15
#import numpy as np

def dataADS(inputA):
		
    frequency = 1	# how often the Pi reads the ADS1115, Hz
    sps = 8			# how long it takes for the ADC to carry out a single conversion (1/sps)
    time1 = 360		# Get how long to sample for from the user
    
    period = 1.0 / frequency		# Calculate sampling period
    
    datapoints = int(time1*frequency)		# Datapoints is the total number of samples to take, which must be an integer
    
    startTime=time.time()					# Time of first sample
    t1=startTime							# T1 is last sample time
    t2=t1									# T2 is current time
    
    for x in range (0,datapoints):		# Loop in which data is sampled
        while (t2-t1 < period):		    # Check if t2-t1 is less then sample period, if it is then update t2
            t2=time.time()				# and check again		
        t1+=period						# Update last sample time by the sampling period
        print (adc.readADCSingleEnded(inputA, pga, sps), " mV		", ("%.2f" % (t2-startTime)) , " s")		# Print sampled value and time to the terminal

pga = 6144					# Set full-scale range of programable gain amplifier (page 13 of data sheet), change depending on the input voltage range
ADS1115 = 0x01				# Specify that the device being used is the ADS1115, for the ADS1015 used 0x00
adc = ADS1x15(ic=ADS1115)	# Create instance of the class ADS1x15 called adc

mode = GPIO.getmode() # Проверяем метод нумерации
GPIO.setmode(GPIO.BOARD) # Устанавливаем метод BOARD, если не установлен ранее

chan_list_right = (18,22,29,31) # Управляющие пины для правой руки 
chan_list_left = (11,13,15,16) # Управляющие пины для левой руки                                  

GPIO.output(chan_list_right, GPIO.LOW) # на левую руку на всё подается LOW
GPIO.output(chan_list_left, GPIO.LOW) # на правую руку на всё подается LOW        
        
muxChannel=[[0,0,0,0], # channel 0 
            [1,0,0,0], # channel 1 
            [0,1,0,0], # channel 2 
            [1,1,0,0], # channel 3 
            [0,0,1,0]] # channel 4
 
for i in range(5):
    GPIO.output(chan_list_right, muxChannel(i))
    dataADS(0)
    GPIO.output(chan_list_left, muxChannel(i))
    dataADS(1)
    time.sleep(1)    
    
GPIO.cleanup() # Очистка
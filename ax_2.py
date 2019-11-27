#!/usr/bin/python

import smbus
import math
import csv
import time
import numpy as np
import matplotlib.pyplot as plt

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

f = open('readings.csv','w')
col = ['X rot','Y rot','X rot(CF)','Y rot(CF)']#,'X rot(CF)','Y rot(CF)'
csv.writer(f).writerow(col)
alpha = 0.1
fs = 100.0
dt=1.0/fs

compfxold = 0.0
compfyold = 0.0

tmp = 1000

list_dt = []
list_dx = []
list_data = []

while tmp > 0:

    tmp = tmp - 1
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)

    gyro_xout_scaled = gyro_xout / 131
    gyro_yout_scaled = gyro_yout / 131
    gyro_zout_scaled = gyro_zout / 131

    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    xrot = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    yrot = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    
    list_dx.append(xrot)
    list_dt.append(dt)

    compfx = (1-alpha) * (compfxold + gyro_xout_scaled * dt) + alpha * (xrot)
    compfy = (1-alpha) * (compfyold + gyro_yout_scaled * dt) + alpha * (yrot)

    compfxold = compfx
    compfyold = compfy

    data = []
    data.append(xrot)
    data.append(yrot)
    data.append(compfx)
    data.append(compfy)
    
    list_data.append(data)
    #plt.plot(xrot,dt)

#     csv.writer(f).writerow(data)
    
    time.sleep(dt)
# print('list_data: ', list_data)
for tmp in list_data:
    csv.writer(f).writerow(tmp)
i = 0
tmp = [i+1 for i in range(1000)]
plt.plot(list_dx, tmp)
# print('list_dx: ', list_dx)
# print('list_dt: ', list_dt)

f.close()
plt.savefig('testplot.png')
#
plt.show()
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

def read_word(adr,address): #считываются значения из регистров
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr,address): #преобразование значений, представленных в дополнительном коде
    val = read_word(adr,address)
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

def activate_addr(address):
    # This is the address value read via the i2cdetect command
    # Now wake the 6050 up as it starts in sleep mode
    bus.write_byte_data(address, power_mgmt_1, 0)
    return address

def mpu_data_acquire(address):
    gyro_xout = read_word_2c(0x43,address)
    gyro_yout = read_word_2c(0x45,address)
    gyro_zout = read_word_2c(0x47,address)

    gyro_xout_scaled = gyro_xout / 131
    gyro_yout_scaled = gyro_yout / 131
    gyro_zout_scaled = gyro_zout / 131

    accel_xout = read_word_2c(0x3b,address)
    accel_yout = read_word_2c(0x3d,address)
    accel_zout = read_word_2c(0x3f,address)

    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0
    
    return gyro_xout_scaled, gyro_yout_scaled, gyro_zout_scaled, accel_xout_scaled, accel_yout_scaled, accel_zout_scaled

def comp_filter(cxold, cyold, gxs, gys, xr, yr, t):
    alpha = 0.1
    compfx = (1-alpha) * (cxold + gxs * t) + alpha * (xr)
    compfy = (1-alpha) * (cyold + gys * t) + alpha * (yr)
    return compfx, compfy

bus = smbus.SMBus(1) #bus = smbus.SMBus(1) for Revision 2 boards, (0) for Rev. 1
address1 = activate_addr(0x68)    # This activates the 1 MPU address value read via the i2cdetect command and assigns the variable
address2 = activate_addr(0x69)    # This activates the 2 MPU the address value read via the i2cdetect command  and assigns the variable

fs = 100.0 #- данные, исползуемые для расчета без реального времени
dt = 1.0/fs
#continuet = 0.0

compfxold1 = 0.0
compfyold1 = 0.0
compfxold2 = 0.0
compfyold2 = 0.0

tmp = 1000

f = open('readings.csv','w')
col = ['X rot','Y rot','X rot(CF)','Y rot(CF)']#названия столбцов данных,'X rot(CF)','Y rot(CF)'
csv.writer(f).writerow(col)

while tmp > 0:
    tmp -= 1
    
    
    #start = continuet + (time.perf_counter()-continuet)
    
    result = mpu_data_acquire(address1)

    xrot = get_x_rotation(result[3], result[4], result[5])
    yrot = get_y_rotation(result[3], result[4], result[5])

    #now = time.perf_counter()
    
    #dt = now - start
    
    compf = comp_filter(compfxold1, compfyold1, result[0], result[1], xrot, yrot, dt)
    compfxold1 = compf[0]
    compfyold1 = compf[1]

    #continuet = time.perf_counter()
    
    result2 = mpu_data_acquire(address2)

    xrot = get_x_rotation(result2[3], result2[4], result2[5])
    yrot = get_y_rotation(result2[3], result2[4], result2[5])

    #now = time.perf_counter()
    
    #dt = now - continuet
    
    compf2 = comp_filter(compfxold2, compfyold2, result2[0], result2[1], xrot, yrot, dt)
    compfxold2 = compf2[0]
    compfyold2 = compf2[1]
    
    #continuet = time.perf_counter()

    data = []
    data.append(xrot)
    data.append(yrot)
    data.append(compf[0])
    data.append(compf[1])

    plt.plot(xrot,dt)

    csv.writer(f).writerow(data)

f.close()
plt.savefig('testplot.png')
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 13 19:03:35 2020

@author: vital
"""

import math
import time
import smbus

#MPU functions - функции для работы с датчиками MPU-6050
def read_register_mpu(address, register): #считываются значения из регистров
    high = bus.read_byte_data(address, register)
    low = bus.read_byte_data(address, register+1)
    val = (high << 8) + low
    if (val >= 0x8000):  #преобразование значений, представленных в дополнительном коде
        return -((65535 - val) + 1)
    else:
        return val

def read_register_mpu_little_endian(address, register): #считываются значения из регистров
    low = bus.read_byte_data(address, register)
    high = bus.read_byte_data(address, register+1)
    val = (low << 8) + high
    if (val >= 0x8000):  #преобразование значений, представленных в дополнительном коде
        return -((65535 - val) + 1)
    else:
        return val
    
def mpu_data_acquire(mpuAddr): #получение данных из регистров датчика
#    bus.write_byte_data(muxAddr, 0, mux_channels[muxMpu]) #выбор канала мультиплексора
    g_x = read_register_mpu(mpuAddr, 0x43)
    g_y = read_register_mpu(mpuAddr, 0x45)
    g_z = read_register_mpu(mpuAddr, 0x47)
    g_x_s = g_x/131
    g_y_s = g_y/131
    g_z_s = g_z/131
    a_x = read_register_mpu(mpuAddr, 0x3b)
    a_y = read_register_mpu(mpuAddr, 0x3d)
    a_z = read_register_mpu(mpuAddr, 0x3f)
    a_x_s = a_x #/16384
    a_y_s = a_y #/16384
    a_z_s = a_z #/16384
    m_x = read_register_mpu_little_endian(0x0C, 0x03)
    m_y = read_register_mpu_little_endian(0x0C, 0x05)
    m_z = read_register_mpu_little_endian(0x0C, 0x07)
    m_x_s = m_x #/6.83
    m_y_s = m_y #/6.83
    m_z_s = m_z #/6.83
    bus.read_byte_data(AK8963_ADDRESS,AK8963_ST2) # needed step for reading magnetic data
    return g_x_s, g_y_s, g_z_s, a_x_s, a_y_s, a_z_s, m_x_s, m_y_s, m_z_s
# Function to compute one filter iteration
def filterUpdate(g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z):
    #local system variables
    global SEq_1, SEq_2, SEq_3, SEq_4, b_x, b_z, g_bx, g_by, g_bz
    # axulirary variables to avoid reapeated calcualtions
    halfSEq_1 = 0.5 * SEq_1
    halfSEq_2 = 0.5 * SEq_2
    halfSEq_3 = 0.5 * SEq_3
    halfSEq_4 = 0.5 * SEq_4
    twoSEq_1 = 2.0 * SEq_1
    twoSEq_2 = 2.0 * SEq_2
    twoSEq_3 = 2.0 * SEq_3
    twoSEq_4 = 2.0 * SEq_4
    twob_x = 2.0 * b_x
    twob_z = 2.0 * b_z
    twob_xSEq_1 = 2.0 * b_x * SEq_1
    twob_xSEq_2 = 2.0 * b_x * SEq_2
    twob_xSEq_3 = 2.0 * b_x * SEq_3
    twob_xSEq_4 = 2.0 * b_x * SEq_4
    twob_zSEq_1 = 2.0 * b_z * SEq_1
    twob_zSEq_2 = 2.0 * b_z * SEq_2
    twob_zSEq_3 = 2.0 * b_z * SEq_3
    twob_zSEq_4 = 2.0 * b_z * SEq_4
    #SEq_1SEq_2
    SEq_1SEq_3 = SEq_1 * SEq_3
    #SEq_1SEq_4
    #SEq_2SEq_3
    SEq_2SEq_4 = SEq_2 * SEq_4
    #SEq_3SEq_4
    twom_x = 2.0 * m_x
    twom_y = 2.0 * m_y
    twom_z = 2.0 * m_z
    # normalise the accelerometer measurement
    norm = math.sqrt(a_x*a_x + a_y*a_y + a_z*a_z)
    if norm == 0.0:
        return
    a_x = a_x / norm
    a_y = a_y / norm
    a_z = a_z / norm
    # normalise the magnetometer measurement
    norm = math.sqrt(m_x*m_x + m_y*m_y + m_z*m_z)
    if norm == 0.0:
        return
    m_x = m_x / norm
    m_y = m_y / norm
    m_z = m_z / norm
    # compute the objective function and Jacobian
    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x
    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y
    f_3 = 1.0 - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z
    f_4 = twob_x * (0.5 - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x
    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y
    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5 - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z
    J_11or24 = twoSEq_3 # J_11 negated in matrix multiplication
    J_12or23 = 2.0 * SEq_4
    J_13or22 = twoSEq_1 # J_12 negated in matrix multiplication
    J_14or21 = twoSEq_2
    J_32 = 2.0 * J_14or21 # negated in matrix multiplication
    J_33 = 2.0 * J_11or24 # negated in matrix multiplication
    J_41 = twob_zSEq_3 # negated in matrix multiplication
    J_42 = twob_zSEq_4
    J_43 = 2.0 * twob_xSEq_3 + twob_zSEq_1 # negated in matrix multiplication
    J_44 = 2.0 * twob_xSEq_4 - twob_zSEq_2 # negated in matrix multiplication
    J_51 = twob_xSEq_4 - twob_zSEq_2 # negated in matrix multiplication
    J_52 = twob_xSEq_3 + twob_zSEq_1
    J_53 = twob_xSEq_2 + twob_zSEq_4
    J_54 = twob_xSEq_1 - twob_zSEq_3 # negated in matrix multiplication
    J_61 = twob_xSEq_3
    J_62 = twob_xSEq_4 - 2.0 * twob_zSEq_2
    J_63 = twob_xSEq_1 - 2.0 * twob_zSEq_3
    J_64 = twob_xSEq_2
    # compute the gradient (matrix multiplication)
    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6
    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6
    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6
    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6
    # normalise the gradient to estimate direction of the gyroscope error
    norm = math.sqrt(SEqHatDot_1*SEqHatDot_1 + SEqHatDot_2*SEqHatDot_2 + SEqHatDot_3*SEqHatDot_3 + SEqHatDot_4*SEqHatDot_4)
    if norm == 0.0:
        return
    SEqHatDot_1 = SEqHatDot_1 / norm
    SEqHatDot_2 = SEqHatDot_2 / norm
    SEqHatDot_3 = SEqHatDot_3 / norm
    SEqHatDot_4 = SEqHatDot_4 / norm
    # compute angular estimated direction of the gyroscope error
    g_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3
    g_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2
    g_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1
    # compute and remove the gyroscope baises
    g_bx = g_bx + g_err_x * deltat * zeta
    g_by = g_by + g_err_y * deltat * zeta
    g_bz = g_bz + g_err_z * deltat * zeta
    g_x = g_x - g_bx
    g_y = g_y - g_by
    g_z = g_z - g_bz
    # compute the quaternion rate measured by gyroscopes
    SEqDot_omega_1 = -halfSEq_2 * g_x - halfSEq_3 * g_y - halfSEq_4 * g_z
    SEqDot_omega_2 = halfSEq_1 * g_x + halfSEq_3 * g_z - halfSEq_4 * g_y
    SEqDot_omega_3 = halfSEq_1 * g_y - halfSEq_2 * g_z + halfSEq_4 * g_x
    SEqDot_omega_4 = halfSEq_1 * g_z + halfSEq_2 * g_y - halfSEq_3 * g_x
    # compute then integrate the estimated quaternion rate
    SEq_1 = SEq_1 + (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat
    SEq_2 = SEq_2 + (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat
    SEq_3 = SEq_3 + (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat
    SEq_4 = SEq_4 + (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat
    # normalise quaternion
    norm = math.sqrt(SEq_1*SEq_1 + SEq_2*SEq_2 + SEq_3*SEq_3 + SEq_4*SEq_4)
    SEq_1 = SEq_1 / norm
    SEq_2 = SEq_2 / norm
    SEq_3 = SEq_3 / norm
    SEq_4 = SEq_4 / norm
    # compute flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2 # recompute axulirary variables
    SEq_1SEq_3 = SEq_1 * SEq_3
    SEq_1SEq_4 = SEq_1 * SEq_4
    SEq_3SEq_4 = SEq_3 * SEq_4
    SEq_2SEq_3 = SEq_2 * SEq_3
    SEq_2SEq_4 = SEq_2 * SEq_4
    h_x = twom_x * (0.5 - SEq_3*SEq_3 - SEq_4*SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3)
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5 - SEq_2*SEq_2 - SEq_4*SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2)
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5 - SEq_2*SEq_2 - SEq_3*SEq_3)
    # normalise the flux vector to have only components in the x and z
    b_x = math.sqrt((h_x*h_x) + (h_y*h_y))
    b_z = h_z
    return SEq_1, SEq_2, SEq_3, SEq_4

MPU9250_ADDRESS = 0x68 #mpu_address_2 = 0x69
AK8963_ADDRESS  = 0x0C
DEVICE_ID       = 0x71
WHO_AM_I        = 0x75
PWR_MGMT_1      = 0x6B
INT_PIN_CFG     = 0x37
INT_ENABLE      = 0x38
# --- Accel ------------------
ACCEL_DATA    = 0x3B
ACCEL_CONFIG  = 0x1C
ACCEL_CONFIG2 = 0x1D
ACCEL_2G      = 0x00
ACCEL_4G      = (0x01 << 3)
ACCEL_8G      = (0x02 << 3)
ACCEL_16G     = (0x03 << 3)
# --- Temp --------------------
TEMP_DATA = 0x41
# --- Gyro --------------------
GYRO_DATA    = 0x43
GYRO_CONFIG  = 0x1B
GYRO_250DPS  = 0x00
GYRO_500DPS  = (0x01 << 3)
GYRO_1000DPS = (0x02 << 3)
GYRO_2000DPS = (0x03 << 3)

# --- AK8963 ------------------
MAGNET_DATA  = 0x03
AK_DEVICE_ID = 0x48
AK_WHO_AM_I  = 0x00
AK8963_8HZ   = 0x02
AK8963_100HZ = 0x06
AK8963_14BIT = 0x00
AK8963_16BIT = (0x01 << 4)
AK8963_CNTL1 = 0x0A
AK8963_CNTL2 = 0x0B
AK8963_ASAX  = 0x10
AK8963_ST1   = 0x02
AK8963_ST2   = 0x09
AK8963_ASTC  = 0x0C
ASTC_SELF    = 0x01<<6
#mux_upp_address = 0x70 #mux_low_address = 0x71
#mux_channel_switch = 0x08

#MPUs = 4 #Setting the number of used MPUs
#mux_channels = [mux_channel_switch | 0, mux_channel_switch | 1, mux_channel_switch | 2, mux_channel_switch | 3,
#                mux_channel_switch | 4, mux_channel_switch | 5, mux_channel_switch | 6, mux_channel_switch | 7]

bus = smbus.SMBus(1) #bus = smbus.SMBus(1) for Revision 2 boards, (0) for Rev. 1
#for MPU in range(MPUs):
#    bus.write_byte_data(muxAddr, 0, mux_channels[muxMpu]) #выбор канала мультиплексора
#    bus.write_byte_data(mpuAddr, mpu_power_mgmt, 0) #выводим датчик из спящего режима
#    print('MPU ' + str(muxMpu) + ' activated')
bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x00)
bus.write_byte_data(MPU9250_ADDRESS, PWR_MGMT_1, 0x01)  # auto select clock source
bus.write_byte_data(MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_4G)
bus.write_byte_data(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_250DPS)
bus.write_byte_data(MPU9250_ADDRESS, INT_PIN_CFG, 0x22)
bus.write_byte_data(MPU9250_ADDRESS, INT_ENABLE, 0x01)
bus.write_byte_data(AK8963_ADDRESS, AK8963_CNTL1, (AK8963_16BIT | AK8963_8HZ)) # cont mode 1
bus.write_byte_data(AK8963_ADDRESS, AK8963_ASTC, 0)

#alsb = 4.0 / 32760 # ACCEL_2G
#glsb = 250.0 / 32760 # GYRO_250DPS
#mlsb = 4800.0 / 32760 # MAGNET range +-4800
# System constants
deltat = 0.2 # sampling period in seconds (shown as 1 ms)
gyroMeasError = math.pi * (-3600.0 / 180.0) # gyroscope measurement error in rad/s (shown as 5 deg/s)
gyroMeasDrift = math.pi * (0.0 / 180.0) # gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
beta = math.sqrt(3.0 / 4.0) * gyroMeasError # compute beta
zeta = math.sqrt(3.0 / 4.0) * gyroMeasDrift # compute zeta
# Global system variables
# a_x, a_y, a_z // accelerometer measurements
# g_x, g_y, g_z // gyroscope measurements in rad/s
# m_x, m_y, m_z // magnetometer measurements
SEq_1 = 1
SEq_2 = 0
SEq_3 = 0
SEq_4 = 0 # estimated orientation quaternion elements with initial conditions
b_x = 1
b_z = 0 # reference direction of flux in earth frame
g_bx = 0
g_by = 0
g_bz = 0 # estimate gyroscope biases error

while True:
    ready = input("Input 1 when you are ready to start: ")
    if ready == 1:
        break
    else:
        print "The input was not a 1."
#mpu_data_acquire(MPU9250_ADDRESS)
#g_x, g_y, g_z, a_x, a_y, a_z, m_x, m_y, m_z
startTime=time.time()                   # Time of first sample
t1=startTime                            # T1 is last sample time
t2=t1                                   # T2 is current time   
#    for x in range (0,datapoints):     # Loop in which data is sampled
while True:
    while (t2-t1 < deltat):             # Check if t2-t1 is less then sample period, if it is then update t2
        t2 = time.time()                # and check again       
    t1 += deltat                        # Update last sample time by the sampling period
    #print (adc.readADCSingleEnded(inputA, pga, sps), " mV      ", ("%.2f" % (t2-startTime)) , " s")        # Print sampled value and time to the terminal
    gxs, gys, gzs, axs, ays, azs, mxs, mys, mzs = mpu_data_acquire(MPU9250_ADDRESS)
    w, x, y, z = filterUpdate(gxs, gys, gzs, axs, ays, azs, mxs, mys, mzs)
    print('quat: ' + str(w) + '  ' + str(x) + '  ' + str(y) + '  ' + str(z))
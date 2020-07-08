# -*- coding: utf-8 -*-
"""
Created on Tue Jun 30 19:11:21 2020

@author: vital
"""

import time
from MPU6050 import MPU6050

# Initialisation functions
def mpuClass(mpu_addr, x_accel_offset, y_accel_offset, z_accel_offset,
             x_gyro_offset, y_gyro_offset, z_gyro_offset):
    mpu = MPU6050(i2c_bus, mpu_addr, x_accel_offset, y_accel_offset,
                  z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
                  enable_debug_output)
    return mpu
def mpuIntStatus_PacketSize_FIFOCount(mpu):
    mpu.dmp_initialize()
    mpu.set_DMP_enabled(True)
    mpu_int_status = mpu.get_int_status()
    print(hex(mpu_int_status))
    packet_size = mpu.DMP_get_FIFO_packet_size()
    print(packet_size)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count)
    return packet_size

# Loop functions
def getQuat(mpu, packet_size):
    FIFO_buffer = [0]*64
    x = True
    while x:
        FIFO_count = mpu.get_FIFO_count()
        mpu_int_status = mpu.get_int_status()
        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            mpu.reset_FIFO()
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading
            # (default is 42 bytes)
            while FIFO_count < packet_size:
                FIFO_count = mpu.get_FIFO_count()
            FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
            x = False
    quat = mpu.DMP_get_quaternion(FIFO_buffer)
    return quat.w, quat.x, quat.y, quat.z
def relativeQuat(w_base, x_base, y_base, z_base, w_rot, x_rot, y_rot, z_rot):
    # w_rot, x_rot, y_rot, z_rot, w_base, x_base, y_base, z_base
    # w_base = w_base
    x_base = -x_base
    y_base = -y_base
    z_base = -z_base
    w_rltv = w_base * w_rot - x_base * x_rot - y_base * y_rot - z_base * z_rot
    x_rltv = w_base * x_rot + x_base * w_rot + y_base * z_rot - z_base * y_rot
    y_rltv = w_base * y_rot - x_base * z_rot + y_base * w_rot + z_base * x_rot
    z_rltv = w_base * z_rot + x_base * y_rot - y_base * x_rot + z_base * w_rot
    return w_rltv, x_rltv, y_rltv, z_rltv
def printQuat(w, x, y, z):
    print('quat: ' + str(w) + '  ' + str(x) + '  ' + str(y) + '  ' + str(z))

##############################################################################
###############################################################################
##### Calibration #############################################################
###############################################################################
i2c_bus = 1
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -5489
y_accel_offset = -1441
z_accel_offset = 1305
x_gyro_offset = -2
y_gyro_offset = -72
z_gyro_offset = -5
enable_debug_output = True

###############################################################################

mpu_address_1 = 0x68
mpu_address_2 = 0x69

MPUs_upper = 1
MPUs_lower = 1

mpu_list_upper = [0]*MPUs_upper
mpu_list_lower = [0]*MPUs_lower

packet_size_list_upper = [0]*MPUs_upper
packet_size_list_lower = [0]*MPUs_lower

for MPU in range(MPUs_upper):
    mpu_list_upper[MPU] = mpuClass(mpu_address_1, x_accel_offset, y_accel_offset,
                  z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset)
for MPU in range(MPUs_lower):
    mpu_list_lower[MPU] = mpuClass(mpu_address_2, x_accel_offset, y_accel_offset,
                  z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset)

for MPU in range(MPUs_upper):
    print 'MPU_upper ', MPU, ':'
    packet_size_list_upper[MPU] = mpuIntStatus_PacketSize_FIFOCount(mpu_list_upper[MPU])
for MPU in range(MPUs_lower):
    print 'MPU_lower ', MPU, ':'
    packet_size_list_lower[MPU] = mpuIntStatus_PacketSize_FIFOCount(mpu_list_lower[MPU])

# raw quaternions from sensors
raw_quat_list_w = [0]*(MPUs_upper + MPUs_lower)
raw_quat_list_x = [0]*(MPUs_upper + MPUs_lower)
raw_quat_list_y = [0]*(MPUs_upper + MPUs_lower)
raw_quat_list_z = [0]*(MPUs_upper + MPUs_lower)

# zero state quaternions in starting positions
zero_state_list_w = [0]*(MPUs_upper + MPUs_lower)
zero_state_list_x = [0]*(MPUs_upper + MPUs_lower)
zero_state_list_y = [0]*(MPUs_upper + MPUs_lower)
zero_state_list_z = [0]*(MPUs_upper + MPUs_lower)

# raw quaternions relative to zero state
zerv_quat_list_w = [0]*(MPUs_upper + MPUs_lower)
zerv_quat_list_x = [0]*(MPUs_upper + MPUs_lower)
zerv_quat_list_y = [0]*(MPUs_upper + MPUs_lower)
zerv_quat_list_z = [0]*(MPUs_upper + MPUs_lower)

# final quaternions relative to previous limb parts quaternions
rltv_quat_list_w = [0]*(MPUs_upper + MPUs_lower)
rltv_quat_list_x = [0]*(MPUs_upper + MPUs_lower)
rltv_quat_list_y = [0]*(MPUs_upper + MPUs_lower)
rltv_quat_list_z = [0]*(MPUs_upper + MPUs_lower)

###############################################################################

tmp = 0
while tmp<100:
    for MPU in range(MPUs_upper):
        raw_quat_list_w[MPU], raw_quat_list_x[MPU], raw_quat_list_y[MPU], raw_quat_list_z[MPU] = getQuat(mpu_list_upper[0],
                                                                                                         packet_size_list_upper[0])
        
    for MPU in range(MPUs_lower):
        raw_quat_list_w[MPU + MPUs_upper], raw_quat_list_x[MPU + MPUs_upper], raw_quat_list_y[MPU + MPUs_upper], raw_quat_list_z[MPU + MPUs_upper] = getQuat(mpu_list_lower[0],
                                                                                                                                                             packet_size_list_lower[0])
    
    for MPU in range(MPUs_upper + MPUs_lower):
        print 'MPU ', MPU, ': ',
        printQuat(raw_quat_list_w[MPU], raw_quat_list_x[MPU], raw_quat_list_y[MPU],
        raw_quat_list_z[MPU])
    
    print ''
    tmp += 1

start_position = input('Input 1 when you have put on the suit and are ready to start? [1/0]')
if start_position:
    ###########################################################################
    ##### SHOULD QUATERNION AVERAGING BE ADDED HERE? ##########################
    ###########################################################################
    print 'ZERO STATES: '
    for MPU in range(MPUs_upper):
        zero_state_list_w[MPU], zero_state_list_x[MPU], zero_state_list_y[MPU], zero_state_list_z[MPU] = getQuat(mpu_list_upper[0],
                                                                                                                 packet_size_list_upper[0])
        
    for MPU in range(MPUs_lower):
        zero_state_list_w[MPU + MPUs_upper], zero_state_list_x[MPU + MPUs_upper], zero_state_list_y[MPU + MPUs_upper], zero_state_list_z[MPU + MPUs_upper] = getQuat(mpu_list_lower[0],
                                                                                                                                                                     packet_size_list_lower[0])
    
    for MPU in range(MPUs_upper + MPUs_lower):
        print 'MPU ', MPU, ': ',
        printQuat(zero_state_list_w[MPU], zero_state_list_x[MPU], zero_state_list_y[MPU],
        zero_state_list_z[MPU])
        print ''
    
    tmp = 0
    while tmp<1000:
        print 'RELATIVE TO ZERO STATES: '
        for MPU in range(MPUs_upper):
            raw_quat_list_w[MPU], raw_quat_list_x[MPU], raw_quat_list_y[MPU], raw_quat_list_z[MPU] = getQuat(mpu_list_upper[0],
                                                                                                             packet_size_list_upper[0])
            
            zerv_quat_list_w[MPU], zerv_quat_list_x[MPU], zerv_quat_list_y[MPU], zerv_quat_list_z[MPU] = relativeQuat(zero_state_list_w[MPU],
                                                                                                                      zero_state_list_x[MPU],
                                                                                                                      zero_state_list_y[MPU],
                                                                                                                      zero_state_list_z[MPU],
                                                                                                                      raw_quat_list_w[MPU],
                                                                                                                      raw_quat_list_x[MPU],
                                                                                                                      raw_quat_list_y[MPU],
                                                                                                                      raw_quat_list_z[MPU])
        
        for MPU in range(MPUs_lower):
            raw_quat_list_w[MPU + MPUs_upper], raw_quat_list_x[MPU + MPUs_upper], raw_quat_list_y[MPU + MPUs_upper], raw_quat_list_z[MPU + MPUs_upper] = getQuat(mpu_list_lower[0],
                                                                                                                                                                 packet_size_list_lower[0])
            
            zerv_quat_list_w[MPU + MPUs_upper], zerv_quat_list_x[MPU + MPUs_upper], zerv_quat_list_y[MPU + MPUs_upper], zerv_quat_list_z[MPU + MPUs_upper] = relativeQuat(zero_state_list_w[MPU + MPUs_upper],
                                                                                                                                                                          zero_state_list_x[MPU + MPUs_upper],
                                                                                                                                                                          zero_state_list_y[MPU + MPUs_upper],
                                                                                                                                                                          zero_state_list_z[MPU + MPUs_upper],
                                                                                                                                                                          raw_quat_list_w[MPU + MPUs_upper],
                                                                                                                                                                          raw_quat_list_x[MPU + MPUs_upper],
                                                                                                                                                                          raw_quat_list_y[MPU + MPUs_upper],
                                                                                                                                                                          raw_quat_list_z[MPU + MPUs_upper])
        
        for MPU in range(MPUs_upper + MPUs_lower):
            print 'MPU ', MPU, ': ',
            printQuat(zerv_quat_list_w[MPU], zerv_quat_list_x[MPU], zerv_quat_list_y[MPU],
            zerv_quat_list_z[MPU])
        
        ##### PART OF CODE TO BE REWRITTEN EVERY TIME #########################
        print 'RELATIVE TO PREVIOUS LIMB PARTS: '
        
        ##### JOINT #####
        rltv_quat_list_w[0], rltv_quat_list_x[0], rltv_quat_list_y[0], rltv_quat_list_z[0] = relativeQuat(zerv_quat_list_w[1],
                                                                                                          zerv_quat_list_x[1],
                                                                                                          zerv_quat_list_y[1],
                                                                                                          zerv_quat_list_z[1],
                                                                                                          zerv_quat_list_w[0],
                                                                                                          zerv_quat_list_x[0],
                                                                                                          zerv_quat_list_y[0],
                                                                                                          zerv_quat_list_z[0])
        print 'JOINT ROTATION QUATERNION: ',
        printQuat(rltv_quat_list_w[0], rltv_quat_list_x[0], rltv_quat_list_y[0],
                  rltv_quat_list_z[0])
        ##### BASE #####
        rltv_quat_list_w[1], rltv_quat_list_x[1], rltv_quat_list_y[1], rltv_quat_list_z[1] = zerv_quat_list_w[1], zerv_quat_list_x[1], zerv_quat_list_y[1], zerv_quat_list_z[1]
        print 'BASE ROTATION QUATERNION: ',
        printQuat(rltv_quat_list_w[1], rltv_quat_list_x[1], rltv_quat_list_y[1],
                  rltv_quat_list_z[1])
        print ''
        time.sleep(1)

        tmp += 1

##############################################################################
# Uncomment to use roll_pitch_yaw
'''
        accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        roll_pitch_yaw = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        #calculated from quat & grav, grav is calculated from quat
        print('roll: ' + str(roll_pitch_yaw.x))
        print('pitch: ' + str(roll_pitch_yaw.y))
        print('yaw: ' + str(roll_pitch_yaw.z))
    tmp += 1
'''
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 29 12:23:14 2020

@author: dasha
"""

from MPU6050 import MPU6050

def relativeQuat(w_base, x_base, y_base, z_base, w_rot, x_rot, y_rot, z_rot):
    # w_base = w_base
    x_base = -x_base
    y_base = -y_base
    z_base = -z_base
    w_rltv = w_base * w_rot - x_base * x_rot - y_base * y_rot - z_base * z_rot
    x_rltv = w_base * x_rot + x_base * w_rot + y_base * z_rot - z_base * y_rot
    y_rltv = w_base * y_rot - x_base * z_rot + y_base * w_rot + z_base * x_rot
    z_rltv = w_base * z_rot + x_base * y_rot - y_base * x_rot + z_base * w_rot
    return w_rltv, x_rltv, y_rltv, z_rltv

i2c_bus = 1
device_address_1 = 0x68
device_address_2 = 0x69
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -5489
y_accel_offset = -1441
z_accel_offset = 1305
x_gyro_offset = -2
y_gyro_offset = -72
z_gyro_offset = -5
enable_debug_output = True

mpu1 = MPU6050(i2c_bus, device_address_1, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)
mpu2 = MPU6050(i2c_bus, device_address_2, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

mpu1.dmp_initialize()
mpu1.set_DMP_enabled(True)
mpu_int_status1 = mpu1.get_int_status()
print(hex(mpu_int_status1))

mpu2.dmp_initialize()
mpu2.set_DMP_enabled(True)
mpu_int_status2 = mpu2.get_int_status()
print(hex(mpu_int_status2))

packet_size1 = mpu1.DMP_get_FIFO_packet_size()
print(packet_size1)
FIFO_count1 = mpu1.get_FIFO_count()
print(FIFO_count1)

packet_size2 = mpu2.DMP_get_FIFO_packet_size()
print(packet_size2)
FIFO_count2 = mpu2.get_FIFO_count()
print(FIFO_count2)

count = 0
FIFO_buffer1 = [0]*64
FIFO_buffer2 = [0]*64

FIFO_count_list = list()

tmp = 1 #tmp = True
while tmp:
    FIFO_count1 = mpu1.get_FIFO_count()
    mpu_int_status1 = mpu1.get_int_status()
    
    FIFO_count2 = mpu2.get_FIFO_count()
    mpu_int_status2 = mpu1.get_int_status()

    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count1 == 1024) or (mpu_int_status1 & 0x10):
        mpu1.reset_FIFO()
    # Check if fifo data is ready
    elif (mpu_int_status1 & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default is 42 bytes
        
        while FIFO_count1 < packet_size1:
            FIFO_count1 = mpu1.get_FIFO_count()
        FIFO_buffer1 = mpu1.get_FIFO_bytes(packet_size1)
        
    quat = mpu1.DMP_get_quaternion(FIFO_buffer1)
        
    quat1w = quat.w
    quat1x = quat.x
    quat1y = quat.y
    quat1z = quat.z
    
    # If overflow is detected by status or fifo count we want to reset    
    if (FIFO_count2 == 1024) or (mpu_int_status2 & 0x10):
        mpu2.reset_FIFO()
    # Check if fifo data is ready
    elif (mpu_int_status2 & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default is 42 bytes
        
        while FIFO_count2 < packet_size2:
            FIFO_count2 = mpu2.get_FIFO_count()
        FIFO_buffer2 = mpu2.get_FIFO_bytes(packet_size2)
        
    quat = mpu2.DMP_get_quaternion(FIFO_buffer2)
        
    quat2w = quat.w
    quat2x = quat.x
    quat2y = quat.y
    quat2z = quat.z
        
    quatw, quatx, quaty, quatz = relativeQuat(quat2w, quat2x, quat2y, quat2z, quat1w, quat1x, quat1y, quat1z)
    print('quat: ' + str(quatw) + '  ' + str(quatx) + '  ' + str(quaty) + '  ' + str(quatz))
        
    tmp += 1

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
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 29 12:23:14 2020

@author: dasha
"""

from MPU6050 import MPU6050

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

mpu = MPU6050(i2c_bus, device_address_1, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print(packet_size)
FIFO_count = mpu.get_FIFO_count()
print(FIFO_count)

count = 0
FIFO_buffer = [0]*64

FIFO_count_list = list()

tmp = 1 #tmp = True
while tmp:
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()

    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default is 42 bytes
        
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #Uncomment to use roll_pitch_yaw
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
        #Uncomment to use quat
    
        quat = mpu.DMP_get_quaternion(FIFO_buffer)
        print('quat: ' + str(quat.w) + '  ' + str(quat.x) + '  ' + str(quat.y) + '  ' + str(quat.z))
    tmp += 1

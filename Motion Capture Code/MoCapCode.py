 #!/usr/bin/python

import time
import RPi.GPIO as GPIO
import Adafruit_ADS1x15 # VERSION?
import smbus
from MPU6050 import MPU6050

# Initialisation functions
def mpu_activate(muxAddr, channel, mpuAddr): #activates the MPUs through multiplexer
    chooseChanell(muxAddr, channel) #выбор канала мультиплексора
    bus.write_byte_data(mpuAddr, mpu_power_mgmt, 0) #выводим датчик из спящего режима
    print('MPU ' + str(channel) + ' activated')
    
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
def chooseChanell(muxAddr, channel):
    bus.write_byte_data(muxAddr, 0, mux_channels[channel]) #выбор канала мул-ра
    
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

##### ADD A getEuler FUNCTION AND A printEuler FUNCTION #######################
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
###############################################################################

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

###############################################################################
def getAnalogInput(hand, sensor):
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
###############################################################################
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
###############################################################################
##### FIND AN ALGORITHM FOR CALIBRATION #######################################
###############################################################################
x_accel_offset = -5489
y_accel_offset = -1441
z_accel_offset = 1305
x_gyro_offset = -2
y_gyro_offset = -72
z_gyro_offset = -5
enable_debug_output = True
###############################################################################
bits = 32767.5
base = 4.096 * 3 / 2 # THIS VALUE IS THE INVERSE OF GAIN!!!
###############################################################################

i2c_bus = 1 #from MPU6050 ### DON'T KNOW IF IT WILL INTERFERE WITH smbus ######
bus = smbus.SMBus(1) # bus = smbus.SMBus(1) for Rev. 2 boards, (0) for Rev. 1
###############################################################################
adc = Adafruit_ADS1x15.ADS1115()
##### SHOULD THERE BE ANY ADS ADDRESS SETUP? #####
GAIN = 2/3
#sps = 128 # CHECK!!!! from a different library! # how long it takes for the ADC to carry out a single conversion (1/sps)
#mode = GPIO.getmode() # Проверяем метод нумерации
GPIO.setmode(GPIO.BOARD) # Устанавливаем метод BOARD, если не установлен ранее
###############################################################################
mpu_address_1 = 0x68
mpu_address_2 = 0x69
mpu_power_mgmt = 0x6b

mux_upp_address = 0x70
mux_low_address = 0x71

mux_channel_switch = 0x08
# I2C multiplexor channels
mux_channels = [mux_channel_switch | 0, mux_channel_switch | 1,
                mux_channel_switch | 2, mux_channel_switch | 3,
                mux_channel_switch | 4, mux_channel_switch | 5,
                mux_channel_switch | 6, mux_channel_switch | 7]
###############################################################################
# Analog multiplexor channels
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
###############################################################################
# Setting the number of used MPUs
MPUs_upper = 8
MPUs_lower = 7
###############################################################################
# Setting the number of analog sensors
number_of_sensors_left = 5
number_of_sensors_right = 5  
###############################################################################
mpu_list_upper = [0]*MPUs_upper
mpu_list_lower = [0]*MPUs_lower

packet_size_list_upper = [0]*MPUs_upper
packet_size_list_lower = [0]*MPUs_lower

print 'Upper MPUs activation and initialisation:'
for MPU in range(MPUs_upper):
    mpu_activate(mux_upp_address, MPU, mpu_address_1)
    mpu_list_upper[MPU] = mpuClass(mpu_address_1, x_accel_offset, y_accel_offset,
                  z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset)
print 'Lower MPUs activation and initialisation:'
for MPU in range(MPUs_lower):
    mpu_activate(mux_low_address, MPU, mpu_address_2)
    mpu_list_lower[MPU] = mpuClass(mpu_address_2, x_accel_offset, y_accel_offset,
                  z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset)

for MPU in range(MPUs_upper):
    print 'MPU_upper ', MPU, ':'
    chooseChanell(mux_upp_address, MPU)
    packet_size_list_upper[MPU] = mpuIntStatus_PacketSize_FIFOCount(mpu_list_upper[MPU])
for MPU in range(MPUs_lower):
    print 'MPU_lower ', MPU, ':'
    chooseChanell(mux_low_address, MPU)
    packet_size_list_lower[MPU] = mpuIntStatus_PacketSize_FIFOCount(mpu_list_lower[MPU])
###############################################################################
chan_list_left = [11,13,15,16] # Управляющие пины для левой руки # S0 S1 S2 S3   
chan_list_right = [18,22,29,31] # Управляющие пины для правой руки # S0 S1 S2 S3                            

GPIO.setup(chan_list_left, GPIO.OUT, initial=GPIO.LOW) # на выход, на правую руку на всё подается LOW  
GPIO.setup(chan_list_right, GPIO.OUT, initial=GPIO.LOW) # на выход, на левую руку на всё подается LOW
###############################################################################
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
values_left = [0]*number_of_sensors_left
values_right = [0]*number_of_sensors_right
###############################################################################

tmp = 0
while tmp<100:
    for MPU in range(MPUs_upper):
        chooseChanell(mux_upp_address, MPU)
        raw_quat_list_w[MPU], raw_quat_list_x[MPU], raw_quat_list_y[MPU], raw_quat_list_z[MPU] = getQuat(mpu_list_upper[0],
                                                                                                         packet_size_list_upper[0])
        
    for MPU in range(MPUs_lower):
        chooseChanell(mux_low_address, MPU)
        raw_quat_list_w[MPU + MPUs_upper], raw_quat_list_x[MPU + MPUs_upper], raw_quat_list_y[MPU + MPUs_upper], raw_quat_list_z[MPU + MPUs_upper] = getQuat(mpu_list_lower[0],
                                                                                                                                                             packet_size_list_lower[0])
    
    for MPU in range(MPUs_upper + MPUs_lower):
        print 'MPU ', MPU, ': ',
        printQuat(raw_quat_list_w[MPU], raw_quat_list_x[MPU], raw_quat_list_y[MPU],
        raw_quat_list_z[MPU])
    
    print ''
    tmp += 1

##### DETERMINING SAMPLING FREQUENCY ##########################################
frequency = 5 # how often the Pi reads the ADS1115, Hz
deltat = 1.0 / frequency        # Calculate sampling period

start_position = input('Input 1 when you have put on the suit and are ready to start? [1/0]')
if start_position:
    ###########################################################################
    ##### SHOULD QUATERNION AVERAGING BE ADDED HERE? ##########################
    ###########################################################################
    print 'ZERO STATES: '
    for MPU in range(MPUs_upper):
        chooseChanell(mux_upp_address, MPU)
        zero_state_list_w[MPU], zero_state_list_x[MPU], zero_state_list_y[MPU], zero_state_list_z[MPU] = getQuat(mpu_list_upper[0],
                                                                                                                 packet_size_list_upper[0])
        
    for MPU in range(MPUs_lower):
        chooseChanell(mux_low_address, MPU)
        zero_state_list_w[MPU + MPUs_upper], zero_state_list_x[MPU + MPUs_upper], zero_state_list_y[MPU + MPUs_upper], zero_state_list_z[MPU + MPUs_upper] = getQuat(mpu_list_lower[0],
                                                                                                                                                                     packet_size_list_lower[0])
    
    for MPU in range(MPUs_upper + MPUs_lower):
        print 'MPU ', MPU, ': ',
        printQuat(zero_state_list_w[MPU], zero_state_list_x[MPU], zero_state_list_y[MPU],
        zero_state_list_z[MPU])
        print ''
    
    ##### TIMING #####    
    startTime=time.time()                   # Time of first sample
    t1=startTime                            # T1 is last sample time
    t2=t1                                   # T2 is current time
    tmp = 0
    while tmp<1000: # Loop in which data is sampled
        while (t2-t1 < deltat):         # Check if t2-t1 is less then sample period, if it is then update t2
            t2=time.time()              # and check again       
        t1+=deltat                  # Update last sample time by the sampling period
        for MPU in range(MPUs_upper):
            chooseChanell(mux_upp_address, MPU)
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
            chooseChanell(mux_low_address, MPU)
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
        ''' #Uncomment to print RELATIVE TO ZERO STATES
        print 'RELATIVE TO ZERO STATES: '
        for MPU in range(MPUs_upper + MPUs_lower):
            print 'MPU ', MPU, ': ',
            printQuat(zerv_quat_list_w[MPU], zerv_quat_list_x[MPU], zerv_quat_list_y[MPU],
            zerv_quat_list_z[MPU])
        '''
        
        print 'ANALOG SENSORS DATA: '
        print 'Left: ',
        for i in range(number_of_sensors_left):
            values_left[i] = getAnalogInput('L', i)
            print i, ': ', "%.2f" % values_left[i], " V    ",
        print '    ',
        print 'Right: ',
        for i in range(number_of_sensors_right):
            values_right[i] = getAnalogInput('R', i)
            print i, ': ', "%.2f" % values_right[i], " V    ",
        print ("%.2f" % (t2 - startTime) , " s")
        ##### LIST OF RELATIVE TO ZERO QUATERNION COMPUTATIONS ################
        # *._ (* - mux number, 1 - address is 0x70, 2 - address is 0x71)
        # _.* (* - channel number)
        
        # 1.0 - zerv_quat_list_w[0], zerv_quat_list_x[0], zerv_quat_list_y[0], zerv_quat_list_z[0]
        # 1.1 - zerv_quat_list_w[1], zerv_quat_list_x[1], zerv_quat_list_y[1], zerv_quat_list_z[1]
        # 1.2 - zerv_quat_list_w[2], zerv_quat_list_x[2], zerv_quat_list_y[2], zerv_quat_list_z[2]
        # 1.3 - zerv_quat_list_w[3], zerv_quat_list_x[3], zerv_quat_list_y[3], zerv_quat_list_z[3]
        # 1.4 - zerv_quat_list_w[4], zerv_quat_list_x[4], zerv_quat_list_y[4], zerv_quat_list_z[4]
        # 1.5 - zerv_quat_list_w[5], zerv_quat_list_x[5], zerv_quat_list_y[5], zerv_quat_list_z[5]
        # 1.6 - zerv_quat_list_w[6], zerv_quat_list_x[6], zerv_quat_list_y[6], zerv_quat_list_z[6]
        # 1.7 - zerv_quat_list_w[7], zerv_quat_list_x[7], zerv_quat_list_y[7], zerv_quat_list_z[7]
        
        # 2.0 - zerv_quat_list_w[MPUs_upper + 0], zerv_quat_list_x[MPUs_upper + 0], zerv_quat_list_y[MPUs_upper + 0], zerv_quat_list_z[MPUs_upper + 0]
        # 2.1 - zerv_quat_list_w[MPUs_upper + 1], zerv_quat_list_x[MPUs_upper + 1], zerv_quat_list_y[MPUs_upper + 1], zerv_quat_list_z[MPUs_upper + 1]
        # 2.2 - zerv_quat_list_w[MPUs_upper + 2], zerv_quat_list_x[MPUs_upper + 2], zerv_quat_list_y[MPUs_upper + 2], zerv_quat_list_z[MPUs_upper + 2]
        # 2.3 - zerv_quat_list_w[MPUs_upper + 3], zerv_quat_list_x[MPUs_upper + 3], zerv_quat_list_y[MPUs_upper + 3], zerv_quat_list_z[MPUs_upper + 3]
        # 2.4 - zerv_quat_list_w[MPUs_upper + 4], zerv_quat_list_x[MPUs_upper + 4], zerv_quat_list_y[MPUs_upper + 4], zerv_quat_list_z[MPUs_upper + 4]
        # 2.5 - zerv_quat_list_w[MPUs_upper + 5], zerv_quat_list_x[MPUs_upper + 5], zerv_quat_list_y[MPUs_upper + 5], zerv_quat_list_z[MPUs_upper + 5]
        # 2.6 - zerv_quat_list_w[MPUs_upper + 6], zerv_quat_list_x[MPUs_upper + 6], zerv_quat_list_y[MPUs_upper + 6], zerv_quat_list_z[MPUs_upper + 6]
        
        ###############################################################################
        # System Description according to the schemes at
        # https://drive.google.com/drive/u/1/folders/12kCs8JDCrMaws85xVSfXheixJFZYA67Z
        ###############################################################################
        
        # MUX 1 addr 0x70
        # 8 MPUs
        #    channel 0 (head)           MPU addr 0x68
        #    channel 1 (right bicep)    MPU addr 0x68
        #    channel 2 (right forearm)  MPU addr 0x68
        #    channel 3 (right hand)     MPU addr 0x68
        #    channel 4 (back)           MPU addr 0x68
        #    channel 5 (left hand)      MPU addr 0x68
        #    channel 6 (left forearm)   MPU addr 0x68
        #    channel 7 (left shoulder)  MPU addr 0x68
        
        # MUX 2 addr 0x71
        # 7 MPUs
        #    channel 0 (waist)          MPU addr 0x69
        #    channel 1 (left thigh)     MPU addr 0x69
        #    channel 2 (left shin)      MPU addr 0x69
        #    channel 3 (left foot)      MPU addr 0x69
        #    channel 4 (right foot)     MPU addr 0x69
        #    channel 5 (right shin)     MPU addr 0x69
        #    channel 6 (right thigh)    MPU addr 0x69
        
        # ADS addr 0x48
        
        ##### SETUP DESCRIPTION ###############################################
        
        # WAIST_MOTION - 2.0                        rltv_quat_list_w[0], rltv_quat_list_x[0], rltv_quat_list_y[0], rltv_quat_list_z[0]
        # BACK_MOTION - 1.4 & WAIST_MOTION          rltv_quat_list_w[1], rltv_quat_list_x[1], rltv_quat_list_y[1], rltv_quat_list_z[1]
        # HEAD_MOTION - 1.0 & BACK_MOTION           rltv_quat_list_w[2], rltv_quat_list_x[2], rltv_quat_list_y[2], rltv_quat_list_z[2]
        # R_BICEP_MOTION - 1.1 & BACK_MOTION        rltv_quat_list_w[3], rltv_quat_list_x[3], rltv_quat_list_y[3], rltv_quat_list_z[3]
        # R_FOREARM_MOTION - 1.2 & R_BICEP_MOTION   rltv_quat_list_w[4], rltv_quat_list_x[4], rltv_quat_list_y[4], rltv_quat_list_z[4]
        # R_HAND_MOTION - 1.3 & R_FOREARM_MOTION    rltv_quat_list_w[5], rltv_quat_list_x[5], rltv_quat_list_y[5], rltv_quat_list_z[5]
        # L_BICEP_MOTION - 1.7 & BACK_MOTION        rltv_quat_list_w[6], rltv_quat_list_x[6], rltv_quat_list_y[6], rltv_quat_list_z[6]
        # L_FOREARM_MOTION - 1.6 & L_BICEP_MOTION   rltv_quat_list_w[7], rltv_quat_list_x[7], rltv_quat_list_y[7], rltv_quat_list_z[7]
        # L_HAND_MOTION - 1.5 & L_FOREARM_MOTION    rltv_quat_list_w[8], rltv_quat_list_x[8], rltv_quat_list_y[8], rltv_quat_list_z[8]
        # R_THIGH_MOTION - 2.6 & WAIST_MOTION       rltv_quat_list_w[9], rltv_quat_list_x[9], rltv_quat_list_y[9], rltv_quat_list_z[9]
        # R_SHIN_MOTION - 2.5 & R_THIGH_MOTION      rltv_quat_list_w[10], rltv_quat_list_x[10], rltv_quat_list_y[10], rltv_quat_list_z[10]
        # R_FOOT_MOTION - 2.4 & R_SHIN_MOTION       rltv_quat_list_w[11], rltv_quat_list_x[11], rltv_quat_list_y[11], rltv_quat_list_z[11]
        # L_THIGH_MOTION - 2.1 & WAIST_MOTION       rltv_quat_list_w[12], rltv_quat_list_x[12], rltv_quat_list_y[12], rltv_quat_list_z[12]
        # L_SHIN_MOTION - 2.2 & L_THIGH_MOTION      rltv_quat_list_w[13], rltv_quat_list_x[13], rltv_quat_list_y[13], rltv_quat_list_z[13]
        # L_FOOT_MOTION - 2.3 & L_SHIN_MOTION       rltv_quat_list_w[14], rltv_quat_list_x[14], rltv_quat_list_y[14], rltv_quat_list_z[14]
        
        ##### PART OF CODE TO BE REWRITTEN DEPENDING ON SETUP #################
        print 'RELATIVE TO PREVIOUS LIMB PARTS: '
        
        ##### WAIST_MOTION - 2.0 #####
        rltv_quat_list_w[0], rltv_quat_list_x[0], rltv_quat_list_y[0], rltv_quat_list_z[0] = zerv_quat_list_w[MPUs_upper + 0], zerv_quat_list_x[MPUs_upper + 0], zerv_quat_list_y[MPUs_upper + 0], zerv_quat_list_z[MPUs_upper + 0]
        print 'WAIST_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[0], rltv_quat_list_x[0], rltv_quat_list_y[0],
                  rltv_quat_list_z[0])
        
        ##### BACK_MOTION - 1.4 & WAIST_MOTION #####
        rltv_quat_list_w[1], rltv_quat_list_x[1], rltv_quat_list_y[1], rltv_quat_list_z[1] = relativeQuat(zerv_quat_list_w[4],
                        zerv_quat_list_x[4],zerv_quat_list_y[4], zerv_quat_list_z[4], rltv_quat_list_w[0], rltv_quat_list_x[0],
                        rltv_quat_list_y[0], rltv_quat_list_z[0])
        print 'BACK_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[1], rltv_quat_list_x[1], rltv_quat_list_y[1],
                  rltv_quat_list_z[1])
        
        ##### HEAD_MOTION - 1.0 & BACK_MOTION #####          
        rltv_quat_list_w[2], rltv_quat_list_x[2], rltv_quat_list_y[2], rltv_quat_list_z[2] = relativeQuat(zerv_quat_list_w[0],
                        zerv_quat_list_x[0], zerv_quat_list_y[0], zerv_quat_list_z[0], rltv_quat_list_w[1], rltv_quat_list_x[1],
                        rltv_quat_list_y[1], rltv_quat_list_z[1])
        print 'HEAD_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[2], rltv_quat_list_x[2], rltv_quat_list_y[2],
                  rltv_quat_list_z[2])
        
        ##### R_BICEP_MOTION - 1.1 & BACK_MOTION #####       
        rltv_quat_list_w[3], rltv_quat_list_x[3], rltv_quat_list_y[3], rltv_quat_list_z[3] = relativeQuat(zerv_quat_list_w[1],
                        zerv_quat_list_x[1], zerv_quat_list_y[1], zerv_quat_list_z[1], rltv_quat_list_w[1], rltv_quat_list_x[1],
                        rltv_quat_list_y[1], rltv_quat_list_z[1])
        print 'R_BICEP_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[3], rltv_quat_list_x[3], rltv_quat_list_y[3],
                  rltv_quat_list_z[3])
        
        ##### R_FOREARM_MOTION - 1.2 & R_BICEP_MOTION #####  
        rltv_quat_list_w[4], rltv_quat_list_x[4], rltv_quat_list_y[4], rltv_quat_list_z[4] = relativeQuat(zerv_quat_list_w[2],
                        zerv_quat_list_x[2], zerv_quat_list_y[2], zerv_quat_list_z[2], rltv_quat_list_w[3], rltv_quat_list_x[3],
                        rltv_quat_list_y[3], rltv_quat_list_z[3])
        print 'R_FOREARM_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[4], rltv_quat_list_x[4], rltv_quat_list_y[4], rltv_quat_list_z[4])
        
        ##### R_HAND_MOTION - 1.3 & R_FOREARM_MOTION #####   
        rltv_quat_list_w[5], rltv_quat_list_x[5], rltv_quat_list_y[5], rltv_quat_list_z[5] = relativeQuat(zerv_quat_list_w[3],
                        zerv_quat_list_x[3], zerv_quat_list_y[3], zerv_quat_list_z[3], rltv_quat_list_w[4], rltv_quat_list_x[4],
                        rltv_quat_list_y[4], rltv_quat_list_z[4])
        print 'R_HAND_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[5], rltv_quat_list_x[5], rltv_quat_list_y[5], rltv_quat_list_z[5])
        
        ##### L_BICEP_MOTION - 1.7 & BACK_MOTION #####       
        rltv_quat_list_w[6], rltv_quat_list_x[6], rltv_quat_list_y[6], rltv_quat_list_z[6] = relativeQuat(zerv_quat_list_w[7],
                        zerv_quat_list_x[7], zerv_quat_list_y[7], zerv_quat_list_z[7], rltv_quat_list_w[1], rltv_quat_list_x[1],
                        rltv_quat_list_y[1], rltv_quat_list_z[1])
        print 'L_BICEP_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[6], rltv_quat_list_x[6], rltv_quat_list_y[6], rltv_quat_list_z[6])
        
        ##### L_FOREARM_MOTION - 1.6 & L_BICEP_MOTION #####  
        rltv_quat_list_w[7], rltv_quat_list_x[7], rltv_quat_list_y[7], rltv_quat_list_z[7] = relativeQuat(zerv_quat_list_w[6],
                        zerv_quat_list_x[6], zerv_quat_list_y[6], zerv_quat_list_z[6], rltv_quat_list_w[6], rltv_quat_list_x[6],
                        rltv_quat_list_y[6], rltv_quat_list_z[6])
        print 'L_FOREARM_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[7], rltv_quat_list_x[7], rltv_quat_list_y[7], rltv_quat_list_z[7])
        
        ##### L_HAND_MOTION - 1.5 & L_FOREARM_MOTION #####   
        rltv_quat_list_w[8], rltv_quat_list_x[8], rltv_quat_list_y[8], rltv_quat_list_z[8] = relativeQuat(zerv_quat_list_w[5],
                        zerv_quat_list_x[5], zerv_quat_list_y[5], zerv_quat_list_z[5], rltv_quat_list_w[7], rltv_quat_list_x[7],
                        rltv_quat_list_y[7], rltv_quat_list_z[7])
        print 'L_HAND_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[8], rltv_quat_list_x[8], rltv_quat_list_y[8], rltv_quat_list_z[8])
        
        ##### R_THIGH_MOTION - 2.6 & WAIST_MOTION #####      
        rltv_quat_list_w[9], rltv_quat_list_x[9], rltv_quat_list_y[9], rltv_quat_list_z[9] = relativeQuat(zerv_quat_list_w[MPUs_upper + 6],
                        zerv_quat_list_x[MPUs_upper + 6], zerv_quat_list_y[MPUs_upper + 6], zerv_quat_list_z[MPUs_upper + 6], rltv_quat_list_w[0],
                        rltv_quat_list_x[0], rltv_quat_list_y[0], rltv_quat_list_z[0])
        print 'R_THIGH_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[9], rltv_quat_list_x[9], rltv_quat_list_y[9], rltv_quat_list_z[9])
        
        ##### R_SHIN_MOTION - 2.5 & R_THIGH_MOTION #####     
        rltv_quat_list_w[10], rltv_quat_list_x[10], rltv_quat_list_y[10], rltv_quat_list_z[10] = relativeQuat(zerv_quat_list_w[MPUs_upper + 5],
                        zerv_quat_list_x[MPUs_upper + 5], zerv_quat_list_y[MPUs_upper + 5], zerv_quat_list_z[MPUs_upper + 5], rltv_quat_list_w[9],
                        rltv_quat_list_x[9], rltv_quat_list_y[9], rltv_quat_list_z[9])
        print 'R_SHIN_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[10], rltv_quat_list_x[10], rltv_quat_list_y[10], rltv_quat_list_z[10])
        
        ##### R_FOOT_MOTION - 2.4 & R_SHIN_MOTION #####      
        rltv_quat_list_w[11], rltv_quat_list_x[11], rltv_quat_list_y[11], rltv_quat_list_z[11] = relativeQuat(zerv_quat_list_w[MPUs_upper + 4],
                        zerv_quat_list_x[MPUs_upper + 4], zerv_quat_list_y[MPUs_upper + 4], zerv_quat_list_z[MPUs_upper + 4], rltv_quat_list_w[10],
                        rltv_quat_list_x[10], rltv_quat_list_y[10], rltv_quat_list_z[10])
        print 'R_FOOT_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[11], rltv_quat_list_x[11], rltv_quat_list_y[11], rltv_quat_list_z[11])
        
        ##### L_THIGH_MOTION - 2.1 & WAIST_MOTION #####      
        rltv_quat_list_w[12], rltv_quat_list_x[12], rltv_quat_list_y[12], rltv_quat_list_z[12] = relativeQuat(zerv_quat_list_w[MPUs_upper + 1],
                        zerv_quat_list_x[MPUs_upper + 1], zerv_quat_list_y[MPUs_upper + 1], zerv_quat_list_z[MPUs_upper + 1], rltv_quat_list_w[0],
                        rltv_quat_list_x[0], rltv_quat_list_y[0], rltv_quat_list_z[0])
        print 'L_THIGH_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[12], rltv_quat_list_x[12], rltv_quat_list_y[12], rltv_quat_list_z[12])
        
        ##### L_SHIN_MOTION - 2.2 & L_THIGH_MOTION #####     
        rltv_quat_list_w[13], rltv_quat_list_x[13], rltv_quat_list_y[13], rltv_quat_list_z[13] = relativeQuat(zerv_quat_list_w[MPUs_upper + 2],
                        zerv_quat_list_x[MPUs_upper + 2], zerv_quat_list_y[MPUs_upper + 2], zerv_quat_list_z[MPUs_upper + 2], rltv_quat_list_w[12],
                        rltv_quat_list_x[12], rltv_quat_list_y[12], rltv_quat_list_z[12])
        print 'L_SHIN_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[13], rltv_quat_list_x[13], rltv_quat_list_y[13], rltv_quat_list_z[13])
        
        ##### L_FOOT_MOTION - 2.3 & L_SHIN_MOTION #####      
        rltv_quat_list_w[14], rltv_quat_list_x[14], rltv_quat_list_y[14], rltv_quat_list_z[14] = relativeQuat(zerv_quat_list_w[MPUs_upper + 3],
                        zerv_quat_list_x[MPUs_upper + 3], zerv_quat_list_y[MPUs_upper + 3], zerv_quat_list_z[MPUs_upper + 3], rltv_quat_list_w[13],
                        rltv_quat_list_x[13], rltv_quat_list_y[13], rltv_quat_list_z[13])
        print 'L_FOOT_MOTION QUATERNION: ',
        printQuat(rltv_quat_list_w[14], rltv_quat_list_x[14], rltv_quat_list_y[14], rltv_quat_list_z[14])
        
        print ''
        
        #time.sleep(1)

        tmp += 1
GPIO.cleanup() # Очистка

# /home/pi/.local/lib/python2.7/site-packages/Adafruit_PureIO/smbus.py REQUIRED FIXING
# AT LINE 216 - ADDED str()
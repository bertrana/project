 #!/usr/bin/python

import smbus
import math
import time
import matplotlib.pyplot as plt

#Math for rotation
def dist(a,b):
    return math.sqrt((a*a)+(b*b))
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z)) 
    return -math.degrees(radians)
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

#MPU functions
def mpu_activate(mux_addr):
    for mpu in range(MPUs):
        bus.write_byte_data(mux_addr, 0, mux_channels[mpu]) #correct - channel select
        bus.write_byte_data(mpu_address, mpu_power_mgmt, 0) # Now wake the 6050 up as it starts in sleep mode
        print('MPU ' + str(mpu) + ' activated')
def read_register_mpu(address, register): #считываются значения из регистров
    high = bus.read_byte_data(address, register)
    low = bus.read_byte_data(address, register+1)
    val = (high << 8) + low
    if (val >= 0x8000):  #преобразование значений, представленных в дополнительном коде
        return -((65535 - val) + 1)
    else:
        return val
def mpu_data_acquire(mpu_addr, mpu):
    bus.write_byte_data(mux_upp_address, 0, mux_channels[mpu]) #correct - channel select
    gyro_xout = read_register_mpu(mpu_addr, 0x43)
    gyro_yout = read_register_mpu(mpu_addr, 0x45)
    gyro_zout = read_register_mpu(mpu_addr, 0x47)
    gyro_xout_scaled = gyro_xout/131
    gyro_yout_scaled = gyro_yout/131
    gyro_zout_scaled = gyro_zout/131
    accel_xout = read_register_mpu(mpu_addr, 0x3b)
    accel_yout = read_register_mpu(mpu_addr, 0x3d)
    accel_zout = read_register_mpu(mpu_addr, 0x3f)
    accel_xout_scaled = accel_xout/16384.0
    accel_yout_scaled = accel_yout/16384.0
    accel_zout_scaled = accel_zout/16384.0
    return gyro_xout_scaled, gyro_yout_scaled, gyro_zout_scaled, accel_xout_scaled, accel_yout_scaled, accel_zout_scaled

#Final rotations
def integral_trap(x, xold, dt): #метод трапеции, реализовать дальше, для этого нужно создать
    #переменную для хранения предыдущего значения гироскопа
    int_trap = (x + xold)*dt/2
    return int_trap
def comp_filter_trap(cold, gs, gold, r, t):
    alpha = 0.05
    compf = (1-alpha) * (cold + integral_trap(gs,gold,t)) + alpha * (r)
    return compf
    
mpu_address = 0x68
mpu_power_mgmt = 0x6b

mux_upp_address = 0x70
#mux_low_address = 0x71
mux_channel_switch = 0x08

MPUs = 4 #Setting the number of used MPUs
mux_channels = [mux_channel_switch | 0, mux_channel_switch | 1, mux_channel_switch | 2, mux_channel_switch | 3,
                mux_channel_switch | 4, mux_channel_switch | 5, mux_channel_switch | 6, mux_channel_switch | 7] #possible to use 0x08 | channel number (MPU)

bus = smbus.SMBus(1) #bus = smbus.SMBus(1) for Revision 2 boards, (0) for Rev. 1
mpu_activate(mux_upp_address)    #activates the MPUs through multiplexer

now = 0.0 #if starting position is not defined
at = 0.0

gyroxold = [0,0,0,0,0,0,0,0]
gyroyold = [0,0,0,0,0,0,0,0]
gyrozold = [0,0,0,0,0,0,0,0]
compfxold = [0,0,0,0,0,0,0,0]
compfyold = [0,0,0,0,0,0,0,0]

gyrox = [0,0,0,0,0,0,0,0]
gyroy = [0,0,0,0,0,0,0,0]
gyroz = [0,0,0,0,0,0,0,0]
xrot = [0,0,0,0,0,0,0,0]
yrot = [0,0,0,0,0,0,0,0]

compfx_ac = [0,0,0,0,0,0,0,0]
compfy_ac = [0,0,0,0,0,0,0,0]
gyrointz_ac = [0,0,0,0,0,0,0,0]

compfx_0 = []
compfy_0 = []
gyrointz_0 = []

listcompfx = [] #Creating a list of lists. The amount of inserted lists is equal to the number of used MPUs.
listcompfy = []
listgyroz = []

for MPU in range(MPUs):
    listcompfx.append([])
    listcompfy.append([])
    listgyroz.append([])

list_at = []

#Asking the user if we shoul use the code to establish the starting position

start_position = bool(input('Input 1 when you are ready to calibrate sensors for starting position? [1/0]')) 

#Establishing the starting position

if start_position:
    print('Calibrating the starting position in 3 seconds. Stay still.')
    
    time.sleep(3)
    
    #Getting the priori state needed for integration and filtering
    
    for MPU in range(MPUs): #This code collects data from all the used MPUs
        result = mpu_data_acquire(mpu_address, MPU)
    
        gyroxold[MPU] = result[0]
        gyroyold[MPU] = result[1]
        gyrozold[MPU] = result[2]
        compfxold[MPU] = get_x_rotation(result[3], result[4], result[5])
        compfyold[MPU] = get_y_rotation(result[3], result[4], result[5])
    
    #Each integration uses dt to get the result
    #Also dt is used generally to compute time at for the plot
    #dt - is the time between two consecutive measurements from MPU
    
    #start always takes the time value at previous measurement
    #now takes the time value at the current measurement
        
    now = time.perf_counter()
    
    #A set of calculations to etablish the starting position
    
    start_iter = 0 #theoretically it should be possible to use as little as 1 iteration here
    while start_iter < 10:
        
        start = now
        
        for MPU in range(MPUs):
            result = mpu_data_acquire(mpu_address, MPU)
            
            gyrox[MPU] = result[0]
            gyroy[MPU] = result[1]
            gyroz[MPU] = result[2]
            xrot[MPU] = get_x_rotation(result[3], result[4], result[5])
            yrot[MPU] = get_y_rotation(result[3], result[4], result[5])
            
        now = time.perf_counter()
        dt = now - start
        
        for MPU in range(MPUs): #if count is reversed it should be possible to use .pop from the list
            
            #The value for each MPU from the last calculation is used to determine the starting points
            compfx_ac[MPU] = comp_filter_trap(compfxold[MPU], gyrox[MPU], gyroxold[MPU], xrot[MPU], dt)
            compfy_ac[MPU] = comp_filter_trap(compfyold[MPU], gyroy[MPU], gyroyold[MPU], yrot[MPU], dt)
            gyrointz_ac[MPU] = integral_trap(gyroz[MPU], gyrozold[MPU], dt)
            
            gyroxold[MPU] = result[0]
            gyroyold[MPU] = result[1]
            gyrozold[MPU] = result[2]
            compfxold[MPU] = compfx_ac[MPU]
            compfyold[MPU] = compfy_ac[MPU]
        
        start_iter += 1

    compfx_0.extend(compfx_ac) #The value for each MPU from the last calculation is used to determine the starting points
    compfy_0.extend(compfy_ac) #.extent adds the whole (list) to the current list
    gyrointz_0.extend(gyrointz_ac)
    
    for MPU in range(MPUs):
        listcompfx[MPU].append(0.0) #This expression gets zero value and is here just to mirror future calculations
        listcompfy[MPU].append(0.0)
        listgyroz[MPU].append(0.0)
    
    list_at.append(at)

plt.ion()
#rows = 1
rows = 2
columns = -(-MPUs//rows)
fig, ax = plt.subplots(rows, columns, sharex='col', sharey='row')

row = int(MPU//columns)
col = int(MPU%columns)

for MPU in range(MPUs):
#        ax.text(0.5, 0.5, 'MPU '+ str(MPU), fontsize=15, ha='center')
#        ax.plot(list_at, listcompfx[MPU], color="purple", label="X (CF)")
#        ax.plot(list_at, listcompfy[MPU], color="blue", label="Y (CF)")
#        ax.plot(list_at, listgyroz[MPU], color="black", label="Z (G)")
#        ax.legend(fontsize = 10, loc="lower left", frameon=True)
        ax[row,col].text(0.5, 0.5, 'MPU '+ str(MPU), fontsize=15, ha='center')
        ax[row,col].plot(list_at, listcompfx[MPU], color="purple", label="X (CF)")
        ax[row,col].plot(list_at, listcompfy[MPU], color="blue", label="Y (CF)")
        ax[row,col].plot(list_at, listgyroz[MPU], color="black", label="Z (G)")
        ax[row,col].legend(fontsize = 10, loc="lower left", frameon=True)

print('Start moving sensors.')

tmp = 1 #tmp = True
while tmp < 1000: #while tmp
        
    start = now
    
    for MPU in range(MPUs):
        result = mpu_data_acquire(mpu_address, MPU)
        
        gyrox[MPU] = result[0]
        gyroy[MPU] = result[1]
        gyroz[MPU] = result[2]
        xrot[MPU] = get_x_rotation(result[3], result[4], result[5])
        yrot[MPU] = get_y_rotation(result[3], result[4], result[5])
    
        now = time.perf_counter()
        dt = now - start
        at += dt
        
    for MPU in range(MPUs):
        compfx_ac[MPU] = comp_filter_trap(compfxold[MPU], gyrox[MPU], gyroxold[MPU], xrot[MPU], dt)
        compfy_ac[MPU] = comp_filter_trap(compfyold[MPU], gyroy[MPU], gyroyold[MPU], yrot[MPU], dt)
        gyrointz_ac[MPU] = integral_trap(gyroz[MPU], gyrozold[MPU], dt)
        
        gyroxold[MPU] = result[0]
        gyroyold[MPU] = result[1]
        gyrozold[MPU] = result[2]
        compfxold[MPU] = compfx_ac[MPU]
        compfyold[MPU] = compfy_ac[MPU]
        
        listcompfx[MPU].append(compfx_ac[MPU] - compfx_0[MPU]) #tmp*MPUs + MPU,
        listcompfy[MPU].append(compfy_ac[MPU] - compfy_0[MPU]) 
        listgyroz[MPU].append(gyrointz_ac[MPU] - gyrointz_0[MPU])
    
    list_at.append(at)
       
    for MPU in range(MPUs):
        #ax[MPU//columns,MPU%columns].axis(0,list_at(len(list_at)),-180,180)
        ax[MPU//columns,MPU%columns].plot(list_at, listcompfx[MPU], color="purple", label="X (CF)") #needs fixing as it won't work with 1 MPU
        ax[MPU//columns,MPU%columns].plot(list_at, listcompfy[MPU], color="blue", label="Y (CF)")
        ax[MPU//columns,MPU%columns].plot(list_at, listgyroz[MPU], color="black", label="Z (G)")
    
    plt.show()
    plt.pause(0.5)
    
    tmp += 1

print('Finished reading data.')

plt.savefig('testplot.png', bbox_inches = 'tight', pad_inches = 0.1)
#requires adjustment, now we are saving the file manually from the showed figure after adjusting it

print('Program has finished all calculations, close the plot Figure to stop the execution.')
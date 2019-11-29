#!/usr/bin/python

import smbus
import math
import csv
import time
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

compfxold1 = 0.0
compfyold1 = 0.0
compfxold2 = 0.0
compfyold2 = 0.0
compfxold3 = 0.0
compfyold3 = 0.0

f1 = open('mpu_1.csv','w')
f2 = open('mpu_2.csv','w')
f3 = open('mpu_diff.csv','w')
col1 = ['X rot 1','Y rot 1','X rot(CF) 1','Y rot(CF) 1']   #названия столбцов данных
col2 = ['X rot 2','Y rot 2','X rot(CF) 2','Y rot(CF) 2']
col3 = ['dX rot','dY rot','dX rot(CF)','dY rot(CF)']
csv.writer(f1).writerow(col1)
csv.writer(f2).writerow(col2)
csv.writer(f3).writerow(col3)

now1 = 0.0
now2 = 0.0
now3 = 0.0
at1 = 0.0
at2 = 0.0
at3 = 0.0

listxrot1 = []
listyrot1 = []
listxrotcf1 = []
listyrotcf1 = []

listxrot2 = []
listyrot2 = []
listxrotcf2 = []
listyrotcf2 = []

listxrot3 = []
listyrot3 = []
listxrotcf3 = []
listyrotcf3 = []

list_at1 = []
list_at2 = []
list_at3 = []

list_data1 = []
list_data2 = []
list_data3 = []

tmp = 1000
while tmp > 0:
    
    if tmp == 1000:
        start = time.perf_counter()
    else:
        start = now1
    
    result1 = mpu_data_acquire(address1)

    xrot1 = get_x_rotation(result1[3], result1[4], result1[5])
    yrot1 = get_y_rotation(result1[3], result1[4], result1[5])

    now1 = time.perf_counter()
    dt1 = now1 - start
    at1 += dt1
    
    compf1 = comp_filter(compfxold1, compfyold1, result1[0], result1[1], xrot1, yrot1, dt1)
    compfxold1 = compf1[0]
    compfyold1 = compf1[1]
    
    data1 = []
    data1.append(xrot1)
    data1.append(yrot1)
    data1.append(compf1[0])
    data1.append(compf1[1])
    
    list_data1.append(data1)
    
    listxrot1.append(xrot1)
    listyrot1.append(yrot1)
    listxrotcf1.append(compf1[0])
    listyrotcf1.append(compf1[1])
    list_at1.append(at1)
    
    if tmp == 1000:
        start = start
    else:
        start = now2
    
    result2 = mpu_data_acquire(address2)

    xrot2 = get_x_rotation(result2[3], result2[4], result2[5])
    yrot2 = get_y_rotation(result2[3], result2[4], result2[5])

    now2 = time.perf_counter()
    dt2 = now2 - start
    at2 += dt2
    
    
    compf2 = comp_filter(compfxold2, compfyold2, result2[0], result2[1], xrot2, yrot2, dt2)
    compfxold2 = compf2[0]
    compfyold2 = compf2[1]
    
    data2 = []
    data2.append(xrot2)
    data2.append(yrot2)
    data2.append(compf2[0])
    data2.append(compf2[1])
    
    list_data2.append(data2)
    
    listxrot2.append(xrot2)
    listyrot2.append(yrot2)
    listxrotcf2.append(compf2[0])
    listyrotcf2.append(compf2[1])
    list_at2.append(at2)
    
    if tmp == 1000:
        start = start
    else:
        start = now3
        
    gyro_result_x = result1[0] - result2[0]
    gyro_result_y = result1[1] - result2[1]
    
    xrot_diff = xrot1 - xrot2
    yrot_diff = yrot1 - yrot2
    
    now3 = time.perf_counter()
    dt3 = now3 - start
    at3 += dt3
    
    compf3 = comp_filter(compfxold3, compfyold3, gyro_result_x, gyro_result_y, xrot_diff, yrot_diff, dt3)
    compfxold3 = compf3[0]
    compfyold3 = compf3[1]
    
    data3 = []
    data3.append(xrot_diff)
    data3.append(yrot_diff)
    data3.append(compf3[0])
    data3.append(compf3[1])
    
    list_data3.append(data3)
    
    listxrot3.append(xrot_diff)
    listyrot3.append(yrot_diff)
    listxrotcf3.append(compf3[0])
    listyrotcf3.append(compf3[1])
    list_at3.append(at3)
    
    tmp -= 1

print('Finished reading data, now creating files.')

for row in list_data1:
    csv.writer(f1).writerow(row)
for row in list_data2:
    csv.writer(f2).writerow(row)
for row in list_data3:
    csv.writer(f3).writerow(row)

fig, (ax1, ax2, ax3) = plt.subplots(1, 3)

ax1.plot(list_at1, listxrot1, color="red", label="X rot")
ax1.plot(list_at1, listyrot1, color="blue", label="Y rot")
ax1.plot(list_at1, listxrotcf1, color="yellow", label="X rot (CF)")
ax1.plot(list_at1, listyrotcf1, color="black", label="Y rot (CF)")
ax1.legend(loc="lower left", title="Legend", frameon=True)

ax2.plot(list_at2, listxrot2, color="red", label="X rot")
ax2.plot(list_at2, listyrot2, color="blue", label="Y rot")
ax2.plot(list_at2, listxrotcf2, color="yellow", label="X rot (CF)")
ax2.plot(list_at2, listyrotcf2, color="black", label="Y rot (CF)")
ax2.legend(loc="lower left", title="Legend", frameon=True)

ax3.plot(list_at3, listxrot3, color="red", label="X rot")
ax3.plot(list_at3, listyrot3, color="blue", label="Y rot")
ax3.plot(list_at3, listxrotcf3, color="yellow", label="X rot (CF)")
ax3.plot(list_at3, listyrotcf3, color="black", label="Y rot (CF)")
ax3.legend(loc="lower left", title="Legend", frameon=True)

#plt.savefig('testplot.png', bbox_inches = 'tight', pad_inches = 0.1)
#requires adjustment, now we are saving the file manually from showed plot after adjusting it

f1.close()
f2.close()
f3.close()

print('Program has finished all calculations, close the plot Figure to stop the execution.')

plt.show()
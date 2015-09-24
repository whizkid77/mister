#!/usr/bin/python

#http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
#http://blog.bitify.co.uk/2013/11/using-complementary-filter-to-combine.html

import smbus
import math
import time
from time import sleep

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

gyro_scale = 131.0
accel_scale = 16384.0

address = 0x68  # This is the address value read via the i2cdetect command

def read_all(bus):
    raw_gyro_data = bus.read_i2c_block_data(address, 0x43, 6)
    raw_accel_data = bus.read_i2c_block_data(address, 0x3b, 6)

    gyro_scaled_x = twos_compliment((raw_gyro_data[0] << 8) + raw_gyro_data[1]) / gyro_scale
    gyro_scaled_y = twos_compliment((raw_gyro_data[2] << 8) + raw_gyro_data[3]) / gyro_scale
    gyro_scaled_z = twos_compliment((raw_gyro_data[4] << 8) + raw_gyro_data[5]) / gyro_scale
    
    accel_scaled_x = twos_compliment((raw_accel_data[0] << 8) + raw_accel_data[1]) / accel_scale
    accel_scaled_y = twos_compliment((raw_accel_data[2] << 8) + raw_accel_data[3]) / accel_scale
    accel_scaled_z = twos_compliment((raw_accel_data[4] << 8) + raw_accel_data[5]) / accel_scale

    return (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z)

def twos_compliment(val):
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
    
def dist(a, b):
    return math.sqrt((a * a) + (b * b))
    
    
def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)
    
def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def loop(bus,duration=60.0,callback=None,debug=False):
    now = time.time()

    K = 0.98
    K1 = 1 - K

    time_diff = 0
    loop_time = 0.025 # 40Hz

    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all(bus)

    last_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
    last_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)

    gyro_offset_x = gyro_scaled_x
    gyro_offset_y = gyro_scaled_y

    gyro_total_x = (last_x) - gyro_offset_x
    gyro_total_y = (last_y) - gyro_offset_y
    
    if debug:
        print "{0:.4f}\trotation_x: {1:.2f}\tgyro_total_x: {2:.2f}\tlast_x: {3:.2f}\trotation_y: {4:.2f}\tgyro_total_y: {5:.2f}\tlast_y: {6:.2f}".format( time.time() - now, (last_x), (gyro_total_x), (last_x), (last_y), (gyro_total_y), (last_y))
    if callback is not None:
        callback(loop_counter=0,rotation_x=last_x,rotation_y=last_y)

    loop_start = time.time()

    for i in xrange(0, int(duration / loop_time)):
        time_diff = time.time() - loop_start
        sleep_time = loop_time - time_diff
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            print "WARNING, looping too slow: " + repr(sleep_time)

        loop_start = time.time()

        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = read_all(bus)
    
        if debug:
            print "gyro_scaled_x: {0:.4f}\tgyro_scaled_y: {1:.4f}\tgyro_scaled_z: {2:.4f}\taccel_scaled_x: {3:.4f}\taccel_scaled_y: {4:.4f}\taccel_scaled_z: {5:.4f}\t".format(gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z)

        gyro_scaled_x -= gyro_offset_x
        gyro_scaled_y -= gyro_offset_y
            
        gyro_x_delta = (gyro_scaled_x * time_diff)
        gyro_y_delta = (gyro_scaled_y * time_diff)

        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta
    
        rotation_x = get_x_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)
        rotation_y = get_y_rotation(accel_scaled_x, accel_scaled_y, accel_scaled_z)

        last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)
        last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)

        if debug:
            print "{0:.4f}\trotation_x: {1:.2f}\tgyro_total_x: {2:.2f}\tlast_x: {3:.2f}\trotation_y: {4:.2f}\tgyro_total_y: {5:.2f}\tlast_y: {6:.2f}".format( time.time() - now, (rotation_x), (gyro_total_x), (last_x), (rotation_y), (gyro_total_y), (last_y))
            pass
        if callback is not None:
            callback(loop_counter=i,rotation_x=last_x,rotation_y=last_y)


def pp(loop_counter,rotation_x,rotation_y):
    print "{0:.0f} {1:.2f} {2:.2f}".format(loop_counter,rotation_x,rotation_y)

if __name__ == "__main__":
    bus = smbus.SMBus(0)  # or bus = smbus.SMBus(1) for Revision 2 boards

    # Now wake the 6050 up as it starts in sleep mode
    bus.write_byte_data(address, power_mgmt_1, 0)

    loop(bus,callback=None,debug=True,duration=120.0)

import tornado.ioloop
import tornado.web
import atexit
import signal
import RPi.GPIO as GPIO
import time
from time import sleep

import MPU6050

import smbus
import math

form = '''<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width;initial-scale=1.0;maximum-scale=1.0;">
<style>
html {width:320px}
body {width:320px}
input[type=submit] {
font-size:26px;
width:calc(100% - 20px);
padding:10px;
}
</style>
</head>
<body><form action="/f" method="post"><input type="submit" value="Forward"/></form><form action="/b" method="post"><input type="submit" value="Backward"/></form></body></html>
'''

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.write(form)

class ForwardHandler(tornado.web.RequestHandler):
    def post(self):

        print "On"
        GPIO.output(InAPin1,GPIO.LOW)
        GPIO.output(InBPin1,GPIO.HIGH)
        p1.start(duty)
        GPIO.output(InAPin2,GPIO.HIGH)
        GPIO.output(InBPin2,GPIO.LOW)
        p2.start(duty)

        print "Off"
        GPIO.output(InAPin1,GPIO.LOW)
        GPIO.output(InBPin1,GPIO.LOW)
        p1.stop()
        GPIO.output(InAPin2,GPIO.LOW)
        GPIO.output(InBPin2,GPIO.LOW)
        p2.stop()
        self.redirect('/')

class BackwardHandler(tornado.web.RequestHandler):
    def post(self):
        
        print "On"
        GPIO.output(InAPin1,GPIO.HIGH)
        GPIO.output(InBPin1,GPIO.LOW)
        p1.start(duty)
        GPIO.output(InAPin2,GPIO.LOW)
        GPIO.output(InBPin2,GPIO.HIGH)
        p2.start(duty)

        print "Off"
        GPIO.output(InAPin1,GPIO.LOW)
        GPIO.output(InBPin1,GPIO.LOW)
        p1.stop()
        GPIO.output(InAPin2,GPIO.LOW)
        GPIO.output(InBPin2,GPIO.LOW)
        p2.stop()
        self.redirect('/')

class QueryHandler(tornado.web.RequestHandler):
    def get(self):
        print "%s,%s" % (GPIO.input(SDAPin), GPIO.input(SCLPin))

application = tornado.web.Application([
        (r"/",MainHandler),
        (r"/f",ForwardHandler),
        (r"/b",BackwardHandler),
        (r"/q",QueryHandler),
])

def cleanup():
    print "Cleaning up"
    GPIO.cleanup()


# Motor encoders
def encoder_init():
    global EncoderPinA1,EncoderPinA2
    GPIO.add_event_detect(EncoderPinA1,GPIO.RISING)
    GPIO.add_event_callback(EncoderPinA1,encodeEventHandler)
    #GPIO.add_event_detect(EncoderPinA2,GPIO.RISING)
    #GPIO.add_event_callback(EncoderPinA2,encodeEventHandler)

def encodeEventHandler(pin):
    global left_encoder_count
    global right_encoder_count

    if pin == EncoderPinA1:
        if GPIO.input(EncoderPinB1) == 1:
            left_encoder_count = left_encoder_count + 1
        else:
            left_encoder_count = left_encoder_count - 1
    elif pin == EncoderPinA2:
        if GPIO.input(EncoderPinB2) == 1:
            right_encoder_count = right_encoder_count + 1
        else:
            right_encoder_count = right_encoder_count - 1

    #print "Encoder counts: %d, %d" % (left_encoder_count, right_encoder_count)

# MPU-6050
# http://blog.bitify.co.uk/2013/11/reading-data-from-mpu-6050-on-raspberry.html
def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

#convert two's complement
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

def get_pitch():
    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)
    
    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    return get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) + 90

def imu_test():

    print "gyro data"
    print "---------"

    gyro_xout = read_word_2c(0x43)
    gyro_yout = read_word_2c(0x45)
    gyro_zout = read_word_2c(0x47)
    
    print "gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131)
    print "gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131)
    print "gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131)
    
    print
    print "accelerometer data"
    print "------------------"
    
    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)
    
    accel_xout_scaled = accel_xout / 16384.0
    accel_yout_scaled = accel_yout / 16384.0
    accel_zout_scaled = accel_zout / 16384.0

    print "accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled
    print "accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled
    print "accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled

    print "x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    print "y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)


def balance(bus):
    global loop_start_time
    loop_start_time = time.time()
    MPU6050.loop(bus,duration=999999.0,callback=adjust_wheels)


def adjust_wheels(loop_counter,rotation_x,rotation_y):
    global last_wheel_position
    global stopped
    global pitch
    global left_encoder_count,right_encoder_count
    global wheel_velocity

    pitch = rotation_y + 90.0
    print 'pitch %f,%f' % (pitch,rotation_y)

    if pitch < 60 or pitch > 120:
        stop_and_reset()
    else:
        PID(target_angle,target_offset)

        if loop_counter % 10 == 0:
            wheel_position = left_encoder_count + right_encoder_count
            wheel_velocity = wheel_position - last_wheel_position
            last_wheel_position = wheel_position
            if abs(wheel_velocity) <= 20 and not stopped:
                target_position = wheel_position
                stopped = True

def PID(rest_angle,offset):
    global steer_forward
    global steer_backward
    global steer_stop
    global p_term,i_term,d_term
    global last_error
    global target_position
    global zone_a,zone_b,position_scale_a,position_scale_b,position_scale_c
    global velocity_scale_move,velocity_scale_stop
    global wheel_velocity

    steer_stop = False

    if steer_forward:
        offset = offset + wheel_velocity/velocity_scale_move
        rest_angle = rest_angle - offset
    elif steer_backward:
        offset = offset - wheel_velocity/velocity_scale_move
        rest_angle = rest_angle + offset
    elif steer_stop:
        position_error = wheel_position - target_position
        if abs(position_error) > zone_a:
            rest_angle = rest_angle - position_error/position_scale_a
        elif abs(position_error) > zone_b:
            rest_angle = rest_angle - position_error/position_scale_b
        else:
            rest_angle = rest_angle - position_error/position_scale_c

        rest_angle = rest_angle - wheel_velocity/velocity_scale_stop
        
        if rest_angle < 80:
            rest_angle = 80
        elif rest_angle > 100:
            rest_angle = 100

    error = (rest_angle - pitch)/100
    p_term = k_p * error
    i_term = i_term + k_i * error
    d_term = k_d * (error - last_error)
    last_error = error
    PIDValue = p_term + i_term + d_term

    if steer_left:
        PIDLeft = PIDValue - turn_speed
        PIDRight = PIDValue + turn_speed
    elif steer_rotate_left:
        PIDLeft = PIDValue - rotate_speed
        PIDRight = PIDValue + rotate_speed
    elif steer_right:
        PIDLeft = PIDValue + turn_speed
        PIDRight = PIDValue - turn_speed
    elif steer_rotate_right:
        PIDLeft = PIDValue + rotate_speed
        PIDRight = PIDValue - rotate_speed
    else:
        PIDLeft = PIDValue
        PIDRight = PIDValue
        
    print "%s : %s (%s %s %s) Error: %s" % (PIDLeft, PIDRight, p_term, i_term, d_term, error)

    if PIDLeft >= 0:
        move('left', 'forward', PIDLeft)
    else:
        move('left', 'backward', PIDLeft * -1)
    if PIDRight >= 0:
        move('right', 'forward', PIDRight)
    else:
        move('right', 'backward', PIDRight * -1)

def move(motor,direction,speed):
    if speed < 0:
        speed = 0
    if speed > 100:
        speed = 100    
    if motor == 'left':
        p1.start(speed)
        if direction == 'forward':
            GPIO.output(InAPin1,GPIO.HIGH)
            GPIO.output(InBPin1,GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(InAPin1,GPIO.LOW)
            GPIO.output(InBPin1,GPIO.HIGH)
    elif motor == 'right':
        p2.start(speed)
        if direction == 'forward':
            GPIO.output(InBPin2,GPIO.HIGH)
            GPIO.output(InAPin2,GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(InBPin2,GPIO.LOW)
            GPIO.output(InAPin2,GPIO.HIGH)
    elif motor == 'both':
        p1.start(speed)
        p2.start(speed)
        if direction == 'forward':
            GPIO.output(InAPin1,GPIO.HIGH)
            GPIO.output(InBPin1,GPIO.LOW)
            GPIO.output(InBPin2,GPIO.HIGH)
            GPIO.output(InAPin2,GPIO.LOW)
        elif direction == 'backward':
            GPIO.output(InAPin1,GPIO.LOW)
            GPIO.output(InBPin1,GPIO.HIGH)
            GPIO.output(InBPin2,GPIO.LOW)
            GPIO.output(InAPin2,GPIO.HIGH)
            

def stop_and_reset():
    global target_position
    stop('both')
    last_error = 0
    i_term = 0
    target_position = wheel_position

def stop(motor):
    if motor == 'left':
        p1.stop()
        GPIO.output(InAPin1,GPIO.LOW)
        GPIO.output(InBPin1,GPIO.LOW)
    elif motor == 'right':
        p2.stop()
        GPIO.output(InAPin2,GPIO.LOW)
        GPIO.output(InBPin2,GPIO.LOW)
    elif motor == 'both':
        p1.stop()
        GPIO.output(InAPin1,GPIO.LOW)
        GPIO.output(InBPin1,GPIO.LOW)
        p2.stop()
        GPIO.output(InAPin2,GPIO.LOW)
        GPIO.output(InBPin2,GPIO.LOW)



def kalman(newAngle, newRate, dtime):
    # KasBot V2  -  Kalman filter module
    # http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418
    # http://www.x-firm.com/?page_id=145
    # with slightly modifications by Kristian Lauszus
    # See http://academic.csuohio.edu/simond/courses/eec644/kalman.pdf and http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information
    dt = dtime / 1000000 # Convert from microseconds to seconds
    
    # Discrete Kalman filter time update equations - Time Update ("Predict")
    # Update xhat - Project the state ahead
    angle = angle + dt * (newRate - bias)
    
    # Update estimation error covariance - Project the error covariance ahead
    P_00 = P_00 + (-dt * (P_10 + P_01) + Q_angle * dt)
    P_01 = P_01 + (-dt * P_11)
    P_10 = P_10 + (-dt * P_11)
    P_11 = P_11 + (+Q_gyro * dt)
    
    # Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    # Calculate Kalman gain - Compute the Kalman gain
    S = P_00 + R_angle
    K_0 = P_00 / S
    K_1 = P_10 / S
    
    # Calculate angle and resting rate - Update estimate with measurement zk
    y = newAngle - angle
    angle = angle + (K_0 * y)
    bias = bias + (K_1 * y)
    
    # Calculate estimation error covariance - Update the error covariance
    P_00 = P_00 - (K_0 * P_00)
    P_01 = P_01 - (K_0 * P_01)
    P_10 = P_10 - (K_1 * P_00)
    P_11 = P_11 - (K_1 * P_01)
    
    return angle


if __name__ == "__main__":

    #PID values
    k_p = 900.0
    k_i = 200.0
    k_d = 300.0
    p_term = 0
    i_term = 0
    d_term = 0
    last_error = 0

    STD_LOOP_TIME = 0.01 # 10ms, 100Hz
    target_angle = 90.0
    target_offset = 0.0

    target_position = 0
    zone_a = 4000
    zone_b = 2000
    position_scale_a = 250
    position_scale_b = 500
    position_scale_c = 1000
    velocity_scale_move = 40
    velocity_scale_stop = 30

    steer_forward = False
    steer_backward = False
    steer_stop = False
    steer_left = False
    steer_rotate_left = False
    steer_right = False
    steer_rotate_right = False

    stopped = False
    wheel_position = 0
    last_wheel_position = 0
    left_encoder_count = 0
    right_encoder_count = 0

    duty = 40

    GPIO.setmode(GPIO.BOARD)

    #Motor 1
    InAPin1 = 8
    InBPin1 = 10
    PWMPin1 = 12
    EncoderPinA1 = 13
    EncoderPinB1 = 15

    #Motor 2
    PWMPin2 = 22
    InBPin2 = 24
    InAPin2 = 26
    EncoderPinA2 = 19
    EncoderPinB2 = 21

    #IMU
    power_mgmt_1 = 0x6b
    power_mgmt_2 = 0x6c
    bus = smbus.SMBus(0) # or bus = smbus.SMBus(1) for Revision 2 boards
    address = 0x68       # This is the address value read via the i2cdetect command
    bus.write_byte_data(address, power_mgmt_1, 0) # Now wake the 6050 up as it starts in sleep mode
    #for i in range(1,20):
    #    imu_test()
    #    sleep(0.1)


    GPIO.setup(PWMPin1,GPIO.OUT)
    GPIO.setup(InAPin1,GPIO.OUT)
    GPIO.setup(InBPin1,GPIO.OUT)
    #GPIO.setup(EnablePin,GPIO.OUT)
    GPIO.setup(PWMPin2,GPIO.OUT)
    GPIO.setup(InAPin2,GPIO.OUT)
    GPIO.setup(InBPin2,GPIO.OUT)
    p1 = GPIO.PWM(PWMPin1, 200)
    p2 = GPIO.PWM(PWMPin2, 200)

    GPIO.setup(EncoderPinA1,GPIO.IN)
    GPIO.setup(EncoderPinB1,GPIO.IN)
    GPIO.setup(EncoderPinA2,GPIO.IN)
    GPIO.setup(EncoderPinB2,GPIO.IN)
    #GPIO.setup(SDAPin,GPIO.IN)
    #GPIO.setup(SCLPin,GPIO.IN)

    signal.signal(signal.SIGTERM,cleanup)
    signal.signal(signal.SIGHUP,cleanup)
    atexit.register(cleanup)

    encoder_init()

    balance(bus)
    #move('both','forward',50)

    port = 8080
    application.listen(port)
    print "Listening on port %d" % port
    tornado.ioloop.IOLoop.current().start()

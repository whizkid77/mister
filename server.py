
# usage:
# sudo python server.py 1000 0 -100
# sudo python server.py calibrate

import os, sys, atexit, signal, time, smbus, math, json
import tornado.ioloop
import tornado.web
import RPi.GPIO as GPIO
from time import sleep

import MPU6050

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

RAD_TO_DEG = 57.295779513082320876798154814105

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
    GPIO.add_event_detect(EncoderPinA2,GPIO.RISING)
    GPIO.add_event_callback(EncoderPinA2,encodeEventHandler)

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
            right_encoder_count = right_encoder_count - 1
        else:
            right_encoder_count = right_encoder_count + 1

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

'''
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
'''

def balance(bus):
    MPU6050.loop(bus,duration=999999.0,callback=adjust_wheels)

def adjust_wheels(loop_counter,rotation_x,rotation_y):
    global bus
    global wheel_position, last_wheel_position, wheel_velocity
    global stopped
    global pitch
    global left_encoder_count,right_encoder_count

    pitch = rotation_y

    print 'pitch %f == %f' % (pitch, target_angle)

    if pitch < -20 or pitch > 20:
        stop_and_reset()
    else:
        PID(target_angle,target_offset)

        if loop_counter % 10 == 0:
            wheel_position = left_encoder_count + right_encoder_count
            print "wheel position " + repr(wheel_position)
            wheel_velocity = wheel_position - last_wheel_position
            last_wheel_position = wheel_position
            if abs(wheel_velocity) <= 20 and not stopped:
                target_position = wheel_position
                print "New target position: " + repr(wheel_position)
                stopped = True

def PID(rest_angle,offset):
    global steer_forward
    global steer_backward
    global steer_stop
    global p_term,i_term,d_term
    global last_error
    global zone_a,zone_b,position_scale_a,position_scale_b,position_scale_c
    global velocity_scale_move,velocity_scale_stop
    global target_position, wheel_position, wheel_velocity

    if steer_forward:
        offset = offset + wheel_velocity/velocity_scale_move
        rest_angle = rest_angle - offset
    elif steer_backward:
        offset = offset - wheel_velocity/velocity_scale_move
        rest_angle = rest_angle + offset
    elif steer_stop:
        
        #try to stay in same place
        position_error = (wheel_position - target_position)*2
        print "position_error: " + repr(position_error)
        print "old rest angle: " + repr(rest_angle)
        if abs(position_error) > zone_a:
            rest_angle = rest_angle - position_error/position_scale_a
        elif abs(position_error) > zone_b:
            rest_angle = rest_angle - position_error/position_scale_b
        else:
            rest_angle = rest_angle - position_error/position_scale_c
        print "new rest angle: " + repr(rest_angle) + " : " + repr(wheel_velocity)

        #rest_angle = rest_angle - wheel_velocity/velocity_scale_stop
        print "final rest angle: " + repr(rest_angle)
        #end try to stay in same place        

        max_target_angle = 5
        if rest_angle < -1 * max_target_angle:
            rest_angle = -1 * max_target_angle
        elif rest_angle > max_target_angle:
            rest_angle = max_target_angle

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
        
    print "%s (%s %s %s) e: %.3f" % (PIDValue, p_term, i_term, d_term, error)

    if PIDLeft >= 0:
        move('left', 'forward', PIDLeft)
    else:
        move('left', 'backward', PIDLeft * -1)
    if PIDRight >= 0:
        move('right', 'forward', PIDRight)
    else:
        move('right', 'backward', PIDRight * -1)


def stop_and_reset():
    global target_position, wheel_position
    stop('both')
    last_error = 0
    i_term = 0
    target_position = wheel_position



def kalman(newAngle, newRate, dtime):
    global angle, Q_angle, Q_gyro, R_angle, bias, P_00, P_01, P_10, P_11

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

# degrees per second from gyro
def get_gyro_y_rate(bus):
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = MPU6050.read_all(bus)
    gyro_rate = -1 * (gyro_scaled_y - zero_values['gyro_y'])
    return gyro_rate

# Y angle/pitch from accelerometer
def get_acc_y(bus):
    (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = MPU6050.read_all(bus)
    
    return (accel_scaled_x - zero_values['acc_y']) * 90 + 90

    acc_x_val = accel_scaled_x - zero_values['acc_x']
    acc_y_val = accel_scaled_y - zero_values['acc_y']
    #acc_y_val = acc_y_val - 1
    acc_z_val = accel_scaled_z - zero_values['acc_z']
    
    #print "\t%s,%s,%s" % (acc_x_val,acc_y_val,acc_z_val)

    R = math.sqrt(math.pow(acc_x_val,2) + math.pow(acc_y_val,2) + math.pow(acc_z_val,2))
    angle_y = math.acos(acc_y_val / R) * RAD_TO_DEG

    return angle_y

def calibrate_sensors(bus):
    global calibration
    calibration = {'pitch':0,'sample_count':0}
    print "Calibrating..."
    MPU6050.loop(bus,duration=3.0,callback=calibrate)
    print "Done."
    return calibration['pitch'] / calibration['sample_count']
    '''
    global zero_values
    sample_count = 100
    gyro_y = 0
    acc_x = 0
    acc_y = 0
    acc_z = 0
    for i in xrange(sample_count):
        (gyro_scaled_x, gyro_scaled_y, gyro_scaled_z, accel_scaled_x, accel_scaled_y, accel_scaled_z) = MPU6050.read_all(bus)
        gyro_y = gyro_y + gyro_scaled_y
        acc_x = acc_x + accel_scaled_x
        acc_y = acc_y + accel_scaled_y
        acc_z = acc_z + accel_scaled_z
        sleep(0.01)
    zero_values['gyro_y'] = gyro_y / sample_count
    zero_values['acc_x'] = acc_x / sample_count
    zero_values['acc_y'] = acc_y / sample_count
    zero_values['acc_z'] = acc_z / sample_count
    '''

def calibrate(loop_counter,rotation_x,rotation_y):
    global calibration
    calibration['pitch'] = calibration['pitch'] + rotation_y
    calibration['sample_count'] = calibration['sample_count'] + 1

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


if __name__ == "__main__":

    zero_values = {'gyro_y':None,
                   'acc_x':None,
                   'acc_y':None,
                   'acc_z':None
                   }
    
    Q_angle = 0.001
    Q_gyro = 0.003
    R_angle = 0.03
    angle = 180
    bias = 0
    P_00 = 0
    P_01 = 0
    P_10 = 0
    P_11 = 0
    
    #PID values
    try:
        if len(sys.argv) >= 2:
            k_p = float(sys.argv[1])
        else:
            k_p = 0.0
        if len(sys.argv) >= 3:
            k_i = float(sys.argv[2])
        else:
            k_i = 0.0
        if len(sys.argv) >= 4:
            k_d = float(sys.argv[3])
        else:
            k_d = 0.0
    except ValueError:
        pass

    p_term = 0
    i_term = 0
    d_term = 0
    last_error = 0

    STD_LOOP_TIME = 0.01 # 10ms, 100Hz

    steer_forward = False
    steer_backward = False
    steer_stop = True
    steer_left = False
    steer_rotate_left = False
    steer_right = False
    steer_rotate_right = False

    stopped = False

    turn_speed = 0.1
    rotate_speed = 0.2

    target_offset = 0.0

    wheel_position = 0
    last_wheel_position = 0
    wheel_velocity = 0
    target_position = 0
    zone_a = 4000
    zone_b = 2000
    position_scale_a = 250.0 * 2
    position_scale_b = 500.0 * 2
    position_scale_c = 1000.0 * 2
    velocity_scale_move = 40.0
    velocity_scale_stop = 30.0

    #mine
    left_encoder_count = 0
    right_encoder_count = 0
    target_angle = 0
    duty = 40
    #end mine

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

    if os.path.exists('config'):
        f = open('config', 'r+')
        try:
            config = json.loads(f.read())
        except:
            config = {}            
        f.close()
    else:
        config = {}

    if len(sys.argv) > 1 and sys.argv[1] == 'calibrate':
        target_angle = calibrate_sensors(bus)
        config['target_angle'] = target_angle
        if os.path.exists('config'):
            f = open('config', 'r+')
        else:
            f = open('config', 'w+')
        print config
        f.write(json.dumps(config))
        f.close()
    elif len(sys.argv) > 1 and sys.argv[1] == 'web':
        port = 8080
        application.listen(port)
        print "Listening on port %d" % port
        tornado.ioloop.IOLoop.current().start()

    else:    
        target_angle = config['target_angle']
        print 'target_angle: ' + repr(target_angle)
        balance(bus)
        
    #move('both','forward',50)
    

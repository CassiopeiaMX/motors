#!/usr/bin/env python
import rospy
from motors_node import Vector2D
import RPi.GPIO as gpio

FREQ = 0.5
RMTR1 = 16
RMTR2 = 26
RMTRE = 13
LMTR1 = 6
LMTR2 = 5
LMTRE = 12
re = None
le = None

def abs(n):
    if(n < 0):
        n = n * -1
    return n

def sign(n):
    if(n < 0):
        return -1
    else:
        return 1

def callback(msg):
    r = msg.y * (1 - msg.x)
    l = msg.y * (1 + msg.x)
    if(abs(r) > 1) r = sign(r)
    if(abs(l) > 1) l = sign(l)
    if(r < 0):
        gpio.output(RMTR1, gpio.LOW)
        gpio.output(RMTR2, gpio.HIGH)
    else:
        gpio.output(RMTR1, gpio.HIGH)
        gpio.output(RMTR2, gpio.LOW)
    if(r < 0):
        gpio.output(LMTR1, gpio.LOW)
        gpio.output(LMTR2, gpio.HIGH)
    else:
        gpio.output(LMTR1, gpio.HIGH)
        gpio.output(LMTR2, gpio.LOW)
    r = abs(r)
    l = abs(l)
    re.start(r * 100)
    le.start(l * 100)
    pass
    
def listener():
    global re
    global le
    rospy.init_node('motors_node', anonymous=False)
    rospy.Subscriber("mtr_ctrl", Vector2D, callback)
    gpio.setmode(gpio.BCM)
    gpio.setup(RMTR1, gpio.OUT)
    gpio.setup(RMTR2, gpio.OUT)
    gpio.setup(RMTRE, gpio.OUT)
    gpio.setup(LMTR1, gpio.OUT)
    gpio.setup(LMTR2, gpio.OUT)
    gpio.setup(LMTRE, gpio.OUT)
    re = gpio.PWM(RMTRE, FREQ)
    le = gpio.PWM(LMTRE, FREQ)
    rospy.spin()
    gpio.cleanup()

if __name__ == '__main__':
    listener()
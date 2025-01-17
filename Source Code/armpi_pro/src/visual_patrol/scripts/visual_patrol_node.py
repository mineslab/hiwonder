#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30
import sys
import cv2
import time
import math
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from chassis_control.msg import *
from visual_patrol.srv import SetTarget
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform

# Line following

lock = RLock()
ik = ik_transform.ArmIK()

move = False
target_en = False
target_color = 'None'
__isRunning = False

img_w = 640
x_pid = PID.PID(P=0.003, I=0.0001, D=0.0001)  # pid initialization

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'yellow': (0, 255, 255),
    'white': (255, 255, 255),
}

# Initial position
def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    if delay:
        rospy.sleep(2)

def off_rgb():
    global rgb_pub
    
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)

# reset variable
def reset():
    global move
    global target_en
    global target_color
    
    with lock:
        move = False
        target_en = False
        x_pid.clear()
        off_rgb()
        target_color = 'None'

# app initialization call
def init():
    rospy.loginfo("visual patrol Init")
    initMove()
    reset()

# image process result callback function
def run(msg):
    global move
    
    center_x = msg.center_x
    center_y = msg.center_y
    width = msg.data
    
    if __isRunning:
        if width > 0:
            # PID algorithm line following
            if abs(center_x - img_w/2) < 20:
                center_x = img_w/2
            x_pid.SetPoint = img_w/2      # set 
            x_pid.update(center_x)        # current
            dx = round(x_pid.output, 2)   # output
            dx = 0.8 if dx > 0.8 else dx
            dx = -0.8 if dx < -0.8 else dx
            set_velocity.publish(100, 90, dx)
            move = True
                
        else:
            if move:
                move = False
                set_velocity.publish(0, 90, 0)
        

result_sub = None
heartbeat_timer = None
# APP enter service callback function
def enter_func(msg):
    global lock
    global result_sub
    
    rospy.loginfo("enter visual patrol")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run)
            
    return [True, 'enter']

# APP exit service callback function
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer
    
    rospy.loginfo("exit visual patrol")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning = False
        reset()
        set_velocity.publish(0, 90, 0)
        try:
            if result_sub is not None:
                result_sub.unregister()
                result_sub = None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel()
                heartbeat_timer = None
        except BaseException as e:
            rospy.loginfo('%s', e)
        
    return [True, 'exit']

# start running function
def start_running():
    global lock
    global __isRunning
    
    rospy.loginfo("start running visual patrol")
    with lock:
        __isRunning = True

# stopping running function
def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running visual patrol")
    with lock:
        reset()
        __isRunning = False
        initMove(delay=False)
        set_velocity.publish(0,90,0)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()

# APP set_running service callback function
def set_running(msg):
    
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

# APP set_target service callback function
def set_target(msg):
    global lock
    global target_en
    global target_color
    
    rospy.loginfo("%s", msg)
    with lock:
        target_color = msg.data
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[target_color][2]
        led.rgb.g = range_rgb[target_color][1]
        led.rgb.b = range_rgb[target_color][0]
        rgb_pub.publish(led)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.1)
        visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
        visual_running('line',target_color)
        rospy.sleep(0.1)
        target_en = True
        
    return [True, 'set_target']

# APP heartbeat service callback function
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/visual_patrol/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('visual_patrol', log_level=rospy.DEBUG)
    # servo publish
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # app communication service
    enter_srv = rospy.Service('/visual_patrol/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/visual_patrol/exit', Trigger, exit_func)
    running_srv = rospy.Service('/visual_patrol/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/visual_patrol/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service('/visual_patrol/heartbeat', SetBool, heartbeat_srv_cb)
    # chassis car control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    set_translation = rospy.Publisher('/chassis_control/set_translation', SetTranslation, queue_size=1)
    # buzzer 
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    # rgb light
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    rospy.sleep(0.5) 
    
    initMove()
    buzzer_pub.publish(0.2)

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        
        msg = SetTarget()
        msg.data = 'red'
        
        set_target(msg)
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

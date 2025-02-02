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
from object_tracking.srv import SetTarget
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from chassis_control.msg import SetTranslation, SetVelocity

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform

# Target tracking 

lock = RLock()
ik = ik_transform.ArmIK()

move = False
__isRunning = False
target_color = 'None'

img_w = 640
img_h = 480
Arm_X = 500
Arm_Y = 100
arm_x = Arm_X
arm_y = Arm_Y
x_pid = PID.PID(P=0.70, I=0.0001, D=0.0001) 
y_pid = PID.PID(P=1.30, I=0.0005, D=0.0001)
arm_x_pid = PID.PID(P=0.1, I=0.0001, D=0.0001)  # pid initialization
arm_y_pid = PID.PID(P=0.18, I=0.0005, D=0.0001)

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
        target = ik.setPitchRanges((0.0, 0.12, 0.08), -145, -180, 0)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, arm_y),
                                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, arm_x)))
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
    global target_color
    global arm_x, arm_y
    
    with lock:
        arm_x = Arm_X
        arm_y = Arm_Y
        x_pid.clear()
        y_pid.clear()
        arm_x_pid.clear()
        arm_y_pid.clear()
        off_rgb()
        target_color = 'None'

# app initialization call
def init():
    rospy.loginfo("object tracking Init")
    initMove()
    reset()

# image process result callback function
def run(msg):
    global lock
    global move
    global arm_x, arm_y
    
    center_x = msg.center_x
    center_y = msg.center_y
    radius = msg.data
    
    with lock:
        if __isRunning:
            if center_x > 0 and center_y > 0:
                # robotic arm x-axis tracking
                if abs(center_x - img_w/2.0) < 15:
                    center_x = img_w/2.0
                arm_x_pid.SetPoint = img_w/2.0  # set 
                arm_x_pid.update(center_x)      # current 
                arm_x += arm_x_pid.output       # output
                arm_x = 200 if arm_x < 200 else arm_x
                arm_x = 800 if arm_x > 800 else arm_x
                
                # robotic arm y-axis tracking
                if abs(center_y - img_h/2.0) < 15:
                    center_y = img_h/2.0
                arm_y_pid.SetPoint = img_h/2.0  # set
                arm_y_pid.update(center_y)      # current 
                arm_y += arm_y_pid.output       # output
                arm_y = 50 if arm_y < 50 else arm_y
                arm_y = 300 if arm_y > 300 else arm_y
                
                # robotic arm movement
                bus_servo_control.set_servos(joints_pub, 20, ((3, arm_y), (6, arm_x)))
                
                # chassis car x-axis tracking
                if abs(arm_x - Arm_X) < 5:
                    arm_x = Arm_X
                x_pid.SetPoint = Arm_X       # set
                x_pid.update(arm_x)          # current 
                dx = x_pid.output    # output
                dx = -200 if dx < -200 else dx
                dx = 200 if dx > 200 else dx
                
                # chassis car y-axis tracking
                if abs(arm_y - Arm_Y) < 5:
                    arm_y = Arm_Y
                y_pid.SetPoint = Arm_Y   # set
                y_pid.update(arm_y)      # current 
                dy = -y_pid.output    # output
                dy = -180 if dy < -180 else dy
                dy = 180 if dy > 180 else dy
                
                # chassis car movement
                set_translation.publish(dx,dy)
                move = True
                        
            else:
                if move:
                    move = False
                    rospy.sleep(0.1)
                    set_translation.publish(0,0)
             

result_sub = None
heartbeat_timer = None
# enter service callback function
def enter_func(msg):
    global lock
    global result_sub
    
    rospy.loginfo("enter object tracking")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run)
            
    return [True, 'enter']

# enter service callback function
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer
    
    rospy.loginfo("exit object tracking")
    with lock:
        __isRunning = False
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        reset()
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
    
    rospy.loginfo("start running object tracking")
    with lock:
        __isRunning = True

# stop running function
def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running object tracking")
    with lock:
        reset()
        __isRunning = False
        initMove(delay=False)
        set_velocity.publish(0,0,0)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()

# set_running service callback function
def set_running(msg):
    
    if msg.data:
        start_running()
    else:
        stop_running()
        
    return [True, 'set_running']

# set_target service callback function
def set_target(msg):
    global lock
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
        visual_running('color',target_color)
        rospy.sleep(0.1)
        
    return [True, 'set_target']

# heartbeat service callback function
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/object_tracking/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('object_tracking', log_level=rospy.DEBUG)
    # servo publish
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # app communication service
    enter_srv = rospy.Service('/object_tracking/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/object_tracking/exit', Trigger, exit_func)
    running_srv = rospy.Service('/object_tracking/set_running', SetBool, set_running)
    set_target_srv = rospy.Service('/object_tracking/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service('/object_tracking/heartbeat', SetBool, heartbeat_srv_cb)
    # chassis car control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    set_translation = rospy.Publisher('/chassis_control/set_translation', SetTranslation, queue_size=1)
    # buzzer 
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    # rgb light
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    rospy.sleep(0.5) 

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

#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30
import sys
import cv2
import math
import time
import rospy
import numpy as np
from threading import RLock, Timer, Thread

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from chassis_control.msg import SetTranslation, SetVelocity

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform

# Tag recognition

lock = RLock()
ik = ik_transform.ArmIK()

img_w = 640
img_h = 480
move_en = False
__isRunning = False
detect_id = 'None'

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# Initial position
def initMove(delay=True):
    with lock:
        target = ik.setPitchRanges((0, 0.167, 0.2), -90, -92, -88) # get the solution through reverse kinematics
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']),(6, servo_data['servo6'])))
    if delay:
        rospy.sleep(2)

def turn_off_rgb():
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)

# Reset variables 
def reset():
    global move_en
    global detect_id

    with lock:
        turn_off_rgb()
        move_en = False
        detect_id = 'None'

# Initialization
def init():
    rospy.loginfo("apriltag detect Init")
    initMove()
    reset()


# Movement control function
def move():
    global move_en
    global detect_id

    while __isRunning:
        if move_en and detect_id != 'None': # Movement enable and detect tag
            rospy.sleep(0.5)
            if detect_id == 1: # The ID of tag is 1 and the robot draw a tirangle.
                set_velocity.publish(100,60,0) # Publish the chassis control message. 80 is the linear velocity ranging 0-200; 45 is the directional angle ranging 0-360; 0 is the yaw rate ranging -1-1. 
                rospy.sleep(2.6)
                set_velocity.publish(100,180,0)
                rospy.sleep(2.6)
                set_velocity.publish(100,300,0)
                rospy.sleep(2.6)

            elif detect_id == 2: # The ID of tag is 2 and the robot draw a circle.
                 for i in range(360):
                    set_velocity.publish(100,i,0)
                    rospy.sleep(0.02)

            elif detect_id == 3: # The ID of tag is 3 and the robot will drift.
                set_velocity.publish(100,180,-0.45)
                rospy.sleep(10.2)

            move_en = False
            detect_id = 'None'
            set_velocity.publish(0,90,0) # Stop moving 

        else:
            rospy.sleep(0.01)


# Image process result callback function
def run(msg):
    global move_en
    global detect_id

    # Get the image process result
    center_x = msg.center_x  # The x coordniate of the centre 
    center_y = msg.center_y  # The y coordiate of the centre
    id = msg.data   # The id of tag

    if not move_en and id != 0:
        move_en = True
        detect_id = id
        buzzer_pub.publish(0.1)  # Publish the message of buzzer node and control the sound of buzzer for 0.1s.


result_sub = None
heartbeat_timer = None
# APP enter service callback function
def enter_func(msg):
    global lock
    global result_sub

    rospy.loginfo("enter apriltag detect")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run) # subscribe to the image process for the node result

    return [True, 'enter']

# APP exit service callback funciton
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer

    rospy.loginfo("exit apriltag detect")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning = False
        reset()
        try:
            if result_sub is not None:
                result_sub.unregister() # Unregister the subsribe of image process result
                result_sub = None
            if heartbeat_timer is not None:
                heartbeat_timer.cancel() # Cancel the heartbeat package service 
                heartbeat_timer = None
        except BaseException as e:
            rospy.loginfo('%s', e)

    return [True, 'exit']

# Start running function
def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running apriltag detect")
    with lock:
        __isRunning = True
        visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
        visual_running('apriltag','')
        rospy.sleep(0.1)
        # Run the child thread
        th = Thread(target=move)
        th.setDaemon(True)
        th.start()

# Stop running function
def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running apriltag detect")
    with lock:
        reset()
        __isRunning = False
        initMove(delay=False)
        rospy.ServiceProxy('/visual_processing/set_running', SetParam)()

# APP running service callback function
def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()

    return [True, 'set_running']

# Set the RGB light of the expansion board
def set_LED(color):
    global lock
    with lock:
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[color][2]
        led.rgb.g = range_rgb[color][1]
        led.rgb.b = range_rgb[color][0]
        rgb_pub.publish(led)
        rospy.sleep(0.05)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.05)

# APP heartbear service callback function
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/apriltag_detect/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # Initialize node 
    rospy.init_node('apriltag_detect', log_level=rospy.DEBUG)
    # Service publish
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # app communication service
    enter_srv = rospy.Service('/apriltag_detect/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/apriltag_detect/exit', Trigger, exit_func)
    running_srv = rospy.Service('/apriltag_detect/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/apriltag_detect/heartbeat', SetBool, heartbeat_srv_cb)
    # mecanum chassis control
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
        start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

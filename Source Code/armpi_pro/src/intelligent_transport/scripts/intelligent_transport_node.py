#!/usr/bin/python3
# coding=utf8
# Date:2022/05/30
import sys
import cv2
import time
import math
import rospy
import numpy as np
from threading import RLock, Timer, Thread

from std_srvs.srv import *
from std_msgs.msg import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from chassis_control.msg import *
from visual_processing.msg import Result
from visual_processing.srv import SetParam
from intelligent_transport.srv import SetTarget
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from armpi_pro import PID
from armpi_pro import Misc
from armpi_pro import bus_servo_control
from kinematics import ik_transform

# Intelligent transportation

lock = RLock()
ik = ik_transform.ArmIK()

set_visual = 'line'   
detect_step = 'color' # STEP:follow line or detect block
line_color = 'yellow' # line color
stable = False        # block gripping determination variable
place_en = False      # block placement dertermination placement 
position_en = False   # Positioning determination variable before gripping bloack
__isRunning = False   # Game Control running variables
block_clamp = False   # Transport blaock tag variables
chassis_move = False  # Chassis movement tag variables

x_dis = 500
y_dis = 0.15
line_width = 0
line_center_x = 0
line_center_y = 0
color_centreX = 320
color_centreY = 410
color_center_x = 0
color_center_y = 0
detect_color = 'None'  
target_color = 'None'

img_h, img_w = 480, 640

line_x_pid = PID.PID(P=0.002, I=0.001, D=0)  # pid initialization
color_x_pid = PID.PID(P=0.06, I=0, D=0) 
color_y_pid = PID.PID(P=0.00003, I=0, D=0)

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

# Turn off RGB light
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


# Reset variable 
def reset():
    global x_dis,y_dis
    global position_en,stable
    global set_visual,detect_step,place_en
    global block_clamp,chassis_move,target_color
    global line_width,line_center_x,line_center_y
    global detect_color,color_center_x,color_center_y
    
    with lock:
        line_x_pid.clear()
        color_x_pid.clear()
        color_y_pid.clear()
        off_rgb()
        set_visual = 'line'
        detect_step = 'color'
        stable = False
        place_en = False
        position_en = False
        block_clamp = False
        chassis_move = False
        x_dis = 500
        y_dis = 0.15
        line_width = 0
        line_center_x = 0
        line_center_y = 0
        color_center_x = 0
        color_center_y = 0
        detect_color = 'None'
        target_color = 'None'
        set_velocity.publish(0, 90, 0)
        

# Initialization calling
def init():
    rospy.loginfo("intelligent transport Init")
    initMove()
    reset()


n = 0
last_x = 0
last_y = 0
# Image processing result callback function
def run(msg):
    global lock,n,last_x,last_y,position_en
    global line_width,line_center_x,line_center_y
    global detect_color,color_center_x,color_center_y
    
    data = int(msg.data)
    center_x = msg.center_x
    center_y = msg.center_y
    color_list = { 0:'None', 1:'red', 2:'green', 3:'blue'}
    
    with lock:
        # Update the position parameter of line or block
        if detect_step == 'line':
            line_center_x = center_x
            line_center_y = center_y
            line_width = data
            
            color_center_x = 0
            color_center_y = 0
            detect_color = color_list[0]
                    
        elif detect_step == 'color':
            color_center_x = center_x
            color_center_y = center_y
            data = 0 if data < 0 else data
            data = 0 if data > 3 else data
            detect_color = color_list[data]
            
            if not position_en: # determine whether the block is stable
                dx = abs(color_center_x - last_x)
                dy = abs(color_center_y - last_y)
                last_x = color_center_x
                last_y = color_center_y
                if dx < 3 and dy < 3:
                    n += 1
                    if n == 10:
                        n = 0
                        position_en = True  # stable
                else:
                    n = 0
            
            line_center_x = 0
            line_center_y = 0
            line_width = 0
            
# Robot movement function
def move():
    global x_dis,y_dis
    global position_en,stable
    global set_visual,detect_step,place_en
    global block_clamp,chassis_move,target_color
    global line_width,line_center_x,line_center_y
    global detect_color,color_center_x,color_center_y
    
    num = 0
    transversae_num = 0
    move_time = time.time()
    place_delay = time.time()
    transversae_time = time.time()
    position = {'red':1, 'green':2, 'blue':3, 'None':-1} # The number of lines corresponding to the block position
    
    while __isRunning:
        if detect_step == 'line': # Line following step
            if set_visual == 'color': 
                set_visual = 'line'
                place_en = False
                visual_running('line', line_color) # switch image processing type 
                # switch the posture of robotic arm
                bus_servo_control.set_servos(joints_pub, 1500, ((1, 500), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
                rospy.sleep(1.5)
                
            elif line_width > 0: #the line is recognized
                # PID algorithm line following
                if abs(line_center_x - img_w/2) < 30:
                    line_center_x = img_w/2
                line_x_pid.SetPoint = img_w/2      # set
                line_x_pid.update(line_center_x)   # current 
                dx = round(line_x_pid.output, 2)   # output
                dx = 0.8 if dx > 0.8 else dx
                dx = -0.8 if dx < -0.8 else dx
                
                set_velocity.publish(100, 90, dx) # control  chassis
                chassis_move = True
                
                if not place_en:
                    if line_width > 100 and block_clamp:  # detect lines when gripping the block
                        if (time.time()-transversae_time) > 1:
                            transversae_num += 1
                            print(transversae_num)
                            transversae_time = time.time()
                    
                        if transversae_num == position[target_color]: # detect whether the number of line numbers is equal to the number of corresponding target
                            place_en = True  # placement enable
                            if transversae_num == 1:
                                place_delay = time.time() + 1.1 # set the delay time
                            elif transversae_num == 2:
                                place_delay = time.time() + 1.1
                            elif transversae_num == 3:
                                place_delay = time.time() + 1.2
                            
                elif place_en:
                    if time.time() >= place_delay: # Stop delay, place the block beside the line
                        rospy.sleep(0.1)
                        set_velocity.publish(0, 0, 0)
                        target = ik.setPitchRanges((-0.24, 0.00, -0.04), -180, -180, 0) #robotic arm moves to the block placement position
                        if target:
                            servo_data = target[1]
                            bus_servo_control.set_servos(joints_pub, 1200, ((6, servo_data['servo6']),)) 
                            rospy.sleep(1)
                            bus_servo_control.set_servos(joints_pub, 1500, ((3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                        rospy.sleep(1.8)

                        bus_servo_control.set_servos(joints_pub, 500, ((1, 150),))  #open the gripper
                        rospy.sleep(0.8)
                        
                        #robotic arm returns to the initial posture
                        bus_servo_control.set_servos(joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625)))
                        rospy.sleep(1.5)
                        bus_servo_control.set_servos(joints_pub, 1500, ((6, 500),))
                        rospy.sleep(1.5)
                        
                        move_time = time.time() + (11.5 - transversae_num) # set the time of line following before placing the block and get the robot back to the reset position.
                            
                        # reset variables
                        place_en = False
                        block_clamp = False
                        target_color = 'None'
                        set_rgb('black')
                        transversae_num = 0
                
                if not block_clamp and time.time() >= move_time: # The robot follows the line to the initial postion after placing block
                    rospy.sleep(0.1)
                    set_velocity.publish(0, 0, 0)
                    detect_step = 'color'
                    
            else:
                if chassis_move:
                    chassis_move = False
                    rospy.sleep(0.1)
                    set_velocity.publish(0, 0, 0)
                else:
                    rospy.sleep(0.01)
            
            
        elif detect_step == 'color': # block detection step
            if set_visual == 'line':
                x_dis = 500
                y_dis = 0.15
                stable = False
                set_visual = 'color'
                visual_running('colors', 'rgb') # switch image processing type 
                # switch the posture of robotic arm
                target = ik.setPitchRanges((0, 0.15, 0.03), -180, -180, 0)
                if target:
                    servo_data = target[1]
                    bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                                    (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
                    rospy.sleep(1.5)
                
            elif detect_color != 'None' and not block_clamp: # The block is stable and starts to following and gripping
                if position_en:
                    diff_x = abs(color_center_x - color_centreX)
                    diff_y = abs(color_center_y - color_centreY)
                    # x-aixs PID tracking
                    if diff_x < 10:
                        color_x_pid.SetPoint = color_center_x  # set
                    else:
                        color_x_pid.SetPoint = color_centreX
                    color_x_pid.update(color_center_x)   # current
                    dx = color_x_pid.output              # output
                    x_dis += int(dx)     
                    x_dis = 200 if x_dis < 200 else x_dis
                    x_dis = 800 if x_dis > 800 else x_dis
                    # Y-axis PID tracking
                    if diff_y < 10:
                        color_y_pid.SetPoint = color_center_y  # set
                    else:
                        color_y_pid.SetPoint = color_centreY
                    color_y_pid.update(color_center_y)   # current
                    dy = color_y_pid.output              # output
                    y_dis += dy  
                    y_dis = 0.12 if y_dis < 0.12 else y_dis
                    y_dis = 0.28 if y_dis > 0.28 else y_dis
                    
                    # The robotic arm move to above the block.      
                    target = ik.setPitchRanges((0, round(y_dis, 4), 0.03), -180, -180, 0)
                    if target:
                        servo_data = target[1]
                        bus_servo_control.set_servos(joints_pub, 20,((3, servo_data['servo3']),         
                             (4, servo_data['servo4']),(5, servo_data['servo5']), (6, x_dis)))
                    
                    if dx < 2 and dy < 0.003 and not stable: # wait for the robotic arm to stop above the block
                        num += 1
                        if num == 10:
                            stable = True  # set the block is gripped 
                            num = 0
                    else:
                        num = 0
                    
                    if stable: #control the robotic arm to grip
                        offset_y = Misc.map(target[2], -180, -150, -0.03, 0.03)
                        set_rgb(detect_color)       # set RGB light color
                        target_color = detect_color # temporaily save the target color 
                        buzzer_pub.publish(0.1) # buzzer makes sound once 
                        
                        bus_servo_control.set_servos(joints_pub, 500, ((1, 120),)) #open gripper
                        rospy.sleep(0.5)
                        target = ik.setPitchRanges((0, round(y_dis + offset_y, 5), -0.07), -180, -180, 0) #The robotic arm extends down
                        if target:
                            servo_data = target[1]
                            bus_servo_control.set_servos(joints_pub, 1000, ((3, servo_data['servo3']),
                                    (4, servo_data['servo4']),(5, servo_data['servo5']), (6, x_dis)))
                        rospy.sleep(1.5)
                        bus_servo_control.set_servos(joints_pub, 500, ((1, 500),)) #close robot gripper
                        rospy.sleep(0.8)
                        
                        bus_servo_control.set_servos(joints_pub, 1500, ((1, 500), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500))) #robotic arm raises up
                        rospy.sleep(1.5)
                        
                        stable = False
                        block_clamp = True
                        position_en = False
                        detect_step = 'line'
                    
                else:
                    rospy.sleep(0.01)
                            
        else:
            rospy.sleep(0.01)

# set RGB light color
def set_rgb(color):
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
        rospy.sleep(0.1)


result_sub = None
heartbeat_timer = None
# enter service callback function
def enter_func(msg):
    global lock
    global result_sub
    
    rospy.loginfo("enter intelligent transport")
    init()
    with lock:
        if result_sub is None:
            rospy.ServiceProxy('/visual_processing/enter', Trigger)()
            result_sub = rospy.Subscriber('/visual_processing/result', Result, run)
            
    return [True, 'enter']

# exit service callback function
def exit_func(msg):
    global lock
    global result_sub
    global __isRunning
    global heartbeat_timer
    
    rospy.loginfo("exit intelligent transport")
    with lock:
        rospy.ServiceProxy('/visual_processing/exit', Trigger)()
        __isRunning = False
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
    
    rospy.loginfo("start running intelligent transport")
    with lock:
        init()
        __isRunning = True
        rospy.sleep(0.1)
        # run sub-thread
        th = Thread(target=move)
        th.setDaemon(True)
        th.start()

# stop running function
def stop_running():
    global lock
    global __isRunning
    
    rospy.loginfo("stop running intelligent transport")
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

# heartbeat service callback function
def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy('/intelligent_transport/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # Initialization node
    rospy.init_node('intelligent_transport', log_level=rospy.DEBUG)
    # Vision processing
    visual_running = rospy.ServiceProxy('/visual_processing/set_running', SetParam)
    # Servo publishing 
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    # app communication service
    enter_srv = rospy.Service('/intelligent_transport/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/intelligent_transport/exit', Trigger, exit_func)
    running_srv = rospy.Service('/intelligent_transport/set_running', SetBool, set_running)
    heartbeat_srv = rospy.Service('/intelligent_transport/heartbeat', SetBool, heartbeat_srv_cb)
    # mecanum chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    set_translation = rospy.Publisher('/chassis_control/set_translation', SetTranslation, queue_size=1)
    # buzzer 
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    # rgb light
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)
    rospy.sleep(0.5) # It must be delayed after pub to take effects

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)
        start_running()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
********************Function:Car Linkage Routine********************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press "Ctrl+C" to close the running program. If fail to close, please try several times! 
----------------------------------------------------------
''')

ik = ik_transform.ArmIK()

start = True
#Process before closing
def stop():
    global start

    start = False
    print('closing...')
    set_velocity.publish(0,0,0)  # close all motors
    # Set the initial position
    target = ik.setPitchRanges((0.0, 0.10, 0.2), -90, -180, 0) # Get the solution with reverse kinematics
    if target: # Determine if there is a solution
        servo_data = target[1]
        # Drive the robotic arm to move
        bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)
    
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('linkage', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Mecanum chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    # Servo publish
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) 
    
    # Set the initial position
    target = ik.setPitchRanges((0.0, 0.10, 0.2), -90, -180, 0) # Get the solution with kinematics
    if target: # Determine if there is a solution
        servo_data = target[1]
        # Drive the robotic arm to move
        bus_servo_control.set_servos(joints_pub, 1800, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(2)
    
    while start:
        
        set_velocity.publish(80,270,0) # The linear velocity is 80; The directional angel is 270; The yaw rate is 0 (When the value is negative, it will rotate clockwise. )
        target = ik.setPitchRanges((0.0, 0.20, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.1)
        
        target = ik.setPitchRanges((0.0, 0.30, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.1)
        set_velocity.publish(0,270,0)
        rospy.sleep(0.3)
        
        
        set_velocity.publish(80,90,0) # The linear velocity is 80; The directional angel is 90; The yaw rate is 0 (When the value is negative, it will rotate clockwise. )
        target = ik.setPitchRanges((0.0, 0.20, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.1)
        
        target = ik.setPitchRanges((0.0, 0.10, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.1)
        set_velocity.publish(0,90,0)
        rospy.sleep(0.3)
        
        ################
        
        set_velocity.publish(85,0,0) # The linear velocity is 85; The directional angel is 0; The yaw rate is 0 (When the value is negative, it will rotate clockwise. )
        target = ik.setPitchRanges((-0.16, 0.1, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.2)
        set_velocity.publish(0,0,0)
        rospy.sleep(0.3)
        
        
        set_velocity.publish(85,180,0) # The linear velocity is 85; The directional angel is 180; The yaw rate is 0 (When the value is negative, it will rotate clockwise. )
        target = ik.setPitchRanges((0.0, 0.1, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.2)
        
        target = ik.setPitchRanges((0.16, 0.1, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.2)
        set_velocity.publish(0,90,0)
        rospy.sleep(0.3)
        
        
        set_velocity.publish(85,0,0) # The linear velocity is 60; The directional angel is 90; The yaw rate is 0 (When the value is negative, it will rotate clockwise. )
        target = ik.setPitchRanges((0.0, 0.1, 0.20), -90, -180, 0) # Get solution with the kinematics
        if target: # Determine if there is a solution
            servo_data = target[1]
            # Drive the robotic arm to move
            bus_servo_control.set_servos(joints_pub, 1200, ((1, 200), (2, 500), (3, servo_data['servo3']),
                            (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
        rospy.sleep(1.2)
        
        
    set_velocity.publish(0,0,0)  # close all motors 
    print('Closed')
        

#!/usr/bin/python3
# coding=utf8
import sys
import rospy
from chassis_control.msg import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)
    
print('''
**********************************************************
***************Function: Car Movement Routine****************
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press "Ctrl+C" to close the running program. If fail to close, please try several times! 
----------------------------------------------------------
''')

start = True
#Process before closing
def stop():
    global start

    start = False
    print('closing...')
    set_velocity.publish(0,0,0)  # close all motors
    
if __name__ == '__main__':
    # Initialize node
    rospy.init_node('car_move_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Mecanum chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    
    while start:
        set_velocity.publish(60,90,0) # The linear velocity is 60; The directional angel is 90; The yaw rate is 0 (When the value is negative, it will rotate clockwise. )
        rospy.sleep(2)
        set_velocity.publish(60,0,0)
        rospy.sleep(2)
        set_velocity.publish(60,270,0)
        rospy.sleep(2)
        set_velocity.publish(60,180,0)
        rospy.sleep(2)
    set_velocity.publish(0,0,0)  # Close all motors
    print('Closed')

        

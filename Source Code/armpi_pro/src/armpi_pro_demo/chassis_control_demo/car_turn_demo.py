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
********************Function:Car Turning Routine ********************
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
    rospy.init_node('car_turn_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Mecanum chassis control
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    
    while start:
        set_velocity.publish(0,90,-0.3)# rotate clockwise 
        rospy.sleep(2)
        set_velocity.publish(0,90, 0.3)# rotate counterclockwise 
        rospy.sleep(2)
    set_velocity.publish(0,0,0)  # close all motors 
    print('Closed')

        

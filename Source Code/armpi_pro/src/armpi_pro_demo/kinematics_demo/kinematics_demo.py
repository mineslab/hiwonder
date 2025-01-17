#!/usr/bin/python3
# coding=utf8
# Date:2022/06/30
import sys
import time
import rospy
from kinematics import ik_transform
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

print('''
**********************************************************
****Function: Hiwonder Raspberry Pi expansion board. Kinematics XYZ Axes Motion Routine *****
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press "Ctrl+C" to close the running program. If fail to close, please try several times!
----------------------------------------------------------
''')

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

ik = ik_transform.ArmIK()

#Process before closing
def stop():
    # Return to the initial position
    target = ik.setPitchRanges((0.00, 0.12, 0.08), -145, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))


if __name__ == '__main__':
    # Initialization node
    rospy.init_node('kinematics_demo', log_level=rospy.DEBUG)
    rospy.on_shutdown(stop)
    # Servo publish
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2) # Delay takes effect
    
    # Set the initial position
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0) # Get the solution with kinematics
    if target: # Determine whether there is a solution
        servo_data = target[1]
        # Drive the robotic arm to move
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    # move to 0.15m along x-axis
    target = ik.setPitchRanges((0.15, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    # Return to the initial position.
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(2)
    # Move to 0.2m along y-axis
    target = ik.setPitchRanges((0.0, 0.20, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    # Return to the initial position
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(2)
    #Move to 0.24m along z-axis
    target = ik.setPitchRanges((0.0, 0.12, 0.24), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(1.5)
    # Return to the initial position
    target = ik.setPitchRanges((0.0, 0.12, 0.15), -90, -180, 0)
    if target:
        servo_data = target[1]
        bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']),
                        (4, servo_data['servo4']),(5, servo_data['servo5']),(6, servo_data['servo6'])))
    time.sleep(2)
    
   
    
#!/usr/bin/python3
# coding=utf8
# Date:2022/06/30
import sys
import time
import signal
import Board as Board

print('''
**********************************************************
*******Function: Hiwonder Raspberry Pi expansion board. Motor Control Routine*********
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

#Process before closing 
def stop(signum, frame):
    Board.setMotor(2, 0)   # No.2 motor stops
    
signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    Board.setMotor(2,100) # No.2 servo rotates at the speed of 100.
    time.sleep(2)         # Delay 2 seconds
    Board.setMotor(2,50)  # No.2 servo rotates at the speed of 50.
    time.sleep(2)
    Board.setMotor(2,-100) # No.2 servo rotates at the speed of -100.
    time.sleep(2)
    Board.setMotor(2, 0)   # No.2 servo stops
    
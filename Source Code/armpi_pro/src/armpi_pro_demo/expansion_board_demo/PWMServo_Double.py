#!/usr/bin/python3
# coding=utf8
import sys
import time
import Board as Board

print('''
**********************************************************
*****Function: Hiwonder Raspberry Pi expansion board. Multiple Servo Control Routine******
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
    
    
if __name__ == '__main__':
    for i in range(5):
        Board.setPWMServoPulse(1, 1500, 1000) # No.1 servo roates to 1500 in 1000ms
        time.sleep(1)
        Board.setPWMServoPulse(1, 2500, 1000) # No.1 servo roates to 2500 in 1000ms
        time.sleep(1)

        Board.setPWMServoPulse(2, 1500, 1000) # No.2 servo roates to 1500 in 1000ms
        time.sleep(1)
        Board.setPWMServoPulse(2, 2500, 1000) # No.2 servo roates to 2500 in 1000ms
        time.sleep(1)












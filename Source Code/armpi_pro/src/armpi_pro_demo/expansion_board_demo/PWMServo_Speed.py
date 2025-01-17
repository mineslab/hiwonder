#!/usr/bin/python3
# coding=utf8
import sys
import time
import threading
import Board

print('''
**********************************************************
******Function: Hiwonder Raspberry Pi expansion board. PWM Servo Variable Routine*******
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
   for i in range(5): # loop 5 times
   
        Board.setPWMServoPulse(1, 500, 1000) # PWM servo connected to No.1 port rotateds to 500 in 1000ms.
        time.sleep(1)
        Board.setPWMServoPulse(1, 1500, 300) # PWM servo connected to No.1 port rotateds to 1500 in 1000ms.
        time.sleep(1)
        Board.setPWMServoPulse(1, 500, 1000) # PWM servo connected to No.1 port rotateds to 500 in 1000ms.
        

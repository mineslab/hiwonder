#!/usr/bin/python3
# coding=utf8
# Date:2022/06/30
import sys
import time
import signal
import Board as Board

print('''
**********************************************************
******Function: Hiwonder Raspberry Pi expansion board. Buzzer Control Routine********
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
    Board.setBuzzer(0) # close buzzer 

signal.signal(signal.SIGINT, stop)

if __name__ == '__main__':
    
    #Buzzer sounds for 0.5s
    Board.setBuzzer(1) # Set buzzer to make sound 
    time.sleep(0.5)    # Dalay 0.5 second
    Board.setBuzzer(0) # Close buzzer
    
    
    
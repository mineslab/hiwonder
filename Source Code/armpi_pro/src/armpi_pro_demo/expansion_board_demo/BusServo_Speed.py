import time
import Board

print('''
**********************************************************
***********Function: Hiwonder Raspberry Pi expansion board. Servo Variable Speed Routine***********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press "Ctrl+C" to close the running program. If fail to close, please try several times!
----------------------------------------------------------
''')

while True:
	# Parameter: parameter 1: servo ID; paramter 2：position; parameter 3: running time 
	# The rotation of servo ranges 0°-240° corresponding to 0-1000, that is, the range of parameter 2 is 0-1000.

	Board.setBusServoPulse(2, 800, 1200) # The ID2 servo rotates to 800 in 1200ms
	time.sleep(0.5) # delay 0.5s

	Board.setBusServoPulse(2, 200, 300) # The ID2 servo rotates to 200 in 300ms
	time.sleep(0.5) # delay 0.5s
	     

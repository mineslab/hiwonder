import time
import Board

print('''
**********************************************************
*********Function: Hiwonder Raspberry Pi expansion board. Servo Reading Status Routine**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press "Ctrl+C" to close the running program. If fail to close, please try several times! 
----------------------------------------------------------
''')

def getBusServoStatus():
    Pulse = Board.getBusServoPulse(6) # get the position information of ID6 servo
    Temp = Board.getBusServoTemp(6)   # get the temperature information of ID6 servo
    Vin = Board.getBusServoVin(6)     # get the voltage information of ID6 servo
    print('Pulse: {}\nTemp:  {}\nVin:   {}\n'.format(Pulse, Temp, Vin)) # Print the status information
    time.sleep(0.5) # delay for viewing

   
Board.setBusServoPulse(6, 500, 1000) # ID6 servo rotates to 500 in 1000ms
time.sleep(1) # delay 1s 
getBusServoStatus() # read the status of bus servo
 

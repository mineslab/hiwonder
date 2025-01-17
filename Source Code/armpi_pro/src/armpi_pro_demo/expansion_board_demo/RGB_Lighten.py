import time
import Board
import signal

print('''
**********************************************************
***********Function: Hiwonder Raspberry Pi expansion board. RGB Light Turning On Routine*************
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
def Stop(signum, frame):
    global start

    start = False
    print('Closing...')

#Turn off all lights
Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))  
Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))  
Board.RGB.show()

signal.signal(signal.SIGINT, Stop)

while True: 
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set RGB1 on expansion board as red 
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set RGB2 on expansion board as red
    Board.RGB.show()
    time.sleep(1)

    if not start:
        #Turn off all lights 
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))  
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))  
        Board.RGB.show()
        print('Closed')
        break
    

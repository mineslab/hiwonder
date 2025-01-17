import time
import Board
import signal

print('''
**********************************************************
**********Function: Hiwonder Raspberry Pi expansion board. RGB Light Changing On Routine************
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
    #Set two lights as red. 0 is RGB1 and 1 is RGB2. 
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
    Board.RGB.show()
    time.sleep(1)
    
    #Set two lights as green. 0 is RGB1 and 1 is RGB2.
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
    Board.RGB.show()
    time.sleep(1)
    
    #Set two lights as blue. 0 is RGB1 and 1 is RGB2.
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
    Board.RGB.show()
    time.sleep(1)  

    if not start:
        #Turn off all lights 
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
        print('Closed')
        break

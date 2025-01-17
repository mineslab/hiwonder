import time
import Board

print('''
**********************************************************
*******Function: Hiwonder Raspberry Pi expansion board. RGB Light Flashing Routine********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * Press "Ctrl+C" to close the running program. If fail to close, please try several times!
----------------------------------------------------------
''')


def stop():         
    print('Closing...')
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off RGB1 on expansion board.
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off RGB2 on expansion board.
    Board.RGB.show()

def start():
    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the RGB1 on expansion board as red. 
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the RGB2 on expansion board as red
    Board.RGB.show()
    time.sleep(1)

    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the RGB1 on expansion board
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the RGB2 on expansion board
    Board.RGB.show()
    time.sleep(1)   

    for i in range(3):
        Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the RGB1 on expansion board as red. 
        Board.RGB.show()
        time.sleep(0.5)
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the RGB1 on expansion board 
        Board.RGB.show()
        time.sleep(0.5)
         
        Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the RGB2 on expansion board as red
        Board.RGB.show()
        time.sleep(0.5)
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the RGB2 on expansion board
        Board.RGB.show()
        time.sleep(0.5)

    Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0)) # Set the RGB1 on expansion board as red. 
    Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0)) # Set the RGB2 on expansion board as red
    Board.RGB.show()
    time.sleep(1)

    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the RGB1 on expansion board 
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the RGB2 on expansion board
    Board.RGB.show()
    time.sleep(1)   


if __name__ == '__main__': 
    Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0)) # Turn off the RGB1 on expansion board 
    Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0)) # Turn off the RGB2 on expansion board
    Board.RGB.show()
    time.sleep(1)	 
    while True:
        try:
            start()	
        except KeyboardInterrupt:
            stop()
            print('Closed')
            break
    

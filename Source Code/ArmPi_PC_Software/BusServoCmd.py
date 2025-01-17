#!/usr/bin/env python3
# encoding: utf-8
import time
import ctypes
import serial
import RPi.GPIO as GPIO
#Hiwonder Bus Servo Communication#

LOBOT_SERVO_FRAME_HEADER         = 0x55
LOBOT_SERVO_MOVE_TIME_WRITE      = 1
LOBOT_SERVO_MOVE_TIME_READ       = 2
LOBOT_SERVO_MOVE_TIME_WAIT_WRITE = 7
LOBOT_SERVO_MOVE_TIME_WAIT_READ  = 8
LOBOT_SERVO_MOVE_START           = 11
LOBOT_SERVO_MOVE_STOP            = 12
LOBOT_SERVO_ID_WRITE             = 13
LOBOT_SERVO_ID_READ              = 14
LOBOT_SERVO_ANGLE_OFFSET_ADJUST  = 17
LOBOT_SERVO_ANGLE_OFFSET_WRITE   = 18
LOBOT_SERVO_ANGLE_OFFSET_READ    = 19
LOBOT_SERVO_ANGLE_LIMIT_WRITE    = 20
LOBOT_SERVO_ANGLE_LIMIT_READ     = 21
LOBOT_SERVO_VIN_LIMIT_WRITE      = 22
LOBOT_SERVO_VIN_LIMIT_READ       = 23
LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE = 24
LOBOT_SERVO_TEMP_MAX_LIMIT_READ  = 25
LOBOT_SERVO_TEMP_READ            = 26
LOBOT_SERVO_VIN_READ             = 27
LOBOT_SERVO_POS_READ             = 28
LOBOT_SERVO_OR_MOTOR_MODE_WRITE  = 29
LOBOT_SERVO_OR_MOTOR_MODE_READ   = 30
LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE = 31
LOBOT_SERVO_LOAD_OR_UNLOAD_READ  = 32
LOBOT_SERVO_LED_CTRL_WRITE       = 33
LOBOT_SERVO_LED_CTRL_READ        = 34
LOBOT_SERVO_LED_ERROR_WRITE      = 35
LOBOT_SERVO_LED_ERROR_READ       = 36

serialHandle = serial.Serial("/dev/ttyS0", 115200)  # Initialize the serial port. The baud rate is 115200

rx_pin = 7
tx_pin = 13

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

def portInit():  # Congigure the IO port to be used
    GPIO.setup(rx_pin, GPIO.OUT)  # Configure RX_CON, i.e, GPIO17 as output
    GPIO.output(rx_pin, 0)
    GPIO.setup(tx_pin, GPIO.OUT)  # Configure TX_CON, i.e, GPIO27 as output
    GPIO.output(tx_pin, 1)

portInit()

def portWrite():  # Configure the single serial port as output
    GPIO.output(tx_pin, 1)  # pull up TX_CON, i.e, GPIO27
    GPIO.output(rx_pin, 0)  # GPIO17 pull down RX_CON, i.e, GPIO17

def portRead():  # Configure the single serial port as intput
    GPIO.output(rx_pin, 1)  # pull up RX_CON, i.e, GPIO17
    GPIO.output(tx_pin, 0)  # GPIO17 pull down TX_CON, i.e, GPIO27

def portRest():
    time.sleep(0.1)
    serialHandle.close()
    GPIO.output(rx_pin, 1)
    GPIO.output(tx_pin, 1)
    serialHandle.open()
    time.sleep(0.1)

def checksum(buf):
    # calculate and check sum
    sum = 0x00
    for b in buf:  # sum
        sum += b
    sum = sum - 0x55 - 0x55  # 0x55 get rid off two 0x55 at the starting of command
    sum = ~sum  # flip
    return sum & 0xff

def serial_serro_wirte_cmd(id=None, w_cmd=None, dat1=None, dat2=None):
    '''
    write command
    :param id:
    :param w_cmd:
    :param dat1:
    :param dat2:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # frame header
    buf.append(id)
    # Command length
    if dat1 is None and dat2 is None:
        buf.append(3)
    elif dat1 is not None and dat2 is None:
        buf.append(4)
    elif dat1 is not None and dat2 is not None:
        buf.append(7)

    buf.append(w_cmd)  # Command
    # write data
    if dat1 is None and dat2 is None:
        pass
    elif dat1 is not None and dat2 is None:
        buf.append(dat1 & 0xff)  # deviaton
    elif dat1 is not None and dat2 is not None:
        buf.extend([(0xff & dat1), (0xff & (dat1 >> 8))])  
        buf.extend([(0xff & dat2), (0xff & (dat2 >> 8))])  
    # check and check sum
    buf.append(checksum(buf))
    # for i in buf:
    #     print('%x' %i)
    serialHandle.write(buf)  # send

def serial_servo_read_cmd(id=None, r_cmd=None):
    '''
    发送读取命令 send a read command
    :param id:
    :param r_cmd:
    :param dat:
    :return:
    '''
    portWrite()
    buf = bytearray(b'\x55\x55')  # frame header
    buf.append(id)
    buf.append(3)  # command length
    buf.append(r_cmd)  # command
    buf.append(checksum(buf))  # check sum
    serialHandle.write(buf)  # send
    time.sleep(0.00034)

def serial_servo_get_rmsg(cmd):
    '''
    # Get the data of the specified read command
    :param cmd: read commamd
    :return: data
    '''
    serialHandle.flushInput()  # clear thr receiving buffer.
    portRead()  #Configure the single serial port as output
    time.sleep(0.005)  # Wait for the receiving to complete
    count = serialHandle.inWaiting()    # receive the number of bytes in buffer
    if count != 0:  # If the receiving data is not none.
        recv_data = serialHandle.read(count)  # read the received data
        # for i in recv_data:
        #     print('%#x' %ord(i))
        # whether to read ID command 
        try:
            if recv_data[0] == 0x55 and recv_data[1] == 0x55 and recv_data[4] == cmd:
                dat_len = recv_data[3]
                serialHandle.flushInput()  # clear the receiving buffer
                if dat_len == 4:
                    # print ctypes.c_int8(ord(recv_data[5])).value    # Convert to signed integer
                    return recv_data[5]
                elif dat_len == 5:
                    pos = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    return ctypes.c_int16(pos).value
                elif dat_len == 7:
                    pos1 = 0xffff & (recv_data[5] | (0xff00 & (recv_data[6] << 8)))
                    pos2 = 0xffff & (recv_data[7] | (0xff00 & (recv_data[8] << 8)))
                    return ctypes.c_int16(pos1).value, ctypes.c_int16(pos2).value
            else:
                return None
        except BaseException as e:
            print(e)
    else:
        serialHandle.flushInput()  # Clear receive cache
        return None

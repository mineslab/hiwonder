#!/usr/bin/python3 
import os
import sys
import time
import signal
 
# Detect the log files once every 30 minutes. If it is greater than 10M, clear it. 
# Too large log files will cause high CPU consumption and abnormal operation
# Close this function and restart to take effect: sudo systemctl disable clear_log.service  

running = True
def handler(signum, frame):
    global running

    running = False
    print('exit')
    sys.exit(0)

signal.signal(signal.SIGINT, handler)

ros_log_path = '/home/ubuntu/.ros/log'

log_size = 0
while running:
    try:
        for root, dirs, files in os.walk(ros_log_path):
            for f in files:
                log_size += os.path.getsize(os.path.join(root, f))
        log_size /= float(1024*1024)
        #print('the log size:{}m'.format(log_size))
        if log_size > 10:
            os.system('sudo rm -rf {}/*'.format(ros_log_path))
        time.sleep(30*60)
    except BaseException as e:
        print(e)

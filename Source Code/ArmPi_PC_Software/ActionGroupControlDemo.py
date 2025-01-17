import time
import ActionGroupControl as AGC

print('''
**********************************************************
*********Function:Hiwonder Raspberry Pi Expansion Board, Action Group Control Routine********
**********************************************************
----------------------------------------------------------
Official website:http://www.hiwonder.com
Online mall:https://huaner.tmall.com/
----------------------------------------------------------
The following command needs to be opened in LX terminal by pressing Ctrl+alt+t or clicking LX terminal icon 
----------------------------------------------------------
Usage:
    sudo python3 ActionGroupControlDemo.py
----------------------------------------------------------
Version: --V1.1  2021/05/04
----------------------------------------------------------
Tips:
 * Press Ctrl+C to close the running program. If failt to close, please try several times.
----------------------------------------------------------
''')

# Action groups should be saved in the path /home/ubuntu/ArmPi_PC_Software/ActionGroups.
AGC.runAction('wave') # The parameter is the action group name including not suffix passed in as a character

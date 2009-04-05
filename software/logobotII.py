'''
the Logobot class communicates with Logobot (running Arduino_Read_Sensor).

...
wbosworth 4/4/09
'''

import serial
import time



class logobot():
    def __init__(self):
        # find your port manaually, before running prog
        self.ser = serial.Serial('COM11', 19200,timeout=0)
        print 'serial port open'

    '''
    from the arduino program:
    data arduino program can recieve:
    #define WAITING 1
    #define MSG_START 2
    #define COMMAND 3
    #define FORWARD 4
    #define BACKWARD 5
    #define LEFT 6
    #define RIGHT 7
    #define MSG_END 8
    #define ACK 9
    #define ACTION_COMPLETE 10
    #define PARAMETER1 11
    #define PARAMETER2 12
    #define REPORTING 13
    #define GET_X 14
    #define GET_Y 15
    #define GET_BATT 16
    #define DATA_IN 17
    #define LISTENING_FOR_THE_END 19
    #define WTF 20
    #define LISTENING_FOR_A_CHECKSUM 21
    #define LISTENING 22
    #define INVALID_COMMAND 23
    #define BAD_CHECKSUM 24
    #define BAD_MESSAGE_STRUCTURE 25
    #define MOVING 26
    #define GET_STATUS 27
    #define PEN_UP 28
    #define PEN_DOWN 29
    #define STOP 30
    '''

    def forward(self):
        # forward is '4'
        delay = 0
        linearparam1 = 2
        linearparam2 = 220
        forward = 4
        command = forward
        msg_start = 2
        msg_end = 8
        checksum = (command + linearparam1 + linearparam2)%255
        self.ser.write(chr(msg_start))
        time.sleep(delay)
        self.ser.write(chr(command))
        time.sleep(delay)
        self.ser.write(chr(linearparam1))
        time.sleep(delay)
        self.ser.write(chr(linearparam2))
        time.sleep(delay)
        self.ser.write(chr(checksum))
        time.sleep(delay)
        self.ser.write(chr(msg_end))
        message = self.ser.read()
        print message
        
       
    

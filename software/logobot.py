#!/usr/bin/python
""" @package logobot

logobot.py sends commands to a serial port (or a USB port imitating a serial port) for transmission to logobot, wirelessly controlled robot which can draw and move.  This is a Python wrapper around the serial commands that are sent wirelessly using the XBee protocol.
"""

import serial
import socket

import re
import time
import math

import logging
logging.basicConfig(level=logging.DEBUG) 
# Available levels: NOSET, DEBUG, INFO, WARN, ERROR, CRITITCAL


""" number of clicks in one inch, should be 400 """
lengthScale = 1 

""" 25.6 = the number of clicks in one degree, should be 29 """
rotationScale = 1 

import os, sys, dircache
portDirectory = '/dev/'
portsAvailable = dircache.opendir(portDirectory)

""" Set the usb port and baudrate """
usb = serial.Serial()
usb.baudrate = 19200
usb.timeout = 0

""" autoselect usb port on linux machines"""
for port in portsAvailable:
    if re.search('USB', port):
        usb.port = portDirectory + port
        break


""" These lookup dictionaries are separated for convenience and clarity of function.
communications : contains the definitions for communication protocol messages
requests       : contains definitions for logobot status or info requests
statuses       : contains definitions for all the status reports logobot could return
controls       : contains the definitions of the commands which control logobot's motion and actions 
errors         : contains all the error messages we use  """
communications = {
    'WAITING':1,
    'MSG_START':2,
    'COMMAND':3,
    'MSG_END':4,
    'COMMAND':3,
    'MSG_END':8,
    'BAD_CHECKSUM':24,
    'PARAMETER1':11,
    'PARAMETER2':12,}

requests = {    
    'GET_STATUS':27,
    'GET_X':14,
    'GET_Y':15,
    'GET_BATT':16,
    'GET_STATUS':27,}

statuses = {
    'REPORTING':13,
    'LISTENING_FOR_THE_END':19,
    'LISTENING_FOR_A_CHECKSUM':21,
    'LISTENING':22,
    'MOVING':26,
    'ACTION_COMPLETE':10,
    'DATA_IN':17,
}

controls = {
    'FORWARD':4,
    'BACKWARDS':5,
    'LEFT':6,
    'RIGHT':7,
    'PEN_UP':28,
    'PEN_DOWN':29,}

errors = {
    'WTF':20,
    'ACK':9,
    'BAD_MESSAGE_STRUCTURE':25,
    'INVALID_COMMAND':23,}


""" messageDictionaries merges the lookup dictionaries for searching""" 
messageDictionaries = [communications, requests, statuses, controls, errors]

""" update merges dictionaries together, this merges all the dictionaries from the messageDictionaries array together """
lookup = reduce(lambda x,y: 
                    x.update(y),
                messageDictionaries)

""" The reverseLookup dictionary will eventually hold the reverse mapping of numbers to messages for use in parsing logobot responses """
reverseLookup = {}

""" Defines interface to scrape, which interprets Scratch commands and translates """
scrapeLookup = {
    'turnRight':'RIGHT',
    'turnLeft':'LEFT',
    'forward':'FORWARD',
    'penUp':'PEN_UP',
    'penDown':'PEN_DOWN',
}



def openConnection(port=usb):
""" open Connection opens a connection on the given port, defaulting to the autolocated usb port and returning a serial object which has been flushed and is ready for reading and writing.
"""
    ser = usb
    if port.isOpen():
        logging.debug('Port %s already open.', port.port)
    else:
        logging.debug('Port %s not yet open.', port.port)
        if port != usb:
            logging.debug('Port %s not open, opening. . .', port.port)
            ser = serial.Serial(port.port,9600,timeout=0)
        else:
            logging.debug('Default port %s used.', port.port)

        logging.debug('Opening port %s . . .', port.port)
        ser.open()
        ser.flushInput()
        logging.debug('Port %s open and flushed', port.port)
    return ser



def encode(message):
""" encode takes a given message and translates it to numbers via the lookup dictionary. """
    logging.debug('Encoding %s', message)

    if message == '':
        logging.warn('Message is blank.')
        return chr(0)
    elif message in lookup:
        translated = chr(lookup[message])
        logging.debug('Message found, translated %s to %s', message, repr(translated))
        # Note that repr is needed to see the hexadecimal.  This can lead to tricky parsing problems.
        return translated
    else:
        logging.warn('Message neither null nor defined, returning original message %s', repr(byteRepresentation))
        byteRepresentation = chr(message)
        
        return byteRepresentation


def decode(value):
""" decode looks up the message it maps to using the reverseLookup dictionary, letting us interpret Logobot commands 
"""

    if len(value) == 0:
        return ''
    else:
        value = ord(value)
        if len(reverseLookup) == 0:
            logging.debug('Constructing reverse lookup dictionary. . .')
            for message, key in lookup.iteritems():
                reverseLookup[key] = message
                logging.debug('%s mapped to %s', key, message)
        try:
            return reverseLookup[value]
        except KeyError:
            logging.warn('Value %s has no matching key, returning %s', value, value)
            reverseLookup[value] = None
            return value



def portWrite(message, port=usb):
""" portWrite is a wrapper around writing to the serial port that will insure that a port is open before attempting to write to it and log a message while doing so.
"""
    logging.debug('Writing %s on %s', message, port.port)
    openConnection(port)
    return port.write(message)



def portRead(port=usb):
""" portRead is a wrapper around writing to the serial port that will insure that a port is open before attempting to read it and log a message while doing so.
"""
    logging.debug('Reading from %s', port.port)
    openConnection(port)
    return port.read()


def msgStart(port=usb):
""" msgStart tells logobot a message is beginning """
    logging.debug('Writing MSG_START byte.')
    portWrite(encode('MSG_START'), port)


def msgEnd(port=usb):
""" msgEnd tells Logobot that a message is finished """
    logging.debug('Sending MSG_END byte.')    
    portWrite(encode('MSG_END'),port)


def say(message, port=usb):    
""" say encodes the message and sends it over the port, well-formed.  If you want to simply send a bit on the port, use portWrite"""
    portWrite(encode(message))
    logging.debug('Wrote %s', repr(message))
    return message


def listen(port=usb):
""" listen waits for 0.1s and then reads whatever is in the serial buffer and decodes it
** TODO Write a listen function which waits for a particular message 
"""
    time.sleep(0.1) """ arbitrarily chosen interval """
    bufferValue = portRead(port)
    logging.debug('Read %s from port %s', repr(bufferValue), port.port)
    if str(repr(bufferValue)) == '':
        logging.warn('Buffer is empty.')
        return None
    else:
        decodedMessage = decode(bufferValue)
        return decodedMessage


def ask(message, port=usb):
""" ask queries logobot for its status and attributes, flushing the input channel and preparing it to read logobot's response """
    msgStart()
    logging.debug('Flushing input channel. . .')
    usb.flushInput()
    logging.debug('Asking about %s', message)
    say(message)
    say(checksum(message,0))
    msgEnd()
    status = listen()
    logging.debug('Read %s from buffer', status)
    return status


def do(command, parameter = '', port=usb):
""" do will take a command, translate it, pass it on, and make sure it is well-formed.  It is the workhorse of logobot.py, and will send the well-formed message to the USB port, printing whatever response received over the serial port 

** TODO ensuring via checksum that the message is transmitted and received. """

    #timeout = port.timeout
    #timeout=parameter/1000
    waitUntilReady(1)
    logging.debug('Flushing input channel. . .')
    usb.flushInput()
    msgStart()
    say(command)

    if parameter:
        logging.debug('Parameter found.')
        if parameter < 256:
            logging.debug('Writing high byte for parameter less than 256')
            say(0)
            logging.debug('Writing low byte for parameter less than 256')
            say(parameter)
            say(checksum(command, parameter))
        else:
            logging.debug('Converting parameter to base 256')
            say(int(parameter/256))
            say(parameter % 256)
            say(checksum(command, parameter))
    else:
        logging.warn('No parameter found.')
        say(checksum(command, 0))
    msgEnd()

    logging.debug('Read %s after completion of [%s, %s]', 
                  listen(), str(command), str(parameter))
    return [command, parameter]



def waitUntilReady(timeout=-1):
""" waitUntilReady

waitUntilReady will pause the script until Logobot is ready to receive commands, so that we do notoverwhelm Logobot with commands while it is still carrying them out """
    if timeout == -1:
        while ask('GET_STATUS') != 'WAITING':
            time.sleep(0.1) """ Sleep for 0.1 seconds """
        return
    else:
        timeSlept = 0
        timeoutInterval = 0.1
        while timeSlept < timeout:
            if ask('GET_STATUS') == 'WAITING':
                time.sleep(0.5)
                 return
            else:
                time.sleep(timeoutInterval)
                timeSlept += timeoutInterval
        

def checksum(command, parameter):
""" checksum

"""
    logging.debug('Computing checksum of %s and %s', command, parameter)
    cmdValue = ord(encode(command))
    if parameter < 256:
        paramValue = parameter
    else:
        paramValue = int(parameter/256) + int(parameter%256)
    checksumValue = (cmdValue + paramValue) % 256 # checksum convention
    logging.debug('Checksum computed as %s', checksumValue)

    return checksumValue


def turn(degrees):
""" turn

turn is a wrapper around turning n degrees.  Positive is clockwise, negative counterclockwise
"""
    angularClicks = abs(degrees)*rotationScale
    if degrees < 0:
        logging.info('Turning left %d degrees', abs(degrees))
        do('LEFT', angularClicks)
    else:
        logging.info('Turning right %d degrees', abs(degrees))
        do('RIGHT', angularClicks)


def move(steps):
""" move

Wrapper around translation """
    lengthClicks = abs(steps)*lengthScale
    if steps < 0:
        logging.info('Moving backwards %d steps', steps)
        do('BACKWARDS', lengthClicks)
    else:
        logging.info('Moving forward %d steps', steps)
        do('FORWARD', lengthClicks)


def scrapeInterface(command, parameter):
""" scrapeInterface

Interface to Scratch customization """
    translatedCommand = ''
    translatedParameter = ''
    logging.debug('Executing scrapeInterface with %s and %s', command, parameter)
    if parameter != '': 
        logging.debug('Parameter is %s', parameter)
        parameter = int(parameter)

    if scrapeLookup[command] == 'FORWARD':
        logging.debug('Parsing forward/backwards command')
        if int(parameter) < 0:
            translatedCommand = 'BACKWARDS'
            translatedParameter = abs(int(parameter)*lengthScale) 
        else:
            translatedCommand = scrapeLookup[command]
            translatedParameter = parameter*lengthScale
    elif scrapeLookup[command] == 'PEN_UP' or scrapeLookup[command] == 'PEN_DOWN':
        translatedCommand = scrapeLookup[command]
        translatedParameter = '' # Default, blank parameter
    else:
        translatedCommand = scrapeLookup[command]
        translatedParameter = parameter*rotationScale

    logging.info('Translated %s and %s into %s and %s for transmission',
                 command, parameter, translatedCommand, translatedParameter)

    return [translatedCommand, translatedParameter]


def queue(messages):
"""
queue

# Queues up jobs to prevent Logobot from overflowing
# TODO: handle multiple sets of jobs being sent (implement threading) """
    for message in messages:
        logging.debug('Trying to use message %s', message)
        originalCommand, originalParameter = message[0], message[1]
        command, parameter = scrapeInterface(originalCommand, originalParameter)
        do(command, parameter)

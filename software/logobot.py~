#!/usr/bin/python

import serial, socket
import re, time, math

import logging
logging.basicConfig(level=logging.DEBUG) 
# Available levels: NOSET, DEBUG, INFO, WARN, ERROR, CRITITCAL

lengthScale = 100 # number of clicks in one inch, should be 400
rotationScale = 1 #25.6 # number of clicks in one degree, should be 29

import os, sys, dircache
portDirectory = '/dev/'
portsAvailable = dircache.opendir(portDirectory)

usb = serial.Serial()
usb.baudrate = 9600
usb.timeout = 0

# Select active USB port
for port in portsAvailable:
    if re.search('USB', port):
        usb.port = portDirectory + port
        break

# Lookup dictionary 
lookup = {
    'WAITING':1,
    'MSG_START':2,
    'COMMAND':3,
    'MSG_END':8,
    'ACK':9,
    'REPORTING':13,
    'DATA_IN':17,
    'LISTENING_FOR_THE_END':19,
    'WTF':20,
    'LISTENING_FOR_A_CHECKSUM':21,
    'LISTTENING':22,
    'ACTION_COMPLETE':10,
    'PARAMETER1':11,
    'PARAMETER2':12,
    'MOVING':26,
    'GET_STATUS':27,
    'FORWARD':4,
    'BACKWARDS':5,
    'LEFT':6,
    'RIGHT':7,
    'GET_X':14,
    'GET_Y':15,
    'GET_BATT':16,
    'INVALID_COMMAND':23,
    'BAD_CHECKSUM':24,
    'BAD_MESSAGE_STRUCTURE':25,
    'MOVING':26,
    'GET_STATUS':27,
    'PEN_UP':28,
    'PEN_DOWN':29,
    }

# Initializes reverseLookup dictionary for decoding later
reverseLookup = {}

# Defines Scratch interface dictionary
scrapeLookup = {
    'turnRight':'RIGHT',
    'turnLeft':'LEFT',
    'forward':'FORWARD',
    'penUp':'PEN_UP',
    'penDown':'PEN_DOWN',
}

# Opens connection with serial/USB port
def openConnection(port=usb):
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

# Translates message for transmission by looking up keyword in dictionary
def encode(message):
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
        byteRepresentation = chr(message)
        logging.warn('Message neither null nor defined, returning original message %s', repr(byteRepresentation))
        return byteRepresentation

# Translates Logobot responses
def decode(value):
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

# Wrapper around serial output on port, insures open connection
def portWrite(message, port=usb):
    logging.debug('Writing %s on %s', message, port.port)
    openConnection(port)
    return port.write(message)

# Wrapper around serial read on port, insures open connection
def portRead(port=usb):
    logging.debug('Reading from %s', port.port)
    openConnection(port)
    return port.read()

# Opening handshake with Logobot
def msgStart(port=usb):
    logging.debug('Writing MSG_START byte.')
    portWrite(encode('MSG_START'), port)

# Closing handshake with Logobot
def msgEnd(port=usb):
    logging.debug('Sending MSG_END byte.')    
    portWrite(encode('MSG_END'),port)

# say is responsible for encoding the message and properly forming it
# For access to the port raw, use portWrite        
def say(message, port=usb):    
    portWrite(encode(message))
    logging.debug('Wrote %s', repr(message))
    return message

# Reads from serial buffer for responses to decode
def listen(port=usb):
    time.sleep(0.1)
    bufferValue = portRead(port)
    logging.debug('Read %s from port %s', repr(bufferValue), port.port)
    if str(repr(bufferValue)) == '':
        logging.warn('Buffer is empty.')
        return None
    else:
        decodedMessage = decode(bufferValue)
        return decodedMessage

# Asks Logobot about its various states
def ask(message, port=usb):
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

# Generic command wrapper which parses and transmits message, complete with handshakes    
def do(command, parameter = '', port=usb):
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

# Waits until Logobot is ready to receive commands
def waitUntilReady(timeout=-1):
    if timeout == -1:
        while ask('GET_STATUS') != 'WAITING':
            time.sleep(0.1)
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
        
# Simple checksum to verify packet integrity
def checksum(command, parameter):
    logging.debug('Computing checksum of %s and %s', command, parameter)
    cmdValue = ord(encode(command))
    if parameter < 256:
        paramValue = parameter
    else:
        paramValue = int(parameter/256) + int(parameter%256)
    checksumValue = (cmdValue + paramValue) % 256 # checksum convention
    logging.debug('Checksum computed as %s', checksumValue)

    return checksumValue

# Wrapper around turning n degrees.  Positive is clockwise, negative counterclockwise
def turn(degrees):
    angularClicks = abs(degrees)*rotationScale
    if degrees < 0:
        logging.info('Turning left %d degrees', abs(degrees))
        do('LEFT', angularClicks)
    else:
        logging.info('Turning right %d degrees', abs(degrees))
        do('RIGHT', angularClicks)

# Wrapper around translation
def move(steps):
    lengthClicks = abs(steps)*lengthScale
    if steps < 0:
        logging.info('Moving backwards %d steps', steps)
        do('BACKWARDS', lengthClicks)
    else:
        logging.info('Moving forward %d steps', steps)
        do('FORWARD', lengthClicks)

# Interface to Scratch customization
def scrapeInterface(command, parameter):
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

# Queues up jobs to prevent Logobot from overflowing
# TODO: handle multiple sets of jobs being sent (implement threading)
def queue(messages):
    for message in messages:
        logging.debug('Trying to use message %s', message)
        originalCommand, originalParameter = message[0], message[1]
        command, parameter = scrapeInterface(originalCommand, originalParameter)
        do(command, parameter)

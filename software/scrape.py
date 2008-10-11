#!/usr/bin/python

from array import array
import socket, time, re

import logobot

HOST = '127.0.0.1'
PORT = 42001 # Port on which Scratch's remote sensors can be enabled
# For more information on the remote sensor system, see

import logging
logging.info('Connecting to socket %s on host %s',PORT,HOST)
scratchSock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
scratchSock.connect((HOST, PORT))
logging.info("Connected.  Waiting for data...")

# Listens to Scratch and extracts broadcast strings
def parseBroadcast(broadcastString):
    messagesBroadcast = []
    if not (broadcastString[0] == '\x00' and
            broadcastString[1] == '\x00' and
            broadcastString[2] == '\x00'):
        logging.warn('Malformed broadcast message received; dropping: %s', broadcastString)
        return
    else:
        i = 3
        while i < len(broadcastString):
            # Hex prefix tells us how long the coming message is
            msgLength = ord(broadcastString[i])
            if msgLength > 0:
                thisMessage = broadcastString[i+1:(i+1)+msgLength]
                messagesBroadcast.append(thisMessage)
            i+=msgLength+1
            logging.debug('Parsed broadcast string %s as %s', broadcastString, thisMessage)
    return messagesBroadcast

# Translates parsed Scratch commands to Logobot commands
def translate(message):
    # Looking to match broadcast "command parameter"
    commandRegex = re.compile('broadcast "(.+) (.+)"')
    # Looking to match broadcast "command"
    broadcastRegex = re.compile('broadcast "([a-zA-z]+)"')
    isCommand = commandRegex.match(message)
    isBroadcast = broadcastRegex.match(message) 
    if isCommand:
        translatedMessage = isCommand
        command, parameter = translatedMessage.group(1), translatedMessage.group(2)
        return [command, parameter]
    elif isBroadcast:
        translatedMessage = isBroadcast
        command = translatedMessage.group(1)
        return [command, '']
    logging.warn('Message %s not recognized.', message)
    return message

# The following code sets up Scrape to listen for Scratch and pass on the parsed broadcasts
job = [] # Array to hold jobs in queue
while True:
    data = str(scratchSock.recv(6000)) # Errors involving characters where you expect integers can often be addressed by increasing the size of the message received
    msgs = parseBroadcast(data)
    for msg in msgs:
        try:
            logging.debug('Captured %s from Scratch', translate(msg))
            job.append(translate(msg))
            logging.debug('Translated %s as %s', msg, translate(msg))
        except AttributeError:
            logging.warn('No command parsed from message: %s', msg)
    logobot.queue(job)
    job = [] # Resets to prevent jobs from being repeatedly executed

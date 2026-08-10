##################################################################################
# Copyright (c) 2025 Matthew Thomas Beck                                         #
#                                                                                #
# Licensed under the Creative Commons Attribution-NonCommercial 4.0              #
# International (CC BY-NC 4.0). Personal and educational use is permitted.       #
# Commercial use by companies or for-profit entities is prohibited.              #
##################################################################################





############################################################
############### IMPORT / CREATE DEPENDENCIES ###############
############################################################


########## IMPORT DEPENDENCIES ##########

##### import necessary libraries #####

import time # import time library for time functions
import serial # import serial for maestro control
import logging # import logging library for debugging
import subprocess
import os
import signal

##### import config #####

from utilities.config import MAESTRO_CONFIG # import servo configuration data





##################################################
############### INITIALIZE MAESTRO ###############
##################################################


########## INITIALIZE MAESTRO ##########

def initialize_maestro( # function to initialize maestro serial connection
        serial_path=MAESTRO_CONFIG['SERIAL_PATH'],
        serial_baud_rate=MAESTRO_CONFIG['SERIAL_BAUD_RATE'],
        serial_timeout=MAESTRO_CONFIG['SERIAL_TIMEOUT']
):

    ##### cleanup attempt #####

    _attempt_serial_cleanup(serial_path) # attempt to clean up serial port before establishing connection

    ##### establish serial connection to maestro #####

    logging.debug("(maestro.py): Attempting to establish connection with maestro...\n")
    MAESTRO = _establish_serial_connection(serial_path, serial_baud_rate, serial_timeout) # establish serial connection
    _send_baud_rate_indication(MAESTRO) # send baud rate indication to maestro to establish communication

    ##### disable servos to clear old state #####

    _disable_all_servos(MAESTRO)

    return MAESTRO


########## ESTABLISH SERIAL CONNECTION ##########

# function to establish serial connection to maestro
def _establish_serial_connection(serial_port_name, serial_baud_rate, serial_timeout):

    ##### attempt to establish serial connection and return maestro object #####

    logging.debug("(maestro.py): Establishing serial connection with maestro...\n")

    try: # try to establish serial connection

        maestro = serial.Serial( # set maestro connection object

            serial_port_name, # port to connect to
            baudrate=serial_baud_rate, # baud rate for serial connection
            timeout=serial_timeout # amount of time to wait for response
        )

        logging.info("(maestro.py): Successfully established connection with maestro.\n")

        return maestro # return maestro connection object

    except:
        logging.error("(maestro.py): Failed to establish serial connection to maestro.\n")
        return 1


########## SEND BAUD RATE ##########

def _send_baud_rate_indication(maestro): # function to send baud rate indication to Maestro to establish communication

    ##### attempt to send baud rate indication #####

    logging.debug("(maestro.py): Sending baud rate initializer...\n") # print initialization statement

    try: # try to send baud rate indication

        maestro.write(bytearray([0xAA])) # send 0xAA byte to indicate baud rate
        time.sleep(0.1) # give time for maestro to process baud rate indication
        logging.info("(maestro.py): Successfully initialized maestro with baud rate.\n")
        return 0

    except: # if baud rate indication failed...

        logging.error("(maestro.py): Failed to send baud rate indication to maestro.\n") # print failure
        return 1


########## DISABLE ALL SERVOS ##########

def _disable_all_servos(maestro): # function to disable all servos at startup

    ##### attempt to disable all servos #####

    logging.debug("(maestro.py): Disabling all servos at startup...\n")

    try:

        for channel in range(12): # iterate through all servo channels (0-11)
            maestro.write(bytearray([0x84, channel, 0, 0])) # send command to set target position to 0 (disabled)

        logging.info("(maestro.py): Disabled all servos at startup.\n")

    except Exception as e:
        logging.warning(f"(maestro.py): Failed to disable servos at startup: {e}")


########## ATTEMPT SERIAL CLEANUP ##########

def _attempt_serial_cleanup(serial_path): # function to attempt cleanup of serial port before establishing connection

    ##### attempt to kill any processes using the serial port #####

    logging.debug(f"(maestro.py): Attempting to clean up serial port {serial_path}...\n")

    try: # try to find and kill processes using the serial port

        # run lsof command to find processes using the serial port
        result = subprocess.run(['lsof', '-t', serial_path], stdout=subprocess.PIPE, text=True)
        pids = result.stdout.strip().split('\n') # get list of process IDs using the serial port

        for pid in pids: # iterate through each process ID
            if pid.isdigit(): # if process ID is a digit...
                os.kill(int(pid), signal.SIGTERM) # kill the process
                logging.warning(f"(maestro.py): Killed process {pid} holding {serial_path}")

        time.sleep(0.2) # give the operating system some time to release the port

    except Exception as e:
        logging.warning(f"(maestro.py): Serial cleanup failed: {e}")

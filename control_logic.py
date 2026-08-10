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


########## MANDATORY DEPENDENCIES ##########

##### mandatory libraries #####

import threading
import queue
import time
import os
import socket
import logging
from collections import deque
import numpy as np

##### mandatory dependencies #####

from utilities.log import initialize_logging
import utilities.config as config
from utilities.receiver import interpret_commands
import utilities.internet as internet  # dynamically import internet utilities to be constantly updated

##### (pre)initialize all utilities #####

LOGGER = initialize_logging()
CAMERA_PROCESS = None
CHANNEL_DATA = {}
SOCK = None
COMMAND_QUEUE = None
ROBOT_ID = None
JOINT_MAP = {}


########## PREPARE ROBOT ##########

##### prepare real robot #####

def set_real_robot_dependencies():  # function to initialize real robot dependencies

    ##### import necessary functions #####

    from utilities.receiver import initialize_receiver  # import receiver initialization functions
    from utilities.camera import initialize_camera  # import to start camera logic
    import utilities.internet as internet  # dynamically import internet utilities to be constantly updated
    from utilities.accelerometer import initialize_accelerometer  # import accelerometer initialization functions

    ##### initialize global variables #####

    global CAMERA_PROCESS, CHANNEL_DATA, SOCK, COMMAND_QUEUE, ROBOT_ID, JOINT_MAP, internet

    ##### initialize PREVIOUS_POSITIONS for physical robot (1 robot) #####

    config.PREVIOUS_POSITIONS = []
    robot_history = deque(maxlen=5)
    for _ in range(5):
        robot_history.append(np.zeros(12, dtype=np.float32))
    config.PREVIOUS_POSITIONS.append(robot_history)
    logging.debug("PREVIOUS_POSITIONS initialized for physical robot with zeros")

    ##### initialize accelerometer #####

    initialize_accelerometer()

    ##### initialize camera process #####

    if config.CONTROL_MODE not in ('isaac_mirror', 'lan'):
        CAMERA_PROCESS = initialize_camera()  # create camera process
        if CAMERA_PROCESS is None:
            logging.error("(control_logic.py): Failed to initialize CAMERA_PROCESS for robot!\n")
    else:
        logging.info(f"(control_logic.py): Skipping camera init ({config.CONTROL_MODE} mode).\n")

    ##### initialize socket and command queue #####

    if config.CONTROL_MODE == 'web':  # if web control mode and robot needs a socket connection for controls and video...
        SOCK = internet.initialize_backend_socket()  # initialize EC2 socket connection
        COMMAND_QUEUE = internet.initialize_command_queue(SOCK)  # initialize command queue for socket communication
        if SOCK is None:
            logging.error("(control_logic.py): Failed to initialize SOCK for robot!\n")
        if COMMAND_QUEUE is None:
            logging.error("(control_logic.py): Failed to initialize COMMAND_QUEUE for robot!\n")

    ##### initialize channel data #####

    elif config.CONTROL_MODE == 'radio':  # if radio control mode...
        CHANNEL_DATA = initialize_receiver()  # get pigpio instance, decoders, and channel data
        if CHANNEL_DATA == None:
            logging.error("(control_logic.py): Failed to initialize CHANNEL_DATA for robot!\n")

    elif config.CONTROL_MODE == 'isaac_mirror':
        from utilities.isaac_mirror import start_isaac_mirror_server
        if not start_isaac_mirror_server():
            logging.error("(control_logic.py): Failed to start Isaac Lab mirror server!\n")
        else:
            logging.info("(control_logic.py): Isaac Lab mirror mode ready (waiting for desktop).\n")

    elif config.CONTROL_MODE == 'lan':
        from utilities.lan_teleop import start_lan_teleop_server
        COMMAND_QUEUE = start_lan_teleop_server()
        if COMMAND_QUEUE is None:
            logging.error("(control_logic.py): Failed to start LAN teleop server!\n")
        else:
            logging.info("(control_logic.py): LAN teleop ready — desktop → TCP :9001 → on-robot RL.\n")

    ##### initialize PREVIOUS_ORIENTATIONS for physical robot (1 robot) #####

    config.PREVIOUS_ORIENTATIONS = []
    orientation_history = deque(maxlen=5)
    for _ in range(5):
        orientation_history.append(np.zeros(6, dtype=np.float32))  # 6 values: shift, move, translate, yaw, roll, pitch
    config.PREVIOUS_ORIENTATIONS.append(orientation_history)
    logging.debug("PREVIOUS_ORIENTATIONS initialized for physical robot with zeros")


########## PREPARE ROBOT ##########

##### prepare robot with correct dependencies #####

set_real_robot_dependencies()

##### post-initialization dependencies #####

from movement.movement_coordinator import *
from utilities.camera import decode_real_frame





#########################################
############### RUN ROBOT ###############
#########################################


########## STATE MACHINE LOOPS ##########

##### set global variables #####

IMAGELESS_GAIT = True  # set global variable for imageless gait
IS_COMPLETE = True  # boolean that tracks if the robot is done moving, independent of it being neutral or not
IS_NEUTRAL = False  # set global neutral standing boolean
CURRENT_LEG = 'FL'  # set global current leg


##### physical loop #####

def _physical_loop(CHANNEL_DATA):  # central function that runs robot in real life

    ##### set/initialize variables #####

    global IS_COMPLETE, IS_NEUTRAL, CURRENT_LEG  # declare as global as these will be edited by function
    mjpeg_buffer = b''  # initialize buffer for MJPEG frames

    ##### isaac_mirror: dedicated joint stream loop (no website / RL) #####

    if config.CONTROL_MODE == 'isaac_mirror':
        _isaac_mirror_loop()
        return

    ##### lan: keyboard teleop into on-robot RL (no camera / website) #####

    if config.CONTROL_MODE == 'lan':
        _lan_teleop_loop()
        return

    ##### run robotic logic #####

    try:  # try to run robot startup sequence
        neutral_position(1)
        time.sleep(3)
        IS_NEUTRAL = True  # set is_neutral to True

    except Exception as e:  # if there is an error, log error
        logging.error(f"(control_logic.py): Failed to move to neutral standing position in runRobot: {e}\n")

    ##### stream video, run inference, and control the robot #####

    try:  # try to run main robotic process

        while True:  # central loop to entire process, commenting out of importance

            mjpeg_buffer, streamed_frame, inference_frame = decode_real_frame(  # run camera and decode frame
                CAMERA_PROCESS,
                mjpeg_buffer
            )
            command = None  # initially no command

            if config.CONTROL_MODE == 'web':  # if web control enabled...
                internet.stream_to_backend(SOCK, streamed_frame)

                if COMMAND_QUEUE is not None and not COMMAND_QUEUE.empty():  # if command queue is not empty...
                    command = COMMAND_QUEUE.get()  # get command from queue
                    if command is not None:
                        if IS_COMPLETE:  # if movement is complete, run command
                            logging.info(f"(control_logic.py): Received command '{command}' from queue (WILL RUN).\n")
                        else:
                            logging.info(f"(control_logic.py): Received command '{command}' from queue (BLOCKED).\n")

            if config.CONTROL_MODE == 'radio':
                commands = interpret_commands(CHANNEL_DATA)
                if IS_COMPLETE:  # if movement is complete, process radio commands
                    logging.debug(f"(control_logic.py): Processing radio commands: {commands}\n")
                    threading.Thread(target=_handle_command, args=(commands, inference_frame), daemon=True).start()

            if command and IS_COMPLETE:  # if command present and movement complete...
                # logging.debug(f"(control_logic.py): Running command: {command}...\n")
                threading.Thread(target=_handle_command, args=(command, inference_frame), daemon=True).start()

            elif not command and IS_COMPLETE and not IS_NEUTRAL:  # if no command and move complete and not neutral...
                logging.debug(f"(control_logic.py): No command received, returning to neutral position...\n")
                threading.Thread(target=_handle_command, args=('n', inference_frame), daemon=True).start()

    except KeyboardInterrupt:  # if user ends program...
        logging.info("(control_logic.py): KeyboardInterrupt received, exiting.\n")

    except Exception as e:  # if something breaks and only God knows what it is...
        logging.error(f"(control_logic.py): Unexpected exception in main loop: {e}\n")
        exit(1)


def _isaac_mirror_loop():
    """Apply absolute joint targets streamed from Isaac Lab over LAN."""
    global IS_NEUTRAL

    from utilities.isaac_mirror import get_latest_message
    from movement.movement_coordinator import apply_isaac_joint_targets, apply_isaac_default_pose

    logging.info("(control_logic.py): Isaac mirror loop starting — moving to Isaac default pose.\n")
    try:
        apply_isaac_default_pose(speed_rad_s=0.5)
        time.sleep(1.0)
        IS_NEUTRAL = True
    except Exception as e:
        logging.error(f"(control_logic.py): Failed Isaac default pose: {e}\n")

    holding_default = True
    last_seq = None

    try:
        while True:
            msg = get_latest_message()
            if msg is None:
                if not holding_default:
                    logging.warning("(control_logic.py): Isaac stream timeout — holding default pose.\n")
                    apply_isaac_default_pose()
                    holding_default = True
                    IS_NEUTRAL = True
                time.sleep(0.05)
                continue

            msg_type = msg.get("type", "joint_targets")
            seq = msg.get("seq")
            if seq is not None and seq == last_seq:
                time.sleep(0.01)
                continue
            last_seq = seq

            if msg_type == "neutral":
                apply_isaac_default_pose(speed_rad_s=msg.get("speed_rad_s"))
                holding_default = True
                IS_NEUTRAL = True
                continue

            joints = msg.get("joints") or {}
            speed = msg.get("speed_rad_s")
            apply_isaac_joint_targets(joints, speed_rad_s=speed)
            holding_default = False
            IS_NEUTRAL = False

    except KeyboardInterrupt:
        logging.info("(control_logic.py): Isaac mirror KeyboardInterrupt — default pose then exit.\n")
        apply_isaac_default_pose(speed_rad_s=0.4)
    except Exception as e:
        logging.error(f"(control_logic.py): Isaac mirror loop failed: {e}\n")
        exit(1)


def _lan_teleop_loop():
    """Receive WASD commands from desktop and run on-robot RL gait."""
    global IS_COMPLETE, IS_NEUTRAL, CURRENT_LEG

    logging.info("(control_logic.py): LAN teleop loop starting — neutral pose, waiting for desktop.\n")
    try:
        neutral_position(1)
        time.sleep(2.0)
        IS_NEUTRAL = True
    except Exception as e:
        logging.error(f"(control_logic.py): Failed LAN neutral pose: {e}\n")

    try:
        while True:
            command = None
            if COMMAND_QUEUE is not None and not COMMAND_QUEUE.empty():
                command = COMMAND_QUEUE.get()

            if command and IS_COMPLETE:
                logging.info(f"(control_logic.py): LAN command '{command}' (WILL RUN).\n")
                threading.Thread(target=_handle_command, args=(command, None), daemon=True).start()
            elif not command and IS_COMPLETE and not IS_NEUTRAL:
                threading.Thread(target=_handle_command, args=('n', None), daemon=True).start()

            time.sleep(0.02)

    except KeyboardInterrupt:
        logging.info("(control_logic.py): LAN teleop KeyboardInterrupt — neutral then exit.\n")
        try:
            neutral_position(1)
        except Exception:
            pass
    except Exception as e:
        logging.error(f"(control_logic.py): LAN teleop loop failed: {e}\n")
        exit(1)


########## HANDLE COMMANDS ##########

def _handle_command(command, camera_frames=None):
    # logging.debug(f"(control_logic.py): Threading command: {command}...\n")

    global IS_COMPLETE, IS_NEUTRAL, CURRENT_LEG
    IS_COMPLETE = False  # block new commands until movement is complete

    if isinstance(command, str):
        if '+' in command:
            keys = command.split('+')
        elif command == 'n':
            keys = []
        else:
            keys = [command]
    elif isinstance(command, (list, tuple)):
        keys = list(command)
    elif isinstance(command, dict):
        keys = []  # not used for radio mode
    else:
        keys = []

    if config.CONTROL_MODE == 'radio':
        try:
            logging.debug(f"(control_logic.py): Executing radio commands: {command}...\n")
            IS_NEUTRAL, CURRENT_LEG = _execute_radio_commands(
                command,  # command should be the full commands dict from interpret_commands
                camera_frames,
                IS_NEUTRAL,
                CURRENT_LEG
            )
            logging.info(f"(control_logic.py): Executed radio commands: {command}\n")
            IS_COMPLETE = True
        except Exception as e:
            logging.error(f"(control_logic.py): Failed to execute radio commands: {e}\n")
            IS_NEUTRAL = False
            IS_COMPLETE = True

    elif config.CONTROL_MODE in ('web', 'lan'):

        intensity = 10

        try:
            IS_NEUTRAL, CURRENT_LEG = _execute_keyboard_commands(
                keys,
                camera_frames,
                IS_NEUTRAL,
                CURRENT_LEG,
                intensity
            )
            # logging.info(f"(control_logic.py): Executed keyboard command: {keys}\n")
            IS_COMPLETE = True
        except Exception as e:
            logging.error(f"(control_logic.py): Failed to execute keyboard command: {e}\n")
            IS_NEUTRAL = False
            IS_COMPLETE = True


########## EXECUTE COMMANDS ##########

def _convert_direction_parts_to_fixed_list(direction_parts):

    fixed_direction = [None, None, None, None] # initialize fixed-length list with None values
    
    for part in direction_parts:
        if part in ['w', 's']:
            fixed_direction[0] = part
        elif part in ['a', 'd']:
            fixed_direction[1] = part
        elif part in ['arrowleft', 'arrowright']:
            fixed_direction[2] = part
        elif part in ['arrowup', 'arrowdown']:
            fixed_direction[3] = part
        elif part in ['w+a', 'w+d', 's+a', 's+d']:
            if 'w' in part:
                fixed_direction[0] = 'w'
            elif 's' in part:
                fixed_direction[0] = 's'
            if 'a' in part:
                fixed_direction[1] = 'a'
            elif 'd' in part:
                fixed_direction[1] = 'd'
    
    return fixed_direction


########## KEYBOARD COMMANDS ##########

def _execute_keyboard_commands(keys, camera_frames, is_neutral, current_leg, intensity):

    ##### set variables #####

    global IMAGELESS_GAIT # set IMAGELESS_GAIT as global to switch between modes via button press
    direction_parts = [] # diretion part list

    ##### handle special toggle keys #####

    if 'i' in keys:  # if user wishes to enable/disable imageless gait...
        IMAGELESS_GAIT = not IMAGELESS_GAIT  # toggle imageless gait mode
        logging.warning(f"(control_logic.py): Toggled IMAGELESS_GAIT to {IMAGELESS_GAIT}\n")
        keys = [k for k in keys if k != 'i']  # remove 'i' from the keys list

    ##### cancel out contradictory keys #####

    if 'w' in keys and 's' in keys:
        keys = [k for k in keys if k not in ['w', 's']]
    if 'a' in keys and 'd' in keys:
        keys = [k for k in keys if k not in ['a', 'd']]
    if 'arrowleft' in keys and 'arrowright' in keys:
        keys = [k for k in keys if k not in ['arrowleft', 'arrowright']]
    if 'arrowup' in keys and 'arrowdown' in keys:
        keys = [k for k in keys if k not in ['arrowup', 'arrowdown']]

    ##### WASD and diagonals #####

    move_forward = 'w' in keys
    move_backward = 's' in keys
    shift_left = 'a' in keys
    shift_right = 'd' in keys

    ##### rotation #####

    rotate_left = 'arrowleft' in keys
    rotate_right = 'arrowright' in keys

    ##### tilt #####

    tilt_up = 'arrowup' in keys
    tilt_down = 'arrowdown' in keys

    ##### handle diagonals #####

    if move_forward and shift_left:
        direction_parts.append('w+a')
    elif move_forward and shift_right:
        direction_parts.append('w+d')
    elif move_backward and shift_left:
        direction_parts.append('s+a')
    elif move_backward and shift_right:
        direction_parts.append('s+d')

    ##### handle single directions #####

    elif move_forward:
        direction_parts.append('w')
    elif move_backward:
        direction_parts.append('s')
    elif shift_left:
        direction_parts.append('a')
    elif shift_right:
        direction_parts.append('d')

    ##### handle rotation #####

    if rotate_left:
        direction_parts.append('arrowleft')
    elif rotate_right:
        direction_parts.append('arrowright')

    ##### handle tilt #####

    if tilt_up:
        direction_parts.append('arrowup')
    elif tilt_down:
        direction_parts.append('arrowdown')

    ##### combine all direction parts into fixed-length list #####

    direction = None
    if direction_parts:
        direction = _convert_direction_parts_to_fixed_list(direction_parts)
    if 'n' in keys or not keys:
        neutral_position(10)
        is_neutral = True
    elif direction:
        move_direction(direction, camera_frames, intensity, IMAGELESS_GAIT)
        is_neutral = False
    else:
        logging.warning(f"(control_logic.py): Invalid command: {keys}.\n")

    return is_neutral, current_leg


########## RADIO COMMANDS ##########

def _execute_radio_commands(commands, camera_frames, is_neutral, current_leg):

    ##### set variables #####

    global IMAGELESS_GAIT  # set IMAGELESS_GAIT as global to switch between modes via button press
    IMAGELESS_GAIT = True
    active_commands = {
        channel: (action, intensity) for channel, (action, intensity) in commands.items() if action != 'NEUTRAL'
    }
    direction_parts = []

    ##### handle no active commands #####

    if not active_commands:
        logging.info(f"(control_logic.py): All channels neutral, returning to neutral position.\n")
        neutral_position(10)
        is_neutral = True
        return is_neutral, current_leg

    ##### cancel out contradictory commands #####

    move_actions = []
    for channel in ['channel_5', 'channel_6']:
        if channel in active_commands:
            action = active_commands[channel][0]
            if 'MOVE' in action or 'SHIFT' in action:
                move_actions.append((channel, action))

    ##### movement combinations (channels 5 and 6) #####

    move_forward = active_commands.get('channel_5', ('NEUTRAL', 0))[0] == 'MOVE FORWARD'
    move_backward = active_commands.get('channel_5', ('NEUTRAL', 0))[0] == 'MOVE BACKWARD'
    shift_left = active_commands.get('channel_6', ('NEUTRAL', 0))[0] == 'SHIFT LEFT'
    shift_right = active_commands.get('channel_6', ('NEUTRAL', 0))[0] == 'SHIFT RIGHT'

    ##### rotation (channel 3) #####

    rotate_left = active_commands.get('channel_3', ('NEUTRAL', 0))[0] == 'ROTATE LEFT'
    rotate_right = active_commands.get('channel_3', ('NEUTRAL', 0))[0] == 'ROTATE RIGHT'

    ##### tilt (channel 4) #####

    tilt_up = active_commands.get('channel_4', ('NEUTRAL', 0))[0] == 'TILT UP'
    tilt_down = active_commands.get('channel_4', ('NEUTRAL', 0))[0] == 'TILT DOWN'

    ###### special actions (channels 0, 1, 2, 7) #####

    look_up = active_commands.get('channel_0', ('NEUTRAL', 0))[0] == 'LOOK UP'
    look_down = active_commands.get('channel_0', ('NEUTRAL', 0))[0] == 'LOOK DOWN'
    trigger_shoot = active_commands.get('channel_1', ('NEUTRAL', 0))[0] == 'TRIGGER SHOOT'
    squat_down = active_commands.get('channel_2', ('NEUTRAL', 0))[0] == 'SQUAT DOWN'
    squat_up = active_commands.get('channel_2', ('NEUTRAL', 0))[0] == 'SQUAT UP'
    plus_action = active_commands.get('channel_7', ('NEUTRAL', 0))[0] == '+'
    minus_action = active_commands.get('channel_7', ('NEUTRAL', 0))[0] == '-'

    ##### handle diagonals #####

    if move_forward and shift_left:
        direction_parts.append('w+a')
        max_intensity = max(
            active_commands.get('channel_5', ('NEUTRAL', 0))[1],
            active_commands.get('channel_6', ('NEUTRAL', 0))[1]
        )
    elif move_forward and shift_right:
        direction_parts.append('w+d')
        max_intensity = max(
            active_commands.get('channel_5', ('NEUTRAL', 0))[1],
            active_commands.get('channel_6', ('NEUTRAL', 0))[1]
        )
    elif move_backward and shift_left:
        direction_parts.append('s+a')
        max_intensity = max(
            active_commands.get('channel_5', ('NEUTRAL', 0))[1],
            active_commands.get('channel_6', ('NEUTRAL', 0))[1]
        )
    elif move_backward and shift_right:
        direction_parts.append('s+d')
        max_intensity = max(
            active_commands.get('channel_5', ('NEUTRAL', 0))[1],
            active_commands.get('channel_6', ('NEUTRAL', 0))[1]
        )

    ##### handle single directions #####

    elif move_forward:
        direction_parts.append('w')
        max_intensity = active_commands.get('channel_5', ('NEUTRAL', 0))[1]
    elif move_backward:
        direction_parts.append('s')
        max_intensity = active_commands.get('channel_5', ('NEUTRAL', 0))[1]
    elif shift_left:
        direction_parts.append('a')
        max_intensity = active_commands.get('channel_6', ('NEUTRAL', 0))[1]
    elif shift_right:
        direction_parts.append('d')
        max_intensity = active_commands.get('channel_6', ('NEUTRAL', 0))[1]

    ##### handle rotation #####

    if rotate_left:
        direction_parts.append('arrowleft')
        max_intensity = max(config.DEFAULT_INTENSITY, active_commands.get('channel_3', ('NEUTRAL', 0))[1])
    elif rotate_right:
        direction_parts.append('arrowright')
        max_intensity = max(config.DEFAULT_INTENSITY, active_commands.get('channel_3', ('NEUTRAL', 0))[1])

    ##### handle tilt #####

    if tilt_up:
        direction_parts.append('arrowup')
        max_intensity = max(config.DEFAULT_INTENSITY, active_commands.get('channel_4', ('NEUTRAL', 0))[1])
    elif tilt_down:
        direction_parts.append('arrowdown')
        max_intensity = max(config.DEFAULT_INTENSITY, active_commands.get('channel_4', ('NEUTRAL', 0))[1])

    ##### handle special actions #####

    special_actions = []
    if look_up:
        special_actions.append('LOOK_UP')
    elif look_down:
        special_actions.append('LOOK_DOWN')
    if trigger_shoot:
        special_actions.append('TRIGGER_SHOOT')
    if squat_down:
        special_actions.append('SQUAT_DOWN')
    elif squat_up:
        special_actions.append('SQUAT_UP')
    if plus_action:
        special_actions.append('PLUS')
    elif minus_action:
        special_actions.append('MINUS')

    ##### execute movement or special actions #####

    if direction_parts:
        direction = _convert_direction_parts_to_fixed_list(direction_parts)
        logging.debug(f"(control_logic.py): Radio commands: ({active_commands}:{direction})\n")
        if special_actions:
            logging.debug(f"(control_logic.py): Special actions: ({special_actions})\n")
        move_direction(direction, camera_frames, config.DEFAULT_INTENSITY, IMAGELESS_GAIT) # TODO locking intensity at 10 for now
        is_neutral = False
    elif special_actions:
        # only special actions, no movement
        logging.debug(f"(control_logic.py): Special actions only: ({special_actions})\n")
        # handle special actions here
        if 'SQUAT_DOWN' in special_actions:
            pass
            is_neutral = False
        elif 'SQUAT_UP' in special_actions:
            neutral_position(10)
            is_neutral = True

    return is_neutral, current_leg


########## MISCELLANEOUS CONTROL FUNCTIONS ##########

def restart_process():  # start background thread to restart robot_dog.service every 30 minutes by checking elapsed time
    start_time = time.time()
    while True:
        elapsed = time.time() - start_time
        if elapsed >= 1800:  # 30 minutes = 1800 seconds
            os.system('sudo systemctl restart robot_dog.service')
            start_time = time.time()  # reset timer after restart
        time.sleep(1)  # check every second


def send_voltage_to_backend(voltage):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('127.0.0.1', 3000))  # Use backend's IP if remote
        msg = f'VOLTAGE:{voltage:.4f}'
        s.sendall(msg.encode())
        s.close()
    except Exception as e:
        logging.error(f"(control_logic.py) Failed to send voltage: {e}\n")


def voltage_monitor():
    while True:
        voltage_output = os.popen('vcgencmd measure_voltage').read()
        voltage_str = voltage_output.strip().replace('volt=', '').replace('V', '')
        try:
            voltage = float(voltage_str)
            if voltage < 0.8600:
                logging.warning(f"(control_logic.py) Low voltage ({voltage:.4f}V) detected!\n")
            send_voltage_to_backend(voltage)
        except ValueError:
            logging.error(f"(control_logic.py) Failed to parse voltage: {voltage_output}\n")
        time.sleep(10)  # check every 10 seconds


########## RUN ROBOTIC PROCESS ##########

restart_thread = threading.Thread(target=restart_process, daemon=True)
restart_thread.start()
# voltage_thread = threading.Thread(target=voltage_monitor, daemon=True)
# voltage_thread.start()
_physical_loop(CHANNEL_DATA)  # run robot process

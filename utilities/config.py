##################################################################################
# Copyright (c) 2025 Matthew Thomas Beck                                         #
#                                                                                #
# Licensed under the Creative Commons Attribution-NonCommercial 4.0              #
# International (CC BY-NC 4.0). Personal and educational use is permitted.       #
# Commercial use by companies or for-profit entities is prohibited.              #
##################################################################################





##########################################################
############### IMPORT/CREATE DEPENDENCIES ###############
##########################################################


########## IMPORT DEPENDENCIES ##########

##### import necessary libraries #####

import time # import time library for gait timing
import logging # import logging library for debugging





#####################################################
############### CREATE CONFIGURATIONS ###############
#####################################################


########## UTILITY CONFIGURATIONS ##########

##### set global fps to be used by all modules #####

LOOP_RATE_HZ = 30 # global loop rate in Hz for all modules TODO DEPRECATED/LEGACY
CONTROL_MODE = 'isaac_mirror'  # 'web' | 'radio' | 'isaac_mirror' | 'lan'
RL_NOT_CNN = True # boolean to switch between testing and RL models (true is RL, false is testing)
DEFAULT_INTENSITY = 10 # default intensity for keyboard commands (1 to 10)

# Gait timing / Maestro speed (NOT a policy output — see note below).
#
# Datasheet no-load speeds for these 45kg digital servos:
#   5.0V: 0.18 s / 60°  →  (π/3)/0.18 ≈ 5.82 rad/s
#   7.4V: 0.13 s / 60°  →  (π/3)/0.13 ≈ 8.06 rad/s
#   8.4V: 0.11 s / 60°  →  (π/3)/0.11 ≈ 9.52 rad/s
# Isaac Lab DCMotorCfg.velocity_limit and servos.map_radian_to_servo_speed
# already use 9.52 as the clamp (8.4V no-load). Under load, true speed is lower.
#
# The PPO policy outputs 12 joint *positions* only (default + 0.5*action).
# It does not output joint speeds. Maestro still needs a speed limit for set_target;
# that used to be hardcoded as 1.0 rad/s in inference.py ("legacy support").
GAIT_CONFIG = {
    'SERVO_SPEED_RAD_S': 9.52,     # Maestro cmd speed: datasheet 8.4V no-load max
    'POLICY_DT_S': 0.0875,         # match Isaac Lab hardware-rate cache
    'SETTLE_MARGIN_S': 0.04,       # extra wait after estimated travel time
    'MAX_STEP_WAIT_S': 0.35,       # cap so a stuck estimate can't freeze the dog
    # Legacy: clip RL / move_joint / PWM-map targets to SERVO_CONFIG FULL_*_ANGLE.
    # False = let policy targets through (still need FULL_* for angle↔PWM mapping).
    'servo_range_clamping': False,
}

# No odometry: fill policy base_lin_vel from the velocity *command* (what sim
# usually tracks closely), not zeros. Scale < 1 if real robot lags the command.
BASE_LIN_VEL_CMD_SCALE = 1.0
BASE_LIN_VEL_EMA_ALPHA = 0.35  # 0→freeze estimate, 1→instant jump to command
EST_BASE_LIN_VEL = None  # runtime EMA state (xyz m/s); set in inference.py

##### set logging configuration #####

LOG_CONFIG = {
    'LOG_PATH': "/home/matthewthomasbeck/Projects/Robot_Dog/robot_dog.log", # path to log file DO NOT CHANGE
    'LOG_LEVEL': logging.INFO # set log level to logging.<DEBUG, INFO, WARNING, ERROR, or CRITICAL>
}

########## CAMERA CONFIGURATION ##########

##### set camera configuration #####

CAMERA_CONFIG = { # TODO BE VERY CAREFUL WITH OUTPUT WIDTHxHEIGHT! Remember, the height gets cut in half via 0.5 crop!
    'FOV': 75, # degrees
    'CAMERA_WIDTH': 4608,
    'CAMERA_HEIGHT': 2592,
    'FOV_HORIZONTAL': 66,  # degrees
    'FOV_VERTICAL': 41,  # degrees
    'PIXEL_SIZE_UM': 1.4,  # pixel size in micrometers
    'DEPTH_OF_FIELD': 0.1,  # depth of field distance in meters
    'APERTURE_RATIO': 1.8,
    'WIDTH': 640, # width of the camera image
    'HEIGHT': 480, # height of the camera image
    'FRAME_RATE': 30, # frame rate of the camera in frames per second
    'CROP_FRACTION': 0.5, # fraction of the image to crop from each side (0.0 to 1.0)
    'OUTPUT_WIDTH': 128, # width of the ML image
    'OUTPUT_HEIGHT': 48, # height of the image for ML inference
}


########## INFERENCE CONFIGURATIONS ##########

##### set ML configurations #####

INFERENCE_CONFIG = {
    'TPU_NAME': "MYRIAD",  # literal device name in code
    'STANDARD_RL_PATH': "/home/matthewthomasbeck/Projects/Robot_Dog/model/standard", # standard all terrain RL model
    'BLIND_RL_PATH': "/home/matthewthomasbeck/Projects/Robot_Dog/model/blind_rl_model.xml", # speedy imageless RL model
    'CNN_PATH': "/home/matthewthomasbeck/Projects/Robot_Dog/model/person-detection-0200.xml",  # person detection
}


########## ROBOT CONTROL CONFIGURATIONS (internet and radio) ##########

##### declare movement channel GPIO pins #####

SIGNAL_TUNING_CONFIG = { # dictionary of signal tuning configuration for sensitivity
    'JOYSTICK_THRESHOLD': 40, # number of times condition must be met to trigger a request on a joystick channel
    'TOGGLE_THRESHOLD': 40, # number of times condition must be met to trigger a request on a button channel
    'TIME_FRAME': 0.10017, # time frame for condition to be met, default: 0.100158
    'DEADBAND_HIGH': 1600, # deadband high for PWM signal
    'DEADBAND_LOW': 1400 # deadband low for PWM signal
}

##### set receiver channels #####

RECEIVER_CHANNELS = { # dictionary of receiver channels' names, GPIO pins, and states
    'channel_0': {'name': 'tilt_up_down', 'gpio_pin': 17, 'counter': 0, 'timestamp': time.time()},
    'channel_1': {'name': 'trigger_shoot', 'gpio_pin': 27, 'counter': 0, 'timestamp': time.time()},
    'channel_2': {'name': 'squat_up_down', 'gpio_pin': 22, 'counter': 0, 'timestamp': time.time()},
    'channel_3': {'name': 'rotate_left_right', 'gpio_pin': 5, 'counter': 0, 'timestamp': time.time()},
    'channel_4': {'name': 'look_up_down', 'gpio_pin': 6, 'counter': 0, 'timestamp': time.time()},
    'channel_5': {'name': 'move_forward_backward', 'gpio_pin': 13, 'counter': 0, 'timestamp': time.time()},
    'channel_6': {'name': 'shift_left_right', 'gpio_pin': 19, 'counter': 0, 'timestamp': time.time()},
    'channel_7': {'name': 'extra_channel', 'gpio_pin': 26, 'counter': 0, 'timestamp': time.time()},
}

##### set receiver configuration #####

MAESTRO_CONFIG = {
    'SERIAL_PATH': "/dev/serial0", # set serial port name to first available
    'SERIAL_BAUD_RATE': 9600, # set baud rate for serial connection
    'SERIAL_TIMEOUT': 1 # set timeout for serial connection
}

##### set internet connectivity configuration #####

INTERNET_CONFIG = {
    'BACKEND_API_URL': "https://api.matthewthomasbeck.com", # URL of the backend API endpoint
    'BACKEND_PUBLIC_IP': "72.177.232.19", # public IP address of backend
    'BACKEND_PORT': 3000, # port number for backend (fixed typo from 'BACKED_PORT')
    'SSH_SOCKET_PATH': "/tmp/robot.sock" # path to unix socket for SSH communication
}

# Direct Isaac Lab → hardware joint mirror (LAN). Desktop connects TO the robot.
ISAAC_MIRROR_CONFIG = {
    'BIND_HOST': '0.0.0.0',
    'PORT': 9000,
    'TIMEOUT_S': 0.75,  # no packet → hold Isaac default standing pose
    'DEFAULT_SPEED_RAD_S': 0.8,
    'MAX_DELTA_RAD': 1.5,  # allow full joint travel for limit_sweep calibration
}

# Desktop keyboard teleop → on-robot RL (no website backend).
LAN_TELEOP_CONFIG = {
    'BIND_HOST': '0.0.0.0',
    'PORT': 9001,
}


########## PHYSICAL CONFIGURATION ##########

##### robot top speed #####

TOP_SPEED = 0.6 # in m/s

##### set dictionary of servos and their ranges #####

SERVO_CONFIG = { # dictionary of leg configurations

    # Calibrated FRONT / NEUTRAL / BACK triples (angle rad ↔ PWM µs).
    # Uppers: FL recipe + FULL_BACK extended 30° further backward (PWM via F→B rate).
    # FRONT unchanged; NEUTRAL PWM unchanged for now.
    'FL': {'hip': {'servo': 3, 'FULL_FRONT': 1782.37, 'FULL_BACK': 1345.88, 'NEUTRAL': 1520.68, 'CURRENT': 1520.68, 'FULL_FRONT_ANGLE': 0.435335, 'FULL_BACK_ANGLE': -0.349066, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'upper': {'servo': 5, 'FULL_FRONT': 1178.86, 'FULL_BACK': 1964.94, 'NEUTRAL': 1681.15, 'CURRENT': 1681.15, 'FULL_FRONT_ANGLE': -0.828533, 'FULL_BACK_ANGLE': 0.741267, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'lower': {'servo': 4, 'FULL_FRONT': 1117.40, 'FULL_BACK': 1947.54, 'NEUTRAL': 1554.32, 'CURRENT': 1554.32, 'FULL_FRONT_ANGLE': -0.785267, 'FULL_BACK_ANGLE': 0.872534, 'CURRENT_ANGLE': 0.087266, 'NEUTRAL_ANGLE': 0.087266}},

    'FR': {'hip': {'servo': 2, 'FULL_FRONT': 1084.63, 'FULL_BACK': 1521.12, 'NEUTRAL': 1346.32, 'CURRENT': 1346.32, 'FULL_FRONT_ANGLE': -0.435335, 'FULL_BACK_ANGLE': 0.349066, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'upper': {'servo': 1, 'FULL_FRONT': 2030.64, 'FULL_BACK': 1244.56, 'NEUTRAL': 1528.35, 'CURRENT': 1528.35, 'FULL_FRONT_ANGLE': 0.828533, 'FULL_BACK_ANGLE': -0.741267, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'lower': {'servo': 0, 'FULL_FRONT': 2009.10, 'FULL_BACK': 1178.96, 'NEUTRAL': 1572.18, 'CURRENT': 1572.18, 'FULL_FRONT_ANGLE': 0.785267, 'FULL_BACK_ANGLE': -0.872534, 'CURRENT_ANGLE': -0.087266, 'NEUTRAL_ANGLE': -0.087266}},

    'BL': {'hip': {'servo': 8, 'FULL_FRONT': 1111.51, 'FULL_BACK': 1548.00, 'NEUTRAL': 1373.20, 'CURRENT': 1373.20, 'FULL_FRONT_ANGLE': -0.435335, 'FULL_BACK_ANGLE': 0.349066, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'upper': {'servo': 7, 'FULL_FRONT': 1362.11, 'FULL_BACK': 2322.99, 'NEUTRAL': 1864.40, 'CURRENT': 1864.40, 'FULL_FRONT_ANGLE': -0.828533, 'FULL_BACK_ANGLE': 0.392201, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'lower': {'servo': 6, 'FULL_FRONT': 1176.15, 'FULL_BACK': 2006.29, 'NEUTRAL': 1613.07, 'CURRENT': 1613.07, 'FULL_FRONT_ANGLE': -0.785267, 'FULL_BACK_ANGLE': 0.872534, 'CURRENT_ANGLE': 0.087266, 'NEUTRAL_ANGLE': 0.087266}},

    'BR': {'hip': {'servo': 11, 'FULL_FRONT': 1772.62, 'FULL_BACK': 1336.13, 'NEUTRAL': 1510.93, 'CURRENT': 1510.93, 'FULL_FRONT_ANGLE': 0.435335, 'FULL_BACK_ANGLE': -0.349066, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'upper': {'servo': 10, 'FULL_FRONT': 1698.26, 'FULL_BACK': 737.40, 'NEUTRAL': 1195.98, 'CURRENT': 1195.98, 'FULL_FRONT_ANGLE': 0.828533, 'FULL_BACK_ANGLE': -0.392201, 'CURRENT_ANGLE': 0.0, 'NEUTRAL_ANGLE': 0.0},
           'lower': {'servo': 9, 'FULL_FRONT': 2004.10, 'FULL_BACK': 1173.96, 'NEUTRAL': 1567.18, 'CURRENT': 1567.18, 'FULL_FRONT_ANGLE': 0.785267, 'FULL_BACK_ANGLE': -0.872534, 'CURRENT_ANGLE': -0.087266, 'NEUTRAL_ANGLE': -0.087266}},
}

##### previous positions #####

PREVIOUS_POSITIONS = [] # array of previous positions for each robot

##### previous orientations #####

PREVIOUS_ORIENTATIONS = [] # array of previous orientations for each robot (shift, move, translate, yaw, roll, pitch)

##### joint ordering configuration #####

# Exact joint order from Isaac Lab JointPositionAction resolution
# (logged as: Resolved joint names for the action term JointPositionAction).
# Do not reorder — obs, last_action, and action decode must all use this list.
ISAAC_JOINT_ORDER = [
    "BL_hip", "BR_hip", "FL_hip", "FR_hip",
    "BL_upper", "BR_upper", "FL_upper", "FR_upper",
    "BL_lower", "BR_lower", "FL_lower", "FR_lower",
]

# Default standing/camber pose from Isaac Lab ROBOT_DOG_CFG.init_state.joint_pos
ISAAC_DEFAULT_JOINT_POS = {
    "BL_hip": -0.1465,
    "BR_hip": 0.1465,
    "FL_hip": 0.1465,
    "FR_hip": -0.1465,
    "BL_upper": 0.1465,
    "BR_upper": -0.1465,
    "FL_upper": -0.1465,
    "FR_upper": 0.1465,
    "BL_lower": 0.087266,
    "BR_lower": -0.087266,
    "FL_lower": 0.087266,
    "FR_lower": -0.087266,
}

# Legacy toggle kept for reference only. Inference now always uses ISAAC_JOINT_ORDER.
# "by_leg": FL/FR/BL/BR each hip-upper-lower
# "by_type_legacy": FL,FR,BL,BR within each joint type (WRONG vs Isaac — was FL/FR/BL/BR, Isaac is BL/BR/FL/FR)
JOINT_ORDERING_SCHEME = "isaac"  # Options: "isaac" (required), legacy: "by_leg", "by_type"

##### set accelerometer configuration #####

ACCELEROMETER_CONFIG = { # dictionary of accelerometer configuration

    'MPU_6050_ADDRESS': 0x68, # address of the accelerometer
    'PWR_MGMT_1': 0x6B, # power management register
    'SMPLRT_DIV': 0x19, # sample rate divider
    'CONFIG_REGISTER': 0x1A, # configuration register
    'GYRO_CONFIG': 0x1B, # gyro configuration register
    'INT_ENABLE': 0x38, # interrupt enable register
    'ACCEL_XOUT_H': 0x3B, # accelerometer x-axis output high register
    'ACCEL_YOUT_H': 0x3D, # accelerometer y-axis output high register
    'ACCEL_ZOUT_H': 0x3F, # accelerometer z-axis output high register
    'GYRO_XOUT_H': 0x43, # gyroscope x-axis output high register
    'GYRO_YOUT_H': 0x45, # gyroscope y-axis output high register
    'GYRO_ZOUT_H': 0x47 # gyroscope z-axis output high register
}
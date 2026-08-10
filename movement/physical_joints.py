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

##### import config #####

import utilities.config as config

##### import necessary libraries #####

import logging
import time

##### import necessary functions #####

from utilities.servos import map_angle_to_servo_position, set_target, map_radian_to_servo_speed





########################################################
############### PHYSICAL JOINT FUNCTIONS ###############
########################################################


########## ANGLE-BASED LEG MOVEMENT ##########

##### swing selected leg #####

def swing_leg(leg_id, target_angles, movement_rates):

    try:
        move_joints_to_angles(leg_id, target_angles, movement_rates)
        time.sleep(0.05)

    except Exception as e:
        logging.error(f"(physical_joints.py): Failed to swing leg {leg_id} with angles: {e}\n")

##### move joint from mid to target angles #####

def move_joints_to_angles(leg_id, end_angles, movement_rates):

    ##### move each joint to its target angle #####
    # Lower (tibia) first, then upper (femur), then hip — pre-position the tibia
    # before the femur swings so compensation isn't "dragging feet".
    joint_order = ['lower', 'upper', 'hip']
    commanded_upper = end_angles.get('upper')

    for joint_name in joint_order:

        try: # attempt to move the joint
            end_angle = end_angles[joint_name]
            speed = movement_rates[joint_name]
            move_joint(
                leg_id,
                joint_name,
                end_angle,
                speed,
                upper_serial_for_compensation=commanded_upper,
            )
            # Give tibia a head start before femur swing (lift foot, then swing leg).
            if joint_name == 'lower':
                time.sleep(0.03)
            
        except Exception as e: # if unable to move joint...
            logging.error(f"(physical_joints.py): Failed to move {leg_id}_{joint_name} to angle: {e}\n")

##### move single joint to target angle #####

def _serial_to_hip_mounted_servo_angle(leg_id, joint_name, serial_angle, upper_serial=None):
    """Convert Isaac/serial joint angle to hip-mounted servo shaft angle.

    Femur and tibia servos are both mounted at the hip with ~equal ligament arc
    lengths (1:1). Holding the lower servo fixed while the femur moves by ``+x``
    passively changes the knee angle (tibia vs femur) by ``-x``.

    Policy / Isaac command **serial** angles (tibia relative to femur). To keep
    that relative angle when the femur moves, advance the lower servo by the same
    femur delta:

        servo_lower = serial_lower + (serial_upper - upper_default)

    Hip and upper map 1:1 (serial == servo).

    When lower moves before upper (preferred), pass this step's commanded
    ``upper_serial`` so compensation isn't based on a stale CURRENT_ANGLE.
    """
    if joint_name != "lower":
        return float(serial_angle)

    upper_key = f"{leg_id}_upper"
    u0 = float(config.ISAAC_DEFAULT_JOINT_POS[upper_key])
    if upper_serial is None:
        u_serial = float(config.SERVO_CONFIG[leg_id]["upper"]["CURRENT_ANGLE"])
    else:
        u_serial = float(upper_serial)
    return float(serial_angle) + (u_serial - u0)


def move_joint(
    leg_id,
    joint_name,
    target_angle,
    speed,
    compensate_lower=True,
    upper_serial_for_compensation=None,
):

    ##### move the joint to the target angle at the specified speed #####
    # ``target_angle`` is serial / Isaac space. CURRENT_ANGLE stays serial for RL.
    # PWM uses hip-mounted servo space (compensated for lower) when enabled.

    servo_data = config.SERVO_CONFIG[leg_id][joint_name]
    if compensate_lower:
        pwm_angle = _serial_to_hip_mounted_servo_angle(
            leg_id,
            joint_name,
            target_angle,
            upper_serial=upper_serial_for_compensation,
        )
    else:
        pwm_angle = float(target_angle)

    if config.GAIT_CONFIG.get("servo_range_clamping", True):
        min_angle = min(servo_data["FULL_BACK_ANGLE"], servo_data["FULL_FRONT_ANGLE"])
        max_angle = max(servo_data["FULL_BACK_ANGLE"], servo_data["FULL_FRONT_ANGLE"])
        pwm_angle = float(max(min_angle, min(max_angle, pwm_angle)))

    pwm = map_angle_to_servo_position(pwm_angle, servo_data)
    speed = map_radian_to_servo_speed(speed)
    set_target(servo_data['servo'], pwm, speed, 255) # use 255 max acceleration
    config.SERVO_CONFIG[leg_id][joint_name]['CURRENT'] = pwm
    config.SERVO_CONFIG[leg_id][joint_name]['CURRENT_ANGLE'] = float(target_angle)


########## NEUTRAL POSITION ##########

def neutral_position_physical(intensity): # used to move all joints to neutral position on physical robot

    ##### set variables #####

    speed = 9.5  # default to max speed
    # Lower before upper (same swing order as gait), then hip.
    joint_order = [
        ('FL', 'lower'), ('FL', 'upper'), ('FL', 'hip'),
        ('FR', 'lower'), ('FR', 'upper'), ('FR', 'hip'),
        ('BL', 'lower'), ('BL', 'upper'), ('BL', 'hip'),
        ('BR', 'lower'), ('BR', 'upper'), ('BR', 'hip'),
    ]
    
    logging.info("(physical_joints.py): Moving all legs to neutral position on physical robot...\n")
    
    ##### move each joint to neutral position #####

    for leg_id, joint_name in joint_order: # loop through each joint in the leg

        try: # attempt to move the joint
            servo_data = config.SERVO_CONFIG[leg_id][joint_name]
            neutral_angle = servo_data['NEUTRAL_ANGLE']
            # Calibrated PWM neutral is raw servo space (no hip-mounted compensation).
            move_joint(leg_id, joint_name, neutral_angle, speed, compensate_lower=False)
            
        except Exception as e: # if unable to move joint...
            logging.error(f"(physical_joints.py): Failed to move {leg_id}_{joint_name} to neutral: {e}\n")

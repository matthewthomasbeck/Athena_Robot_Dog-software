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

import logging # import logging for debugging

##### import necessary functions #####

import utilities.config as config # gait flags (servo_range_clamping)
from utilities.maestro import initialize_maestro # import maestro initialization functions


########## CREATE DEPENDENCIES ##########

##### create maestro object #####

MAESTRO = initialize_maestro() # create maestro object





#############################################################
############### FUNDAMENTAL MOVEMENT FUNCTION ###############
#############################################################


########## MOVE A SINGLE SERVO ##########

def set_target(channel, target, speed, acceleration): # function to set target position of a singular servo

    ##### move a servo to a desired position using its number and said position #####

    logging.debug(f"(servos.py): Attempting to move servo {channel} to target {target} with speed {speed} and acceleration {acceleration}...\n")

    try: # attempt to move desired servo

        target = int(round(target * 4)) # convert target from microseconds to quarter-microseconds
        speed = max(0, min(16383, speed)) # ensure speed is within valid range
        acceleration = max(0, min(255, acceleration)) # ensure acceleration is within valid range
        speed_command = bytearray([0x87, channel, speed & 0x7F, (speed >> 7) & 0x7F]) # create speed command
        MAESTRO.write(speed_command) # send speed command to maestro

        # create acceleration command
        accel_command = bytearray([0x89, channel, acceleration & 0x7F, (acceleration >> 7) & 0x7F])
        MAESTRO.write(accel_command) # send acceleration command to maestro
        command = bytearray([0x84, channel, target & 0x7F, (target >> 7) & 0x7F]) # create target position command
        MAESTRO.write(command) # send target position command to maestro

    except:
        logging.error("(servos.py): Failed to move servo.\n") # print failure statement


########## ANGLE TO TARGET ##########

def _lerp(x, x0, x1, y0, y1):
    """Linear map x in [x0, x1] → y in [y0, y1]."""
    denom = x1 - x0
    if abs(denom) < 1e-12:
        return float(y0)
    t = (x - x0) / denom
    return float(y0 + t * (y1 - y0))


def map_angle_to_servo_position(angle, joint_data):
    """Map serial/Isaac joint angle (rad) → Maestro PWM (µs).

    Uses the calibrated triple in ``SERVO_CONFIG``:
        (FULL_FRONT_ANGLE ↔ FULL_FRONT),
        (NEUTRAL_ANGLE ↔ NEUTRAL),
        (FULL_BACK_ANGLE ↔ FULL_BACK)

    Piecewise-linear through neutral so asymmetric front/back spans and
    shifted neutrals stay exact. The old fixed ``0.001997 rad/us`` rate is
    no longer used — it drifted after per-joint range tuning.
    """

    neutral_pwm = float(joint_data["NEUTRAL"])
    neutral_angle = float(joint_data.get("NEUTRAL_ANGLE", 0.0))
    full_back_angle = float(joint_data["FULL_BACK_ANGLE"])
    full_front_angle = float(joint_data["FULL_FRONT_ANGLE"])
    full_back_pwm = float(joint_data["FULL_BACK"])
    full_front_pwm = float(joint_data["FULL_FRONT"])

    angle = float(angle)
    clamp_ranges = bool(config.GAIT_CONFIG.get("servo_range_clamping", True))

    # Clamp to calibrated angle range (legacy). Off → keep angle and extrapolate PWM.
    a_lo = min(full_front_angle, full_back_angle)
    a_hi = max(full_front_angle, full_back_angle)
    if clamp_ranges:
        angle = max(a_lo, min(a_hi, angle))

    def _between(a, a0, a1, eps=1e-9):
        return (a - a0) * (a - a1) <= eps

    if _between(angle, neutral_angle, full_front_angle):
        pwm = _lerp(angle, neutral_angle, full_front_angle, neutral_pwm, full_front_pwm)
    elif _between(angle, neutral_angle, full_back_angle):
        pwm = _lerp(angle, neutral_angle, full_back_angle, neutral_pwm, full_back_pwm)
    elif abs(angle - full_front_angle) <= abs(angle - full_back_angle):
        # Past FRONT (or misaligned neutral): use front-side slope.
        pwm = _lerp(angle, neutral_angle, full_front_angle, neutral_pwm, full_front_pwm)
    else:
        # Past BACK: use back-side slope.
        pwm = _lerp(angle, neutral_angle, full_back_angle, neutral_pwm, full_back_pwm)

    if clamp_ranges:
        p_lo = min(full_front_pwm, full_back_pwm)
        p_hi = max(full_front_pwm, full_back_pwm)
        pwm = max(p_lo, min(p_hi, pwm))
    else:
        # Soft Maestro absolute floor/ceiling only (µs); no SERVO_CONFIG span clip.
        pwm = max(500.0, min(2500.0, pwm))

    logging.debug(
        f"(servos.py): angle {angle:.4f} rad -> pwm {pwm:.1f} us "
        f"(N {neutral_angle:.4f}->{neutral_pwm:.1f}, "
        f"F {full_front_angle:.4f}->{full_front_pwm:.1f}, "
        f"B {full_back_angle:.4f}->{full_back_pwm:.1f})\n"
    )

    return int(round(pwm))


########## RADIAN TO SERVO SPEED ##########

def map_radian_to_servo_speed(radian_speed): # function to map radian speed to servo speed

    ##### mao radian speed to servo speed #####

    logging.debug(f"(servos.py): Mapping radian speed {radian_speed} to servo speed...\n")

    radian_speed = max(0.0, min(9.52, radian_speed)) # clamp radian speed to valid range (0 to 9.52 rad/s)
    servo_speed = (radian_speed / 9.52) * 16383 # map radian speed to servo speed (0 to 16383)
    servo_speed = int(round(servo_speed)) # round servo speed to nearest integer
    servo_speed = max(0, min(16383, servo_speed)) # ensure servo speed is within valid range
    
    logging.debug(f"(servos.py): Radian speed {radian_speed:.3f} rad/s -> Servo speed {servo_speed}\n")
    
    return servo_speed

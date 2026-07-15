"""Shared utility functions for ROV Pixhawk control nodes."""

import math


def apply_dead_zone(value, threshold=0.05):
    """Filter out joystick noise near centre, then re-scale to [0, ±1].

    Args:
        value:  Raw axis value in [-1, 1].
        threshold:  Dead-zone width (default 5 %).

    Returns:
        Rescaled value in [-1, 1], or 0.0 inside the dead-zone.
    """
    if abs(value) < threshold:
        return 0.0
    return (value - math.copysign(threshold, value)) / (1.0 - threshold)


def cmd_to_pwm(cmd, center=1500, range_us=400, pwm_min=1100, pwm_max=1900):
    """Convert a normalised command [-1, 1] to a clamped PWM value.

    Args:
        cmd:       Normalised command in [-1, 1].
        center:    PWM centre (μs), default 1500.
        range_us:  Half-swing from centre, default 400.
        pwm_min:   Lower clamp, default 1100.
        pwm_max:   Upper clamp, default 1900.

    Returns:
        Clamped PWM integer in [pwm_min, pwm_max].
    """
    pwm = int(center + cmd * range_us)
    if pwm < pwm_min:
        pwm = pwm_min
    elif pwm > pwm_max:
        pwm = pwm_max
    return pwm


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.

    Uses the standard aerospace convention:
        roll  – rotation about X
        pitch – rotation about Y
        yaw   – rotation about Z

    Returns:
        (roll_deg, pitch_deg, yaw_deg)
    """
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch — standard asin with clamp for numerical stability
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

from typing import Final

"""
Respect the naming convention : "subsystem" _ "component type" _ "precision"

Put port variables into the right category: CAN - PWM - DIO

Order port numbers, ex:
    drivetrain_motor_fl: Final = 0
    drivetrain_motor_fr: Final = 1
    drivetrain_motor_rr: Final = 2
"""

# CAN
drivetrain_motor_front_right = 2
drivetrain_motor_rear_right = 3
drivetrain_motor_front_left = 6
drivetrain_motor_rear_left = 1
arm_motor_elevator = 4
arm_motor_extension = 5

# PWM
led_strip = 0

# DIO
arm_switch_extension_min = 1
arm_switch_extension_max = 0
arm_photocell = 2
arm_switch_elevator_min = 3

# PCM
claw_piston_forward = 0
claw_piston_reverse = 1

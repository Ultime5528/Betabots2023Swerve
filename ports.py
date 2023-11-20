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
drivetrain_motor_driving_fl: Final = 0
drivetrain_motor_turning_fl: Final = 1
drivetrain_motor_driving_fr: Final = 2
drivetrain_motor_turning_fr: Final = 3
drivetrain_motor_driving_bl: Final = 4
drivetrain_motor_turning_bl: Final = 5
drivetrain_motor_driving_br: Final = 6
drivetrain_motor_turning_br: Final = 7

# PWM

# DIO
blocker_limitswitch: Final = 1
# PCM
blocker_solenoid_forward: Final = 2
blocker_solenoid_reverse: Final = 3

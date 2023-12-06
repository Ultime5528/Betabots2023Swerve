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
catapult_motor = 8

# PWM
catapult_servo: Final = 0

# DIO
catapult_limitswitch: Final = 0


# PCM
catapult_solenoid_forward: Final = 0
catapult_solenoid_reverse: Final = 1

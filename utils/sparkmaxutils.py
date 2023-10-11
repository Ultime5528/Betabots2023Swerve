from typing import Literal

import rev
import wpilib

IdleMode = Literal["brake", "coast"]

__all__ = ["configureLeader", "configureFollower"]


def configureLeader(motor: rev.CANSparkMax, mode: IdleMode, inverted: bool = False):
    _handleCanError(motor.restoreFactoryDefaults(), "restoreFactoryDefaults", motor)
    motor.setInverted(inverted)
    _configureMotor(motor, mode)


def configureFollower(follower: rev.CANSparkMax, leader: rev.CANSparkMax, mode: IdleMode, inverted: bool = False):
    _handleCanError(follower.restoreFactoryDefaults(), "restoreFactoryDefaults", follower)
    _handleCanError(follower.follow(leader, inverted), "follow", follower)
    _handleCanError(follower.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus0, 1000), "set status0 rate", follower)
    _handleCanError(follower.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus1, 1000), "set status1 rate", follower)
    _handleCanError(follower.setPeriodicFramePeriod(rev.CANSparkMax.PeriodicFrame.kStatus2, 1000), "set status2 rate", follower)
    _configureMotor(follower, mode)


def _configureMotor(motor: rev.CANSparkMax, mode: IdleMode):
    _handleCanError(motor.setIdleMode(_idleModeToEnum(mode)), "setIdleMode", motor)
    _handleCanError(motor.burnFlash(), "burnFlash", motor)
    _handleCanError(motor.clearFaults(), "clearFaults", motor)
    wpilib.wait(1.0)


def _idleModeToEnum(mode: IdleMode):
    if mode == "brake":
        return rev.CANSparkMax.IdleMode.kBrake
    elif mode == "coast":
        return rev.CANSparkMax.IdleMode.kCoast
    raise ValueError(f"mode is not 'brake' or 'coast' : {mode}")


def _handleCanError(error: rev.REVLibError, function: str, motor: rev.CANSparkMax):
    if error != rev.REVLibError.kOk:
        wpilib.reportError(f"CANError on motor ID {motor.getDeviceId()} during {function} : {error}", printTrace=True)

from typing import Callable

import commands2.button
import wpilib
from wpimath.filter import LinearFilter, SlewRateLimiter

import subsystems.drivetrain
import utils.swervemodule
from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand


class Drive(SafeCommand):
    def __init__(
        self,
        getPeriod: Callable[[], int],
        drivetrain: Drivetrain,
        xbox_remote: commands2.button.CommandXboxController,
    ):
        super().__init__()
        self.addRequirements(drivetrain)
        self.xbox_remote = xbox_remote
        self.drivetrain = drivetrain
        self.get_period = getPeriod

        self.m_xspeedLimiter = SlewRateLimiter(3)
        self.m_yspeedLimiter = SlewRateLimiter(3)
        self.m_rotLimiter = SlewRateLimiter(3)

    def execute(self):
        x_speed = (
            self.m_xspeedLimiter.calculate(self.xbox_remote.getLeftY())
            * -1
            * utils.swervemodule.k_max_speed
        )
        y_speed = (
            self.m_yspeedLimiter.calculate(self.xbox_remote.getLeftX())
            * -1
            * utils.swervemodule.k_max_speed
        )
        rot = (
            self.m_rotLimiter.calculate(self.xbox_remote.getRightX())
            * -1
            * subsystems.drivetrain.k_max_angular_speed
        )

        self.drivetrain.drive(x_speed, y_speed, rot, True, self.get_period())

    def end(self, interrupted: bool) -> None:
        self.drivetrain.drive(0.0, 0.0, 0.0, True, self.get_period())

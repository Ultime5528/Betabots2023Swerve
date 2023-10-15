#!/usr/bin/env python3
import math
from typing import Optional

import commands2
import wpilib

from commands.drive import Drive
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.stick = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain()

        self.drivetrain.setDefaultCommand(Drive(self.drivetrain, self.stick, self))


if __name__ == "__main__":
    wpilib.run(Robot)

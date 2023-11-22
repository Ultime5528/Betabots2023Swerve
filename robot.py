#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib

from commands.drive import Drive
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    xboxremote: commands2.button.CommandXboxController = None
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain()

        self.drivetrain.setDefaultCommand(
            Drive(lambda: self.getPeriod(), self.drivetrain, self.xboxremote)
        )


if __name__ == "__main__":
    wpilib.run(Robot)

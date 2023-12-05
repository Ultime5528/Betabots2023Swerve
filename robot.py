#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib

from commands.drive import Drive
from subsystems.drivetrain import Drivetrain
from commands.drivedistance import DriveDistance
from wpimath.geometry import Pose2d


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain(self.getPeriod())

        self.drivetrain.setDefaultCommand(
            Drive(self.drivetrain, self.xboxremote)
        )

        wpilib.SmartDashboard.putData("DriveDistance", DriveDistance(self.drivetrain, Pose2d(4, 4, 0), 2))


if __name__ == "__main__":
    wpilib.run(Robot)

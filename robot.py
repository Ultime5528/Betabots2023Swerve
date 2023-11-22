#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib
from commands2 import Trigger
from wpilib.event import BooleanEvent

from commands.closelock import CloseLock
from commands.drive import Drive
from commands.openlock import OpenLock
from subsystems.blocker import Blocker
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain()

        self.blocker = Blocker()


        self.drivetrain.setDefaultCommand(
            Drive(lambda: self.getPeriod(), self.drivetrain, self.xboxremote)
        )

        wpilib.SmartDashboard.putData("Close", CloseLock(self.blocker))
        wpilib.SmartDashboard.putData("Open", OpenLock(self.blocker))


if __name__ == "__main__":
    wpilib.run(Robot)

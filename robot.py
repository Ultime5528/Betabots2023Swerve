#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib
from commands2 import Trigger
from wpilib.event import BooleanEvent

from commands.lock import Lock
from commands.drive import Drive
from commands.unlock import Unlock
from subsystems.blocker import Blocker
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain(self.getPeriod())

        self.blocker = Blocker()


        self.drivetrain.setDefaultCommand(
            Drive(self.drivetrain, self.xboxremote)
        )

        wpilib.SmartDashboard.putData("Close", Lock(self.blocker))
        wpilib.SmartDashboard.putData("Open", Unlock(self.blocker))


if __name__ == "__main__":
    wpilib.run(Robot)

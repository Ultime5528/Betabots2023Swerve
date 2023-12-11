#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib
from commands2 import Trigger
from wpilib.event import BooleanEvent

from commands.launch import Launch
from commands.resetarm import ResetArm
from commands.lock import Lock
from commands.drive import Drive
from commands.unlock import Unlock
from subsystems.catapult import Catapult
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.CameraServer.launch("vision.py:main")

        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain(self.getPeriod())

        self.catapult = Catapult()


        self.drivetrain.setDefaultCommand(
            Drive(self.drivetrain, self.xboxremote)
        )

        wpilib.SmartDashboard.putData("Lock", Lock(self.catapult))
        wpilib.SmartDashboard.putData("Unlock", Unlock(self.catapult))
        wpilib.SmartDashboard.putData("ResetLauncher", ResetArm(self.catapult))
        wpilib.SmartDashboard.putData("Launch", Launch(self.catapult))
        wpilib.SmartDashboard.putData("Launch uninterrupt", Launch(self.catapult))


if __name__ == "__main__":
    wpilib.run(Robot)

#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib
from commands2 import Trigger
from wpilib.event import BooleanEvent

from commands.charge import Charge
from commands.lock import Lock
from commands.drive import Drive
from commands.unlock import Unlock
from subsystems.catapult import Catapult
from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain(self.getPeriod())

        self.catapult = Catapult()


        self.drivetrain.setDefaultCommand(
            Drive(self.drivetrain, self.xboxremote)
        )

        wpilib.SmartDashboard.putData("Close", Lock(self.catapult))
        wpilib.SmartDashboard.putData("Open", Unlock(self.catapult))
        wpilib.SmartDashboard.putData("Charge1", Charge(self.catapult, 1))
        wpilib.SmartDashboard.putData("Charge2", Charge(self.catapult, 2))
        wpilib.SmartDashboard.putData("Charge3", Charge(self.catapult, 3))


if __name__ == "__main__":
    wpilib.run(Robot)

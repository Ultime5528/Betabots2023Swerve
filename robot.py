#!/usr/bin/env python3
import math
from typing import Optional

import commands2.button
import wpilib

from commands.autonomous.automiddleflowergreen import AutoMiddleFlowerGreen
from commands.autonomous.automiddlefloweryellow import AutoMiddleFlowerYellow
from subsystems.catapult import Catapult
from commands.drive import Drive
from subsystems.drivetrain import Drivetrain
from commands.drivedistance import DriveDistance
from wpimath.geometry import Pose2d


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.auto_command: Optional[commands2.CommandBase] = None

        self.xboxremote = commands2.button.CommandXboxController(0)

        self.drivetrain = Drivetrain(self.getPeriod())
        self.catapult = Catapult()

        self.drivetrain.setDefaultCommand(Drive(self.drivetrain, self.xboxremote))

        self.auto_chooser = wpilib.SendableChooser()
        self.auto_chooser.addOption("Nothing", None)
        self.auto_chooser.addOption(
            "AutoMiddleFlower Green",
            AutoMiddleFlowerGreen(self.drivetrain, self.catapult),
        )
        self.auto_chooser.addOption(
            "AutoMiddleFlower Yellow",
            AutoMiddleFlowerYellow(self.drivetrain, self.catapult),
        )
        self.auto_chooser.setDefaultOption("Nothing", None)
        wpilib.SmartDashboard.putData("AutonomousMode", self.auto_chooser)

    def autonomousInit(self):
        self.auto_command: commands2.CommandBase = self.auto_chooser.getSelected()
        if self.auto_command:
            self.auto_command.schedule()

    def teleopInit(self):
        if self.auto_command:
            self.auto_command.cancel()


if __name__ == "__main__":
    wpilib.run(Robot)

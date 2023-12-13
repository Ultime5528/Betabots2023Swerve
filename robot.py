#!/usr/bin/env python3

import commands2.button
import wpilib

from commands.drive import Drive
from commands.launch import Launch
from commands.load import Load
from commands.resetarm import ResetArm
from commands.charge import Charge
from commands.lock import Lock
from commands.resetarm import ResetArm
from commands.unlock import Unlock
from subsystems.catapult import Catapult
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

        self.catapult = Catapult()

        self.drivetrain.setDefaultCommand(
            Drive(self.drivetrain, self.xboxremote)
        )

        wpilib.SmartDashboard.putData("Lock", Lock(self.catapult))
        wpilib.SmartDashboard.putData("Unlock", Unlock(self.catapult))
        wpilib.SmartDashboard.putData("Load", Load(self.catapult))
        wpilib.SmartDashboard.putData("ResetLauncher", ResetArm(self.catapult))
        wpilib.SmartDashboard.putData("Launch", Launch(self.catapult))
        wpilib.SmartDashboard.putData("Launch uninterrupt", Launch(self.catapult))
        wpilib.SmartDashboard.putData("DriveDistance", DriveDistance(self.drivetrain, Pose2d(4, 4, 0), 2))
        wpilib.SmartDashboard.putData("Charge1", Charge(self.catapult, 1))
        wpilib.SmartDashboard.putData("Charge2", Charge(self.catapult, 2))
        wpilib.SmartDashboard.putData("Charge3", Charge(self.catapult, 3))


if __name__ == "__main__":
    wpilib.run(Robot)

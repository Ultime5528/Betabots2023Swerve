#!/usr/bin/env python3

import commands2.button
import wpilib
from wpimath.geometry import Pose2d

from commands.autonomous.automiddleflowergreen import AutoMiddleFlowerGreen
from commands.autonomous.automiddlefloweryellow import AutoMiddleFlowerYellow
from subsystems.catapult import Catapult
from commands.charge import Charge
from commands.drive import Drive
from commands.drivedistance import DriveDistance
from commands.launch import Launch
from commands.load import Load
from commands.lock import Lock
from commands.resetarm import ResetArm
from commands.unlock import Unlock
from subsystems.catapult import Catapult
from subsystems.drivetrain import Drivetrain


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
        self.catapult = Catapult()

        self.drivetrain.setDefaultCommand(
            Drive(self.drivetrain, self.xboxremote)
        )
        self.auto_chooser.addOption(
            "AutoMiddleFlower Yellow",
            AutoMiddleFlowerYellow(self.drivetrain, self.catapult),
        )
        self.auto_chooser.setDefaultOption("Nothing", None)
        
        wpilib.SmartDashboard.putData("AutonomousMode", self.auto_chooser)
        wpilib.SmartDashboard.putData("Lock", Lock(self.catapult))
        wpilib.SmartDashboard.putData("Unlock", Unlock(self.catapult))
        wpilib.SmartDashboard.putData("Load", Load(self.catapult))
        wpilib.SmartDashboard.putData("ResetArm", ResetArm(self.catapult))
        wpilib.SmartDashboard.putData("Launch", Launch(self.catapult))
        wpilib.SmartDashboard.putData("Launch uninterrupt", Launch(self.catapult))
        wpilib.SmartDashboard.putData("DriveDistance", DriveDistance(self.drivetrain, Pose2d(4, 4, 0), 2))
        wpilib.SmartDashboard.putData("Charge1", Charge(self.catapult, 1))
        wpilib.SmartDashboard.putData("Charge2", Charge(self.catapult, 2))
        wpilib.SmartDashboard.putData("Charge3", Charge(self.catapult, 3))

          
    def autonomousInit(self):
        self.auto_command: commands2.CommandBase = self.auto_chooser.getSelected()
        if self.auto_command:
            self.auto_command.schedule()

    def teleopInit(self):
        if self.auto_command:
            self.auto_command.cancel()


if __name__ == "__main__":
    wpilib.run(Robot)

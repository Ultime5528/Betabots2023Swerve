#!/usr/bin/env python3
import math
from typing import Optional

import commands2
import wpilib

from subsystems.drivetrain import Drivetrain


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        wpilib.LiveWindow.enableAllTelemetry()
        wpilib.LiveWindow.setEnabled(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

        self.stick = wpilib.Joystick(0)

        self.drivetrain = Drivetrain()


if __name__ == "__main__":
    wpilib.run(Robot)

#!/usr/bin/env python3
import math
from typing import Optional

import commands2
import wpilib


class Robot(commands2.TimedCommandRobot):
    def robotInit(self):
        self.stick = commands2.button.CommandJoystick(0)

    def autonomousInit(self) -> None:
        self.autoCommand: commands2.CommandBase = self.autoChooser.getSelected()
        if self.autoCommand:
            self.autoCommand.schedule()

    def teleopInit(self) -> None:
        if self.autoCommand:
            self.autoCommand.cancel()

    def robotPeriodic(self) -> None:
        super().robotPeriodic()
        self.loop.poll()


if __name__ == "__main__":
    wpilib.run(Robot)

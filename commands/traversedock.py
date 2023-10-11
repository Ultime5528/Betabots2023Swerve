import math

import wpilib

from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand
from enum import Enum


class State(Enum):
    Start = "start"
    Up = "up"
    Down = "down"
    End = "end"


class TraverseDock(SafeCommand):
    start_speed = autoproperty(0.32)
    up_speed = autoproperty(0.25)
    down_speed = autoproperty(0.25)
    up_threshold = autoproperty(10.0)
    end_threshold = autoproperty(2.5)
    max_distance = autoproperty(4.0)

    def __init__(self, drivetrain: Drivetrain):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.state = State.Start

    def initialize(self) -> None:
        self.state = State.Start

    def execute(self) -> None:
        pitch = -self.drivetrain.getPitch()
        speed = self.start_speed

        if pitch > self.up_threshold:
            self.state = State.Up

        if self.state == State.Up:
            speed = self.up_speed
            if pitch < -self.up_threshold:
                self.state = State.Down
                self.up_distance = self.drivetrain.getAverageEncoderPosition()

        if self.state == State.Down:
            speed = self.down_speed
            if abs(self.drivetrain.getAverageEncoderPosition() - self.up_distance) > self.end_threshold:
                self.state = State.End

        self.drivetrain.arcadeDrive(-speed, 0)
        print(self.state)

    def isFinished(self) -> bool:
        return self.state == State.End

    def end(self, interrupted: bool) -> None:
        self.drivetrain.arcadeDrive(0, 0)

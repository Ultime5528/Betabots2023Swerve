import math

import commands2
from subsystems.drivetrain import Drivetrain
from utils.safecommand import SafeCommand


class DriveStraight(SafeCommand):
    def __init__(self, drivetrain: Drivetrain, distance: float, speed: float) -> None:
        super().__init__()
        self.drivetrain = drivetrain
        self.distance = distance
        self.speed = speed
        self.initial_encoder_pos = 0
        self.encoder_delta = 0
        self.addRequirements(drivetrain)

    def initialize(self) -> None:
        self.initial_encoder_pos = self.drivetrain.getAverageEncoderPosition()
        self.encoder_delta = 0

    def execute(self) -> None:
        self.encoder_delta = self.drivetrain.getAverageEncoderPosition() - self.initial_encoder_pos
        self.drivetrain.arcadeDrive(math.copysign(self.speed, self.distance), 0)

    def end(self, interrupted: bool) -> None:
        self.drivetrain.arcadeDrive(0, 0)

    def isFinished(self) -> bool:
        return abs(self.encoder_delta) >= abs(self.distance)
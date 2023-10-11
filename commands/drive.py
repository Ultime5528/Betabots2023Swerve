from abc import ABC

import commands2.button
import wpilib
from wpimath.filter import LinearFilter

from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand


class Interpolator(ABC):
    def reset(self):
        self.filter = LinearFilter.movingAverage(int(self.smoothing_window))

    def interpolate(self, value: float):
        value = self.filter.calculate(value)

        curve = self.interpolation_curve
        deadzone_x = self.deadzone_x
        deadzone_y = self.deadzone_y

        if value >= deadzone_x:
            return deadzone_y + (1 - deadzone_y) * (curve * value ** 3 + (1 - curve) * value)
        elif value <= -deadzone_x:
            return -deadzone_y + (1 - deadzone_y) * (curve * value ** 3 + (1 - curve) * value)
        else:
            return 0.0


class DriveForward(Interpolator):
    smoothing_window = autoproperty(1)
    interpolation_curve = autoproperty(0.6)
    deadzone_x = autoproperty(0.1)
    deadzone_y = autoproperty(0.1)


class DriveTurn(Interpolator):
    smoothing_window = autoproperty(1)
    interpolation_curve = autoproperty(0.6)
    deadzone_x = autoproperty(0.05)
    deadzone_y = autoproperty(0.05)


forward_interpolator = DriveForward()
turn_interpolator = DriveTurn()


class Drive(SafeCommand):
    def __init__(self, drivetrain: Drivetrain, stick: wpilib.Joystick):
        super().__init__()
        self.addRequirements(drivetrain)
        self.stick = stick
        self.drivetrain = drivetrain

    def initialize(self) -> None:
        forward_interpolator.reset()
        turn_interpolator.reset()

    def execute(self):
        if isinstance(self.stick, commands2.button.CommandXboxController):
            forward = self.stick.getLeftY()
            turn = self.stick.getLeftX()
        else:
            forward = self.stick.getY()
            turn = self.stick.getX()

        forward = forward_interpolator.interpolate(forward * -1)
        turn = turn_interpolator.interpolate(turn * -1)

        self.drivetrain.arcadeDrive(forward, turn)

    def end(self, interrupted: bool) -> None:
        self.drivetrain.arcadeDrive(0.0, 0.0)

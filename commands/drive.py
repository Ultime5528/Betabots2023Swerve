import commands2.button
import wpilib
from wpimath.filter import LinearFilter, SlewRateLimiter

import subsystems.drivetrain
import utils.swervemodule
from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand


class Drive(SafeCommand):
    interpolation_curve = autoproperty(0.6)
    deadzone_x = autoproperty(0.1)
    deadzone_y = autoproperty(0.1)

    def __init__(self, drivetrain: Drivetrain, stick: commands2.button.CommandXboxController(0), robot: wpilib.TimedRobot):
        super().__init__()
        self.addRequirements(drivetrain)
        self.stick = stick
        self.drivetrain = drivetrain
        self.robot = robot

        self.m_xspeedLimiter = SlewRateLimiter(3)
        self.m_yspeedLimiter = SlewRateLimiter(3)
        self.m_rotLimiter = SlewRateLimiter(3)

    def initialize(self) -> None:
        pass

    def execute(self):
        xSpeed = self.m_xspeedLimiter.calculate(self.stick.getLeftY())*-1*utils.swervemodule.k_max_speed
        ySpeed = self.m_yspeedLimiter.calculate(self.stick.getLeftX())*-1*utils.swervemodule.k_max_speed
        rot = self.m_rotLimiter.calculate(self.stick.getRightX())*-1*subsystems.drivetrain.k_max_angular_speed

        self.drivetrain.drive(xSpeed, ySpeed, rot, True, self.robot.getPeriod())

    def end(self, interrupted: bool) -> None:
        self.drivetrain.arcadeDrive(0.0, 0.0)

    def interpolate(self, value: float):
        curve = self.interpolation_curve
        deadzone_x = self.deadzone_x
        deadzone_y = self.deadzone_y

        if value >= deadzone_x:
            return deadzone_y + (1 - deadzone_y) * (curve * value ** 3 + (1 - curve) * value)
        elif value <= -deadzone_x:
            return -deadzone_y + (1 - deadzone_y) * (curve * value ** 3 + (1 - curve) * value)
        else:
            return 0.0
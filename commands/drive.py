import commands2.button
from wpimath.filter import SlewRateLimiter

from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand
from utils.property import autoproperty


class Drive(SafeCommand):
    is_field_relative = autoproperty(True)
    has_rate_limiter = autoproperty(False)
    deadzone = autoproperty(0.1)

    def __init__(
        self,
        drivetrain: Drivetrain,
        xbox_remote: commands2.button.CommandXboxController,
    ):
        super().__init__()
        self.addRequirements(drivetrain)
        self.xbox_remote = xbox_remote
        self.drivetrain = drivetrain

        self.m_xspeedLimiter = SlewRateLimiter(3)
        self.m_yspeedLimiter = SlewRateLimiter(3)
        self.m_rotLimiter = SlewRateLimiter(3)

    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        else:
            return value

    def execute(self):
        x_speed = self.apply_deadzone(
            self.m_xspeedLimiter.calculate(self.xbox_remote.getLeftY())
            * -1
        )
        y_speed = self.apply_deadzone(
            self.m_yspeedLimiter.calculate(self.xbox_remote.getLeftX())
            * -1
        )
        rot = self.apply_deadzone(
            self.m_rotLimiter.calculate(self.xbox_remote.getRightX())
            * -1
        )
        
        self.drivetrain.drive(x_speed, y_speed, rot, self.is_field_relative, self.has_rate_limiter)

    def end(self, interrupted: bool) -> None:
        self.drivetrain.drive(0.0, 0.0, 0.0, is_field_relative=False, rate_limiter=False)

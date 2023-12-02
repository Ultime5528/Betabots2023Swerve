import math

from wpimath.geometry import Pose2d

from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand


class DriveDistance(SafeCommand):
    max_error = autoproperty(0.1)

    def __init__(self, drivetrain: Drivetrain, pose: Pose2d, x_speed, y_speed):
        super().__init__()
        self.drivetrain = drivetrain

        self.x_distance = pose.x
        self.y_distance = pose.y
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.x_error = math.inf
        self.y_error = math.inf

        self.addRequirements(drivetrain)

    def execute(self):
        self.x_error = (
            self.x_distance - self.drivetrain.swerve_estimator.getEstimatedPosition().x
        )
        self.y_error = (
            self.y_distance - self.drivetrain.swerve_estimator.getEstimatedPosition().y
        )

        self.vx = math.copysign(self.x_speed, self.x_error)
        self.vy = math.copysign(self.y_speed, self.y_error)

        if abs(self.x_error) <= self.max_error:
            self.vx = 0
        if abs(self.y_error) <= self.max_error:
            self.vy = 0

        self.drivetrain.drive(self.vy, self.vy, 0, True)

    def isFinished(self) -> bool:
        return self.vx == 0 and self.vy == 0

    def end(self, interrupted):
        self.drivetrain.drive(0, 0, 0, True)

import math

from wpimath.geometry import Pose2d

from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty
from utils.safecommand import SafeCommand
from utils.trapezoidalmotion import TrapezoidalMotion


class DriveDistance(SafeCommand):
    max_error = autoproperty(0.3)
    min_speed = autoproperty(0.15)
    accel = autoproperty(0.0005)

    def __init__(self, drivetrain: Drivetrain, pose: Pose2d, speed):
        super().__init__()
        self.drivetrain = drivetrain

        self.x_distance = pose.x
        self.y_distance = pose.y
        self.speed = speed
        self.x_error = math.inf
        self.y_error = math.inf

        self.estimator = drivetrain.getSwerveEstimator()

        self.addRequirements(drivetrain)

        self.initial_position = Pose2d()

    def initialize(self):
        self.initial_position = self.estimator.getEstimatedPosition()
        self.motion_x = TrapezoidalMotion(
            min_speed=self.min_speed,
            max_speed=self.speed,
            accel=self.accel,
            start_position=0,
            end_position=abs(self.initial_position.x-self.x_distance)
        )
        self.motion_y = TrapezoidalMotion(
            min_speed=self.min_speed,
            max_speed=self.speed,
            accel=self.accel,
            start_position=0,
            end_position=abs(self.initial_position.y-self.y_distance)
        )

    def execute(self):
        """
        Faire une fonction dans drivetrain qui retourne l'estimation position.
        L'appeler une seule fois, garder dans une variable, et prendre x et y.
        """

        self.x_error = (
            self.x_distance - self.estimator.getEstimatedPosition().x
        )
        self.y_error = (
            self.y_distance - self.estimator.getEstimatedPosition().y
        )

        moved_x = abs(self.estimator.getEstimatedPosition().x - self.initial_position.x)
        moved_y = abs(self.estimator.getEstimatedPosition().y - self.initial_position.y)

        self.motion_x.setPosition(moved_x)
        self.motion_y.setPosition(moved_y)

        self.vx = math.copysign(self.motion_x.getSpeed(), self.x_error)
        self.vy = math.copysign(self.motion_y.getSpeed(), self.y_error)

        self.drivetrain.drive(self.vx, self.vy, 0, True)

    def isFinished(self) -> bool:
        return self.motion_x.isFinished() and self.motion_y.isFinished()

    def end(self, interrupted):
        self.drivetrain.drive(0, 0, 0, True)

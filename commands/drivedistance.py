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

    def __init__(self, drivetrain: Drivetrain, goal: Pose2d, speed: float):
        super().__init__()
        self.drivetrain = drivetrain

        self.goal_x = goal.x
        self.goal_y = goal.y
        self.speed = speed
        self.error_x = math.inf
        self.error_y = math.inf


        self.addRequirements(drivetrain)

        self.initial_position = Pose2d()

    def initialize(self):
        self.initial_position = self.drivetrain.getPose()
        self.motion_x = TrapezoidalMotion(
            min_speed=self.min_speed,
            max_speed=self.speed,
            accel=self.accel,
            start_position=self.initial_position.x,
            end_position=self.goal_x
        )
        self.motion_y = TrapezoidalMotion(
            min_speed=self.min_speed,
            max_speed=self.speed,
            accel=self.accel,
            start_position=self.initial_position.y,
            end_position=self.goal_y
        )

    def execute(self):
        current_position = self.drivetrain.getPose()
        current_x = current_position.x
        current_y = current_position.y


        self.error_x = (
                self.goal_x - current_x
        )
        self.error_y = (
                self.goal_y - current_y
        )

        moved_x = abs(current_x - self.initial_position.x)
        moved_y = abs(current_y - self.initial_position.y)

        self.motion_x.setPosition(moved_x)
        self.motion_y.setPosition(moved_y)

        self.vx = math.copysign(self.motion_x.getSpeed(), self.error_x)
        self.vy = math.copysign(self.motion_y.getSpeed(), self.error_y)

        self.drivetrain.drive(self.vx, self.vy, 0)

    def isFinished(self) -> bool:
        return self.motion_x.isFinished() and self.motion_y.isFinished()

    def end(self, interrupted):
        self.drivetrain.drive(0, 0, 0)

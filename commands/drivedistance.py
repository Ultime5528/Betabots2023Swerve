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

        self.motion_x.setPosition(current_x)
        self.motion_y.setPosition(current_y)

        vx = self.motion_x.getSpeed()
        vy = self.motion_y.getSpeed()

        if self.motion_x.isFinished():
            vx = 0
        if self.motion_y.isFinished():
            vy = 0

        self.drivetrain.drive(vx, vy, 0, is_field_relative=False, rate_limiter=False)

    def isFinished(self) -> bool:
        return self.motion_x.isFinished() and self.motion_y.isFinished()

    def end(self, interrupted):
        self.drivetrain.drive(0, 0, 0)

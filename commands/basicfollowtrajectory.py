from typing import Literal, Iterable, Union

from commands2 import ConditionalCommand
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from subsystems.drivetrain import Drivetrain, april_tag_field
from utils.property import autoproperty, FloatProperty, asCallable
from utils.safecommand import SafeCommand
from utils.trapezoidalmotion import TrapezoidalMotion


blue_offset = Transform2d(Translation2d(-2, 0), Rotation2d(0))
blue_loading_pose = april_tag_field.getTagPose(4).toPose2d().transformBy(blue_offset)
red_offset = Transform2d(Translation2d(2, 0), Rotation2d.fromDegrees(180))
red_loading_pose = april_tag_field.getTagPose(5).toPose2d().transformBy(red_offset)


class BasicFollowTrajectory(SafeCommand):
    """
    Pour une trajectoire inversée, il faut :
    - path_reversed=True
    - Les angles doivent être inversés (0 devient 180, -30 devient 150...)
    - Les coordonnées doivent être multipliées par -1 : (3, -1) devient (-3, 1)

    Example of a command:
    FollowTrajectory(self.drivetrain, [self.drivetrain.getPose(), Pose2d(0, 3, 90), Pose2d(3, 3, 0)], 0.5)
    """
    start_speed = autoproperty(0.1)
    accel = autoproperty(0.5)
    correction_factor = autoproperty(0.016)

    @classmethod
    def toLoading(cls, drivetrain: Drivetrain):
        cmd = ConditionalCommand(
            cls(drivetrain, red_loading_pose, lambda: properties.to_loading_speed, origin="absolute"),
            cls(drivetrain, blue_loading_pose, lambda: properties.to_loading_speed, origin="absolute"),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed
        )
        cmd.setName(cmd.getName() + ".toLoading")
        return cmd

    @classmethod
    def driveStraight(cls, drivetrain: Drivetrain, distance: float, speed: float):
        cmd = cls(drivetrain, Pose2d(distance, 0, 0), speed, origin="relative")
        cmd.setName(cmd.getName() + ".driveStraight")
        return cmd

    def __init__(
            self,
            drivetrain: Drivetrain,
            waypoints: Union[Pose2d, Iterable[Pose2d]],
            speed: FloatProperty,
            origin: Literal["absolute", "relative"],
            direction: Literal["forward", "backward"] = "forward"
    ) -> None:
        super().__init__()
        self.waypoints = waypoints if isinstance(waypoints, Iterable) else [waypoints]
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.speed = asCallable(speed)
        self.path_reversed = (direction == "backward")
        self.config = TrajectoryConfig(10, 10)
        self.config.setReversed(self.path_reversed)
        self.origin = origin

        if self.origin == "relative":
            self.relative_trajectory = TrajectoryGenerator.generateTrajectory(
                [Pose2d(0, 0, 0), *self.waypoints],
                self.config
            )

    def initialize(self) -> None:
        if self.origin == "relative":
            self.trajectory = self.relative_trajectory.transformBy(Transform2d(Pose2d(), self.drivetrain.getPose()))
        else:
            self.trajectory = TrajectoryGenerator.generateTrajectory(
                [self.drivetrain.getPose(), *self.waypoints],
                self.config
            )
        self.states = self.trajectory.states()
        self.motion = TrapezoidalMotion(
            min_speed=self.start_speed,
            max_speed=self.speed(),
            accel=self.accel,
            start_position=0,
            displacement=self.states[0].pose.translation().distance(self.states[-1].pose.translation())
        )

        self.index = 0
        self.cumulative_dist = 0
        self.start_dist = self.drivetrain.getAverageEncoderPosition()
        self.drivetrain.getField().getObject("traj").setTrajectory(self.trajectory)

    def execute(self) -> None:
        current_pose = self.drivetrain.getPose()

        while (
                self.index < len(self.states) - 1
                and abs(self.drivetrain.getAverageEncoderPosition() - self.start_dist) >= self.cumulative_dist
        ):
            self.index += 1
            self.cumulative_dist += self.states[self.index].pose.translation().distance(
                self.states[self.index - 1].pose.translation())

        destination_pose = self.states[self.index].pose
        distance_traveled = self.states[0].pose.translation().distance(destination_pose.translation())
        self.motion.setPosition(distance_traveled)
        speed = self.motion.getSpeed() * (-1 if self.path_reversed else 1)

        error = current_pose.rotation() - destination_pose.rotation()

        correction = self.correction_factor * error.degrees()
        self.drivetrain.tankDrive(speed + correction, speed - correction)

    def isFinished(self) -> bool:
        return self.index >= len(self.states) - 1 and abs(
            self.drivetrain.getAverageEncoderPosition() - self.start_dist) >= self.cumulative_dist

    def end(self, interrupted: bool) -> None:
        self.drivetrain.tankDrive(0, 0)


class _ClassProperties:
    to_loading_speed = autoproperty(0.6, subtable=BasicFollowTrajectory.__name__)


properties = _ClassProperties()

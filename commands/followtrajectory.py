from typing import Literal, Iterable, Union, Optional

import wpiutil
from commands2 import ConditionalCommand
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d
from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator

from subsystems.drivetrain import Drivetrain, april_tag_field
from utils.controller import RearWheelFeedbackController
from utils.property import autoproperty, FloatProperty, asCallable, defaultSetter
from utils.safecommand import SafeCommand
from utils.trapezoidalmotion import TrapezoidalMotion


blue_offset = Transform2d(Translation2d(2, -1), Rotation2d.fromDegrees(270))
blue_loading_pose = april_tag_field.getTagPose(4).toPose2d().transformBy(blue_offset)
red_offset = Transform2d(Translation2d(2, 1), Rotation2d.fromDegrees(90))
red_loading_pose = april_tag_field.getTagPose(5).toPose2d().transformBy(red_offset)


class FollowTrajectory(SafeCommand):
    """
    Pour une trajectoire inversée, il faut :
    - direction="backward"
    - Les angles doivent être inversés (0 devient 180, -30 devient 150...)
    - Les coordonnées doivent être multipliées par -1 : (3, -1) devient (-3, 1)

    Example of a command:
    FollowTrajectory(self.drivetrain, [self.drivetrain.getPose(), Pose2d(0, 3, 90), Pose2d(3, 3, 0)], 0.5)
    """
    start_speed = autoproperty(0.12)
    accel = autoproperty(0.55)
    curvature_factor = autoproperty(0.1)
    angle_factor = autoproperty(5.0)
    track_error_factor = autoproperty(2.5)

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
        self._delta = 0.0
        self._computed_speed = 0.0
        self._controller: Optional[RearWheelFeedbackController] = None

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

        self.motion = TrapezoidalMotion(
            min_speed=self.start_speed,
            max_speed=self.speed(),
            accel=self.accel,
            start_position=0,
            end_position=self.trajectory.totalTime()
        )
        self.drivetrain.getField().getObject("traj").setTrajectory(self.trajectory)
        self._controller = RearWheelFeedbackController(self.trajectory)
        self.drivetrain.use_vision = False

    def execute(self) -> None:
        current_pose = self.drivetrain.getPose()
        self.motion.setPosition(self._controller.closest_t)
        self._computed_speed = self.motion.getSpeed() * (-1 if self.path_reversed else 1)
        self._delta = self._controller.update(current_pose, self._computed_speed, self.curvature_factor, self.angle_factor, self.track_error_factor)
        self.drivetrain.tankDrive(self._computed_speed - self._delta, self._computed_speed + self._delta)
        self.drivetrain.getField().getObject("closest").setPose(self._controller.closest_sample.pose)

    def isFinished(self) -> bool:
        return self._controller.closest_t >= 0.99 * self.trajectory.totalTime()

    def end(self, interrupted: bool) -> None:
        self.drivetrain.tankDrive(0, 0)
        self.drivetrain.use_vision = True

    def initSendable(self, builder: wpiutil.SendableBuilder) -> None:
        super().initSendable(builder)
        builder.addDoubleProperty("closest_t", lambda: self._controller.closest_t if self._controller else 0.0, defaultSetter)
        builder.addDoubleProperty("curvature", lambda: self._controller.closest_sample.curvature if self._controller and self._controller.closest_sample else 0.0, defaultSetter)
        builder.addDoubleProperty("computed_speed", lambda: self._computed_speed, defaultSetter)
        builder.addDoubleProperty("delta", lambda: self._delta, defaultSetter)
        builder.addDoubleProperty("angle_error", lambda: self._controller.angle_error.degrees() if self._controller else 0.0, defaultSetter)
        builder.addDoubleProperty("angle_error_cos", lambda: self._controller.angle_error.cos() if self._controller else 0.0, defaultSetter)
        builder.addDoubleProperty("angle_error_sin", lambda: self._controller.angle_error.sin() if self._controller else 0.0, defaultSetter)
        builder.addDoubleProperty("error", lambda: self._controller.error if self._controller else 0.0, defaultSetter)
        builder.addDoubleProperty("omega_0", lambda: self._controller.omega_0 if self._controller else 0.0, defaultSetter)
        builder.addDoubleProperty("omega_1", lambda: self._controller.omega_1 if self._controller else 0.0, defaultSetter)


class _ClassProperties:
    to_loading_speed = autoproperty(0.2, subtable=FollowTrajectory.__name__)


properties = _ClassProperties()

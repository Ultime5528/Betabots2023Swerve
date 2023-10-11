import wpiutil

from subsystems.drivetrain import Drivetrain
from utils.property import autoproperty, defaultSetter
from utils.safecommand import SafeCommand
from utils.trapezoidalmotion import TrapezoidalMotion


class Turn(SafeCommand):
    min_speed = autoproperty(0.15)
    accel = autoproperty(0.0005)

    def __init__(self, drivetrain: Drivetrain, angle: float, speed: float):
        """
        Parameters
        ----------
        drivetrain
        angle: Positif = antihoraire
        speed: SPEEED
        """

        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.angle = angle
        self.speed = speed
        self.cumul = 0
        self.previous_rotation = self.drivetrain.getRotation()
        self._computed_speed = 0.0

    def initialize(self) -> None:
        self.cumul = 0
        self.previous_rotation = self.drivetrain.getRotation()
        self.motion = TrapezoidalMotion(
            start_position=0,
            end_position=self.angle,
            start_speed=self.speed,
            max_speed=self.speed,
            end_speed=self.min_speed,
            accel=self.accel
        )

    def execute(self):
        current_rotation = self.drivetrain.getRotation()
        moved_rotation = current_rotation - self.previous_rotation
        self.cumul += moved_rotation.degrees()
        self.motion.setPosition(self.cumul)
        self._computed_speed = self.motion.getSpeed()
        self.drivetrain.arcadeDrive(0, self._computed_speed)
        self.previous_rotation = current_rotation

    def isFinished(self) -> bool:
        return self.motion.isFinished()

    def end(self, interrupted: bool) -> None:
        self.drivetrain.tankDrive(0.0, 0.0)

    def initSendable(self, builder: wpiutil.SendableBuilder) -> None:
        super().initSendable(builder)
        builder.addDoubleProperty("cumul", lambda: self.cumul, defaultSetter)
        builder.addDoubleProperty("previous", lambda: self.previous_rotation.degrees(), defaultSetter)
        builder.addDoubleProperty("computed_speed", lambda: self._computed_speed, defaultSetter)

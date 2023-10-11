import commands2
from commands2 import ConditionalCommand

from utils.property import autoproperty, FloatProperty, asCallable
from utils.safecommand import SafeCommand, SafeMixin
from utils.trapezoidalmotion import TrapezoidalMotion
from subsystems.arm import Arm


class MoveArm(SafeMixin, ConditionalCommand):
    @classmethod
    def toLevel1(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.level1_extension, lambda: properties.level1_elevation)
        cmd.setName(cmd.getName() + ".toLevel1")
        return cmd

    @classmethod
    def toLevel2(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.level2_extension, lambda: properties.level2_elevation)
        cmd.setName(cmd.getName() + ".toLevel2")
        return cmd

    @classmethod
    def toLevel3(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.level3_extension, lambda: properties.level3_elevation)
        cmd.setName(cmd.getName() + ".toLevel3")
        return cmd

    @classmethod
    def toLevel3Drop(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.level3_drop_extension, lambda: properties.level3_drop_elevation)
        cmd.setName(cmd.getName() + ".toLevel3_drop")
        return cmd

    @classmethod
    def toFloor(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.floor_extension, lambda: properties.floor_elevation)
        cmd.setName(cmd.getName() + ".toFloor")
        return cmd

    @classmethod
    def toBase(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.base_extension, lambda: properties.base_elevation)
        cmd.setName(cmd.getName() + ".toBase")
        return cmd

    @classmethod
    def toBin(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.bin_extension, lambda: properties.bin_elevation)
        cmd.setName(cmd.getName() + ".toBin")
        return cmd

    def __init__(self, arm: Arm, extension_end_position: FloatProperty, elevator_end_position: FloatProperty):
        extension_end_position = asCallable(extension_end_position)
        elevator_end_position = asCallable(elevator_end_position)

        def cond():
            return arm.shouldTransition(extension_end_position(), elevator_end_position())

        super().__init__(
            commands2.SequentialCommandGroup(
                MoveArmToTransition(arm),
                MoveArmDirect(arm, extension_end_position, elevator_end_position)
            ),
            MoveArmDirect(arm, extension_end_position, elevator_end_position),
            cond
        )


class MoveArmDirect(SafeCommand):
    @classmethod
    def toTransition(cls, arm: Arm):
        cmd = cls(arm, lambda: properties.transition_extension, lambda: properties.transition_elevation)
        cmd.setName(cmd.getName() + ".toTransition")
        return cmd

    def __init__(self, arm: Arm, extension_end_position: FloatProperty, elevator_end_position: FloatProperty):
        super().__init__()
        self.arm = arm
        self.extension_end_position = asCallable(extension_end_position)
        self.elevator_end_position = asCallable(elevator_end_position)
        self.addRequirements(arm)

    def initialize(self) -> None:
        self.elevator_motion = TrapezoidalMotion(
            start_position=self.arm.getElevatorPosition(),
            end_position=self.elevator_end_position(),
            min_speed=properties.elevator_min_speed,
            max_speed=properties.elevator_max_speed,
            accel=properties.elevator_acceleration
        )
        self.extension_motion = TrapezoidalMotion(
            start_position=self.arm.getExtensionPosition(),
            end_position=self.extension_end_position(),
            start_speed=max(properties.extension_min_speed, abs(self.arm.getExtensionSpeed())),
            end_speed=properties.extension_min_speed,
            max_speed=properties.extension_max_speed,
            accel=properties.extension_acceleration
        )

    def execute(self) -> None:
        # Elevation
        if self.elevator_motion.isFinished():
            self.arm.setElevatorSpeed(0)
        else:
            current_elevation = self.arm.getElevatorPosition()
            self.elevator_motion.setPosition(current_elevation)
            self.arm.setElevatorSpeed(self.elevator_motion.getSpeed())

        # Extension
        if self.extension_motion.isFinished():
            self.arm.setExtensionSpeed(0)
        else:
            current_extension = self.arm.getExtensionPosition()
            self.extension_motion.setPosition(current_extension)
            self.arm.setExtensionSpeed(self.extension_motion.getSpeed())

    def isFinished(self) -> bool:
        return self.elevator_motion.isFinished() and self.extension_motion.isFinished()

    def end(self, interrupted: bool) -> None:
        self.arm.setElevatorSpeed(0)
        self.arm.setExtensionSpeed(0)


class MoveArmToTransition(SafeCommand):
    def __init__(self, arm: Arm):
        super().__init__()
        self.arm = arm
        self.addRequirements(arm)

    def initialize(self) -> None:
        self.elevator_motion = TrapezoidalMotion(
            start_position=self.arm.getElevatorPosition(),
            end_position=properties.transition_elevation,
            min_speed=properties.elevator_min_speed,
            max_speed=properties.elevator_max_speed,
            accel=properties.elevator_acceleration
        )
        self.extension_motion = TrapezoidalMotion(
            start_position=self.arm.getExtensionPosition(),
            end_position=properties.transition_extension,
            start_speed=properties.extension_min_speed,
            end_speed=properties.extension_min_speed,
            max_speed=properties.extension_max_speed,
            accel=properties.extension_acceleration
        )
        self._elevator_finished = False

    def execute(self) -> None:
        # Elevation
        if self.elevator_motion.isFinished():
            self.arm.setElevatorSpeed(0)
            if not self._elevator_finished:
                self._elevator_finished = True
                self.extension_motion = TrapezoidalMotion(
                    start_position=self.arm.getExtensionPosition(),
                    end_position=properties.transition_extension,
                    start_speed=max(properties.extension_min_speed, abs(self.arm.getExtensionSpeed())),
                    end_speed=properties.extension_max_speed,
                    max_speed=properties.extension_max_speed,
                    accel=properties.extension_acceleration
                )
        else:
            current_elevation = self.arm.getElevatorPosition()
            self.elevator_motion.setPosition(current_elevation)
            self.arm.setElevatorSpeed(self.elevator_motion.getSpeed())

        # Extension
        if self.extension_motion.isFinished():
            self.arm.setExtensionSpeed(0)
        else:
            current_extension = self.arm.getExtensionPosition()
            self.extension_motion.setPosition(current_extension)
            self.arm.setExtensionSpeed(self.extension_motion.getSpeed())

    def isFinished(self) -> bool:
        return self.extension_motion.isFinished() and self.elevator_motion.isFinished()

    def end(self, interrupted: bool) -> None:
        self.arm.setElevatorSpeed(0)


class _ClassProperties:
    # Elevator Properties #
    level1_extension = autoproperty(70.0, subtable=MoveArm.__name__)
    level2_extension = autoproperty(158.0, subtable=MoveArm.__name__)
    level3_extension = autoproperty(232.0, subtable=MoveArm.__name__)
    level3_drop_extension = autoproperty(232.0, subtable=MoveArm.__name__)
    floor_extension = autoproperty(80.0, subtable=MoveArm.__name__)
    base_extension = autoproperty(15.0, subtable=MoveArm.__name__)
    bin_extension = autoproperty(0.0, subtable=MoveArm.__name__)
    transition_extension = autoproperty(60.0, subtable=MoveArm.__name__)

    elevator_min_speed = autoproperty(0.2, subtable=MoveArmDirect.__name__)
    elevator_max_speed = autoproperty(1.0, subtable=MoveArmDirect.__name__)
    elevator_acceleration = autoproperty(0.03, subtable=MoveArmDirect.__name__)

    # Extension Properties #
    level1_elevation = autoproperty(60.0, subtable=MoveArm.__name__)
    level2_elevation = autoproperty(55.0, subtable=MoveArm.__name__)
    level3_elevation = autoproperty(50.0, subtable=MoveArm.__name__)
    level3_drop_elevation = autoproperty(70.0, subtable=MoveArm.__name__)
    floor_elevation = autoproperty(210.0, subtable=MoveArm.__name__)
    base_elevation = autoproperty(0.0, subtable=MoveArm.__name__)
    bin_elevation = autoproperty(0.0, subtable=MoveArm.__name__)
    transition_elevation = autoproperty(0.0, subtable=MoveArm.__name__)

    extension_min_speed = autoproperty(0.1, subtable=MoveArmDirect.__name__)
    extension_max_speed = autoproperty(1.0, subtable=MoveArmDirect.__name__)
    extension_acceleration = autoproperty(0.035, subtable=MoveArmDirect.__name__)


properties = _ClassProperties()

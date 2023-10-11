import commands2
from subsystems.arm import Arm
from utils.property import autoproperty
from utils.safecommand import SafeCommand, SafeMixin


class ResetArm(SafeMixin, commands2.SequentialCommandGroup):
    def __init__(self, arm: Arm):
        super().__init__(
            _ResetElevator(arm),
            _ResetExtension(arm).withTimeout(5.0),
        )


class _ResetElevator(SafeCommand):
    elevator_speed = autoproperty(0.25, subtable=ResetArm.__name__)

    def __init__(self, arm: Arm):
        super().__init__()
        self.arm = arm
        self.addRequirements(self.arm)

    def initialize(self) -> None:
        self.arm.is_reset = True

    def execute(self) -> None:
        self.arm.setElevatorSpeed(-self.elevator_speed)

    def isFinished(self) -> bool:
        return self.arm.isSwitchElevatorMinOn()

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.arm.is_reset = False
        else:
            self.arm.resetElevator()
        self.arm.setElevatorSpeed(0.0)


class _ResetExtension(SafeCommand):
    extension_speed = autoproperty(0.25, subtable=ResetArm.__name__)

    def __init__(self, arm: Arm):
        super().__init__()
        self.arm = arm
        self.addRequirements(self.arm)

    def initialize(self) -> None:
        self.arm.is_reset = True

    def execute(self) -> None:
        self.arm.setExtensionSpeed(-self.extension_speed)

    def isFinished(self) -> bool:
        return self.arm.isSwitchExtensionMinOn()

    def end(self, interrupted: bool) -> None:
        if interrupted:
            self.arm.is_reset = False
        else:
            self.arm.resetExtension()
        self.arm.setExtensionSpeed(0.0)

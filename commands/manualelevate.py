import math
from typing import Literal

from utils.property import autoproperty
from utils.safecommand import SafeCommand
from subsystems.arm import Arm


class ManualElevate(SafeCommand):
    speed = autoproperty(0.15)

    @classmethod
    def up(cls, arm: Arm):
        cmd = cls(arm, "up")
        cmd.setName(cmd.getName() + ".up")
        return cmd

    @classmethod
    def down(cls, arm: Arm):
        cmd = cls(arm, "down")
        cmd.setName(cmd.getName() + ".down")
        return cmd

    def __init__(self, arm: Arm, direction: Literal["up", "down"]):
        super().__init__()
        self.arm = arm
        self.direction = direction
        self.addRequirements(self.arm)

    def execute(self) -> None:
        self.arm.motor_elevator.set(math.copysign(self.speed, 1.0 if self.direction == "up" else -1.0))

    def end(self, interrupted: bool) -> None:
        self.arm.setElevatorSpeed(0.0)

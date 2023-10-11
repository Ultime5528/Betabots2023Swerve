from utils.safecommand import SafeCommand
from subsystems.arm import Arm


class StopArm(SafeCommand):
    def __init__(self, arm: Arm):
        super().__init__()
        self.arm = arm
        self.addRequirements(arm)

    def execute(self) -> None:
        self.arm.setExtensionSpeed(0)
        self.arm.setElevatorSpeed(0)

import wpilib
from utils.safecommand import SafeCommand
from subsystems.claw import Claw
from utils.property import autoproperty


class OpenClaw(SafeCommand):
    duration = autoproperty(1.0)

    def __init__(self, claw: Claw):
        super().__init__()
        self.claw = claw
        self.timer = wpilib.Timer()
        self.addRequirements(self.claw)

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.claw.open()

    def isFinished(self) -> bool:
        return self.timer.get() >= self.duration

    def end(self, interrupted: bool) -> None:
        self.claw.stop()

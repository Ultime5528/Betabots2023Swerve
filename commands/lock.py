import wpilib

from utils.property import autoproperty
from utils.safecommand import SafeCommand
from subsystems.blocker import Blocker


class Lock(SafeCommand):
    timeout = autoproperty(1.0)

    def __init__(self, blocker: Blocker):
        super().__init__()
        self.blocker = blocker
        self.addRequirements(blocker)
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.stop()
        self.timer.reset()

    def execute(self) -> None:
        # unroll
        if self.blocker.isArmDown():
            # stop unroll
            self.blocker.lock()
            self.timer.start()

    def isFinished(self) -> bool:
        return self.timer.get() >= self.timeout

    def end(self, interrupted: bool) -> None:
        self.lock.stop()
        # stop unroll

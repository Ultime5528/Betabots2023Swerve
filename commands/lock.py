import wpilib

from utils.property import autoproperty
from utils.safecommand import SafeCommand
from subsystems.catapult import Catapult


class Lock(SafeCommand):
    timeout = autoproperty(1.0)

    def __init__(self, catapult: Catapult):
        super().__init__()
        self.catapult = catapult
        self.addRequirements(catapult)
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.catapult.lock()

    def isFinished(self) -> bool:
        return self.timer.get() >= self.timeout

    def end(self, interrupted: bool) -> None:
        self.catapult.stopLocker()

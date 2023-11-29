from utils.property import autoproperty
from utils.safecommand import SafeCommand
from subsystems.catapult import Catapult
import wpilib


class Unlock(SafeCommand):
    timeout = autoproperty(1.0)

    def __init__(self, catapult: Catapult):
        super().__init__()
        self.catapult = catapult
        self.addRequirements(self.catapult)
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.stop()
        self.timer.reset()

    def execute(self) -> None:
        if not self.catapult.isArmDown():
            self.catapult.unlock()
            self.timer.start()

    def isFinished(self) -> bool:
        return self.timer.get() >= self.timeout

    def end(self, interrupted: bool) -> None:
        self.catapult.idle()

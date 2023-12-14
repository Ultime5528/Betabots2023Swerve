from subsystems.catapult import Catapult
from utils.property import autoproperty
from utils.safecommand import SafeCommand
import wpilib


class Load(SafeCommand):
    open_time = autoproperty(0.75)

    def __init__(self, catapult: Catapult):
        super().__init__()
        self.addRequirements(catapult)
        self.catapult = catapult
        self.timer = wpilib.Timer()

    def initialize(self) -> None:
        self.timer.reset()
        self.timer.start()

    def execute(self) -> None:
        self.catapult.open()

    def isFinished(self) -> bool:
        return self.timer.get() >= self.open_time

    def end(self, interrupted: bool) -> None:
        self.timer.stop()
        self.catapult.close()

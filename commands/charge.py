from subsystems.catapult import Catapult
from utils.property import autoproperty
from utils.safecommand import SafeCommand


class Charge(SafeCommand):
    distance1 = autoproperty(0.0)
    distance2 = autoproperty(0.0)
    distance3 = autoproperty(0.0)
    threshold = autoproperty(0.0)

    def __init__(self, catapult: Catapult, distanceWanted: int):
        super().__init__()
        self.catapult = catapult
        self.addRequirements(self.catapult)
        self.get_distance = {
            1: lambda: self.distance1,
            2: lambda: self.distance2,
            3: lambda: self.distance3
        }[distanceWanted]

    def execute(self) -> None:
        self.catapult.moveUp()

    def isFinished(self) -> bool:
        return abs(self.catapult.encoder.getPosition() - self.get_distance()) < self.threshold

    def end(self, interrupted: bool) -> None:
        self.catapult.stopArm()

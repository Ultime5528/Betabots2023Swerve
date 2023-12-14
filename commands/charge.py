from subsystems.catapult import Catapult
from utils.property import autoproperty
from utils.safecommand import SafeCommand


class Charge(SafeCommand):
    # Multiply wanted turns by 36 for gearbox
    distance1 = autoproperty(36.0)  # 1 * 36
    distance2 = autoproperty(54.0)  # 1.5 * 36
    distance3 = autoproperty(72.0)  # 2 * 36
    threshold = autoproperty(0.1)

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
        move_down = self.catapult.encoder.getPosition() > self.get_distance()
        if move_down:
            self.catapult.moveDown()
        else:
            self.catapult.moveUp()

    def isFinished(self) -> bool:
        return abs(self.catapult.encoder.getPosition() - self.get_distance()) < self.threshold

    def end(self, interrupted: bool) -> None:
        self.catapult.stopArm()

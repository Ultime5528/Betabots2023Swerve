from utils.safecommand import SafeCommand
from subsystems.catapult import Catapult


class ResetLauncher(SafeCommand):
    def __init__(self, catapult: Catapult):
        super().__init__()
        self.catapult = catapult
        self.addRequirements(self.catapult)

    def execute(self) -> None:
        self.catapult.moveDown()

    def isFinished(self) -> bool:
        return self.catapult.isArmDown()

    def end(self, interrupted: bool) -> None:
        self.catapult.stopMotor()

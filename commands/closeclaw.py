from utils.safecommand import SafeCommand
from subsystems.claw import Claw


class CloseClaw(SafeCommand):
    def __init__(self, claw: Claw):
        super().__init__()
        self.claw = claw
        self.addRequirements(self.claw)

    def execute(self) -> None:
        self.claw.close()

    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        self.claw.stop()

from utils.safecommand import SafeCommand
from subsystems.blocker import Blocker


class Unlock(SafeCommand):
    def __init__(self, blocker: Blocker):
        super().__init__()
        self.blocker = blocker
        self.addRequirements(self.blocker)

    def execute(self) -> None:
        if not self.blocker.isArmDown():
            self.blocker.unlock()

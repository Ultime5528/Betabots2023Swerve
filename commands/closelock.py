from utils.safecommand import SafeCommand
from subsystems.blocker import Blocker


class CloseLock(SafeCommand):
    def __init__(self, blocker: Blocker):
        super().__init__()
        self.blocker = blocker
        self.addRequirements(blocker)

    def execute(self) -> None:
        if self.blocker.limit_switch_blocker.get():
            self.blocker.lock()

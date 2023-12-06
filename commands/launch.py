from commands2 import SequentialCommandGroup

from commands.load import Load
from subsystems.catapult import Catapult
from commands.resetarm import ResetArm
from commands.unlock import Unlock
from commands.lock import Lock

from utils.safecommand import SafeMixin


class Launch(SequentialCommandGroup, SafeMixin):
    def __init__(self, catapult: Catapult):
        super().__init__(
            Unlock(catapult),
            ResetArm(catapult),
            Lock(catapult),
            Load(catapult)
        )
        self.setName("Launch")

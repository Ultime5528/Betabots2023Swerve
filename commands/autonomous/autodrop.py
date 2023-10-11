import commands2

from commands.openclaw import OpenClaw
from commands.resetarm import ResetArm
from subsystems.claw import Claw
from subsystems.arm import Arm

from commands.movearm import MoveArm
from commands.closeclaw import CloseClaw
from utils.safecommand import SafeMixin


class AutoDrop(SafeMixin, commands2.SequentialCommandGroup):
    def __init__(self, claw: Claw, arm: Arm):
        super().__init__(
            ResetArm(arm),
            CloseClaw(claw).withTimeout(0.5),
            MoveArm.toLevel3(arm),
            MoveArm.toLevel3Drop(arm),
            OpenClaw(claw).withTimeout(0.5)
        )

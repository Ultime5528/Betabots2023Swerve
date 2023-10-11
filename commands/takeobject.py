from subsystems.arm import Arm
from subsystems.claw import Claw

from commands.movearm import MoveArm
from commands.closeclaw import CloseClaw

from commands2 import SequentialCommandGroup


class TakeObject(SequentialCommandGroup):
    def __init__(self, claw: Claw, arm: Arm):
        super().__init__(
            MoveArm.toBin(arm),
            CloseClaw(claw),
            MoveArm.toBase(arm)
        )
        self.setName("TakeObject")

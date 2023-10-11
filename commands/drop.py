import commands2.cmd

from subsystems.claw import Claw
from subsystems.arm import Arm

from commands.openclaw import OpenClaw
from commands.movearm import MoveArm, properties

from commands2 import SequentialCommandGroup, ConditionalCommand

from utils.property import autoproperty
from utils.safecommand import SafeMixin


class Drop(SafeMixin, SequentialCommandGroup):
    level3_arm_threshold = autoproperty(5.0)

    def __init__(self, claw: Claw, arm: Arm):
        super().__init__(
            ConditionalCommand(
                MoveArm.toLevel3Drop(arm),
                commands2.cmd.nothing(),
                lambda: abs(arm.getExtensionPosition() - properties.level3_extension) <= self.level3_arm_threshold
            ),
            OpenClaw(claw),
            MoveArm.toBin(arm)
        )

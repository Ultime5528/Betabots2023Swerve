
import commands2

from commands.autonomous.autodrop import AutoDrop
from commands.movearm import MoveArm
from commands.traversedock import TraverseDock
from subsystems.drivetrain import Drivetrain
from subsystems.claw import Claw
from subsystems.arm import Arm


class AutoTraverse(commands2.SequentialCommandGroup):
    def __init__(self, drivetrain: Drivetrain, claw: Claw, arm: Arm, drop: bool):
        commands = [
            commands2.ParallelCommandGroup(
                commands2.SequentialCommandGroup(
                    MoveArm.toLevel2(arm),
                    commands2.WaitCommand(1.5),
                    MoveArm.toBase(arm)
                ),
                TraverseDock(drivetrain)
            )
        ]

        if drop:
            commands.insert(0, AutoDrop(claw, arm))

        super().__init__(
            *commands
        )

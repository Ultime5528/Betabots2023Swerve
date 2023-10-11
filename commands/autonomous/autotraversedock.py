import commands2

from subsystems.drivetrain import Drivetrain
from subsystems.claw import Claw
from subsystems.arm import Arm

from commands.autonomous.autodrop import AutoDrop
from commands.movearm import MoveArm
from commands.drivetodock import DriveToDock
from commands.traversedock import TraverseDock
from utils.safecommand import SafeMixin


class AutoTraverseDock(SafeMixin, commands2.SequentialCommandGroup):
    def __init__(self, drivetrain: Drivetrain, claw: Claw, arm: Arm, drop: bool):
        commands = [
            commands2.ParallelCommandGroup(
                commands2.SequentialCommandGroup(
                    # MoveArm.toLevel2(arm),
                    # commands2.WaitCommand(1),
                    MoveArm.toBase(arm)
                ),
                commands2.SequentialCommandGroup(
                    TraverseDock(drivetrain),
                    # commands2.WaitCommand(0.75),
                    DriveToDock(drivetrain)
                )
            )
        ]

        if drop:
            commands.insert(0, AutoDrop(claw, arm))

        super().__init__(
            *commands
        )

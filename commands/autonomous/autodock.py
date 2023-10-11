import commands2

from commands.autonomous.autodrop import AutoDrop
from commands.drivetodock import DriveToDock
from commands.movearm import MoveArm
from subsystems.drivetrain import Drivetrain
from subsystems.claw import Claw
from subsystems.arm import Arm
from utils.property import autoproperty


class AutoDock(commands2.SequentialCommandGroup):
    backwards_distance = autoproperty(10.0)

    def __init__(self, drivetrain: Drivetrain, claw: Claw, arm: Arm, drop: bool):
        commands = [commands2.ParallelCommandGroup(
            MoveArm.toBase(arm),
            commands2.SequentialCommandGroup(
                commands2.WaitCommand(1),
                DriveToDock(drivetrain, True)
            )
        )]
        if drop:
            commands.insert(0, AutoDrop(claw, arm))

        super().__init__(
            *commands
        )
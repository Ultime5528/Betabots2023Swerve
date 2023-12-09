from wpimath.geometry import Pose2d
from commands2 import SequentialCommandGroup
from commands2 import ParallelCommandGroup

from commands.drivedistance import DriveDistance

from commands.charge import Charge
from commands.load import Load
from commands.launch import Launch

from subsystems.drivetrain import Drivetrain


from subsystems.catapult import Catapult

from utils.safecommand import SafeMixin


class AutoMiddleFlowerGreen(SequentialCommandGroup, SafeMixin):
    def __init__(self, drivetrain: Drivetrain, catapult: Catapult):
        super().__init__(
            ParallelCommandGroup(
                DriveDistance(drivetrain, Pose2d(0.91, -0.91, 0), 0.5),
                Load(catapult),
                Charge(catapult, 3)
            ),
            Launch(catapult)
        )
        self.addRequirements(drivetrain, catapult)

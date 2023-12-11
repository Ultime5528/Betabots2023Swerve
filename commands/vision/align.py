from subsystems.drivetrain import Drivetrain
from utils.safecommand import SafeCommand
from commands.


class Align(SafeCommand):
    def __init__(self, drivetrain: Drivetrain):
        super().__init__()
        self.drivetrain = Drivetrain

    
from subsystems.drivetrain import Drivetrain
from utils.safecommand import SafeCommand
from commands.


class Align(SafeCommand):
    centering_speed = autoproperty(0.5)
    centering_rot_speed = autoproperty(0.25)
    def __init__(self, drivetrain: Drivetrain):
        super().__init__()
        self.drivetrain = Drivetrain
        self.centered_tag_x = lambda: NetworkTables.getEntry("vision/tag_x")
        self.centered_tag_y = lambda: NetworkTables.getEntry("vision/tag_y")
        self.centered_tag_rot = lambda: NetworkTables.getEntry("vision/tag_rot")
        self.estimator = self.drivetrain.getEstimator()

    def execute(self):
        self.rot_error = self.centered_tag_rot - self.estimator.getEstimatedPosition().rot
        vx = math.copysign(self. centered_tag_x, centering_speed) * -1
        vr = math.copysign(self.rot_error, self.centering_rot_speed)

        self.drivetrain.drive(0, vx, vr, True, True)

    def isFinished(self) -> book:
        return self.centered_tag_x <= 0.1 and self.centered_tag_rot <= 1

    def end(self, interrupted):
        self.drivetrain.drive(0,0,0,True, True)


    
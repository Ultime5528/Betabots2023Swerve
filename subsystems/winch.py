import wpilib
import ports
from utils.property import autoproperty
from utils.safesubsystem import SafeSubsystem


class Winch(SafeSubsystem):
    spool_speed = autoproperty(0.25)
    unspool_speed = autoproperty(-0.25)

    def __init__(self):
        super().__init__()
        self.motor = wpilib.Spark(ports.winch_motor)

    def spool(self):
        self.motor.set(self.spool_speed)

    def unspool(self):
        self.motor.set(self.unspool_speed)

    def stop(self):
        self.motor.stopMotor()

import wpilib
import ports
from utils.safesubsystem import SafeSubsystem
from utils.property import autoproperty


class Loader(SafeSubsystem):
    revolve_speed = autoproperty(0.37)

    def __init__(self):
        super().__init__()
        self.motor = wpilib.Spark(ports.loader_motor)

    def revolve(self):
        self.motor.set(self.spin_speed)

    def stop(self):
        self.motor.stopMotor()

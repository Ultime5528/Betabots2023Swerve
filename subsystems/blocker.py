import wpilib
from wpilib import DigitalInput, PneumaticsModuleType, RobotBase
import wpilib.simulation
import ports
from utils.safesubsystem import SafeSubsystem


class Blocker(SafeSubsystem):
    def __init__(self):
        super().__init__()
        self.limit_switch_blocker = DigitalInput(ports.blocker_limitswitch)
        self.blocker = wpilib.DoubleSolenoid(PneumaticsModuleType.REVPH, ports.blocker_solenoid_forward, ports.blocker_solenoid_reverse)

    def lock(self):
        self.blocker.set(wpilib.DoubleSolenoid.Value.kForward)

    def unlock(self):
        self.blocker.set(wpilib.DoubleSolenoid.Value.kReverse)
        
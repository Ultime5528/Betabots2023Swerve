import wpilib
import ports
from utils.safesubsystem import SafeSubsystem


class Claw(SafeSubsystem):
    def __init__(self):
        super().__init__()
        self.piston = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM,
            ports.claw_piston_forward,
            ports.claw_piston_reverse
        )
        self.is_closed = False

        self.addChild("piston", self.piston)

    def open(self):
        self.is_closed = False
        self.piston.set(wpilib.DoubleSolenoid.Value.kReverse)

    def close(self):
        self.is_closed = True
        self.piston.set(wpilib.DoubleSolenoid.Value.kForward)

    def stop(self):
        self.piston.set(wpilib.DoubleSolenoid.Value.kOff)

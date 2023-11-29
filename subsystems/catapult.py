import wpilib
from wpilib import DigitalInput, PneumaticsModuleType, RobotBase
import wpilib.simulation
import ports
from utils.property import autoproperty
from utils.safesubsystem import SafeSubsystem

    
class Catapult(SafeSubsystem):
    speed_up = autoproperty(0.25)
    speed_down = autoproperty(-0.25)

    def __init__(self):
        super().__init__()
        self.limit_switch_catapult = DigitalInput(ports.catapult_limitswitch)
        self.piston_catapult = wpilib.DoubleSolenoid(PneumaticsModuleType.REVPH, ports.blocker_solenoid_forward,
                                                     ports.blocker_solenoid_reverse)
        self.winch_motor = wpilib.Spark(ports.catapult_motor)

    def lock(self):
        self.piston_catapult.set(wpilib.DoubleSolenoid.Value.kForward)

    def unlock(self):
        self.piston_catapult.set(wpilib.DoubleSolenoid.Value.kReverse)

    def idle(self):
        self.piston_catapult.set(wpilib.DoubleSolenoid.Value.kOff)

    def isDown(self):
        return self.limit_switch_catapult.get()

    def moveUp(self):
        self.winch_motor.set(self.up_speed)

    def moveDown(self):
        self.winch_motor.set(self.down_speed)

    def stopArm(self):
        self.winch_motor.stopMotor()

    def isArmDown(self):
        return self.limit_switch_catapult.get()

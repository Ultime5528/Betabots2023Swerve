import wpilib
from wpilib import DigitalInput, PneumaticsModuleType, RobotBase
import wpilib.simulation
import ports
from utils.property import autoproperty
from utils.safesubsystem import SafeSubsystem
import rev

from utils.sparkmaxsim import SparkMaxSim


class Catapult(SafeSubsystem):
    speed_up = autoproperty(0.25)
    speed_down = autoproperty(-0.25)
    servo_angle_open = autoproperty(90)
    servo_angle_close = autoproperty(0)

    def __init__(self):
        super().__init__()
        self.switch = DigitalInput(ports.catapult_limitswitch)
        self.motor = rev.CANSparkMax(ports.catapult_motor, rev.CANSparkMax.MotorType.kBrushless)
        self.servo = wpilib.Servo(ports.catapult_servo)
        self.encoder = self.motor.getEncoder()

        self.piston = wpilib.DoubleSolenoid(PneumaticsModuleType.CTREPCM, ports.catapult_solenoid_forward,
                                            ports.catapult_solenoid_reverse)

        if RobotBase.isSimulation():
            self.sim_motor = SparkMaxSim(self.motor)

    def close(self):
        self.servo.setAngle(self.servo_angle_close)

    def open(self):
        self.servo.setAngle(self.servo_angle_open)

    def lock(self):
        self.piston.set(wpilib.DoubleSolenoid.Value.kForward)

    def unlock(self):
        self.piston.set(wpilib.DoubleSolenoid.Value.kReverse)

    def stopLocker(self):
        self.piston.set(wpilib.DoubleSolenoid.Value.kOff)

    def moveUp(self):
        self.motor.set(self.speed_up)

    def moveDown(self):
        self.motor.set(self.speed_down)

    def stopArm(self):
        self.motor.stopMotor()

    def isArmDown(self):
        return not self.switch.get()

    def simulationPeriodic(self) -> None:
        self.sim_motor.setVelocity(self.motor.get())

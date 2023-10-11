import rev
import wpilib
import wpiutil
import wpilib
from wpilib import DigitalInput, RobotBase, Mechanism2d, Color8Bit
from wpilib.event import EventLoop, BooleanEvent
from wpilib.simulation import DIOSim

import ports
from utils.property import autoproperty, defaultSetter
from utils.safesubsystem import SafeSubsystem
from utils.sparkmaxsim import SparkMaxSim
from utils.sparkmaxutils import configureLeader


def checkIsInDeadzone(extension: float):
    return extension <= properties.deadzone_extension


class Arm(SafeSubsystem):
    elevator_max_position = autoproperty(235.0)

    # Simulation
    elevator_min_position = autoproperty(0.0)
    extension_max_position = autoproperty(10.0)
    extension_min_position = autoproperty(0.0)

    def __init__(self):
        super().__init__()
        # Switches
        self.is_reset = False
        self.switch_extension_min = DigitalInput(ports.arm_switch_extension_min)
        self.addChild("switch_extension_min", self.switch_extension_min)

        self.switch_extension_max = DigitalInput(ports.arm_switch_extension_max)
        self.addChild("switch_extension_max", self.switch_extension_max)

        self.switch_elevator_min = DigitalInput(ports.arm_switch_elevator_min)
        self.addChild("switch_elevator_min", self.switch_elevator_min)

        # Motors
        self.motor_elevator = rev.CANSparkMax(ports.arm_motor_elevator,
                                              rev.CANSparkMax.MotorType.kBrushless)
        configureLeader(self.motor_elevator, "brake", True)

        self.motor_extension = rev.CANSparkMax(ports.arm_motor_extension,
                                               rev.CANSparkMax.MotorType.kBrushless)
        configureLeader(self.motor_extension, "brake", True)
        self.motor_extension.setSmartCurrentLimit(15, 30)

        self.encoder_extension = self.motor_extension.getEncoder()
        self.encoder_elevator = self.motor_elevator.getEncoder()

        self.photocell = wpilib.DigitalInput(ports.arm_photocell)
        self.addChild("photocell", self.photocell)

        self._extension_offset = 0.0
        self._elevator_offset = 0.0

        # self.loop = EventLoop()
        # self._min_elevator_event = BooleanEvent(
        #     self.loop, self.isSwitchElevatorMinOn
        # )
        # self._min_elevator_event.ifHigh(self.resetElevator)
        #
        # self._min_extension_event = BooleanEvent(
        #     self.loop, self.isSwitchExtensionMinOn
        # )
        # self._min_extension_event.ifHigh(self.resetExtension)

        # self._max_extension_event = BooleanEvent(
        #     self.loop, self.isSwitchExtensionMaxOn
        # ).rising()
        # self._max_extension_event.ifHigh(self.maximizeExtension)

        if RobotBase.isSimulation():
            self.motor_elevator_sim = SparkMaxSim(self.motor_elevator)
            self.motor_extension_sim = SparkMaxSim(self.motor_extension)
            self.switch_extension_min_sim = DIOSim(self.switch_extension_min)
            self.switch_extension_max_sim = DIOSim(self.switch_extension_max)
            self.switch_elevator_min_sim = DIOSim(self.switch_elevator_min)
            self.mech = Mechanism2d(350, 200)
            self.root = self.mech.getRoot("Arm root", 340, 80)
            support = self.root.appendLigament("Support", 50, 90)
            self.mech_elevator = support.appendLigament("Elevator", 10, 90)
            claw = self.mech_elevator.appendLigament("Claw", 30, 90, lineWidth=4, color=Color8Bit(255, 0, 0))
            self.addChild("Mechanism", self.mech)

    def simulationPeriodic(self):
        motor_elevator_sim_increment = self.motor_elevator.get() * 0.5
        motor_extension_sim_increment = self.motor_extension.get() * 4.0
        self.motor_elevator_sim.setPosition(self.motor_elevator_sim.getPosition() + motor_elevator_sim_increment)
        self.motor_extension_sim.setPosition(self.motor_extension_sim.getPosition() + motor_extension_sim_increment)
        self.switch_extension_min_sim.setValue(self.getExtensionPosition() <= 0.05)
        self.switch_extension_max_sim.setValue(self.getExtensionPosition() >= 227.0)
        self.mech_elevator.setAngle((135 - -10) * (self.encoder_elevator.getPosition()) / self.elevator_max_position)
        self.mech_elevator.setLength(self.encoder_extension.getPosition())

    # def periodic(self):
    #     self.loop.poll()

    def resetExtension(self):
        self._extension_offset = self.encoder_extension.getPosition()

    def resetElevator(self):
        self._elevator_offset = self.encoder_elevator.getPosition()

    def getElevatorPosition(self):
        if self.isElevatorMin():
            return 0.0
        return self.encoder_elevator.getPosition() - self._elevator_offset

    def getExtensionPosition(self):
        if self.isExtensionMin():
            return 0.0
        return self.encoder_extension.getPosition() - self._extension_offset

    def isSwitchExtensionMinOn(self):
        return not self.switch_extension_min.get()

    def isSwitchExtensionMaxOn(self):
        return not self.switch_extension_max.get()

    def isSwitchElevatorMinOn(self):
        return not self.switch_elevator_min.get()

    def isExtensionMax(self):
        return self.isSwitchExtensionMaxOn()

    def isExtensionMin(self):
        return self.isSwitchExtensionMinOn()

    def isElevatorMax(self):
        return self.getElevatorPosition() > self.elevator_max_position

    def isElevatorMin(self):
        return self.isSwitchElevatorMinOn()

    def setElevatorSpeed(self, speed: float):
        if self.is_reset:
            if self.isElevatorMin() and speed < 0:
                speed = 0
            if self.isElevatorMax() and speed > 0:
                speed = 0
            self.motor_elevator.set(speed)
        else:
            self.motor_elevator.set(0)

    def setExtensionSpeed(self, speed: float):
        if self.is_reset:
            if self.isExtensionMin() and speed < 0:
                speed = 0
            if self.isExtensionMax() and speed > 0:
                speed = 0
            self.motor_extension.set(speed)
        else:
            self.motor_extension.set(0)

    def getExtensionSpeed(self):
        return self.motor_extension.get()

    def isInDeadzone(self):
        return checkIsInDeadzone(self.getExtensionPosition())

    def shouldTransition(self, extension: float, elevator: float):
        return self.isInDeadzone() != checkIsInDeadzone(extension)

    def initSendable(self, builder: wpiutil.SendableBuilder) -> None:
        super().initSendable(builder)
        builder.addDoubleProperty("Elevator position", self.getElevatorPosition, defaultSetter)
        builder.addDoubleProperty("Extension position", self.getExtensionPosition, defaultSetter)
        builder.addDoubleProperty("Elevator speed", self.motor_elevator.get, defaultSetter)
        builder.addDoubleProperty("Extension speed", self.motor_extension.get, defaultSetter)

    def hasObject(self):
        return self.photocell.get()


class _ClassProperties:
    deadzone_extension = autoproperty(60.0, subtable=Arm.__name__)


properties = _ClassProperties()

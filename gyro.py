import math
from abc import ABC, abstractmethod

import hal
import navx
import wpilib
from wpilib.simulation import SimDeviceSim
from wpimath.geometry import Rotation2d
from wpiutil import Sendable, SendableBuilder

from utils.property import defaultSetter, autoproperty


class AbstractSendableMetaclass(type(ABC), type(Sendable)):
    pass


class AbstractSendable(ABC, Sendable, metaclass=AbstractSendableMetaclass):
    def initSendable(self, builder: SendableBuilder) -> None:
        super().__init__(builder)


class Gyro(AbstractSendable):
    def __init__(self):
        super().__init__()
        self.calibrate()

    @abstractmethod
    def getAngle(self): ...

    @abstractmethod
    def getPitch(self): ...

    @abstractmethod
    def setSimAngle(self, angle: float): ...

    @abstractmethod
    def setSimPitch(self, angle: float): ...

    def reset(self):
        self.gyro.reset()

    def calibrate(self):
        self.gyro.calibrate()

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngle())

    def initSendable(self, builder: SendableBuilder) -> None:
        super().initSendable(builder)
        builder.addDoubleProperty("angle", self.getAngle, defaultSetter)
        builder.addDoubleProperty("pitch", self.getPitch, defaultSetter)


class NavX(Gyro):
    def __init__(self):
        self.gyro = navx.AHRS(wpilib.SerialPort.Port.kMXP)
        super().__init__()
        gyro_sim_device = SimDeviceSim("navX-Sensor[1]")
        self._gyro_sim_angle = gyro_sim_device.getDouble("Yaw")
        self._gyro_sim_pitch = gyro_sim_device.getDouble("Roll")

    def getAngle(self):
        return -math.remainder(self.gyro.getAngle(), 360.0)

    def getPitch(self):
        return -self.gyro.getRoll()

    def setSimAngle(self, angle: float):
        self._gyro_sim_angle.set(-angle)

    def setSimPitch(self, pitch: float):
        self._gyro_sim_pitch.set(pitch)


class ADIS16448(Gyro):
    def __init__(self):
        self.gyro = wpilib.ADIS16448_IMU()
        super().__init__()
        gyro_sim_device = SimDeviceSim("Gyro:ADIS16448[4]")
        self._gyro_sim_angle = gyro_sim_device.getDouble("gyro_angle_z")
        self._gyro_sim_pitch = gyro_sim_device.getDouble("gyro_angle_y")

    def getAngle(self):
        return -math.remainder(self.gyro.getAngle(), 360.0)

    def getPitch(self):
        return math.remainder(self.gyro.getGyroAngleY(), 360.0)

    def setSimAngle(self, angle: float):
        self._gyro_sim_angle.set(angle)

    def setSimPitch(self, pitch: float):
        self._gyro_sim_pitch.set(pitch)


class ADIS16470(Gyro):
    pitch_offset = autoproperty(4.1)
    def __init__(self):
        self.gyro = wpilib.ADIS16470_IMU()
        super().__init__()
        gyro_sim_device = SimDeviceSim("Gyro:ADIS16470[0]")
        self._gyro_sim_angle = gyro_sim_device.getDouble("gyro_angle_z")
        self._gyro_sim_pitch = gyro_sim_device.getDouble("gyro_angle_y")

    def getAngle(self):
        return math.remainder(self.gyro.getAngle(), 360.0)

    def getPitch(self):
        return math.remainder(self.gyro.getYComplementaryAngle() + self.pitch_offset, 360.0)

    def setSimAngle(self, angle: float):
        self._gyro_sim_angle.set(angle)

    def setSimPitch(self, pitch: float):
        self._gyro_sim_pitch.set(pitch)

    def calibrate(self):
        if wpilib.RobotBase.isReal():
            self.gyro.calibrate()


class ADXRS(Gyro):
    def __init__(self):
        self.gyro = wpilib.ADXRS450_Gyro()
        super().__init__()
        gyro_sim_device = SimDeviceSim("Gyro:ADXRS450[0]")
        self._gyro_sim_angle = gyro_sim_device.getDouble("angle_x")
        self.pitch = 0

    def getAngle(self):
        return -math.remainder(self.gyro.getAngle(), 360.0)

    def getPitch(self):
        return self.pitch

    def setSimAngle(self, angle: float):
        self._gyro_sim_angle.set(angle)

    def setSimPitch(self, pitch: float):
        self.pitch = pitch


class Empty(Gyro):
    def __init__(self):
        super().__init__()
        self._device = hal.SimDevice("Empty-Gyro")
        self._gyro_sim_angle = self._device.createDouble("yaw", hal.SimValueDirection.HAL_SimValueOutput, 0)
        self._gyro_sim_pitch = self._device.createDouble("pitch", hal.SimValueDirection.HAL_SimValueOutput, 0)
        self.angle = 0
        self.pitch = 0

    def getAngle(self):
        return self.angle

    def getPitch(self):
        return self.pitch

    def setSimAngle(self, angle: float):
        self.angle = angle
        self._gyro_sim_angle.set(angle)

    def setSimPitch(self, pitch: float):
        self.pitch = pitch
        self._gyro_sim_pitch.set(pitch)

    def reset(self):
        pass

    def calibrate(self):
        pass

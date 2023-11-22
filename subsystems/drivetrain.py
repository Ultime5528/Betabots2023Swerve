import math
from typing import Literal, Callable

import wpilib
import wpiutil
from wpilib import RobotBase
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
    SwerveModuleState,
)

import ports
import utils.swervemodule
from gyro import ADIS16448, ADIS16470, ADXRS, Empty, Gyro, NavX
from utils.property import defaultSetter, autoproperty
from utils.safesubsystem import SafeSubsystem
from utils.swervemodule import SwerveModule
from utils.swerveutils import discretize

select_gyro: Literal["navx", "adis16448", "adis16470", "adxrs", "empty"] = "adis16470"



class Drivetrain(SafeSubsystem):
    width = autoproperty(0.68)
    length = autoproperty(0.68)
    max_angular_speed = autoproperty(math.pi)

    def __init__(self, getPeriod: Callable[[], int],) -> None:
        super().__init__()
        self.period_seconds = getPeriod

        # Swerve Module motor positions
        self.motor_fl_loc = Translation2d(self.width/2, self.length/2)
        self.motor_fr_loc = Translation2d(self.width/2, -(self.length/2))
        self.motor_bl_loc = Translation2d(-(self.width/2), self.length/2)
        self.motor_br_loc = Translation2d(-(self.width/2), -(self.length/2))

        self.swerve_module_fl = SwerveModule(
            ports.drivetrain_motor_driving_fl,
            ports.drivetrain_motor_turning_fl
        )
        self.swerve_module_fr = SwerveModule(
            ports.drivetrain_motor_driving_fr,
            ports.drivetrain_motor_turning_fr
        )
        self.swerve_module_bl = SwerveModule(
            ports.drivetrain_motor_driving_bl,
            ports.drivetrain_motor_turning_bl
         )
        self.swerve_module_br = SwerveModule(
            ports.drivetrain_motor_driving_br,
            ports.drivetrain_motor_turning_br
        )

        # Gyro
        self._gyro = {
            "navx": NavX,
            "adis16448": ADIS16448,
            "adis16470": ADIS16470,
            "adxrs": ADXRS,
            "empty": Empty,
        }[select_gyro]()

        self.addChild("Gyro", self._gyro)

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        self.swervedrive_kinematics = SwerveDrive4Kinematics(
            self.motor_fl_loc, self.motor_fr_loc, self.motor_bl_loc, self.motor_br_loc
        )

        self.swervedrive_odometry = SwerveDrive4Odometry(
            self.swervedrive_kinematics,
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.getPosition(),
                self.swerve_module_fr.getPosition(),
                self.swerve_module_bl.getPosition(),
                self.swerve_module_br.getPosition(),
            ),
            Pose2d(0, 0, 0),
        )

        self.swerve_estimator = SwerveDrive4PoseEstimator(
            self.swervedrive_kinematics,
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.getPosition(),
                self.swerve_module_fr.getPosition(),
                self.swerve_module_bl.getPosition(),
                self.swerve_module_br.getPosition(),
            ),
            Pose2d(0, 0, 0),
        )

        if RobotBase.isSimulation():
            self.sim_yaw = 0

    def joystickDrive(self,
        x_speed: float,
        y_speed: float,
        rot_speed: float,
        is_field_relative: bool,
    ):
        x_speed *= self.swerve_module_fr.max_speed
        y_speed *= self.swerve_module_fr.max_speed
        rot_speed *= self.max_angular_speed

        self.drive(x_speed, y_speed, rot_speed, is_field_relative)

    def drive(
        self,
        x_speed: float,
        y_speed: float,
        rot_speed: float,
        is_field_relative: bool,
    ):
        swerve_module_states = self.swervedrive_kinematics.toSwerveModuleStates(
            discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    x_speed, y_speed, rot_speed, self._gyro.getRotation2d()
                )
                if is_field_relative
                else ChassisSpeeds(x_speed, y_speed, rot_speed),
                self.period_seconds(),
            )
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, self.swerve_module_fr.max_speed)
        self.swerve_module_fl.setDesiredState(swerve_module_states[0])
        self.swerve_module_fr.setDesiredState(swerve_module_states[1])
        self.swerve_module_bl.setDesiredState(swerve_module_states[2])
        self.swerve_module_br.setDesiredState(swerve_module_states[3])

    def getRotation(self):
        return self._gyro.getRotation2d()

    def getPitch(self):
        return self._gyro.getPitch()

    def getPose(self):
        return self.swerve_estimator.getEstimatedPosition()

    def periodic(self):
        self.swerve_estimator.update(
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.getPosition(),
                self.swerve_module_fr.getPosition(),
                self.swerve_module_bl.getPosition(),
                self.swerve_module_br.getPosition(),
            ),
        )

        self._field.setRobotPose(self.swerve_estimator.getEstimatedPosition())

    def simulationPeriodic(self):
        self.swerve_module_fl.simulationUpdate()
        self.swerve_module_fr.simulationUpdate()
        self.swerve_module_bl.simulationUpdate()
        self.swerve_module_br.simulationUpdate()

        self.swervedrive_odometry.update(
            self._gyro.getRotation2d(),
            self.swerve_module_fl.getPosition(),
            self.swerve_module_fr.getPosition(),
            self.swerve_module_bl.getPosition(),
            self.swerve_module_br.getPosition(),
        )

        module_states = (
            self.swerve_module_fl.getState(),
            self.swerve_module_fr.getState(),
            self.swerve_module_bl.getState(),
            self.swerve_module_br.getState(),
        )

        chassis_speed = self.swervedrive_kinematics.toChassisSpeeds(*module_states)
        chassis_rotation_speed = chassis_speed.omega
        self.sim_yaw += chassis_rotation_speed * self.period_seconds()
        self._gyro.setSimAngle(-math.degrees(self.sim_yaw))

        self._field.setRobotPose(self.swervedrive_odometry.getPose())

    def initSendable(self, builder: wpiutil.SendableBuilder) -> None:
        super().initSendable(builder)

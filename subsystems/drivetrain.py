import math
from typing import Literal

import wpilib
import wpiutil
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, SwerveModulePosition, ChassisSpeeds
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from utils.safesubsystem import SafeSubsystem
from wpimath.estimator import SwerveDrive4PoseEstimator

from gyro import NavX, ADIS16448, ADIS16470, ADXRS, Empty
from utils.swervemodule import SwerveModule
from utils.swerveutils import discretize

select_gyro: Literal["navx", "adis16448", "adis16470", "adxrs", "empty"] = "adis16470"

kMaxSpeed = 3  # 3 meters per second
kMaxAngularSpeed = math.pi / 2  # 1/2 radian per second


class Drivetrain(SafeSubsystem):
    def __init__(self) -> None:
        super().__init__()
        # Swerve Module motor positions
        self.motor_fl_loc = Translation2d(0.33, 0.33)
        self.motor_fr_loc = Translation2d(0.33, -0.33)
        self.motor_bl_loc = Translation2d(-0.33, 0.33)
        self.motor_br_loc = Translation2d(-0.33, -0.33)

        self.swerve_module_fl = SwerveModule(1, 2, 0, 1, 2, 3)
        self.swerve_module_fr = SwerveModule(3, 4, 4, 5, 6, 7)
        self.swerve_module_bl = SwerveModule(5, 6, 8, 9, 10, 11)
        self.swerve_module_br = SwerveModule(7, 8, 12, 13, 14, 15)

        # Gyro
        self._gyro = {
            "navx": NavX,
            "adis16448": ADIS16448,
            "adis16470": ADIS16470,
            "adxrs": ADXRS,
            "empty": Empty,
        }[select_gyro]()

        self._field = wpilib.Field2d()

        self.swervedrive_kinematics = SwerveDrive4Kinematics(self.motor_fl_loc, self.motor_fr_loc, self.motor_bl_loc,
                                                             self.motor_br_loc)

        self.swervedrive_odometry = SwerveDrive4Odometry(
            self.swervedrive_kinematics,
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.get_position(), self.swerve_module_fr.get_position(),
                self.swerve_module_bl.get_position(), self.swerve_module_br.get_position()
            ),
            Pose2d(0, 0, 0)
        )

        self.swerve_estimator = SwerveDrive4PoseEstimator(
            self.swervedrive_kinematics,
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.get_position(), self.swerve_module_fr.get_position(),
                self.swerve_module_bl.get_position(), self.swerve_module_br.get_position()
            ),
            Pose2d(0, 0, 0)
        )

    def drive(self, x_speed: float, y_speed: float, rot_speed: float, is_field_relative: bool, period_seconds: float):
        swerve_module_states = self.swervedrive_kinematics.toSwerveModuleStates(
            discretize(
                ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, rot_speed, self._gyro.getRotation2d())
                if is_field_relative else ChassisSpeeds(x_speed, y_speed, rot_speed),
                period_seconds
            )
        )
        SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, kMaxSpeed)
        self.swerve_module_fl.set_desired_state(swerve_module_states[0])
        self.swerve_module_fr.set_desired_state(swerve_module_states[1])
        self.swerve_module_bl.set_desired_state(swerve_module_states[2])
        self.swerve_module_br.set_desired_state(swerve_module_states[3])

    def get_rotation(self):
        return self._gyro.getRotation2d()

    def get_pitch(self):
        return self._gyro.getPitch()

    def getPose(self):
        return self.swerve_estimator.getEstimatedPosition()

    def periodic(self):
        self.swerve_estimator.update(
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.get_position(), self.swerve_module_fr.get_position(),
                self.swerve_module_bl.get_position(), self.swerve_module_br.get_position()
            ),
        )

        self._field.setRobotPose(self._estimator.getEstimatedPosition())

    def initSendable(self, builder: wpiutil.SendableBuilder) -> None:
        super().initSendable(builder)
        builder.addDoubleProperty("Left motor", lambda: self.swerve_module_fl.get or -999.0, defaultSetter)
        builder.addDoubleProperty("Right Motor", lambda: self._motor_right.get() or -999.0, defaultSetter)
        builder.addDoubleProperty("Left Encoder Position", self.getLeftEncoderPosition, defaultSetter)
        builder.addDoubleProperty("Right Encoder Position", self.getRightEncoderPosition, defaultSetter)

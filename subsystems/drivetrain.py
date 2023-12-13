import math
from typing import Literal, Callable

import wpilib
import wpiutil
from wpilib import RobotBase, RobotController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModuleState,
)

import ports
from gyro import ADIS16448, ADIS16470, ADXRS, Empty, NavX
from utils.property import autoproperty
from utils.safesubsystem import SafeSubsystem
from utils.swervemodule import SwerveModule
from utils.swerveutils import *

select_gyro: Literal["navx", "adis16448", "adis16470", "adxrs", "empty"] = "adis16470"


class Drivetrain(SafeSubsystem):
    width = autoproperty(0.68)
    length = autoproperty(0.68)
    max_angular_speed = autoproperty(math.pi)

    mag_slew_rate = autoproperty(1.8)
    rotation_slew_rate = autoproperty(2.0)
    direction_slew_rate = autoproperty(1.2)

    angular_offset_fl = autoproperty(-math.pi / 2)
    angular_offset_fr = autoproperty(0.0)
    angular_offset_bl = autoproperty(math.pi)
    angular_offset_br = autoproperty(math.pi / 2)

    acceptable_wheel_rotation = autoproperty(0.45)  # Will multiply pi, is radians. Tolerance in which the wheel can be in
    wheel_flip_rotation = autoproperty(0.85)  # If wheel has this angle, wheel will lock and flip

    def __init__(self, period: float) -> None:
        super().__init__()
        self.period_seconds = period

        # Swerve Module motor positions
        self.motor_fl_loc = Translation2d(self.width / 2, self.length / 2)
        self.motor_fr_loc = Translation2d(self.width / 2, -self.length / 2)
        self.motor_bl_loc = Translation2d(-self.width / 2, self.length / 2)
        self.motor_br_loc = Translation2d(-self.width / 2, -self.length / 2)

        if RobotBase.isReal():
            self.swerve_module_fl = SwerveModule(
                ports.drivetrain_motor_driving_fl,
                ports.drivetrain_motor_turning_fl,
                self.angular_offset_fl,
            )
            wpilib.wait(4)
            self.swerve_module_fr = SwerveModule(
                ports.drivetrain_motor_driving_fr,
                ports.drivetrain_motor_turning_fr,
                self.angular_offset_fr,
            )
            wpilib.wait(4)
            self.swerve_module_bl = SwerveModule(
                ports.drivetrain_motor_driving_bl,
                ports.drivetrain_motor_turning_bl,
                self.angular_offset_bl,
            )
            wpilib.wait(4)
            self.swerve_module_br = SwerveModule(
                ports.drivetrain_motor_driving_br,
                ports.drivetrain_motor_turning_br,
                self.angular_offset_br,
            )
            wpilib.wait(4)
        else:

            self.swerve_module_fl = SwerveModule(
                ports.drivetrain_motor_driving_fl,
                ports.drivetrain_motor_turning_fl,
                self.angular_offset_fl,
            )
            self.swerve_module_fr = SwerveModule(
                ports.drivetrain_motor_driving_fr,
                ports.drivetrain_motor_turning_fr,
                self.angular_offset_fr,
            )
            self.swerve_module_bl = SwerveModule(
                ports.drivetrain_motor_driving_bl,
                ports.drivetrain_motor_turning_bl,
                self.angular_offset_bl,
            )
            self.swerve_module_br = SwerveModule(
                ports.drivetrain_motor_driving_br,
                ports.drivetrain_motor_turning_br,
                self.angular_offset_br,
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

        self.current_rotation = 0.0
        self.current_translation_dir = 0.0
        self.current_translation_mag = 0.0

        self.mag_limiter = SlewRateLimiter(self.mag_slew_rate)
        self.rot_limiter = SlewRateLimiter(self.rotation_slew_rate)

        self.prev_time = RobotController.getFPGATime() * 1e-6


        if RobotBase.isSimulation():
            self.sim_yaw = 0

    def drive(
        self,
        x_speed_input: float,
        y_speed_input: float,
        rot_speed: float,
        is_field_relative: bool,
        rate_limiter: bool = True,
    ):
        if rate_limiter:
            # Convert XY to polar for rate limiting
            input_translation_direction = math.atan2(y_speed_input, x_speed_input)
            input_translation_mag = math.sqrt(
                math.pow(x_speed_input, 2) + math.pow(y_speed_input, 2)
            )

            if self.current_translation_mag != 0.0:
                direction_slew_rate = abs(
                    self.direction_slew_rate / self.current_translation_mag
                )
            else:
                direction_slew_rate = 500  # some high number that means the slew rate is effectively instantaneous

            current_time = RobotController.getFPGATime() * 1e-6
            elapsed_time = current_time - self.prev_time
            angle_diff = angleDifference(
                input_translation_direction, self.current_translation_dir
            )

            if angle_diff < self.acceptable_wheel_rotation * math.pi:
                self.current_translation_dir = stepTowardsCircular(
                    self.current_translation_dir,
                    input_translation_direction,
                    direction_slew_rate * elapsed_time,
                )
                self.current_translation_mag = self.mag_limiter.calculate(
                    input_translation_mag
                )
            elif angle_diff > self.wheel_flip_rotation * math.pi:
                if (
                    self.current_translation_mag > 1e-4
                ):  # small number to avoid floating point errors
                    self.current_translation_mag = self.mag_limiter.calculate(0.0)
                else:
                    self.current_translation_dir = wrapAngle(
                        self.current_translation_dir + math.pi
                    )
                    self.current_translation_mag = self.mag_limiter.calculate(
                        input_translation_mag
                    )
            else:
                self.current_translation_dir = stepTowardsCircular(
                    self.current_translation_dir,
                    input_translation_direction,
                    direction_slew_rate * elapsed_time,
                )
                self.current_translation_mag = self.mag_limiter.calculate(0.0)

            self.prev_time = current_time
            x_speed = self.current_translation_mag * math.cos(
                self.current_translation_dir
            )
            y_speed = self.current_translation_mag * math.sin(
                self.current_translation_dir
            )
            self.current_rotation = self.rot_limiter.calculate(rot_speed)
        else:
            x_speed = x_speed_input
            y_speed = y_speed_input
            self.current_rotation = rot_speed

        x_speed *= self.swerve_module_fr.max_speed
        y_speed *= self.swerve_module_fr.max_speed
        rot_speed = self.current_rotation * self.max_angular_speed

        swerve_module_states = self.swervedrive_kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                x_speed, y_speed, rot_speed, self._gyro.getRotation2d()
            )
            if is_field_relative
            else ChassisSpeeds(x_speed, y_speed, rot_speed)
        )

        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerve_module_states, self.swerve_module_fr.max_speed
        )
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

    def getSwerveEstimator(self):
        return self.swerve_estimator

    def setXFormation(self):
        """
        Points all the wheels into the center to prevent movement
        """
        self.swerve_module_fl.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45))
        )
        self.swerve_module_fr.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.swerve_module_bl.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.swerve_module_br.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(45))
        )

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
        wpilib.SmartDashboard.putNumberArray(
            "SwerveStates",
            [
                self.swerve_module_fl.getState().angle.radians(),
                self.swerve_module_fl.getState().speed,
                self.swerve_module_fr.getState().angle.radians(),
                self.swerve_module_fr.getState().speed,
                self.swerve_module_bl.getState().angle.radians(),
                self.swerve_module_bl.getState().speed,
                self.swerve_module_br.getState().angle.radians(),
                self.swerve_module_br.getState().speed,
            ],
        )

        self.swerve_module_fl.simulationUpdate(self.period_seconds)
        self.swerve_module_fr.simulationUpdate(self.period_seconds)
        self.swerve_module_bl.simulationUpdate(self.period_seconds)
        self.swerve_module_br.simulationUpdate(self.period_seconds)

        self.swerve_estimator.update(
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl.getPosition(),
                self.swerve_module_fr.getPosition(),
                self.swerve_module_bl.getPosition(),
                self.swerve_module_br.getPosition(),
            ),
        )

        module_states = (
            self.swerve_module_fl.getState(),
            self.swerve_module_fr.getState(),
            self.swerve_module_bl.getState(),
            self.swerve_module_br.getState(),
        )
        chassis_speed = self.swervedrive_kinematics.toChassisSpeeds(*module_states)
        chassis_rotation_speed = chassis_speed.omega
        self.sim_yaw += chassis_rotation_speed * self.period_seconds
        self._gyro.setSimAngle(-math.degrees(self.sim_yaw))

        self._field.setRobotPose(self.swerve_estimator.getEstimatedPosition())

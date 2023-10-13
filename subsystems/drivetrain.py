from typing import Literal
from wpimath.kinematics import SwerveDrive4Odometry, SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from utils.safesubsystem import SafeSubsystem

from gyro import NavX, ADIS16448, ADIS16470, ADXRS, Empty

select_gyro: Literal["navx", "adis16448", "adis16470", "adxrs", "empty"] = "adis16470"

class Drivetrain(SafeSubsystem):
    def __init__(self) -> None:
        super().__init__()
        # Swerve Module motor positions
        self.motor_fl_loc = Translation2d(0.33, 0.33)
        self.motor_fr_loc = Translation2d(0.33, -0.33)
        self.motor_bl_loc = Translation2d(-0.33, 0.33)
        self.motor_br_loc = Translation2d(-0.33, -0.33)

        self.swerve_module_fl = SwerveModulePosition(0, Rotation2d(0))
        self.swerve_module_fr = SwerveModulePosition(0, Rotation2d(0))
        self.swerve_module_bl = SwerveModulePosition(0, Rotation2d(0))
        self.swerve_module_br = SwerveModulePosition(0, Rotation2d(0))

        # Gyro
        self._gyro = {
            "navx": NavX,
            "adis16448": ADIS16448,
            "adis16470": ADIS16470,
            "adxrs": ADXRS,
            "empty": Empty,
        }[select_gyro]()

        self.swervedrive_kinematics = SwerveDrive4Kinematics(self.motor_fl_loc, self.motor_fr_loc, self.motor_bl_loc, self.motor_br_loc)

        self.swervedrive_odometry = SwerveDrive4Odometry(
            self.swervedrive_kinematics,
            self._gyro.getRotation2d(),
            (
                self.swerve_module_fl, self.swerve_module_fr,
                self.swerve_module_bl, self.swerve_module_br
            ),
            Pose2d(0, 0, 0)
        )

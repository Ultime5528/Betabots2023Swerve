import math

import rev
import wpilib
from wpilib import Encoder, RobotBase, RobotController
from wpilib.simulation import EncoderSim, FlywheelSim
from wpimath.controller import (
    PIDController,
    ProfiledPIDController,
    SimpleMotorFeedforwardMeters,
)
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.system.plant import DCMotor, LinearSystemId
from wpimath.trajectory import TrapezoidProfile

from utils.property import autoproperty
from utils.sparkmaxsim import SparkMaxSim
from utils.sparkmaxutils import configureLeader

module_max_angular_velocity = math.pi / 2  # 1/2 radian per second
module_max_angular_acceleration = 2 * math.pi  # radians per second squared
encoder_resolution = 4096
# TODO Add robot specific parameters
wheel_radius = 0.0381  # meters

turn_motor_gear_ratio = 12.8  # //12 to 1
turn_encoder_conversion_factor = 2 * math.pi / encoder_resolution
turn_encoder_distance_per_pulse = (2 * math.pi) / (
        encoder_resolution * turn_motor_gear_ratio
)

drive_motor_gear_ratio = 8.16  # //6.89 to 1
drive_encoder_distance_per_pulse = (2 * math.pi) / (
        encoder_resolution * drive_motor_gear_ratio
)
drive_encoder_conversion_factor = 2 * math.pi * wheel_radius / encoder_resolution


class SwerveModule:
    max_speed = autoproperty(3)

    def __init__(
        self,
        drive_motor_port,
        turning_motor_port
    ):
        # TODO Changer la convention "m_..." pour seulement "_..."
        self.m_drive_motor = rev.CANSparkMax(
            drive_motor_port, rev.CANSparkMax.MotorType.kBrushless
        )
        configureLeader(self.m_drive_motor, "brake", False)

        self.m_turning_motor = rev.CANSparkMax(
            turning_motor_port, rev.CANSparkMax.MotorType.kBrushless
        )
        configureLeader(self.m_turning_motor, "brake", False)

        self.m_drive_encoder = self.m_drive_motor.getEncoder()
        self.m_turning_encoder = self.m_turning_motor.getEncoder()
        self.m_drive_encoder.setPositionConversionFactor(
            drive_encoder_conversion_factor
        )
        self.m_turning_encoder.setPositionConversionFactor(
            turn_encoder_conversion_factor
        )

        self.m_turningPIDController = ProfiledPIDController(
            1,
            0,
            0,
            TrapezoidProfile.Constraints(
                module_max_angular_velocity, module_max_angular_acceleration
            ),
        )
        self.m_turningPIDController.enableContinuousInput(-math.pi, math.pi)

        # TODO Find robot specific parameters
        self.m_drivePIDController = PIDController(1, 0, 0)
        self.m_driveFeedforward = SimpleMotorFeedforwardMeters(1, 3)
        self.m_turnFeedforward = SimpleMotorFeedforwardMeters(1, 0.5)

        if RobotBase.isSimulation():
            # Simulation things
            self.sim_drive_encoder = SparkMaxSim(self.m_drive_motor)
            self.sim_turn_encoder = SparkMaxSim(self.m_turning_motor)

            self.drive_output: float = 0.0
            self.turn_output: float = 0.0
            self.sim_turn_encoder_distance: float = 0.0
            self.sim_drive_encoder_distance: float = 0.0

            # Flywheels allow simulation of a more physically realistic rendering of swerve module properties
            # Magical values for sim pulled from :
            # https://github.com/4201VitruvianBots/2021SwerveSim/blob/main/Swerve2021/src/main/java/frc/robot/subsystems/SwerveModule.java
            self.sim_turn_motor = FlywheelSim(
                LinearSystemId.identifyVelocitySystemMeters(0.16, 0.0348),
                DCMotor.NEO550(1),
                turn_motor_gear_ratio,
            )
            self.sim_drive_motor = FlywheelSim(
                LinearSystemId.identifyVelocitySystemMeters(2, 1.24),
                DCMotor.NEO550(1),
                drive_motor_gear_ratio,
            )

    def getVelocity(self) -> float:
        return self.m_drive_encoder.getVelocity()

    def getTurningRadians(self) -> float:
        """
        Returns radians
        """
        return self.m_turning_encoder.getPosition()

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.getVelocity(), Rotation2d(self.getTurningRadians())
        )

    def getModuleEncoderPosition(self) -> float:
        return self.m_drive_encoder.getPosition()

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self.getModuleEncoderPosition(), Rotation2d(self.getTurningRadians())
        )

    def setDesiredState(self, desired_state):
        # Works for both sim and real because all functions related to getting values from encoders take care of
        # returning the correct value internally
        encoder_rotation = Rotation2d(self.getTurningRadians())
        state = SwerveModuleState.optimize(desired_state, encoder_rotation)
        state.speed *= (state.angle - encoder_rotation).cos()
        self.drive_output = self.m_drivePIDController.calculate(
            self.getVelocity(), state.speed
        )
        drive_feedforward = self.m_driveFeedforward.calculate(state.speed)
        self.turn_output = self.m_turningPIDController.calculate(
            self.getTurningRadians(), state.angle.radians()
        )
        turn_feedforward = self.m_turnFeedforward.calculate(
            self.m_turningPIDController.getSetpoint().velocity
        )
        self.m_drive_motor.setVoltage(self.drive_output + drive_feedforward)
        self.m_turning_motor.setVoltage(self.turn_output + turn_feedforward)

    def simulationUpdate(self, period: float):
        self.sim_turn_motor.setInputVoltage(
            self.turn_output
            / module_max_angular_acceleration
            * RobotController.getBatteryVoltage()
        )
        self.sim_drive_motor.setInputVoltage(
            self.drive_output / self.max_speed * RobotController.getBatteryVoltage()
        )

        self.sim_drive_motor.update(period)
        self.sim_turn_motor.update(period)

        self.sim_turn_encoder_distance += (
                self.sim_turn_motor.getAngularVelocity() * period
        )
        self.sim_turn_encoder.setPosition(self.sim_turn_encoder_distance)
        self.sim_turn_encoder.setVelocity(self.sim_turn_motor.getAngularVelocity())

        self.sim_drive_encoder_distance += (
                self.sim_drive_motor.getAngularVelocity() * period
        )
        self.sim_drive_encoder.setPosition(self.sim_drive_encoder_distance)
        self.sim_drive_encoder.setVelocity(self.sim_drive_motor.getAngularVelocity())

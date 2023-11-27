import math

import rev
import wpilib
from rev._rev import SparkMaxAbsoluteEncoder, CANSparkMax, SparkMaxRelativeEncoder
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

# 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
drive_motor_pinion_teeth = 14
drive_motor_gear_ratio = (45.0 * 22) / (drive_motor_pinion_teeth * 15);
drive_encoder_position_conversion_factor = math.pi * wheel_radius / drive_motor_gear_ratio  # meters
drive_encoder_velocity_conversion_factor = drive_encoder_position_conversion_factor / 60  # meters per second
drive_motor_free_rps = 5676 / 60  # Neo motor max free RPM into rotations per second
drive_wheel_free_rps = (drive_motor_free_rps * (2 * math.pi))
driving_PID_feedforward = 1 / drive_wheel_free_rps

turning_encoder_position_conversion_factor = math.pi * 2  # radians
turning_encoder_velocity_conversion_factor = math.pi * 2 / 60  # radians per second

turning_encoder_position_PID_min_input = 0
turning_encoder_position_PID_max_input = turning_encoder_position_conversion_factor


class SwerveModule:
    max_speed = autoproperty(0.25)

    driving_PID_P = autoproperty(0.04)
    driving_PID_I = autoproperty(0.0)
    driving_PID_D = autoproperty(0.0)
    driving_PID_feedforward = autoproperty(1 / drive_wheel_free_rps)

    turning_PID_P = autoproperty(1.0)
    turning_PID_I = autoproperty(0.0)
    turning_PID_D = autoproperty(0.0)
    turning_PID_feedforward = autoproperty(0.0)

    driving_PID_output_min = autoproperty(-1.0)
    driving_PID_output_max = autoproperty(1.0)
    turning_PID_output_min = autoproperty(-1.0)
    turning_PID_output_max = autoproperty(1.0)

    def __init__(
            self,
            drive_motor_port,
            turning_motor_port,
            chassis_angular_offset: float,
            turning_motor_inverted: bool = False,
    ):
        # TODO Changer la convention "m_..." pour seulement "_..."
        self.m_drive_motor = CANSparkMax(
            drive_motor_port, CANSparkMax.MotorType.kBrushless
        )
        configureLeader(self.m_drive_motor, "brake", False)

        self.m_turning_motor = CANSparkMax(
            turning_motor_port, CANSparkMax.MotorType.kBrushless
        )
        configureLeader(self.m_turning_motor, "brake", turning_motor_inverted)

        # Restore SparkMax controllers to factory defaults
        self.m_turning_motor.restoreFactoryDefaults()
        self.m_drive_motor.restoreFactoryDefaults()

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        self.m_drive_encoder: SparkMaxRelativeEncoder = self.m_drive_motor.getEncoder()
        self.m_turning_encoder = self.m_turning_motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.m_drive_PIDController = self.m_drive_motor.getPIDController()
        self.m_turning_PIDController = self.m_turning_motor.getPIDController()
        self.m_drive_PIDController.setFeedbackDevice(self.m_drive_encoder)
        self.m_turning_PIDController.setFeedbackDevice(self.m_turning_encoder)

        self.m_drive_encoder.setPositionConversionFactor(drive_encoder_position_conversion_factor)
        self.m_drive_encoder.setVelocityConversionFactor(drive_encoder_velocity_conversion_factor)
        self.m_turning_encoder.setPositionConversionFactor(turning_encoder_position_conversion_factor)
        self.m_turning_encoder.setVelocityConversionFactor(turning_encoder_velocity_conversion_factor)

        self.m_turning_encoder.setInverted(True)

        self.m_turning_PIDController.setPositionPIDWrappingEnabled(True)
        self.m_turning_PIDController.setPositionPIDWrappingMinInput(turning_encoder_position_PID_min_input)
        self.m_turning_PIDController.setPositionPIDWrappingMaxInput(turning_encoder_position_PID_max_input)

        self.m_drive_PIDController.setP(self.driving_PID_P)
        self.m_drive_PIDController.setI(self.driving_PID_I)
        self.m_drive_PIDController.setD(self.driving_PID_D)
        self.m_drive_PIDController.setFF(self.driving_PID_feedforward)
        self.m_drive_PIDController.setOutputRange(self.driving_PID_output_min,
                                                  self.driving_PID_output_max)

        self.m_turning_PIDController.setP(self.turning_PID_P)
        self.m_turning_PIDController.setI(self.turning_PID_I)
        self.m_turning_PIDController.setD(self.turning_PID_D)
        self.m_turning_PIDController.setFF(self.turning_PID_feedforward)
        self.m_turning_PIDController.setOutputRange(self.turning_PID_output_min,
                                                    self.turning_PID_output_max)

        self.m_drive_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.m_turning_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        # Save the SPARK MAX configurations. If a SPARK MAX browns out during
        # operation, it will maintain the above configurations.
        self.m_drive_motor.burnFlash()
        self.m_turning_motor.burnFlash()

        self.m_desired_state = SwerveModuleState(0.0, Rotation2d())
        self.m_chassis_angular_offset = chassis_angular_offset
        self.m_desired_state.angle = Rotation2d(self.m_turning_encoder.getPosition())
        self.m_drive_encoder.setPosition(0)

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

    def setDesiredState(self, desired_state: SwerveModuleState):
        corrected_desired_state = SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle + Rotation2d(self.m_chassis_angular_offset)

        optimized_desired_state = SwerveModuleState.optimize(corrected_desired_state,
                                                             Rotation2d(self.m_turning_encoder.getPosition()))
        self.m_drive_PIDController.setReference(optimized_desired_state.speed, CANSparkMax.ControlType.kVelocity)
        self.m_turning_PIDController.setReference(optimized_desired_state.angle.radians(),
                                                  CANSparkMax.ControlType.kPosition)

        self.m_desired_state = desired_state

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

import math
import rev
import wpilib
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpilib import Encoder
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.system.plant import DCMotor

from wpimath.trajectory import TrapezoidProfile

from utils.sparkmaxsim import SparkMaxSim

k_wheel_radius = 0.0508
k_encoder_resolution = 4096
k_module_max_angular_velocity = math.pi / 2  # 1/2 radian per second
k_module_max_angular_acceleration = 2 * math.pi  # radians per second squared
k_max_speed = 3  # 3 meters per second
k_drive_encoder_conversion_factor = 2 * math.pi * k_wheel_radius / k_encoder_resolution
k_turning_encoder_conversion_factor = 2 * math.pi / k_encoder_resolution

class SwerveModule:
    def __init__(self, drive_motor_port, turning_motor_port, drive_encoder_channel_a, drive_encoder_channel_b,
                 turning_encoder_channel_a, turning_encoder_channel_b):
        self.m_drive_motor = rev.CANSparkMax(drive_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.m_turning_motor = rev.CANSparkMax(turning_motor_port, rev.CANSparkMax.MotorType.kBrushless)

        self.m_drive_encoder = self.m_drive_motor.getEncoder()
        self.m_turning_encoder = self.m_turning_motor.getEncoder()
        self.m_drive_encoder.setPositionConversionFactor(k_drive_encoder_conversion_factor)
        self.m_turning_encoder.setPositionConversionFactor(k_turning_encoder_conversion_factor)

        self.m_turningPIDController = ProfiledPIDController(
            1, 0, 0, TrapezoidProfile.Constraints(k_module_max_angular_velocity, k_module_max_angular_acceleration))
        self.m_turningPIDController.enableContinuousInput(-math.pi, math.pi)

        # TODO Find robot specific parameters
        self.m_drivePIDController = PIDController(1, 0, 0)
        self.m_driveFeedforward = SimpleMotorFeedforwardMeters(1, 3)
        self.m_turnFeedforward = SimpleMotorFeedforwardMeters(1, 0.5)

        if wpilib.RobotBase.isSimulation():
            self.m_sim_drive_motor = SparkMaxSim(self.m_drive_motor)
            self.m_sim_turning_motor = SparkMaxSim(self.m_turning_motor)

    def get_state(self):
        return SwerveModuleState(self.m_drive_encoder.getVelocity(), Rotation2d(self.m_turning_encoder.getPosition())) if wpilib.RobotBase.isReal() else SwerveModuleState(self.m_sim_drive_motor.getVelocity(), Rotation2d(self.m_sim_turning_motor.getPosition() * k_turning_encoder_conversion_factor))

    def get_position(self):
        return SwerveModulePosition(self.m_drive_encoder.getPosition(), Rotation2d(self.m_turning_encoder.getPosition())) if wpilib.RobotBase.isReal() else SwerveModulePosition(self.m_sim_drive_motor.getPosition(), Rotation2d(self.m_sim_turning_motor.getPosition() * k_turning_encoder_conversion_factor))

    def set_desired_state(self, desiredState):
        if wpilib.RobotBase.isReal():
            encoder_rotation = Rotation2d(self.m_turning_encoder.getPosition())
            state = SwerveModuleState.optimize(desiredState, encoder_rotation)
            state.speed *= (state.angle-encoder_rotation).cos()
            drive_output = self.m_drivePIDController.calculate(self.m_drive_encoder.getVelocity(), state.speed)
            drive_feedforward = self.m_driveFeedforward.calculate(state.speed)
            turn_output = self.m_turningPIDController.calculate(self.m_turning_encoder.getPosition(),
                                                                state.angle.radians())
            turn_feedforward = self.m_turnFeedforward.calculate(self.m_turningPIDController.getSetpoint().velocity)
            self.m_drive_motor.setVoltage(drive_output + drive_feedforward)
            self.m_turning_motor.setVoltage(turn_output + turn_feedforward)
        else:
            encoder_rotation = Rotation2d(self.m_sim_turning_motor.getPosition() * k_turning_encoder_conversion_factor)
            state = SwerveModuleState.optimize(desiredState, encoder_rotation)
            state.speed *= (state.angle - encoder_rotation).cos()
            drive_output = self.m_drivePIDController.calculate(self.m_drive_encoder.getVelocity(), state.speed)
            drive_feedforward = self.m_driveFeedforward.calculate(state.speed)
            turn_output = self.m_turningPIDController.calculate(self.m_turning_encoder.getPosition(),
                                                                state.angle.radians())
            turn_feedforward = self.m_turnFeedforward.calculate(self.m_turningPIDController.getSetpoint().velocity)
            self.m_sim_drive_motor.setVoltage(drive_output + drive_feedforward)
            self.m_sim_turning_motor.setVoltage(turn_output + turn_feedforward)
            test = self.m_sim_turning_motor.getVoltage()
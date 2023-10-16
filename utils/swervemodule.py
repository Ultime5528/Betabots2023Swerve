import math
import rev
import wpilib
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpilib import Encoder, RobotBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib.simulation import FlywheelSim, EncoderSim

from wpimath.trajectory import TrapezoidProfile

from utils.sparkmaxsim import SparkMaxSim

k_wheel_radius = 0.0508
k_encoder_resolution = 4096
k_drive_motor_gear_ratio = 8.16  # //6.89 to 1
k_turning_motor_gear_ratio = 12.8  # //12 to 1
k_module_max_angular_velocity = math.pi / 2  # 1/2 radian per second
k_module_max_angular_acceleration = 2 * math.pi  # radians per second squared
k_max_speed = 3  # 3 meters per second
k_drive_encoder_distance_per_pulse = (2 * math.pi) / (k_encoder_resolution * k_drive_motor_gear_ratio)
k_turning_encoder_distance_per_pulse = (2 * math.pi) / (k_encoder_resolution * k_turning_motor_gear_ratio)
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

        if RobotBase.isSimulation():
            # Simulation things
            self.sim_drive_encoder = EncoderSim(Encoder(drive_encoder_channel_a, drive_encoder_channel_b))
            self.sim_drive_encoder.setDistancePerPulse(k_drive_encoder_distance_per_pulse)
            self.sim_turning_encoder = EncoderSim(Encoder(turning_encoder_channel_a, turning_encoder_channel_b))
            self.sim_turning_encoder.setDistancePerPulse(k_turning_encoder_distance_per_pulse)
            # TODO add flywheel and hook them up
            self.sim_turning_motor = FlywheelSim(
                DCMotor.NEO(1),
                k_turning_motor_gear_ratio,

            )

    def get_velocity(self) -> float:
        if RobotBase.isReal():
            return self.m_drive_encoder.getVelocity()
        else:
            return self.sim_drive_encoder.getRate()

    def get_turning_angle(self) -> float:
        """
        Returns radians
        """
        if RobotBase.isReal():
            return self.m_turning_encoder.getPosition()
        else:
            self.sim_turning_encoder.getDistance()

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_velocity(), Rotation2d(self.get_turning_angle()))
    
    def get_module_position(self) -> float:
        if RobotBase.isReal():
            return self.m_drive_encoder.getPosition()
        else:
            return self.sim_drive_encoder.getDistance()

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_position(),
                                    Rotation2d(self.get_turning_angle()))

    def set_desired_state(self, desiredState):
        # Works for both sim and real because all functions related to getting values from encoders take care of
        # returning the correct value internally
        encoder_rotation = Rotation2d(self.get_turning_angle())
        state = SwerveModuleState.optimize(desiredState, encoder_rotation)
        state.speed *= (state.angle - encoder_rotation).cos()
        drive_output = self.m_drivePIDController.calculate(self.get_velocity(), state.speed)
        drive_feedforward = self.m_driveFeedforward.calculate(state.speed)
        turn_output = self.m_turningPIDController.calculate(self.get_turning_angle(),
                                                            state.angle.radians())
        turn_feedforward = self.m_turnFeedforward.calculate(self.m_turningPIDController.getSetpoint().velocity)
        self.m_drive_motor.setVoltage(drive_output + drive_feedforward)
        self.m_turning_motor.setVoltage(turn_output + turn_feedforward)

    # TODO add things to do after each simulation step in drivetrain.py
    def simulation_update(self):
        pass

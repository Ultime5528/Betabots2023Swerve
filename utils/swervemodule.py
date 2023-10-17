import math
import rev
import wpilib
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpilib import Encoder, RobotBase, RobotController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.system.plant import DCMotor, LinearSystemId
from wpilib.simulation import FlywheelSim, EncoderSim

from wpimath.trajectory import TrapezoidProfile

from utils.sparkmaxsim import SparkMaxSim

k_module_max_angular_velocity = math.pi / 2  # 1/2 radian per second
k_module_max_angular_acceleration = 2 * math.pi  # radians per second squared
k_max_speed = 3  # 3 meters per second
k_encoder_resolution = 4096
# TODO Add robot specific parameters
k_wheel_radius = 0.0381  # meters
k_wheel_moment_of_inertia = 0.0001146  # I = 0.079kg * 0.0381m ^ 2 = 0.0001146 kg*m^2

k_turn_motor_gear_ratio = 12.8  # //12 to 1
k_turn_encoder_conversion_factor = 2 * math.pi / k_encoder_resolution
k_turn_encoder_distance_per_pulse = (2 * math.pi) / (k_encoder_resolution * k_turn_motor_gear_ratio)

k_drive_motor_gear_ratio = 8.16  # //6.89 to 1
k_drive_encoder_distance_per_pulse = (2 * math.pi) / (k_encoder_resolution * k_drive_motor_gear_ratio)
k_drive_encoder_conversion_factor = 2 * math.pi * k_wheel_radius / k_encoder_resolution


class SwerveModule:
    def __init__(self, drive_motor_port, turning_motor_port, drive_encoder_channel_a, drive_encoder_channel_b,
                 turning_encoder_channel_a, turning_encoder_channel_b):
        self.m_drive_motor = rev.CANSparkMax(drive_motor_port, rev.CANSparkMax.MotorType.kBrushless)
        self.m_turning_motor = rev.CANSparkMax(turning_motor_port, rev.CANSparkMax.MotorType.kBrushless)

        self.m_drive_encoder = self.m_drive_motor.getEncoder()
        self.m_turning_encoder = self.m_turning_motor.getEncoder()
        self.m_drive_encoder.setPositionConversionFactor(k_drive_encoder_conversion_factor)
        self.m_turning_encoder.setPositionConversionFactor(k_turn_encoder_conversion_factor)

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
            self.sim_turn_encoder = EncoderSim(Encoder(turning_encoder_channel_a, turning_encoder_channel_b))
            self.sim_turn_encoder.setDistancePerPulse(k_turn_encoder_distance_per_pulse)
            self.drive_output: float = 0
            self.turn_output: float = 0
            self.sim_turn_encoder_distance: float = 0
            self.sim_drive_encoder_distance: float = 0
            # TODO Add robot specific parameters
            self.sim_turn_motor = FlywheelSim(
                LinearSystemId.identifyVelocitySystemMeters(0.16, 0.0348),
                # Magical values for sim pulled from : https://github.com/4201VitruvianBots/2021SwerveSim/blob/main/Swerve2021/src/main/java/frc/robot/subsystems/SwerveModule.java
                DCMotor.NEO550(1),
                k_turn_motor_gear_ratio
            )
            self.sim_drive_motor = FlywheelSim(
                LinearSystemId.identifyVelocitySystemMeters(2, 1.24),
                # Magical values for sim pulled from : https://github.com/4201VitruvianBots/2021SwerveSim/blob/main/Swerve2021/src/main/java/frc/robot/subsystems/SwerveModule.java
                DCMotor.NEO550(1),
                k_drive_motor_gear_ratio
            )

    def get_velocity(self) -> float:
        if RobotBase.isReal():
            return self.m_drive_encoder.getVelocity()
        else:
            return self.sim_drive_encoder.getRate()

    def get_turning_radians(self) -> float:
        """
        Returns radians
        """
        if RobotBase.isReal():
            return self.m_turning_encoder.getPosition()
        else:
            return self.sim_turn_encoder.getDistance()

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_velocity(), Rotation2d(self.get_turning_radians()))

    def get_module_encoder_position(self) -> float:
        if RobotBase.isReal():
            return self.m_drive_encoder.getPosition()
        else:
            return self.sim_drive_encoder.getDistance()

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_module_encoder_position(),
                                    Rotation2d(self.get_turning_radians()))

    def set_desired_state(self, desired_state):
        # Works for both sim and real because all functions related to getting values from encoders take care of
        # returning the correct value internally
        encoder_rotation = Rotation2d(self.get_turning_radians())
        state = SwerveModuleState.optimize(desired_state, encoder_rotation)
        state.speed *= (state.angle - encoder_rotation).cos()
        self.drive_output = self.m_drivePIDController.calculate(self.get_velocity(), state.speed)
        drive_feedforward = self.m_driveFeedforward.calculate(state.speed)
        self.turn_output = self.m_turningPIDController.calculate(self.get_turning_radians(),
                                                                 state.angle.radians())
        turn_feedforward = self.m_turnFeedforward.calculate(self.m_turningPIDController.getSetpoint().velocity)
        self.m_drive_motor.setVoltage(self.drive_output + drive_feedforward)
        self.m_turning_motor.setVoltage(self.turn_output + turn_feedforward)

    def simulation_update(self):
        print("Turn Output: ", round(self.turn_output, 2), "Drive Output: ", round(self.drive_output, 2))
        self.sim_turn_motor.setInputVoltage(self.turn_output / k_module_max_angular_acceleration * RobotController.getBatteryVoltage())
        self.sim_drive_motor.setInputVoltage(self.drive_output / k_max_speed * RobotController.getBatteryVoltage())

        self.sim_drive_motor.update(0.02)
        self.sim_turn_motor.update(0.02)

        self.sim_turn_encoder_distance += self.sim_turn_motor.getAngularVelocity() * 0.02
        self.sim_turn_encoder.setDistance(self.sim_turn_encoder_distance)
        self.sim_turn_encoder.setRate(self.sim_turn_motor.getAngularVelocity())

        self.sim_drive_encoder_distance += self.sim_drive_motor.getAngularVelocity() * 0.02
        self.sim_drive_encoder.setDistance(self.sim_drive_encoder_distance)
        self.sim_drive_encoder.setRate(self.sim_drive_motor.getAngularVelocity())

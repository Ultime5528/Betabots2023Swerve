import math
import wpilib
from wpimath.controller import PIDController
from wpimath.controller import ProfiledPIDController
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpilib import Encoder
from wpimath.geometry import Rotation2d
from wpilib import PWM
from wpimath.kinematics import SwerveModuleState
from wpimath.kinematics import SwerveModulePosition
from wpimath.system.plant import DCMotor

from wpimath.trajectory import TrapezoidProfile

from subsystems.drivetrain import Drivetrain

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed
kModuleMaxAngularAcceleration = 2 * math.pi  # radians per second squared


class SwerveModule:
    def __init__(self, driveMotorChannel, turningMotorChannel, driveEncoderChannelA, driveEncoderChannelB,
                 turningEncoderChannelA, turningEncoderChannelB):
        self.m_driveMotor = DCMotor.NEO(driveMotorChannel)
        self.m_turningMotor = DCMotor.NEO(turningMotorChannel)

        self.m_driveEncoder = Encoder(driveEncoderChannelA, driveEncoderChannelB)
        self.m_turningEncoder = Encoder(turningEncoderChannelA, turningEncoderChannelB)

        self.m_driveEncoder.setDistancePerPulse(2 * math.pi * kWheelRadius / kEncoderResolution)
        self.m_turningEncoder.setDistancePerPulse(2 * math.pi / kEncoderResolution)

        self.m_turningPIDController = ProfiledPIDController(
            1, 0, 0, TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration))
        self.m_turningPIDController.enableContinuousInput(-math.pi, math.pi)

        # TODO Find robot specific parameters
        self.m_drivePIDController = PIDController(1, 0, 0)
        self.m_driveFeedforward = SimpleMotorFeedforwardMeters(1, 3)
        self.m_turnFeedforward = SimpleMotorFeedforwardMeters(1, 0.5)

    def get_state(self):
        return SwerveModuleState(self.m_driveEncoder.getRate(), Rotation2d(self.m_turningEncoder.getDistance()))

    def get_position(self):
        return SwerveModulePosition(self.m_driveEncoder.getDistance(), Rotation2d(self.m_turningEncoder.getDistance()))

    def set_desired_state(self, desiredState):
        encoder_rotation = Rotation2d(self.m_turningEncoder.getDistance())
        state = SwerveModuleState.optimize(desiredState, encoder_rotation)
        state.speed *= state.angle.minus(encoder_rotation).cos()
        drive_output = self.m_drivePIDController.calculate(self.m_driveEncoder.getRate(), state.speed)
        drive_feedforward = self.m_driveFeedforward.calculate(state.speed)
        turn_output = self.m_turningPIDController.calculate(self.m_turningEncoder.getDistance(),
                                                            state.angle.getRadians())
        turn_feedforward = self.m_turnFeedforward.calculate(self.m_turningPIDController.getSetpoint().velocity)
        self.m_driveMotor.setVoltage(drive_output + drive_feedforward)
        self.m_turningMotor.setVoltage(turn_output + turn_feedforward)

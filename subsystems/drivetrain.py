from typing import Literal

import rev
import wpilib
import wpilib.drive
import wpiutil
from photonvision import PhotonCamera, SimVisionSystem, SimVisionTarget, RobotPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField
from wpilib import DriverStation
from wpilib import RobotBase, RobotController
from wpilib.simulation import DifferentialDrivetrainSim
from wpimath.estimator import DifferentialDrivePoseEstimator
from wpimath.geometry import Pose2d, Rotation3d, Translation3d, Transform3d
from wpimath.kinematics import DifferentialDriveKinematics
from wpimath.system import LinearSystemId
from wpimath.system.plant import DCMotor

import ports
from gyro import NavX, ADIS16448, ADIS16470, ADXRS, Empty
from utils.property import autoproperty, defaultSetter
from utils.safesubsystem import SafeSubsystem
from utils.sparkmaxsim import SparkMaxSim
from utils.sparkmaxutils import configureFollower, configureLeader
from utils.vision import Vision

select_gyro: Literal["navx", "adis16448", "adis16470", "adxrs", "empty"] = "adis16470"
april_tag_field = loadAprilTagLayoutField(AprilTagField.k2023ChargedUp)
cam_to_robot = Transform3d(Translation3d(-0.375, 0.0, -0.165), Rotation3d(0, 0, 0))


class Drivetrain(SafeSubsystem):
    encoder_conversion_factor = autoproperty(0.056)

    def __init__(self) -> None:
        super().__init__()
        self.vision = Vision()

        # Motors
        self._motor_left = rev.CANSparkMax(ports.drivetrain_motor_front_left, rev.CANSparkMax.MotorType.kBrushless)
        configureLeader(self._motor_left, "brake")

        self._motor_left_follower = rev.CANSparkMax(ports.drivetrain_motor_rear_left,
                                                    rev.CANSparkMax.MotorType.kBrushless)
        configureFollower(self._motor_left_follower, self._motor_left, "brake")

        self._motor_right = rev.CANSparkMax(ports.drivetrain_motor_front_right,
                                            rev.CANSparkMax.MotorType.kBrushless)
        configureLeader(self._motor_right, "brake", True)

        self._motor_right_follower = rev.CANSparkMax(ports.drivetrain_motor_rear_right,
                                                     rev.CANSparkMax.MotorType.kBrushless)
        configureFollower(self._motor_right_follower, self._motor_right, "brake")

        self._drive = wpilib.drive.DifferentialDrive(self._motor_left, self._motor_right)
        self.addChild("DifferentialDrive", self._drive)

        # Photon Vision
        self.latest = None

        # Encoders
        self._encoder_left = self._motor_left.getEncoder()
        self._encoder_right = self._motor_right.getEncoder()
        self._left_encoder_offset = self._encoder_left.getPosition()
        self._right_encoder_offset = self._encoder_right.getPosition()

        # Gyro
        self._gyro = {
            "navx": NavX,
            "adis16448": ADIS16448,
            "adis16470": ADIS16470,
            "adxrs": ADXRS,
            "empty": Empty,
        }[select_gyro]()

        # Odometry
        self._kinematics = DifferentialDriveKinematics(trackWidth=0.48)
        self._estimator = DifferentialDrivePoseEstimator(self._kinematics, self._gyro.getRotation2d(), 0, 0,
                                                         Pose2d(0, 0, 0), (0.02, 0.02, 0.01), (0.01, 0.01, 0.01))

        self._field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self._field)

        self.alliance = DriverStation.getAlliance()

        self.addChild("Gyro", self._gyro)

        if RobotBase.isReal():
            self.cam = PhotonCamera("mainCamera")
            PhotonCamera.setVersionCheckEnabled(False)
        else:  # sim
            self._motor_left_sim = SparkMaxSim(self._motor_left)
            self._motor_right_sim = SparkMaxSim(self._motor_right)
            self._system = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 5, 0.3)
            self._drive_sim = DifferentialDrivetrainSim(self._system, 0.64, DCMotor.NEO(4), 1.5, 0.08, [
                0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005])

            # Cam sim
            cam_diag_fov = 75.0
            max_led_range = 20
            cam_resolution_width = 320
            cam_resolution_height = 240
            min_target_area = 10
            self.sim_vision = SimVisionSystem("cam", cam_diag_fov, cam_to_robot, max_led_range,
                                              cam_resolution_width, cam_resolution_height, min_target_area)
            for i in range(1, 9):
                self.sim_vision.addSimVisionTarget(SimVisionTarget(april_tag_field.getTagPose(i), 8, 8, i))
            self.cam = self.sim_vision.cam

        self.cam_estimator = RobotPoseEstimator(april_tag_field, PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT, [(self.cam, cam_to_robot.inverse())])
        self.use_vision = True

    def arcadeDrive(self, forward: float, rotation: float) -> None:
        self._drive.arcadeDrive(forward, rotation, False)

    def tankDrive(self, left: float, right: float) -> None:
        self._drive.tankDrive(left, right, False)

    def simulationPeriodic(self):
        self._drive_sim.setInputs(
            self._motor_left.get() * RobotController.getInputVoltage(),
            self._motor_right.get() * RobotController.getInputVoltage())
        self._drive_sim.update(0.02)
        self._motor_left_sim.setPosition(self._drive_sim.getLeftPosition() / self.encoder_conversion_factor + self._left_encoder_offset)
        self._motor_left_sim.setVelocity(self._drive_sim.getLeftVelocity())
        self._motor_right_sim.setPosition(self._drive_sim.getRightPosition() / self.encoder_conversion_factor + self._right_encoder_offset)
        self._motor_right_sim.setVelocity(self._drive_sim.getRightVelocity())
        self._gyro.setSimAngle(self._drive_sim.getHeading().degrees())
        self.sim_vision.processFrame(self._drive_sim.getPose())

    def getRotation(self):
        return self._gyro.getRotation2d()

    def getPitch(self):
        return self._gyro.getPitch()

    def getLeftEncoderPosition(self):
        return (self._encoder_left.getPosition() - self._left_encoder_offset) * self.encoder_conversion_factor

    def getRightEncoderPosition(self):
        return (self._encoder_right.getPosition() - self._right_encoder_offset) * self.encoder_conversion_factor

    def getAverageEncoderPosition(self):
        return (self.getLeftEncoderPosition() + self.getRightEncoderPosition()) / 2

    def getPose(self):
        return self._estimator.getEstimatedPosition()

    def getField(self):
        return self._field

    def periodic(self):
        self._estimator.update(self._gyro.getRotation2d(), self.getLeftEncoderPosition(),
                               self.getRightEncoderPosition())

        self.latest = self.cam.getLatestResult()
        if self.use_vision and self.latest.hasTargets():
            # img_capture_time = self.latest.getTimestamp()
            # best = self.latest.getBestTarget()
            # cam_to_target = best.getBestCameraToTarget()
            # best.getAlternateCameraToTarget()
            pose, lag = self.cam_estimator.update()
            pose = self.vision.update(pose.toPose2d(), self.latest.getBestTarget().getPoseAmbiguity())
            if pose:
                self._estimator.addVisionMeasurement(pose, lag)
            # best.
            # target_to_cam = cam_to_target.inverse()
            # target_on_field = april_tag_field.getTagPose(self.latest.getBestTarget().getFiducialId())
            # if target_on_field is not None:
            #     camera_on_field = target_on_field.transformBy(target_to_cam)
            #     robot_on_field = camera_on_field.transformBy(cam_to_robot).toPose2d()
            #     robot_on_field = self.vision.update(robot_on_field, best.getPoseAmbiguity())
            #     if robot_on_field:
            #         self._estimator.addVisionMeasurement(robot_on_field, img_capture_time)

        self._field.setRobotPose(self._estimator.getEstimatedPosition())

    def initSendable(self, builder: wpiutil.SendableBuilder) -> None:
        super().initSendable(builder)
        builder.addDoubleProperty("Left motor", lambda: self._motor_left.get() or -999.0, defaultSetter)
        builder.addDoubleProperty("Right Motor", lambda: self._motor_right.get() or -999.0, defaultSetter)
        builder.addDoubleProperty("Left Encoder Position", self.getLeftEncoderPosition, defaultSetter)
        builder.addDoubleProperty("Right Encoder Position", self.getRightEncoderPosition, defaultSetter)


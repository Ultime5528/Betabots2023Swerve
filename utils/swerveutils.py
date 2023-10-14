from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds


def discretize(chassis_speeds: ChassisSpeeds, dtSeconds: float):
    vx, vy, omega = chassis_speeds.vx, chassis_speeds.vy, chassis_speeds.omega
    desired_delta_pose = Pose2d(
        vx * dtSeconds,
        vy * dtSeconds,
        Rotation2d(omega * dtSeconds)
    )
    _twist = Pose2d().log(desired_delta_pose)
    return ChassisSpeeds(_twist.dx / dtSeconds, _twist.dy / dtSeconds, _twist.dtheta / dtSeconds)

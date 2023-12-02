from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds
import math


def wrapAngle(angle):
    two_pi = 2 * math.pi

    if angle == two_pi:
        return 0.0
    elif angle > two_pi:
        return angle - two_pi * math.floor(angle / two_pi)
    elif angle < 0.0:
        return angle + two_pi * (math.floor(-angle / two_pi) + 1)
    else:
        return angle


def stepTowardsCircular(current, target, step_size):
    current = wrapAngle(current)
    target = wrapAngle(target)

    step_direction = math.copysign(1, target - current)
    difference = abs(current - target)

    if difference <= step_size:
        return target
    elif difference > math.pi:
        # Handle the special case where you can reach the target in one step while also wrapping
        if current + 2 * math.pi - target < step_size or target + 2 * math.pi - current < step_size:
            return target
        else:
            return wrapAngle(current - step_direction * step_size)
    else:
        return current + step_direction * step_size


def angleDifference(angleA, angleB):
    difference = abs(angleA - angleB)
    return (2 * math.pi) - difference if difference > math.pi else difference

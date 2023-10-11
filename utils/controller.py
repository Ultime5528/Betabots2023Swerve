import math
from typing import Optional

import numpy as np
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import Trajectory


class RearWheelFeedbackController:
    """
    Adapted from: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/rear_wheel_feedback/rear_wheel_feedback.py
    """

    def __init__(self, trajectory: Trajectory, curvature_factor=1.0, angle_factor=2.5, track_error_factor=30.0):
        self.trajectory = trajectory
        self.states = trajectory.states()
        self.poses_array = np.array([(state.pose.X(), state.pose.Y()) for state in self.states])
        self.current_pose = Pose2d()
        self.closest_t = 0.0
        self.angle_error = Rotation2d()
        self.error = 0.0
        self.omega_0 = 0.0
        self.omega_1 = 0.0
        self.closest_sample: Optional[Trajectory.State] = None
        self.curvature_factor = curvature_factor
        self.angle_factor = angle_factor
        self.track_error_factor = track_error_factor

    def _updateClosestScipy(self):
            from scipy import optimize

            def calcDistance(t, *args):
                pose = self.trajectory.sample(t).pose
                x = pose.X()
                y = pose.Y()
                current_x = self.current_pose.X()
                current_y = self.current_pose.Y()
                return (x - current_x)**2 + (y - current_y)**2

            res = optimize.minimize_scalar(calcDistance, bounds=(0, self.trajectory.totalTime()))

            self.closest_t = res.x
            self.error = res.fun
            self.closest_sample = self.trajectory.sample(self.closest_t)

    def _updateClosest(self):
        current_array = np.array([self.current_pose.X(), self.current_pose.Y()])
        diffs = self.poses_array - current_array
        dists = np.linalg.norm(diffs, axis=1)
        argmin = dists.argmin()
        min_state = self.states[argmin]
        self.closest_t = min_state.t
        self.error = dists[argmin]
        self.closest_sample = min_state

    def update(self, current_pose: Pose2d, speed=1.0, curvature_factor: Optional[float] = None, angle_factor: Optional[float] = None, track_error_factor: Optional[float] = None):
        if curvature_factor is not None:
            self.curvature_factor = curvature_factor

        if angle_factor is not None:
            self.angle_factor = angle_factor

        if track_error_factor is not None:
            self.track_error_factor = track_error_factor

        self.current_pose = current_pose
        self._updateClosest()

        curvature = self.closest_sample.curvature
        target_yaw = self.closest_sample.pose.rotation()
        delta_translation = self.closest_sample.pose.translation() - self.current_pose.translation()
        d_angle = target_yaw - delta_translation.angle()

        if d_angle.radians() < 0:
            self.error *= -1

        angle_error = self.current_pose.rotation() - self.closest_sample.pose.rotation()
        self.angle_error = angle_error

        omega = self.curvature_factor * speed * curvature * angle_error.cos() / (1.0 - curvature * self.error)
        self.omega_0 = omega
        omega -= self.angle_factor * abs(speed) * angle_error.radians()
        omega -= self.track_error_factor * speed * angle_error.sin() * self.error / angle_error.radians()
        self.omega_1 = omega

        if angle_error.radians() == 0.0 or omega == 0.0:
            delta = 0
        else:
            delta = math.atan2(omega, 1.0)

        return delta

from dataclasses import dataclass
from typing import Tuple

import wpilib
import numpy as np
from numpy import ndarray
from photonvision import PhotonTrackedTarget
from wpimath.geometry import Transform3d, Pose2d, Rotation2d

from utils.property import autoproperty


@dataclass
class Data:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    is_good: bool = False

    def update(self, pose: Pose2d, ambiguity) -> None:
        self.is_good = (ambiguity <= properties.max_ambiguity)
        if self.is_good:
            self.x = pose.x
            self.y = pose.y
            self.yaw = pose.rotation().degrees()


class Vision:
    buffer_size = autoproperty(15.0)
    min_good_tags = autoproperty(10.0)
    pos_std_deviation_max = autoproperty(2.0)
    yaw_std_deviation_max = autoproperty(10.0)

    def __init__(self):
        self.buffer = [Data() for _ in range(int(self.buffer_size))]
        self.index = 0

    def update(self, best: Pose2d, ambiguity: float) -> Pose2d | None:
        self.buffer[self.index].update(best, ambiguity)
        self.index = 0 if self.index == self.buffer_size - 1 else self.index + 1

        good_buffer = list(filter(lambda pose: pose.is_good, self.buffer))
        if len(good_buffer) >= self.min_good_tags:
            x_vals = [pose.x for pose in good_buffer]
            y_vals = [pose.y for pose in good_buffer]
            yaw_vals = [pose.yaw for pose in good_buffer]
            x_std, y_std, yaw_std = [np.std(vals) for vals in [x_vals, y_vals, yaw_vals]]
            if x_std < self.pos_std_deviation_max and y_std < self.pos_std_deviation_max and yaw_std < self.yaw_std_deviation_max:
                return Pose2d(np.mean(x_vals), np.mean(y_vals), Rotation2d.fromDegrees(np.mean(yaw_vals)))
        else:
            return None


class _ClassProperties:
    max_ambiguity = autoproperty(0.2, subtable=Vision.__name__)


properties = _ClassProperties()

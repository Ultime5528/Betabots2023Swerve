import math
from dataclasses import dataclass
from typing import Optional

__all__ = ["TrapezoidalMotion"]


@dataclass
class MotionConfig:
    start_position: Optional[float]
    end_position: Optional[float]
    displacement: Optional[float]
    start_speed: Optional[float]
    end_speed: Optional[float]
    max_speed: Optional[float]
    accel: Optional[float]

    def check_final_state(self):
        if self.end_position is None and self.displacement is None:
            raise ValueError("'end_position' and 'displacement' cannot both be None")
        elif self.end_position is not None and self.displacement is not None:
            raise ValueError("'end_position' and 'displacement' cannot both be set")


class TrapezoidalMotion:
    def __init__(
        self,
        min_speed: Optional[float] = None,
        start_speed: Optional[float] = None,
        end_speed: Optional[float] = None,
        max_speed: Optional[float] = None,
        accel: Optional[float] = None,
        start_position: Optional[float] = None,
        end_position: Optional[float] = None,
        displacement: Optional[float] = None,
    ):
        if min_speed is None:
            assert start_speed is not None and end_speed is not None
        else:
            assert start_speed is None and end_speed is None, "If min_speed is specfied, start_speed and end_speed cannot be specified"
            start_speed = min_speed
            end_speed = min_speed

        self._initial_config = MotionConfig(start_position, end_position, displacement, start_speed, end_speed, max_speed, accel)
        self._real_config: Optional[MotionConfig] = None
        self._position = None
        self._inverted = False

        if start_position is not None:
            self._compute()

    def update(
        self,
        start_position: Optional[float] = None,
        displacement: Optional[float] = None,
        end_position: Optional[float] = None,
        min_speed: Optional[float] = None,
        start_speed: Optional[float] = None,
        end_speed: Optional[float] = None,
        max_speed: Optional[float] = None,
        accel: Optional[float] = None,
    ):
        if start_position is not None:
            self._initial_config.start_position = start_position
        if displacement is not None:
            self._initial_config.displacement = displacement
        if end_position is not None:
            self._initial_config.end_position = end_position
        if min_speed is not None:
            self._initial_config.start_speed = min_speed
            self._initial_config.end_speed = min_speed
        if start_speed is not None:
            self._initial_config.start_speed = start_speed
        if end_speed is not None:
            self._initial_config.end_speed = end_speed
        if max_speed is not None:
            self._initial_config.max_speed = max_speed
        if accel is not None:
            self._initial_config.accel = accel

        self._compute()

    def _compute(self):
        assert self._initial_config.start_position is not None, "'start_position' is not set."
        assert self._initial_config.start_speed is not None, "'start_speed' is not set."
        assert self._initial_config.end_speed is not None, "'end_speed' is not set."
        assert self._initial_config.max_speed is not None, "'max_speed' is not set."
        assert self._initial_config.accel is not None, "'accel' is not set."

        self._initial_config.check_final_state()

        self._position = self._initial_config.start_position

        if self._initial_config.end_position is not None:
            end_position = self._initial_config.end_position
            displacement = end_position - self._initial_config.start_position
        else:
            displacement = self._initial_config.displacement
            end_position = self._initial_config.start_position + displacement

        self._inverted = displacement < 0
        displacement = abs(displacement)

        self._real_config = MotionConfig(
            start_position=self._initial_config.start_position,
            end_position=end_position,
            displacement=displacement,
            start_speed=abs(self._initial_config.start_speed),
            end_speed=abs(self._initial_config.end_speed),
            max_speed=abs(self._initial_config.max_speed),
            accel=abs(self._initial_config.accel),
        )

        assert self._real_config.start_speed <= self._real_config.max_speed
        assert self._real_config.end_speed <= self._real_config.max_speed

        self._start_accel_window = (self._real_config.max_speed - self._real_config.start_speed) / self._real_config.accel
        self._end_accel_window = (self._real_config.max_speed - self._real_config.end_speed) / self._real_config.accel

        if self._start_accel_window + self._end_accel_window > self._real_config.displacement:
            self._start_accel_window = self._real_config.displacement / 2 + (self._real_config.end_speed - self._real_config.start_speed) / (2 * self._real_config.accel)
            self._end_accel_window = self._real_config.displacement - self._start_accel_window
            self._real_config.max_speed = self._real_config.start_speed + self._real_config.accel * self._start_accel_window

    def setPosition(self, position: float):
        assert (
            self._real_config
        ), "Motion is not yet computed. 'start_position' and ('end_position' or 'displacement') must be set."
        self._position = position

    def getSpeed(self) -> float:
        assert self._position is not None, "Position has not been set."

        if self._inverted:
            s = self._real_config.start_position - self._position
        else:
            s = self._position - self._real_config.start_position

        v = 0

        if s < 0:
            v = self._real_config.start_speed
        elif s < self._start_accel_window:
            v = math.sqrt(
                self._real_config.start_speed ** 2
                + (s / self._start_accel_window) * (self._real_config.max_speed ** 2 - self._real_config.start_speed ** 2)
            )
        elif s < self._real_config.displacement - self._end_accel_window:
            v = self._real_config.max_speed
        elif s < self._real_config.displacement:
            v = math.sqrt(
                self._real_config.end_speed ** 2
                + (self._real_config.displacement - s) / self._end_accel_window
                * (self._real_config.max_speed ** 2 - self._real_config.end_speed ** 2)
            )
        else:
            v = self._real_config.end_speed

        if self._inverted:
            v *= -1

        return v

    def isFinished(self):
        if self._inverted:
            return self._position <= self._real_config.end_position
        else:
            return self._position >= self._real_config.end_position

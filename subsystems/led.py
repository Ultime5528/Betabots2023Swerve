import random
from enum import Enum
import math
from typing import Callable, Union, Tuple, List
import wpilib
import ports
import numpy as np

from utils.property import autoproperty
from utils.safesubsystem import SafeSubsystem


def interpolate(t, color1, color2):
    assert 0 <= t <= 1
    return ((1 - t) * color1 + t * color2).astype(int)


def numpy_interpolation(t: np.ndarray, color1: np.ndarray, color2: np.ndarray):
    assert 0 <= t.min() and t.max() <= 1
    return ((1 - t)[:, np.newaxis] * color1 + t[:, np.newaxis] * color2).astype(int)


Color = Union[np.ndarray, Tuple[int, int, int], List[int]]


class ModeLED(Enum):
    NONE = "none"
    CONE = "cone"
    CUBE = "cube"


class LEDController(SafeSubsystem):
    # HSV: [Hue(color 0 to 180), Saturation( amount of gray 0 to 255), Value(brightness 0 to 255)
    red_rgb = np.array([255, 0, 0])
    blue_rgb = np.array([0, 0, 255])
    sky_blue_rgb = np.array([0, 205, 255])
    purple_rgb = np.array([150, 0, 200])
    violet_rgb = np.array([205, 0, 255])
    yellow_rgb = np.array([255, 255, 0])
    orange_rgb = np.array([255, 100, 0])
    black = np.array([0, 0, 0])
    white = np.array([255, 255, 255])
    beige_rgb = np.array([225, 198, 153])

    led_number = 203

    speed = autoproperty(1.25)
    white_length = autoproperty(6.0)
    color_period = autoproperty(20.0)
    brightness = autoproperty(100)

    last = 0

    def __init__(self):
        super().__init__()
        self.led_strip = wpilib.AddressableLED(ports.led_strip)
        self.buffer = [wpilib.AddressableLED.LEDData() for _ in range(self.led_number)]
        self.led_strip.setLength(len(self.buffer))
        self.time = 0
        self.explosiveness = 0.0
        self.led_strip.start()
        self.mode = ModeLED.NONE

    def setRGB(self, i: int, color: Color):
        brightness = max(min(100, self.brightness), 0) / 100
        color = (color * brightness).astype(int)
        self.buffer[i].setRGB(*color)

    def dim(self, x):
        return round(x * max(min(1, self.brightness), 0))

    def setAll(self, color_func: Callable[[int], Color]):
        a = np.arange(len(self.buffer))
        for i in np.nditer(a):
            self.setRGB(i, color_func(i))

    def pulse(self):
        # Convert percentage to good value
        brightness = max(min(100, self.brightness), 0) / 100

        pixel_value = abs(round(255 * math.cos((self.time / (18 * math.pi)))))

        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            r = round(pixel_value * brightness)
            g = 0
            b = 0
        else:
            r = 0
            g = 0
            b = round(pixel_value * brightness)

        a = np.arange(self.led_number)
        for i in np.nditer(a):
            self.buffer[i].setRGB(r, g, b)

    def selectTeam(self):
        brightness = max(min(100, self.brightness), 0) / 100
        pixel_value = round((255 * math.cos((self.time / (18 * math.pi)))) * brightness)
        if pixel_value >= 0:
            r = pixel_value
            g = 0
            b = 0
        else:
            r = 0
            g = 0
            b = abs(pixel_value)

        for i in range(self.led_number):
            self.buffer[i].setRGB(r, g, b)

    def gradient(self):
        brightness = max(min(100, self.brightness), 0) / 100
        color = self.getAllianceColor()

        i_values = np.arange(self.led_number)
        y_values = 0.5 * np.sin(2 * math.pi ** 2 * (i_values - 2 * self.time) / 200) + 0.5

        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            color1 = numpy_interpolation(y_values, color, numpy_interpolation(y_values, color, self.purple_rgb))
            color2 = numpy_interpolation(y_values, color, numpy_interpolation(y_values, color, self.sky_blue_rgb))
            final_colors = numpy_interpolation(y_values, color1, color2)
            final_colors = (final_colors * brightness).astype(int)
            for i, y in enumerate(final_colors):
                self.buffer[i].setRGB(*y)
        elif wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            color1 = numpy_interpolation(y_values, color, numpy_interpolation(y_values, color, self.orange_rgb))
            color2 = numpy_interpolation(y_values, color, numpy_interpolation(y_values, color, self.beige_rgb))
            final_colors = numpy_interpolation(y_values, color1, color2)
            final_colors = (final_colors * brightness).astype(int)
            for i, y in enumerate(final_colors):
                self.buffer[i].setRGB(*y)
        else:
            for i in i_values:
                self.buffer[i].setRGB(0, 0, 0)

    def halfWaves(self, color):
        brightness = max(min(100, self.brightness), 0) / 100
        i_values = np.arange(self.led_number)
        prop = 0.5 * np.cos((2 * math.pi / 50) * (self.time + i_values)) + 0.5
        t_values = np.zeros_like(prop)
        for i in i_values:
            if self.last - prop[i] <= 0:
                t_values[i] = prop[i]
            else:
                t_values[i] = abs(prop[i] - 1)
            self.last = prop[i]
        y_values = numpy_interpolation(t_values, color, self.black)
        y_values = (y_values * brightness).astype(int)
        for i, y in enumerate(y_values):
            self.buffer[i].setRGB(*y)

    def flash(self, color, speed):
        brightness = max(min(100, self.brightness), 0) / 100
        color = (color * brightness).astype(int)
        i_values = np.arange(self.led_number)
        if self.time % speed == 0:
            if self.time % (speed * 2) == 0:
                for i in i_values:
                    self.buffer[i].setRGB(*color)
            else:
                for i in i_values:
                    self.buffer[i].setRGB(0, 0, 0)

    def explode(self, color):
        brightness = max(min(100, self.brightness), 0) / 100
        color = (color * brightness).astype(int)
        if self.time % 3 == 0:
            def getColor(i: int):
                y = random.random()
                if y <= self.explosiveness:
                    return interpolate(y / self.explosiveness, color, np.array([0, 0, 0]))
                else:
                    return self.black

            self.explosiveness -= 0.02
            self.setAll(getColor)

    def getAllianceColor(self):
        alliance = wpilib.DriverStation.getAlliance()
        if alliance == wpilib.DriverStation.Alliance.kInvalid:
            color = self.black
        elif alliance == wpilib.DriverStation.Alliance.kRed:
            color = self.red_rgb
        else:  # kBlue
            color = self.blue_rgb
        return color

    def getModeColor(self):
        if self.mode == ModeLED.CUBE:
            return self.purple_rgb
        elif self.mode == ModeLED.CONE:
            return self.yellow_rgb
        else:
            return self.getAllianceColor()

    def teleop(self):
        brightness = max(min(100, self.brightness), 0) / 100
        a = 1 / (1 - math.cos(math.pi * self.white_length / self.color_period))
        k = 1 - a

        i_values = np.arange(self.led_number)
        y_values = a * np.sin(2 * math.pi / self.color_period * (i_values - self.speed * self.time)) + k
        y_values = np.maximum(y_values, 0)
        y_values = numpy_interpolation(y_values, self.getModeColor(), self.white)
        y_values = (y_values * brightness).astype(int)
        for i, y in enumerate(y_values):
            self.buffer[i].setRGB(*y)

    def e_stopped(self):
        interval = 10
        flash_time = 20
        state = round(self.time / flash_time) % 2

        def getColor(i: int):
            is_color = state - round(i / interval) % 2
            if is_color:
                return self.red_rgb
            else:
                return self.black

        self.setAll(getColor)

    def setMode(self, mode: ModeLED):
        self.mode = mode

    def periodic(self) -> None:
        start_time = wpilib.getTime()

        self.time += 1
        if wpilib.DriverStation.isEStopped():
            self.e_stopped()
        elif self.explosiveness > 0.0:
            self.explode(self.getAllianceColor())
        else:
            if wpilib.DriverStation.isAutonomousEnabled():  # auto
                self.gradient()
            elif wpilib.DriverStation.isTeleopEnabled():  # teleop
                if wpilib.DriverStation.getMatchTime() == -1.0 or wpilib.DriverStation.getMatchTime() > 30:
                    self.teleop()
                elif wpilib.DriverStation.getMatchTime() > 25:
                    self.flash(self.getAllianceColor(), 10)
                elif wpilib.DriverStation.getMatchTime() > 1:
                    self.halfWaves(self.getModeColor())
                else:
                    self.explosiveness = 1
                    self.explode(self.getAllianceColor())
            else:  # game hasn't started
                if wpilib.DriverStation.isDSAttached():
                    self.pulse()
                else:
                    self.selectTeam()

        self.led_strip.setData(self.buffer)
        wpilib.SmartDashboard.putNumber("led_time", wpilib.getTime() - start_time)

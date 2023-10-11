from subsystems.led import LEDController, ModeLED
from utils.safecommand import SafeCommand


class SignalCube(SafeCommand):
    def __init__(self, led_controller: LEDController):
        super().__init__()
        self.led_controller = led_controller
        self.addRequirements(self.led_controller)

    def initialize(self) -> None:
        self.led_controller.setMode(ModeLED.CUBE)

    def end(self, interrupted: bool) -> None:
        self.led_controller.setMode(ModeLED.NONE)


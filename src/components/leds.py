from wpilib import Color
from magicbot import will_reset_to, feedback

from components.arm_control import ArmControl
from components.climber import Climber
from components.claw import Claw

from lemonlib.util import LEDController, AlertManager, AlertType


class LEDStrip:
    climber: Climber
    claw: Claw

    leds: LEDController

    justin_bool = will_reset_to(False)

    """
    INITIALIZATION METHODS
    """

    def setup(self):
        self.leds.set_solid_color((10, 10, 10))
        self.coral_detected = (0, 0, 255)
        self.aligned_branch = (0, 255, 0)
        self.fully_climbed = (255, 0, 255)
        self.error_color = (255, 0, 0)
        self.warning_color = (255, 255, 0)

    def on_disable(self):
        self.leds.set_solid_color((10, 10, 10))

    """
    INFORMATIONAL METHODS
    """

    @feedback
    def get_color(self) -> str:
        """Returns color of first LED as a hex string"""
        return Color(
            self.leds.buffer[0].r, self.leds.buffer[0].g, self.leds.buffer[0].b
        ).hexString()

    @feedback
    def has_warnings_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.WARNING)) > 0

    @feedback
    def has_errors_present(self) -> bool:
        return len(AlertManager.get_strings(AlertType.ERROR)) > 0

    """
    CONTROL METHODS
    """

    def justin_fun(self):
        self.justin_bool = True

    """
    EXECUTE
    """

    def execute(self):
        if self.has_errors_present():
            self.leds.set_solid_color(self.error_color)
        elif self.has_warnings_present:
            self.leds.set_solid_color(self.warning_color)
        elif self.climber.is_deployed():
            self.leds.set_solid_color(self.fully_climbed)
        elif self.claw.get_intake_limit():
            self.leds.set_solid_color(self.coral_detected)
        elif self.justin_bool:
            self.leds.scolling_rainbow(6)
        else:
            self.leds.move_across((255, 255, 0), 15, 12)
